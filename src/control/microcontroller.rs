use std::{cell::RefCell, collections::VecDeque, rc::Rc};

use esp32rs::{
    esp32::CPU_FREQUENCY,
    esp32s3::{
        esp32s3::{ESP32S3, S3_CPU_SLOWDOWN_FACTOR},
        i2c_device::I2cDevice,
    },
    symbols::Symbols,
};
use gibbon_electronics::{
    SignalLevel::{HIGH, LOW},
    imu::icm42670::ICM42670,
    servo::petoi_p1l::PetoiP1L,
};
use gorilla_physics::{
    GRAVITY,
    hybrid::{Rigid, articulated::Articulated, control::ArticulatedController},
    joint::Joint,
    na::{DVector, Vector3},
    types::Float,
};

/// The I2C controller the Q_Motion board hangs its IMU off: `Wire1`, on
/// SDA=34/SCL=33, a bus of its own (`IMU_SDA`/`IMU_SCL` in OpenCat.h).
const IMU_BUS: usize = 1;

pub(crate) struct QuaddleESP32S3Controller {
    esp32s3: ESP32S3,
    leg_servos: [PetoiP1L; 4],

    /// The ICM-42670 on the board, AP_AD0 high (0x69). Shared: the chip's bus
    /// reads its registers, and this controller tells it how the body moves.
    imu: Rc<RefCell<ICM42670>>,
    /// The velocity of the IMU at the previous step, world frame, from which
    /// its acceleration is differenced. `None` until there is one.
    imu_last_velocity: Option<Vector3<Float>>,

    uart_payload: VecDeque<u8>, // data pending to be fed into esp32 uart0
}

/// The IMU as the chip's I2C bus sees it.
struct ImuOnBus(Rc<RefCell<ICM42670>>);

impl I2cDevice for ImuOnBus {
    fn address(&self) -> u8 {
        self.0.borrow().address()
    }
    fn start(&mut self, read: bool) {
        self.0.borrow_mut().i2c_start(read);
    }
    fn write(&mut self, byte: u8) {
        self.0.borrow_mut().i2c_write(byte);
    }
    fn read(&mut self) -> u8 {
        self.0.borrow_mut().i2c_read()
    }
    fn stop(&mut self) {
        self.0.borrow_mut().i2c_stop();
    }
}

/// A body-frame vector in the IMU's axes. The body frame of the URDF has the
/// head along -y and the left side along +x (the front thighs sit at
/// y = -0.039, the left ones at x = +0.024), while the firmware reads the
/// chip as x forward, y left, z up -- the frame `getImuException` is written
/// in, and the only one its flip test (|roll| > 85, z below -0.1 g) and lift
/// test (|pitch| > 80) make sense in.
fn body_to_imu(v: Vector3<Float>) -> [Float; 3] {
    [-v.y, v.x, v.z]
}

impl QuaddleESP32S3Controller {
    pub async fn new() -> Self {
        let project = "OpenCatEsp32S3";

        let mut symbols = Symbols::new(); // symbols for printing
        let rom_data: Vec<u8>;
        let bootloader_data: Vec<u8>;
        let partition_table_data: Vec<u8>;
        let app_data: Vec<u8>;

        #[cfg(not(target_arch = "wasm32"))]
        {
            use std::fs;

            use gorilla_physics::util::read_file;

            rom_data = fs::read("rom/wokwi/esp32s3-rom.bin").unwrap();
            symbols.add(&read_file("rom/symbols.txt"));

            bootloader_data =
                fs::read("OpenCatEsp32/build/OpenCatEsp32.ino.bootloader.bin").unwrap();
            partition_table_data =
                fs::read("OpenCatEsp32/build/OpenCatEsp32.ino.partitions.bin").unwrap();
            app_data = fs::read("OpenCatEsp32/build/OpenCatEsp32.ino.bin").unwrap();
            symbols.add(&read_file("OpenCatEsp32/build/symbols.txt"));
            symbols.add(&read_file("OpenCatEsp32/bootloader_symbols.txt"));
        }

        #[cfg(target_arch = "wasm32")]
        {
            use gorilla_physics::interface::util::read_web_file;
            use gorilla_physics::interface::util::read_web_file_bytes;

            rom_data = read_web_file_bytes("rom/wokwi/esp32s3-rom.bin").await;
            symbols.add(&read_web_file("rom/esp32s3_rom_symbols.txt").await);

            let build_dir = format!("{}/build", project);

            bootloader_data =
                read_web_file_bytes(&format!("{}/{}.ino.bootloader.bin", build_dir, project)).await;
            partition_table_data =
                read_web_file_bytes(&format!("{}/{}.ino.partitions.bin", build_dir, project)).await;
            app_data = read_web_file_bytes(&format!("{}/{}.ino.bin", build_dir, project)).await;
            symbols.add(&read_web_file(&format!("{}/symbols.txt", build_dir)).await);
            symbols.add(&read_web_file(&format!("{}/bootloader_symbols.txt", build_dir)).await);
        }

        let mut esp32s3 = ESP32S3::new(
            rom_data,
            bootloader_data,
            partition_table_data,
            app_data,
            symbols,
        );
        let imu = Rc::new(RefCell::new(ICM42670::new(true)));
        esp32s3.attach_i2c_device(IMU_BUS, Box::new(ImuOnBus(imu.clone())));

        Self {
            esp32s3,
            leg_servos: std::array::from_fn(|_| PetoiP1L::new()),
            imu,
            imu_last_velocity: None,
            uart_payload: VecDeque::new(),
        }
    }

    /// Tell the IMU how the body is moving, and let it sample.
    fn sense_motion(&mut self, dt: Float, articulated: &Articulated) {
        let (specific_force, angular_velocity, velocity) =
            imu_motion(&articulated.bodies[0], self.imu_last_velocity, dt);
        self.imu_last_velocity = Some(velocity);

        let mut imu = self.imu.borrow_mut();
        imu.measure(specific_force, angular_velocity);
        imu.step(dt);
    }
}

/// What the IMU on `body` feels -- specific force in m/s^2 and angular
/// velocity in rad/s, in the chip's axes -- along with the chip's velocity in
/// the world frame, which the next step differences against.
///
/// The chip is taken to sit at the body frame's origin, on the main board; a
/// few centimetres either way only changes the rotational part of what it
/// feels, which is small next to gravity at the rates this robot turns. Its
/// acceleration is the step-to-step change in its velocity: that is all a
/// velocity-level solver has, and it is what the chip's own low-pass smooths
/// contact impulses out of.
fn imu_motion(
    body: &Rigid,
    last_velocity: Option<Vector3<Float>>,
    dt: Float,
) -> ([Float; 3], [Float; 3], Vector3<Float>) {
    let rotation = body.pose.rotation;
    let position = body.pose.translation;
    // The twist's linear part is the velocity of the point at the world
    // origin; carry it over to the body.
    let velocity = body.twist.linear + body.twist.angular.cross(&position);

    let acceleration = match last_velocity {
        Some(last) if dt > 0. => (velocity - last) / dt,
        _ => Vector3::zeros(),
    };

    // What an accelerometer senses is the acceleration minus gravity: +1 g up
    // when nothing is moving.
    let specific_force = acceleration + Vector3::new(0., 0., GRAVITY);
    let specific_force = rotation.inverse_transform_vector(&specific_force);
    let angular_velocity = rotation.inverse_transform_vector(&body.twist.angular);

    (
        body_to_imu(specific_force),
        body_to_imu(angular_velocity),
        velocity,
    )
}

impl ArticulatedController for QuaddleESP32S3Controller {
    fn step(&mut self, dt: Float, articulated: &Articulated) {
        let cpu_freq_hz = (CPU_FREQUENCY * 1000_000 / S3_CPU_SLOWDOWN_FACTOR) as Float;
        let cycle_dt = 1. / cpu_freq_hz;
        let n_steps = (dt / cycle_dt) as usize;

        let pins = [11, 13, 14, 10];

        self.sense_motion(dt, articulated);

        for _ in 0..n_steps {
            self.esp32s3.step(cycle_dt);
            if let Some(char) = self.uart_payload.pop_front() {
                self.esp32s3.feed_uart(char);
            }

            for (i, servo) in self.leg_servos.iter_mut().enumerate() {
                // An unreadable pin is an idle one, i.e. no pulse.
                let pin = self.esp32s3.read_pin(pins[i]).unwrap_or(false);
                servo.read(if pin { HIGH } else { LOW });
                servo.step(cycle_dt);
            }
        }
    }

    fn control(&mut self, articulated: &Articulated, input: &Vec<Float>) -> DVector<Float> {
        let mut torques = vec![];
        let body_dof = if let Joint::FloatingJoint(_) = articulated.joints[0] {
            6
        } else {
            0
        };

        let qs = articulated.q();
        let vs = articulated.v();

        let dof = vs.len();
        for _ in 0..dof {
            torques.push(0.); // default to 0 torque for unactuated joints
        }

        let leg_dof = 6;
        let mut index = body_dof;
        let mut actuated_joint_indices = vec![];
        let mut damped_joint_indices = vec![];
        let mut wheel_joint_indices = vec![];
        for _ in 0..4 {
            actuated_joint_indices.push(index + 1);
            damped_joint_indices.push(index);
            wheel_joint_indices.push(index + 4);
            index += leg_dof;
        }
        // let actuated_joint_indices = [1, 6, 11, 16];
        // let damped_joint_indices = [0, 5, 10, 15];

        for (i, servo) in self.leg_servos.iter_mut().enumerate() {
            let joint_index = actuated_joint_indices[i];
            let offset = if body_dof != 0 { 1 } else { 0 };
            let q = qs[joint_index + offset];
            let v = vs[joint_index];
            servo.measure(q, v);
            let torque = servo.torque();
            let damping = 0.; // -1e-4 * v;
            torques[joint_index] += torque + damping;
        }

        for joint_index in damped_joint_indices {
            let v = vs[joint_index];

            // // Angle-dependent damping
            // let offset = if body_dof != 0 { 1 } else { 0 };
            // let q = qs[joint_index + offset];
            // let range = (20 as Float).to_radians();
            // let d_max = -2e-3;
            // let d_min = -1e-4;
            // let torque = if (-range..range).contains(&q) {
            //     (d_max - d_min) * (1. - q.abs() / range) + d_min
            // } else {
            //     d_min
            // } * v;

            let torque = -2e-3 * v;
            torques[joint_index] += torque;
        }

        // virtual springs at the wheel joint
        for joint_index in wheel_joint_indices {
            let offset = if body_dof != 0 { 1 } else { 0 };
            let q = qs[joint_index + offset];
            let diff = q - (30 as Float).to_radians();
            let torque = -2e-3 * diff;
            torques[joint_index] += torque;
        }

        // // Damping on all joints
        // for joint_index in body_dof..dof {
        //     let v = vs[joint_index];
        //     torques[joint_index] += -1e-4 * v;
        // }

        DVector::from_vec(torques)
    }

    fn reboot_esp32(&mut self, app_bin: Vec<u8>, new_symbols: &str) {
        // TODO: add reboot function to ESP32S3
        let mut symbols = self.esp32s3.symbols.clone();
        symbols.add(new_symbols);

        let rom_data: Vec<u8> = self.esp32s3.rom_data.clone();
        let bootloader_data: Vec<u8> = self.esp32s3.bootloader_data.clone();
        let partition_table_data: Vec<u8> = self.esp32s3.partition_table_data.clone();
        let app_data: Vec<u8> = app_bin;

        let mut esp32s3 = ESP32S3::new(
            rom_data,
            bootloader_data,
            partition_table_data,
            app_data,
            symbols,
        );
        // The same part, on the same bus: only the chip was replaced.
        esp32s3.attach_i2c_device(IMU_BUS, Box::new(ImuOnBus(self.imu.clone())));

        self.esp32s3 = esp32s3;
        for servo in self.leg_servos.iter_mut() {
            servo.reset();
        }
        self.imu_last_velocity = None;
    }

    fn reboot(&mut self, _code: &str) {
        self.esp32s3.reboot();
        for servo in self.leg_servos.iter_mut() {
            servo.reset();
        }
        self.imu_last_velocity = None;
    }

    /// Return the content in UART
    fn get_uart(&self) -> String {
        self.esp32s3.get_uart()
    }

    /// Send UART data to esp32
    fn send_uart(&mut self, payload: &str) {
        self.uart_payload.extend(String::from(payload).into_bytes());
    }
}

#[cfg(test)]
mod imu_motion_tests {
    use gorilla_physics::{
        GRAVITY, PI,
        hybrid::Rigid,
        na::{UnitQuaternion, Vector3},
        spatial::{pose::Pose, spatial_vector::SpatialVector},
        types::Float,
    };

    use super::imu_motion;

    /// The body at the spawn point, turned to `rotation` and spinning at
    /// `angular` (world frame).
    fn body_at(rotation: UnitQuaternion<Float>, angular: Vector3<Float>) -> Rigid {
        let mut body = Rigid::new_cuboid(0.1, 0.071, 0.105, 0.027, "body");
        body.pose = Pose {
            rotation,
            translation: Vector3::new(0., -0.15, 0.88),
        };
        body.twist = SpatialVector::new(angular, Vector3::zeros());
        body
    }

    fn assert_close(a: [Float; 3], b: [Float; 3]) {
        for (a, b) in a.iter().zip(b) {
            assert!((a - b).abs() < 1e-9, "{a:?} != {b:?}");
        }
    }

    /// Standing as `build_quaddle` spawns it, yawed -90 degrees: yaw does not
    /// change what the chip feels.
    #[test]
    fn standing_reads_one_g_up() {
        let body = body_at(
            UnitQuaternion::from_euler_angles(0., 0., -PI / 2.),
            Vector3::zeros(),
        );
        let (force, rate, _) = imu_motion(&body, None, 1e-3);
        assert_close(force, [0., 0., GRAVITY]);
        assert_close(rate, [0., 0., 0.]);
    }

    /// Rolled onto its back about the head-to-tail axis: gravity reverses on
    /// the chip's z, which with the roll the firmware's filter integrates is
    /// what its fall-over test looks for.
    #[test]
    fn on_its_back_reads_one_g_down() {
        let yaw = UnitQuaternion::from_euler_angles(0., 0., -PI / 2.);
        let roll = UnitQuaternion::from_axis_angle(&-Vector3::y_axis(), PI);
        let body = body_at(yaw * roll, Vector3::zeros());
        let (force, _, _) = imu_motion(&body, None, 1e-3);
        assert_close(force, [0., 0., -GRAVITY]);
    }

    /// Head down: the specific force points up, out of the tail, so the
    /// chip's +x (forward) reads -1 g -- the forward axis really is the
    /// URDF's -y.
    #[test]
    fn nose_down_reads_gravity_along_forward() {
        // 90 degrees about the body's +x (left) axis takes +y to +z, so the
        // head (-y) to -z.
        let pitch = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), PI / 2.);
        let body = body_at(pitch, Vector3::zeros());
        let (force, _, _) = imu_motion(&body, None, 1e-3);
        assert_close(force, [-GRAVITY, 0., 0.]);
    }

    /// Rolling about the head-to-tail axis shows on the chip's x gyro.
    #[test]
    fn rolling_reads_on_the_forward_gyro() {
        let yaw = UnitQuaternion::from_euler_angles(0., 0., -PI / 2.);
        let head = yaw * -Vector3::y(); // the head direction, in the world

        let body = body_at(yaw, head * 2.);
        let (_, rate, _) = imu_motion(&body, None, 1e-3);
        assert_close(rate, [2., 0., 0.]);
    }

    /// Falling freely, an accelerometer reads nothing.
    #[test]
    fn free_fall_reads_nothing() {
        let body = body_at(UnitQuaternion::identity(), Vector3::zeros());
        let dt = 1e-3;
        let last = Some(Vector3::new(0., 0., GRAVITY * dt));
        let (force, _, _) = imu_motion(&body, last, dt);
        assert_close(force, [0., 0., 0.]);
    }
}
