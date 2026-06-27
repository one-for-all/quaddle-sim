use std::collections::VecDeque;

use esp32rs::{
    esp32::CPU_FREQUENCY,
    esp32s3::esp32s3::{ESP32S3, S3_CPU_SLOWDOWN_FACTOR},
    servo::petoi_p1l::PetoiP1L,
    symbols::Symbols,
};
use gorilla_physics::{
    hybrid::{articulated::Articulated, control::ArticulatedController},
    joint::Joint,
    na::DVector,
    types::Float,
};

pub(crate) struct QuaddleESP32S3Controller {
    esp32s3: ESP32S3,
    leg_servos: [PetoiP1L; 4],

    uart_payload: VecDeque<u8>, // data pending to be fed into esp32 uart0
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

        let esp32s3 = ESP32S3::new(
            rom_data,
            bootloader_data,
            partition_table_data,
            app_data,
            symbols,
        );

        Self {
            esp32s3,
            leg_servos: [PetoiP1L::new(); 4],
            uart_payload: VecDeque::new(),
        }
    }
}

impl ArticulatedController for QuaddleESP32S3Controller {
    fn step(&mut self, dt: Float, articulated: &Articulated) {
        let cpu_freq_hz = (CPU_FREQUENCY * 1000_000 / S3_CPU_SLOWDOWN_FACTOR) as Float;
        let cycle_dt = 1. / cpu_freq_hz;
        let n_steps = (dt / cycle_dt) as usize;

        let pins = [11, 13, 14, 10];

        for _ in 0..n_steps {
            self.esp32s3.step(cycle_dt);
            if let Some(char) = self.uart_payload.pop_front() {
                self.esp32s3.feed_uart(char);
            }

            for (i, servo) in self.leg_servos.iter_mut().enumerate() {
                if let Some(pin) = self.esp32s3.read_pin(pins[i]) {
                    servo.step(cycle_dt, pin);
                } else {
                    servo.step(cycle_dt, false);
                }
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
            servo.angle = q;
            servo.vel = v;
            let torque = servo.torque() * 0.2;
            let damping = 0.; // -1e-4 * v;
            torques[joint_index] += torque + damping;
        }

        for joint_index in damped_joint_indices {
            let v = vs[joint_index];

            // Angle-dependent damping
            // let offset = if body_dof != 0 { 1 } else { 0 };
            // let q = qs[joint_index + offset];
            // let range = (30 as Float).to_radians();
            // let d_max = -1e-4;
            // let d_min = -1e-4;
            // let torque = if (-range..range).contains(&q) {
            //     (d_max - d_min) * (1. - q.abs() / range) + d_min
            // } else {
            //     d_min
            // } * v;

            let torque = -1e-4 * v;
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

    /// Return the content in UART
    fn get_uart(&self) -> String {
        self.esp32s3.get_uart()
    }

    /// Send UART data to esp32
    fn send_uart(&mut self, payload: &str) {
        self.uart_payload.extend(String::from(payload).into_bytes());
    }
}
