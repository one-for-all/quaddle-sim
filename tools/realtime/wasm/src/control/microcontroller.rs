// The controller itself, plus a constructor taking the images as bytes: in
// wasm `new` fetches them, which Node has no page to fetch from.
include!("../../../../../src/control/microcontroller.rs");

impl QuaddleESP32S3Controller {
    /// What `new` builds, from images handed in rather than fetched.
    pub fn from_images(rom: Vec<u8>, boot: Vec<u8>, part: Vec<u8>, app: Vec<u8>, symbols: Symbols) -> Self {
        let mut esp32s3 = ESP32S3::new(rom, boot, part, app, symbols);
        let imu = Rc::new(RefCell::new(ICM42670::new(true)));
        esp32s3.attach_i2c_device(IMU_BUS, Box::new(ImuOnBus(imu.clone())));
        Self {
            esp32s3,
            leg_servos: std::array::from_fn(|_| PetoiP1L::new()),
            imu,
            imu_track: None,
            uart_payload: VecDeque::new(),
            pending: 0.,
        }
    }
}
