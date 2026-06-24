use esp32rs::servo::petoi_p1s::PetoiP1S;
use gorilla_physics::{
    hybrid::{articulated::Articulated, control::ArticulatedController},
    na::DVector,
    types::Float,
};

pub(crate) mod microcontroller;

pub struct QuaddleController {
    pub servos: [PetoiP1S; 4],

    t: Float,
}

const DT: Float = 1. / 60. / 100.;

impl QuaddleController {
    pub fn new() -> Self {
        let mut servos = [PetoiP1S::new(); 4];
        // servos[0].command_angle = Some((0. as Float).to_radians());
        let command_angles = [0, 0, 0, 0];
        for (i, servo) in servos.iter_mut().enumerate() {
            servo.command_angle = Some((command_angles[i] as Float).to_radians());
        }

        Self { servos, t: 0. }
    }
}

impl ArticulatedController for QuaddleController {
    fn control(&mut self, articulated: &Articulated, input: &Vec<Float>) -> DVector<Float> {
        let command_angle = self.t.sin() * 30.;
        self.servos[0].command_angle = Some(command_angle.to_radians());

        let command_angle = -self.t.sin() * 45. + 45.;
        self.servos[3].command_angle = Some(command_angle.to_radians());

        let mut torques = vec![];

        let qs = articulated.q();
        let vs = articulated.v();

        let dof = vs.len();
        for _ in 0..dof {
            torques.push(0.); // default to 0 torque for unactuated joints
        }

        let actuated_joint_indices = [1, 6, 11, 16];

        for i in 0..self.servos.len() {
            let index = actuated_joint_indices[i];
            let q = qs[index];
            let v = vs[index];
            self.servos[i].angle = q;
            self.servos[i].vel = v;
            let torque = self.servos[i].torque() * 0.1;
            // println!("q: {}, v: {}, tau: {}", q, v, torque);
            torques[index] = torque;
        }

        let damped_joint_indices = [0, 5, 10, 15];
        for index in damped_joint_indices {
            let v = vs[index];
            let torque = -2e-3 * v;
            torques[index] += torque;
        }

        self.t += DT;

        DVector::from_vec(torques)
    }
}
