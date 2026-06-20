use esp32rs::servo::petoi_p1s::PetoiP1S;
use gorilla_physics::{
    hybrid::{articulated::Articulated, control::ArticulatedController},
    na::DVector,
    types::Float,
};

pub struct QuaddleController {
    pub servos: [PetoiP1S; 1],
}

impl QuaddleController {
    pub fn new() -> Self {
        let mut servos = [PetoiP1S::new(); 1];
        // servos[0].command_angle = Some((0. as Float).to_radians());
        servos[0].command_angle = Some((-90. as Float).to_radians());

        Self { servos }
    }
}

impl ArticulatedController for QuaddleController {
    fn control(&mut self, articulated: &Articulated, input: &Vec<Float>) -> DVector<Float> {
        let mut torques = vec![];

        let qs = articulated.q();
        let vs = articulated.v();

        let dof = vs.len();
        for _ in 0..dof {
            torques.push(0.); // default to 0 torque for unactuated joints
        }

        let actuated_joint_indices = [1];

        for i in 0..self.servos.len() {
            let index = actuated_joint_indices[i];
            let q = qs[index];
            let v = vs[index];
            self.servos[i].angle = q;
            self.servos[i].vel = v;
            let mult = if i == 0 { 0.1 } else { 0.1 };
            let torque = self.servos[i].torque() * 0.1;
            // println!("q: {}, v: {}, tau: {}", q, v, torque);
            torques[index] = torque;
        }

        let damped_joint_indices = [0];
        for index in damped_joint_indices {
            let v = vs[index];
            let torque = -2e-3 * v;
            torques[index] += torque;
        }

        DVector::from_vec(torques)
    }
}
