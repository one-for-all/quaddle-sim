#[cfg(any(target_arch = "wasm32", rust_analyzer))]
use gorilla_physics::hybrid::control::NullArticulatedController;
use gorilla_physics::{
    PI, WORLD_FRAME,
    hybrid::{
        Hybrid, Rigid,
        articulated::Articulated,
        mesh::URDFMeshes,
        rigid::helper::{build_joint, build_revolute_constraint, build_rigid},
    },
    joint::{
        Joint,
        constraint::{Constraint, RangeConstraint, RelativeRangeConstraint},
    },
    na::Vector3,
    spatial::transform::Transform3D,
    types::Float,
};
use urdf_rs::Robot;

#[cfg(any(target_arch = "wasm32", rust_analyzer))]
use crate::control::QuaddleController;

#[cfg(any(target_arch = "wasm32", rust_analyzer))]
use {
    gorilla_physics::interface::{hybrid::InterfaceHybrid, util::read_web_file},
    wasm_bindgen::prelude::wasm_bindgen,
};

pub fn build_quaddle(meshes: &mut URDFMeshes, urdf: &Robot) -> Hybrid {
    let mut state = Hybrid::empty();

    let body_frame = "body";
    let body = build_rigid(body_frame, "body", urdf, meshes);
    let body_joint = Joint::new_fixed(Transform3D::new_xyz_rpy(
        body_frame,
        WORLD_FRAME,
        &vec![0., 0., 0.],
        &vec![0., 0., -PI / 2.],
    ));

    // left-front
    let (lf_frames, lf_rigids, lf_joints) = build_leg("left", "front", body_frame, urdf, meshes);

    // right-front
    let (rf_frames, rf_rigids, rf_joints) = build_leg("right", "front", body_frame, urdf, meshes);

    // right-back
    let (rb_frames, rb_rigids, rb_joints) = build_leg("right", "back", body_frame, urdf, meshes);

    // left-back
    let (lb_frames, lb_rigids, lb_joints) = build_leg("left", "back", body_frame, urdf, meshes);

    let mut articulated = Articulated::new(
        vec![body]
            .into_iter()
            .chain(lf_rigids)
            .chain(rf_rigids)
            .chain(rb_rigids)
            .chain(lb_rigids)
            .chain(vec![
                // closing_1,
                // closing_2,
            ])
            .collect(),
        vec![body_joint]
            .into_iter()
            .chain(lf_joints)
            .chain(rf_joints)
            .chain(rb_joints)
            .chain(lb_joints)
            .chain(vec![
                // closing_1_joint,
                // closing_2_joint,
            ])
            .collect(),
    );

    add_constraints("left", "front", &lf_frames, &mut articulated, urdf);
    add_constraints("right", "front", &rf_frames, &mut articulated, urdf);
    add_constraints("right", "back", &rb_frames, &mut articulated, urdf);
    add_constraints("left", "back", &lb_frames, &mut articulated, urdf);

    state.add_articulated(articulated);

    state
}

#[cfg(any(target_arch = "wasm32", rust_analyzer))]
#[allow(non_snake_case)]
#[wasm_bindgen]
pub async fn createQuaddle() -> InterfaceHybrid {
    let urdf_path = "robot.urdf";
    let urdf_file = read_web_file(urdf_path).await;
    let urdf_robot = urdf_rs::read_from_string(&urdf_file).unwrap();

    let mut meshes = URDFMeshes::new(&urdf_robot).await;

    let mut state = build_quaddle(&mut meshes, &urdf_robot);

    let controller = QuaddleController::new();
    state.set_controller(0, controller);

    InterfaceHybrid::new(state)
}

// side be "left" or "right"
// direction be "front" or "back"
fn build_leg(
    side: &str,
    direction: &str,
    body_frame: &str,
    urdf: &Robot,
    meshes: &mut URDFMeshes,
) -> (Vec<String>, Vec<Rigid>, Vec<Joint>) {
    let name = format!("{}_{}", side, direction);

    // left-front
    let thigh_frame = format!("{}_thigh", name); // same as link name
    let thigh = build_rigid(&thigh_frame, &thigh_frame, urdf, meshes);
    let thigh_joint = build_joint(
        &thigh_frame,
        body_frame,
        &format!("{}_thigh", name),
        urdf,
        -Vector3::z_axis(),
        0.,
    );
    // let lf_thigh_joint = build_fixed_joint(lf_thigh_frame, body_frame, "left_front_thigh", urdf);

    let motor_arm_frame = format!("{}_motor_arm", name);
    let motor_arm = build_rigid(&motor_arm_frame, &motor_arm_frame, urdf, meshes);
    let motor_arm_joint = build_joint(
        &motor_arm_frame,
        body_frame,
        &format!("{}_motor_arm", name),
        urdf,
        -Vector3::z_axis(),
        0.,
    );

    let spring_frame = format!("{}_spring", name);
    let spring = build_rigid(&spring_frame, &spring_frame, urdf, meshes);
    let spring_joint = build_joint(
        &spring_frame,
        &motor_arm_frame,
        &format!("{}_spring", name),
        urdf,
        -Vector3::z_axis(),
        0.,
    );

    let leg_frame = format!("{}_leg", name);
    let leg = build_rigid(&leg_frame, &leg_frame, urdf, meshes);
    let leg_joint = build_joint(
        &leg_frame,
        &spring_frame,
        &format!("{}_leg", name),
        urdf,
        -Vector3::z_axis(),
        0.,
    );

    let wheel_frame = format!("{}_wheel", name);
    let wheel = build_rigid(&wheel_frame, &wheel_frame, urdf, meshes);
    let wheel_joint = build_joint(
        &wheel_frame,
        &leg_frame,
        &format!("{}_wheel", name),
        urdf,
        -Vector3::z_axis(),
        0.,
    );

    let frames = vec![
        thigh_frame,
        motor_arm_frame,
        spring_frame,
        leg_frame,
        wheel_frame,
    ];
    let rigids = vec![thigh, motor_arm, spring, leg, wheel];
    let joints = vec![
        thigh_joint,
        motor_arm_joint,
        spring_joint,
        leg_joint,
        wheel_joint,
    ];
    (frames, rigids, joints)
}

fn add_constraints(
    side: &str,
    direction: &str,
    frames: &Vec<String>,
    articulated: &mut Articulated,
    urdf: &Robot,
) {
    let leg_frame = &frames[3];
    let thigh_frame = &frames[0];
    let motor_arm_frame = &frames[1];
    let wheel_frame = &frames[4];

    let name = format!("{}_{}", side, direction);

    articulated.add_constraints(vec![Constraint::Revolute(build_revolute_constraint(
        leg_frame,
        thigh_frame,
        &format!("{}_leg", name),
        urdf,
    ))]);

    let lower = if side == "left" { -14.48 } else { -8.48 };
    let upper = if side == "left" { 8.48 } else { 14.48 };
    articulated.add_relative_range_constraints(vec![RelativeRangeConstraint::new(
        motor_arm_frame,
        thigh_frame,
        (lower as Float).to_radians(),
        (upper as Float).to_radians(),
    )]);

    articulated.add_range_constraints(vec![RangeConstraint::new(
        wheel_frame,
        (0. as Float).to_radians(),
        (30. as Float).to_radians(),
    )]);
}
