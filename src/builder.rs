use gibbon_electronics::servo::petoi_p1l::PetoiP1L;
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
    na::{Vector3, vector},
    spatial::transform::Transform3D,
    types::Float,
};
#[cfg(any(target_arch = "wasm32", rust_analyzer))]
use gorilla_physics::{
    collision::halfspace::HalfSpace, hybrid::control::NullArticulatedController,
};
use urdf_rs::Robot;

#[cfg(any(target_arch = "wasm32", rust_analyzer))]
use crate::control::{QuaddleController, microcontroller::QuaddleESP32S3Controller};

#[cfg(any(target_arch = "wasm32", rust_analyzer))]
use {
    gorilla_physics::interface::{hybrid::InterfaceHybrid, util::read_web_file},
    wasm_bindgen::prelude::wasm_bindgen,
};

pub fn build_quaddle(meshes: &mut URDFMeshes, urdf: &Robot) -> Hybrid {
    let mut state = Hybrid::empty();
    state.set_friction_mu(1.0);

    let body_frame = "body";
    let mut body = build_rigid(body_frame, "body", urdf, meshes);
    add_quaddle_body_collision(&mut body, urdf);
    let body_joint = Joint::new_floating(Transform3D::new_xyz_rpy(
        body_frame,
        WORLD_FRAME,
        &vec![0., 0., 0.06],
        &vec![0., 0., -PI / 2.],
    ));

    let zero_positions: [Float; _] = [90., 135., 210., 60., 240., 30.];

    // left-front
    let (lf_frames, lf_rigids, lf_joints) = build_leg(
        "left",
        "front",
        body_frame,
        urdf,
        meshes,
        zero_positions[2].to_radians(),
    );

    // right-front
    let (rf_frames, rf_rigids, rf_joints) = build_leg(
        "right",
        "front",
        body_frame,
        urdf,
        meshes,
        zero_positions[3].to_radians(),
    );

    // right-back
    let (rb_frames, rb_rigids, rb_joints) = build_leg(
        "right",
        "back",
        body_frame,
        urdf,
        meshes,
        zero_positions[4].to_radians(),
    );

    // left-back
    let (lb_frames, lb_rigids, lb_joints) = build_leg(
        "left",
        "back",
        body_frame,
        urdf,
        meshes,
        zero_positions[5].to_radians(),
    );

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

    add_constraints(
        "left",
        "front",
        &lf_frames,
        &mut articulated,
        urdf,
        zero_positions[2],
    );
    add_constraints(
        "right",
        "front",
        &rf_frames,
        &mut articulated,
        urdf,
        zero_positions[3],
    );
    add_constraints(
        "right",
        "back",
        &rb_frames,
        &mut articulated,
        urdf,
        zero_positions[4],
    );
    add_constraints(
        "left",
        "back",
        &lb_frames,
        &mut articulated,
        urdf,
        zero_positions[5],
    );

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
    state.add_halfspace(HalfSpace::new(Vector3::z_axis(), 0.));
    state.articulated[0].show_visual = false;

    // let controller = QuaddleController::new();
    let controller = QuaddleESP32S3Controller::new().await;
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
    zero_q: Float, // zero position q in radians
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
    // The P1L driving this joint is part of its drivetrain: the rotor seen
    // through the gearbox is most of what the servo torque accelerates, and
    // the gear friction is what holds the joint still under a small torque.
    let servo = PetoiP1L::new().params;
    let motor_arm_joint = build_joint(
        &motor_arm_frame,
        body_frame,
        &format!("{}_motor_arm", name),
        urdf,
        -Vector3::z_axis(),
        zero_q,
    )
    .with_armature(servo.rotor_inertia)
    .with_dry_friction(servo.dry_friction);

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
    let mut leg = build_rigid(&leg_frame, &leg_frame, urdf, meshes);
    add_quaddle_leg_collision(&mut leg, &name, urdf);
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

    let wheel_tip_frame = format!("{}_wheel_tip", name);
    let mut wheel_tip = build_rigid(&wheel_tip_frame, &wheel_tip_frame, urdf, meshes);
    add_quaddle_wheel_tip_collision(&mut wheel_tip, &name, urdf);
    let wheel_tip_joint = build_joint(
        &wheel_tip_frame,
        &wheel_frame,
        &format!("{}_wheel_tip", name),
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
        wheel_tip_frame,
    ];
    let rigids = vec![thigh, motor_arm, spring, leg, wheel, wheel_tip];
    let joints = vec![
        thigh_joint,
        motor_arm_joint,
        spring_joint,
        leg_joint,
        wheel_joint,
        wheel_tip_joint,
    ];
    (frames, rigids, joints)
}

fn add_constraints(
    side: &str,
    direction: &str,
    frames: &Vec<String>,
    articulated: &mut Articulated,
    urdf: &Robot,
    zero_q: Float, // zero position in degrees
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

    let lower = zero_q + if side == "left" { -14.48 } else { -8.48 };
    let upper = zero_q + if side == "left" { 8.48 } else { 14.48 };
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

fn add_quaddle_leg_collision(rigid: &mut Rigid, which_leg: &str, urdf: &Robot) {
    let joint_name = format!("{}_foot_frame", which_leg);
    let point_joint = urdf.joints.iter().find(|&j| j.name == joint_name).unwrap();
    rigid.add_collision_sphere_at(&Vector3::from(point_joint.origin.xyz.0), 0.0035);

    let joint_name = format!("{}_knee_frame", which_leg);
    let point_joint = urdf.joints.iter().find(|&j| j.name == joint_name).unwrap();
    rigid.add_collision_sphere_at(&Vector3::from(point_joint.origin.xyz.0), 0.0025);
}

fn add_quaddle_wheel_tip_collision(rigid: &mut Rigid, which_leg: &str, urdf: &Robot) {
    let joint_name = format!("{}_wheel_tip_collision_frame", which_leg);
    let point_joint = urdf.joints.iter().find(|&j| j.name == joint_name).unwrap();
    rigid.add_collision_sphere_at(&Vector3::from(point_joint.origin.xyz.0), 0.002);
}

// body length 10.5cm, width 7.1cm, height 2.7cm
fn add_quaddle_body_collision(rigid: &mut Rigid, urdf: &Robot) {
    let joint_name = "body_collision_frame";
    let point_joint = urdf.joints.iter().find(|&j| j.name == joint_name).unwrap();
    let p = Vector3::from(point_joint.origin.xyz.0);
    let w = 0.071;
    let d = 0.105;
    let h = 0.027;
    let com = p - vector![0., 0., h / 2.];
    rigid.add_collision_cuboid_at(&com, w, d, h);
}
