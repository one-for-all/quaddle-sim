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
        Joint, JointPosition,
        constraint::{Constraint, RangeConstraint, RelativeRangeConstraint},
    },
    na::{Vector3, vector},
    spatial::transform::Transform3D,
    types::Float,
};
#[cfg(any(target_arch = "wasm32", rust_analyzer))]
use gorilla_physics::{
    collision::halfspace::HalfSpace,
    hybrid::control::NullArticulatedController,
    joint::JointVelocity,
    spatial::{pose::Pose, spatial_vector::SpatialVector},
};
#[cfg(any(target_arch = "wasm32", rust_analyzer))]
use nalgebra::{Isometry3, Translation3, UnitQuaternion};
use urdf_rs::Robot;

#[cfg(any(target_arch = "wasm32", rust_analyzer))]
use crate::can::build_beverage_can;
#[cfg(any(target_arch = "wasm32", rust_analyzer))]
use crate::control::{QuaddleController, microcontroller::QuaddleESP32S3Controller};

#[cfg(any(target_arch = "wasm32", rust_analyzer))]
use {
    gorilla_physics::interface::{hybrid::InterfaceHybrid, util::read_web_file},
    wasm_bindgen::prelude::wasm_bindgen,
};

/// Servo zero positions in degrees, per firmware joint index.
const ZERO_POSITIONS: [Float; 6] = [90., 135., 210., 60., 240., 30.];

pub fn build_quaddle(meshes: &mut URDFMeshes, urdf: &Robot) -> Hybrid {
    let mut state = Hybrid::empty();
    state.set_friction_mu(1.0);

    let body_frame = "body";
    let mut body = build_rigid(body_frame, "body", urdf, meshes, false);
    add_quaddle_body_collision(&mut body, urdf);
    let body_joint = Joint::new_floating(Transform3D::new_xyz_rpy(
        body_frame,
        WORLD_FRAME,
        &vec![0., -0.15, 0.06],
        &vec![0., 0., -PI / 2.],
    ));

    let zero_positions = ZERO_POSITIONS;

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

    for (index, q) in power_on_joint_qs() {
        articulated.set_joint_q(index, JointPosition::Float(q));
    }

    state.add_articulated(articulated);

    state
}

/// The firmware's power-on pose (currentAng in OpenCat.h, the rest posture) as
/// (joint index, q) pairs, so the robot spawns and resets to where the joints
/// really are when its first transform() runs.
/// Firmware maps a joint angle to servo angle zeroPosition + rotationDirection * angle.
/// The URDF pose stays labelled zero_q; the thigh and motor arm share one axis,
/// so turning both by the same amount turns the whole leg linkage rigidly.
pub fn power_on_joint_qs() -> Vec<(usize, Float)> {
    let zero_positions = ZERO_POSITIONS;
    let rotation_directions: [Float; _] = [1., -1., 1., -1., -1., 1.];
    let power_on_angles: [Float; _] = [0., 90., 0., 0., 40., 40.];
    let leg_dof = 6; // thigh, motor arm, spring, leg, wheel, wheel tip
    let mut qs = vec![];
    for leg in 0..4 {
        let servo = leg + 2; // firmware joints 2..5 are LF, RF, RB, LB
        let spawn_q = (rotation_directions[servo] * power_on_angles[servo]).to_radians();
        let thigh_index = 1 + leg * leg_dof; // joint 0 is the floating body
        let motor_arm_index = thigh_index + 1;
        qs.push((thigh_index, spawn_q));
        qs.push((
            motor_arm_index,
            zero_positions[servo].to_radians() + spawn_q,
        ));
    }
    qs
}

/// `power_on_joint_qs` flattened to [index, q, index, q, ...] for JS, which
/// applies it with `set_joint_q` after `reset()` zeroes every joint.
#[cfg(any(target_arch = "wasm32", rust_analyzer))]
#[allow(non_snake_case)]
#[wasm_bindgen]
pub fn quaddlePowerOnPose() -> Vec<Float> {
    power_on_joint_qs()
        .into_iter()
        .flat_map(|(index, q)| [index as Float, q])
        .collect()
}

/// Dev tool for testing side-fall recovery: tips the robot onto its left or
/// right side where it stands, keeping its heading, joint angles and the
/// firmware's state, as if it had just been knocked over. It is dropped from
/// spawn height so it lands on the ground rather than starting in contact.
/// Only wired into the UI on the dev server, see DEV_TOOLS in rspack.config.js.
#[cfg(any(target_arch = "wasm32", rust_analyzer))]
#[allow(non_snake_case)]
#[wasm_bindgen]
pub fn quaddleLayOnSide(hybrid: &mut InterfaceHybrid, left: bool) {
    let articulated = &mut hybrid.hybrid_mut().articulated[0];
    let Joint::FloatingJoint(body_joint) = &articulated.joints[0] else {
        panic!("joint 0 should be the floating body joint");
    };
    // The body's parent is the world, so this is the body's world pose
    let iso = body_joint.transform.iso;
    let init_iso = body_joint.init_iso;

    // Heading, from where the body's long axis (y) points in the world xy plane
    let long_axis = iso.rotation * Vector3::y();
    let yaw = long_axis.y.atan2(long_axis.x) - PI / 2.;

    // Body +x is its left side; rolling about the long axis by +90° points it
    // down at the ground
    let roll = if left { PI / 2. } else { -PI / 2. };
    let rotation = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), yaw)
        * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), roll);
    let translation =
        Translation3::new(iso.translation.x, iso.translation.y, init_iso.translation.z);

    // The floating joint's q is relative to the spawn transform
    let q = init_iso.inverse() * Isometry3::from_parts(translation, rotation);
    articulated.set_joint_q(
        0,
        JointPosition::Pose(Pose {
            rotation: q.rotation,
            translation: q.translation.vector,
        }),
    );

    for i in 0..articulated.joints.len() {
        let v = match articulated.joints[i] {
            Joint::FloatingJoint(_) => JointVelocity::Spatial(SpatialVector::zero()),
            Joint::FixedJoint(_) => continue,
            _ => JointVelocity::Float(0.),
        };
        articulated.set_joint_v(i, v);
    }
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
    state.articulated[0].show_visual = false;

    // let controller = QuaddleController::new();
    let controller = QuaddleESP32S3Controller::new().await;
    state.set_controller(0, controller);

    // A beverage can to judge the robot's size by, and for it to push around.
    // It walks off toward -x (screen left, from the default camera), so the can
    // stands behind its line, to the left of where it starts.
    let can = build_beverage_can(&vector![-0.15, 0.05, 0.]);
    state.add_articulated(can);

    // Add ground
    state.add_halfspace(HalfSpace::new(Vector3::z_axis(), 0.));

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
    let mut thigh = build_rigid(&thigh_frame, &thigh_frame, urdf, meshes, false);
    add_quaddle_thigh_collision(&mut thigh, side);
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
    let mut motor_arm = build_rigid(&motor_arm_frame, &motor_arm_frame, urdf, meshes, false);
    add_quaddle_motor_arm_collision(&mut motor_arm);
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
    let spring = build_rigid(&spring_frame, &spring_frame, urdf, meshes, false);
    let spring_joint = build_joint(
        &spring_frame,
        &motor_arm_frame,
        &format!("{}_spring", name),
        urdf,
        -Vector3::z_axis(),
        0.,
    );

    let leg_frame = format!("{}_leg", name);
    let mut leg = build_rigid(&leg_frame, &leg_frame, urdf, meshes, false);
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
    let wheel = build_rigid(&wheel_frame, &wheel_frame, urdf, meshes, false);
    let wheel_joint = build_joint(
        &wheel_frame,
        &leg_frame,
        &format!("{}_wheel", name),
        urdf,
        -Vector3::z_axis(),
        0.,
    );

    let wheel_tip_frame = format!("{}_wheel_tip", name);
    let mut wheel_tip = build_rigid(&wheel_tip_frame, &wheel_tip_frame, urdf, meshes, false);
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

// The thigh is a flat plate pivoting on the body side: in its link frame it
// lies in the xy plane, reaching from its pivot out along +y (left) or -y
// (right) to the knee bearing, with its outer face -- the one that touches the
// ground when the robot lies on its side -- toward -z. Taken from the extents
// of thigh.stl / thigh_right.stl.
fn add_quaddle_thigh_collision(rigid: &mut Rigid, side: &str) {
    let y = if side == "left" { 0.0133 } else { -0.0133 };
    rigid.add_collision_cuboid_at(&vector![0., y, -0.0015], 0.017, 0.0436, 0.008);
}

// The motor arm is the servo horn, flush with the thigh on the outside of the
// body: in its link frame it reaches along +x, outer face toward -z. Taken
// from the extents of motor_arm.stl.
fn add_quaddle_motor_arm_collision(rigid: &mut Rigid) {
    rigid.add_collision_cuboid_at(&vector![0.0045, 0., 0.00055], 0.02, 0.011, 0.0065);
}

// body length 10.5cm, width 5.0cm, height 2.7cm. The width is the shell's
// alone (body.stl): what sticks out past it on either side are the thighs and
// motor arms, which turn with the legs and so collide on their own.
fn add_quaddle_body_collision(rigid: &mut Rigid, urdf: &Robot) {
    let joint_name = "body_collision_frame";
    let point_joint = urdf.joints.iter().find(|&j| j.name == joint_name).unwrap();
    let p = Vector3::from(point_joint.origin.xyz.0);
    let w = 0.0498;
    let d = 0.105;
    let h = 0.027;
    let com = p - vector![0., 0., h / 2.];
    rigid.add_collision_cuboid_at(&com, w, d, h);
}
