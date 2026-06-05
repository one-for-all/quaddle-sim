#[cfg(any(target_arch = "wasm32", rust_analyzer))]
use gorilla_physics::hybrid::control::NullArticulatedController;
use gorilla_physics::{
    PI, WORLD_FRAME,
    hybrid::{
        Hybrid,
        articulated::Articulated,
        mesh::URDFMeshes,
        rigid::helper::{build_joint, build_revolute_constraint, build_rigid},
    },
    joint::{
        Joint,
        constraint::{
            Constraint, RangeConstraint, RelativeRangeConstraint,
            constraint_revolute::RevoluteConstraintJoint,
        },
    },
    na::{Isometry3, Rotation3, Translation3, UnitQuaternion, Vector3},
    spatial::transform::Transform3D,
    types::Float,
};
use urdf_rs::Robot;

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
    let lf_thigh_frame = "left_front_thigh";
    let lf_thigh = build_rigid(lf_thigh_frame, "thigh", urdf, meshes);
    let lf_thigh_joint = build_joint(
        lf_thigh_frame,
        body_frame,
        "left_front_thigh",
        urdf,
        -Vector3::z_axis(),
        0.,
    );

    let lf_motor_arm_frame = "left_front_motor";
    let lf_motor_arm = build_rigid(lf_motor_arm_frame, "motor_arm", urdf, meshes);
    let lf_motor_arm_joint = build_joint(
        lf_motor_arm_frame,
        body_frame,
        "left_front_motor_arm",
        urdf,
        -Vector3::z_axis(),
        0.,
    );

    let lf_spring_frame = "left_front_spring";
    let lf_spring = build_rigid(lf_spring_frame, "long_spring", urdf, meshes);
    let lf_spring_joint = build_joint(
        lf_spring_frame,
        lf_motor_arm_frame,
        "left_front_spring",
        urdf,
        -Vector3::z_axis(),
        0.,
    );

    let lf_leg_frame = "left_front_leg";
    let lf_leg = build_rigid(lf_leg_frame, "leg", urdf, meshes);
    let lf_leg_joint = build_joint(
        lf_leg_frame,
        lf_spring_frame,
        "left_front_leg",
        urdf,
        -Vector3::z_axis(),
        0.,
    );

    let lf_wheel_frame = "left_front_wheel";
    let lf_wheel = build_rigid(lf_wheel_frame, "wheel", urdf, meshes);
    let lf_wheel_joint = build_joint(
        lf_wheel_frame,
        lf_leg_frame,
        "left_front_wheel",
        urdf,
        -Vector3::z_axis(),
        0.,
    );

    let mut articulated = Articulated::new(
        vec![body, lf_thigh, lf_motor_arm, lf_spring, lf_leg, lf_wheel],
        vec![
            body_joint,
            lf_thigh_joint,
            lf_motor_arm_joint,
            lf_spring_joint,
            lf_leg_joint,
            lf_wheel_joint,
        ],
    );

    articulated.add_constraints(vec![Constraint::Revolute(build_revolute_constraint(
        lf_leg_frame,
        lf_thigh_frame,
        "left_front_leg",
        urdf,
    ))]);

    articulated.add_relative_range_constraints(vec![RelativeRangeConstraint::new(
        lf_motor_arm_frame,
        lf_thigh_frame,
        (-14.48 as Float).to_radians(),
        (8.48 as Float).to_radians(),
    )]);

    articulated.add_range_constraints(vec![RangeConstraint::new(
        lf_wheel_frame,
        (0. as Float).to_radians(),
        (30. as Float).to_radians(),
    )]);

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

    InterfaceHybrid::new(state)
}
