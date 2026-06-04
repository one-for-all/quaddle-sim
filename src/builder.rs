#[cfg(any(target_arch = "wasm32", rust_analyzer))]
use gorilla_physics::hybrid::control::NullArticulatedController;
use gorilla_physics::hybrid::{Hybrid, mesh::URDFMeshes};
use urdf_rs::Robot;

#[cfg(any(target_arch = "wasm32", rust_analyzer))]
use {
    gorilla_physics::interface::{hybrid::InterfaceHybrid, util::read_web_file},
    wasm_bindgen::prelude::wasm_bindgen,
};

pub fn build_quaddle(meshes: &mut URDFMeshes, urdf: &Robot) -> Hybrid {
    let mut state = Hybrid::empty();

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

    let controller = NullArticulatedController {};
    state.set_controller(0, controller);

    InterfaceHybrid::new(state)
}
