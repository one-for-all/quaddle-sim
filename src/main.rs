use gorilla_physics::{hybrid::mesh::URDFMeshes, util::read_file};
use quaddle::builder::build_quaddle;

#[tokio::main]
async fn main() {
    let mut meshes = URDFMeshes::empty();
    let urdf_path = "onshape/robot.urdf";
    let urdf_file = read_file(urdf_path);
    let urdf_robot = urdf_rs::read_from_string(&urdf_file).unwrap();
    let mut state = build_quaddle(&mut meshes, &urdf_robot);

    let dt = 1. / 60. / 10.;
    let t_final = 2.0;
    let num_steps = (t_final / dt) as usize;

    for s in 0..num_steps {
        state.step(dt, &vec![]);
    }
}
