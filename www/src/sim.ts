import { createQuaddle } from "quaddle-sim";
import { Simulator } from "gorilla-physics-ui";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";
import { Euler, Vector3 } from "three";

let _simulator: Simulator | null = null;

export const cameraPosition = {
  eye: { x: 0.0, y: -0.6, z: 0.2 + 0.82 },
  target: { x: 0.0, y: 0, z: 0.82 },
};

export function initSimulator() {
  let interfaceSimulator = null;
  let showGrid = true;
  let simulator = new Simulator(interfaceSimulator, showGrid);
  simulator.showHalfspaces = false;

  let scenes = [
    {
      scenePath: "gamer_setup_pack.glb",
      rotation: new Euler(Math.PI / 2),
      position: new Vector3(0.6, 0, 0),
    },
  ];
  let gltfLoader = new GLTFLoader();
  for (const scene of scenes) {
    gltfLoader.load(
      scene.scenePath,
      (gltf) => {
        const gltfScene = gltf.scene;
        gltfScene.setRotationFromEuler(scene.rotation);
        gltfScene.position.set(
          scene.position.x,
          scene.position.y,
          scene.position.z,
        );
        simulator.graphics.scene.add(gltfScene);
      },
      undefined,
      (error) => {
        console.error(error);
      },
    );
  }

  createQuaddle().then((state) => {
    simulator.addHybrid(state);
    simulator.updateHybrid();

    simulator.graphics.lookAt(cameraPosition);

    simulator.run(30, 0);

    setSimulator(simulator);

    setInterval(() => {
      const realtimeRatio = document.getElementById("realtimeRatio");
      realtimeRatio.innerHTML =
        "realtime rate: " + simulator.realtimeRatio.toFixed(2);
    }, 500);

    const loadingUI = document.getElementById("loading");
    if (loadingUI) {
      loadingUI.remove();
    }
  });
}

function setSimulator(sim: Simulator) {
  _simulator = sim;
}

export function getSimulator(): Simulator | null {
  return _simulator;
}
