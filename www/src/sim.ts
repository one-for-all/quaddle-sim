import { createQuaddle, InterfaceHybrid } from "quaddle-sim";
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

    showRealtimeRate(simulator, state);

    const loadingUI = document.getElementById("loading");
    if (loadingUI) {
      loadingUI.remove();
    }
  });
}

// `Simulator.realtimeRatio` is recomputed once per smoothing window inside the
// run loop, so it is polled rather than pushed -- twice a second, which is slow
// enough to read and fast enough to show a motion dragging the loop down.
//
// The rate says the scene is behind; the breakdown under it says what is
// holding it up. That is a click away rather than always on: it is the answer
// to a question most sessions never ask, and open it covers the movement
// buttons it hangs over. `hybrid.profile_take` accumulates whether or not
// anyone is looking, so opening it mid-run needs nothing switched on first.
function showRealtimeRate(simulator: Simulator, hybrid: InterfaceHybrid) {
  const rate = document.getElementById("realtimeRatio");
  const value = document.getElementById("realtime-value");
  const profile = document.getElementById("realtime-profile");
  if (!rate || !value || !profile) {
    return;
  }

  const labels = hybrid.profile_labels().split(",");
  let lastPoll = performance.now();

  rate.addEventListener("click", () => {
    profile.hidden = !profile.hidden;
    // Whatever piled up while it was closed describes a window the user did not
    // ask about, and would land on screen as the first reading.
    hybrid.profile_take();
    lastPoll = performance.now();
  });

  setInterval(() => {
    value.textContent = `realtime rate: ${simulator.realtimeRatio.toFixed(2)}`;

    // Reading resets the accumulation, so this has to run every tick whether or
    // not the panel is up -- otherwise the first reading after it opens covers
    // however long it was shut.
    const taken = hybrid.profile_take();
    const now = performance.now();
    const wall = now - lastPoll;
    lastPoll = now;
    if (profile.hidden) {
      return;
    }

    const steps = taken[0];
    const phases = Array.from(taken.slice(1));
    if (steps === 0 || wall <= 0) {
      profile.textContent = "not stepping";
      return;
    }

    // Two numbers per phase, because they answer different questions: the share
    // of the wall clock is what identifies the bottleneck, and the cost per
    // step is what a change to that phase moves. Sorted, since the first line is
    // the answer.
    const order = phases
      .map((ms, i) => ({ name: labels[i] ?? `phase ${i}`, ms }))
      .sort((a, b) => b.ms - a.ms);
    const total = phases.reduce((sum, ms) => sum + ms, 0);
    const lines = order.map(
      ({ name, ms }) =>
        `${name.padEnd(12)}${((100 * ms) / wall).toFixed(0).padStart(3)}% ` +
        `${((1000 * ms) / steps).toFixed(0).padStart(5)} us`,
    );
    profile.textContent = [
      `${(steps / (wall / 1000)).toFixed(0)} steps/s, ` +
        `${((100 * total) / wall).toFixed(0)}% of wall in step`,
      ...lines,
    ].join("\n");
  }, 500);
}

function setSimulator(sim: Simulator) {
  _simulator = sim;
}

export function getSimulator(): Simulator | null {
  return _simulator;
}
