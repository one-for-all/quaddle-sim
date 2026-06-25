import { createQuaddle } from "quaddle-sim";
import { Simulator } from "gorilla-physics-ui";

let _simulator: Simulator | null = null;

export function initSimulator() {
  let interfaceSimulator = null;
  let showGrid = true;
  let simulator = new Simulator(interfaceSimulator, showGrid);
  simulator.showHalfspaces = true;

  createQuaddle().then((state) => {
    simulator.addHybrid(state);
    simulator.updateHybrid();

    let cameraPosition = {
      eye: { x: 0.0, y: -0.6, z: 0.2 },
      target: { x: 0.0, y: 0, z: 0 },
    };
    simulator.graphics.lookAt(cameraPosition);

    simulator.run(50, 0);

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
