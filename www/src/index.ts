// Import shared CSS
import "chimpanzee-ui/css";

import { initSimulator } from "./sim";
import { getSimulator } from "./sim";
import {
  createSerialMonitorPanel,
  setupResize,
  updateUIforMode,
  createMotionButton,
} from "chimpanzee-ui";

initSimulator();
const panel = createSerialMonitorPanel({ getSimulator });
updateUIforMode();
setupResize();

const motions = [
  { payload: "kbbbk", label: "Back Walk" },
  { payload: "kbiped", label: "Biped Walk" },
  { payload: "kbkL", label: "Backward Left Turn" },
  { payload: "kboat", label: "Boat" },
];
for (const { payload, label } of motions) {
  createMotionButton({ getSimulator }, payload, label);
}
