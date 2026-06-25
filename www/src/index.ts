import { initSimulator } from "./sim";
import { getSimulator } from "./sim";
import {
  createSerialMonitorPanel,
  setupResize,
  updateUIforMode,
} from "chimpanzee-ui";

// Import shared CSS
import "chimpanzee-ui/css";

initSimulator();
const panel = createSerialMonitorPanel({ getSimulator });
updateUIforMode();
setupResize();
