// Import shared CSS
import "chimpanzee-ui/css";

import { initSimulator } from "./sim";
import { getSimulator } from "./sim";
import {
  createSerialMonitorPanel,
  setupResize,
  updateUIforMode,
} from "chimpanzee-ui";

initSimulator();
const panel = createSerialMonitorPanel({ getSimulator });
updateUIforMode();
setupResize();
