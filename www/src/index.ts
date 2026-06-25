import { initSimulator } from "./sim";
import { getSimulator } from "./sim";
import { createSerialMonitorPanel } from "chimpanzee-ui";

// Import shared CSS
import "chimpanzee-ui/index.css";

const panel = createSerialMonitorPanel({ getSimulator });

initSimulator();
