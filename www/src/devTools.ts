import { quaddleLayOnSide } from "quaddle-sim";
import { getSimulator } from "./sim";

// Buttons for testing the robot in situations that are slow to set up by
// driving it there. Only on the dev server, see DEV_TOOLS in rspack.config.js.
export function setupDevTools() {
  const container = document.getElementById("movementButtonContainer");
  if (!container) return;

  // Tips the robot over with the firmware still running, to test its recovery
  // from a side pose (e.g. with "krc")
  for (const { left, label } of [
    { left: true, label: "Lay on Left Side (dev)" },
    { left: false, label: "Lay on Right Side (dev)" },
  ]) {
    const button = document.createElement("button");
    button.className = "movement-button";
    button.textContent = label;
    button.addEventListener("click", () => {
      const hybrid = getSimulator()?.hybrid;
      if (hybrid) {
        quaddleLayOnSide(hybrid, left);
      }
    });
    container.appendChild(button);
  }
}
