const CONTROL_KEY = navigator.platform === "MacIntel" ? "Meta" : "Control";

window.addEventListener("keydown", (event) => {
  console.log(event.key, CONTROL_KEY);
  if (event.key === CONTROL_KEY) {
    window.ctrlKeyPressed = true;
  }
});
window.addEventListener("keyup", (event) => {
  window.ctrlKeyPressed = false;
});
