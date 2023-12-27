export default class FirstPersonControl {
  constructor(camera, dom) {
    this.camera = camera;
    this.dom = dom;
    this.onMouseMove = this.onMouseMove.bind(this);
    this.onMouseDown = this.onMouseDown.bind(this);
    this.onMouseUp = this.onMouseUp.bind(this);
    dom.addEventListener("mousedown", this.onMouseDown);
    dom.addEventListener("mouseup", this.onMouseUp);
  }
  onMouseDown(event) {
    this.dom.addEventListener("mousemove", this.onMouseMove);
  }
  onMouseUp(event) {
    this.dom.removeEventListener("mousemove", this.onMouseMove);
  }
  onMouseMove(event) {
    const x = event.clientX;
    if (this.lastX) {
      const dx = x - this.lastX;
      if (dx > 0) {
        this.camera.rotateY(Math.PI / 90);
      } else {
        this.camera.rotateY(-Math.PI / 90);
      }
    }
    this.lastX = x;
  }
  dispose() {
    this.dom.removeEventListener("mousedown", this.onMouseDown);
    this.dom.removeEventListener("mouseup", this.onMouseUp);
    this.dom.removeEventListener("mousemove", this.onMouseMove);
  }
}
