import * as log from "loglevel";

export class FPSCounter {
  private msg: string;
  private t0?: number;
  private fpsBefore?: number;
  public fps: number;
  constructor(msg = "") {
    this.msg = msg;
    this.fps = 0;
  }
  public hit(this: FPSCounter) {
    let dt, fps;
    if (!this.t0) {
      this.fpsBefore = 0;
      this.t0 = performance.now();
      return 0;
    }
    dt = performance.now() - this.t0;
    fps = (1000 / dt) * 0.2 + this.fpsBefore! * 0.8;
    // if (this.msg) log.debug(this.msg, fps);
    this.fpsBefore = fps;
    this.t0 = performance.now();
    this.fps = fps;
    return fps;
  }
}
