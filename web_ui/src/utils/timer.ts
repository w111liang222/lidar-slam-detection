export function setIntervalBlock(cb: any, t: number) {
  let exit = false;
  // fn.constructor.name is diff between dev and build.
  const loop = async () => {
    if (exit) return;
    const t0 = performance.now();
    await cb();
    const dt = performance.now() - t0;
    setTimeout(loop, t - dt);
  };
  loop();
  return () => {
    exit = true;
  };
}
