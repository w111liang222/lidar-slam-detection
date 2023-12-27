import React from "react";
import Controls, { CONTROL_TYPE } from "./Controls";
import * as THREE from "three";

export const DEFAULT_CONFIG = {
  axis: { size: 2, visible: true },
  polarGrid: {
    visible: true,
    radius: 50,
    radials: 16,
    circles: 5,
    divisions: 64,
    color: 0x222222,
  },
  grid: {
    visible: false,
    size: 100,
    divisions: 20,
    color: 0x222222,
  },
  camera: { type: CONTROL_TYPE.NORMAL },
};

export type Config = typeof DEFAULT_CONFIG;

export default function CanvasHelper({ config = DEFAULT_CONFIG }: { config?: Config }) {
  const axisScale = new THREE.Vector3(...new Array(3).fill(config ? config.axis.size : 1));
  return (
    <>
      <axesHelper {...(config && config.axis)} scale={axisScale} material-linewidth={2} />
      {!config.polarGrid.visible || (
        <polarGridHelper
          args={[
            config.polarGrid.radius,
            config.polarGrid.radials,
            config.polarGrid.circles,
            config.polarGrid.divisions,
            config.polarGrid.color,
            config.polarGrid.color,
          ]}
          visible={config.polarGrid.visible}
          rotation-x={Math.PI / 2}
        />
      )}
      {!config.grid.visible || (
        <gridHelper
          args={[config.grid.size, config.grid.divisions, config.grid.color, config.grid.color]}
          visible={config.grid.visible}
          rotation-x={Math.PI / 2}
        />
      )}
      <Controls {...(config && config.camera)} />
    </>
  );
}
