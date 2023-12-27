import Pointcloud from "@components/3d/Pointcloud";
import Sketchpad, { Ref as SketchpadRef } from "@components/3d/SketchpadROI";
import { Canvas } from "@react-three/fiber";
import React, { useState, useEffect } from "react";
import { useImperativeHandle } from "react";
import { calibrateGround } from "@rpc/http";
import { useRef } from "react";
import ContextMenu from "@components/general/ContextMenu";
import { useCtrlKey } from "@hooks/keyboard";
import Helper, { Config, DEFAULT_CONFIG } from "@components/3d/Helper";
import Controls, { CONTROL_TYPE } from "@components/3d/Controls";
import * as THREE from "three";

export type Props = {
  points: Float32Array | undefined;
  lidarIndex?: number;
  setCalibEnable?: any;
  disable: boolean;
};
export type Ref = {
  calibrate: () => Promise<THREE.Matrix4>;
};

function CalibrateGround({ points, lidarIndex, setCalibEnable, disable }: Props, ref: React.Ref<Ref>) {
  const [roi, setRoi] = useState<THREE.Vector3[]>([]);
  useImperativeHandle(ref, () => ({
    calibrate: async () => {
      if (lidarIndex === undefined || !points) {
        return new THREE.Matrix4().identity();
      }
      return await calibrateGround({
        points,
        contour: roi.map((pt) => [pt.x, pt.y]),
        key: lidarIndex,
      });
    },
  }));

  const [padState, setPadState] = useState("idle");

  useEffect(() => {
    if (disable) {
      setRoi([]);
    } else {
      setPadState("drawing");
    }
  }, [disable]);

  useEffect(() => {
    if (ref) {
      if (lidarIndex === undefined || !points || roi.length < 3 || padState != "idle") {
        setCalibEnable(false);
      } else {
        setCalibEnable(true);
      }
    } else {
      setCalibEnable(lidarIndex != undefined);
    }
  }, [ref, lidarIndex, roi, padState]);

  const config = JSON.parse(JSON.stringify(DEFAULT_CONFIG));
  config.polarGrid.visible = false;
  config.grid.visible = true;
  config.camera.type = CONTROL_TYPE.UP;

  const ctrlKeyPressed = useCtrlKey();
  const padRef = useRef<SketchpadRef>();
  let contextMenu = (
    <ContextMenu
      enabled={ctrlKeyPressed && padState === "idle"}
      menuItems={{
        redraw: () => {
          padRef.current?.start();
          setRoi([]);
        },
      }}
      onStart={ctrlKeyPressed ? () => padRef.current?.stop() : undefined}
    />
  );

  return (
    <div className="calibration-tab">
      <Canvas
        onCreated={() => {
          padRef.current?.start();
        }}>
        <Helper config={config} />
        <Pointcloud points={points} color={"height"} size={1.5} />
        {!disable && (
          <Sketchpad
            ref={padRef}
            enabled={ctrlKeyPressed}
            roi={roi}
            state={padState}
            onChange={setRoi}
            onStart={() => setPadState("drawing")}
            onEnd={(state: string) => {
              setPadState(state);
            }}
          />
        )}
      </Canvas>
      {!disable && contextMenu}
    </div>
  );
}

export default React.forwardRef(CalibrateGround);
