import React, { useImperativeHandle } from "react";
import { useEffect, useState, useRef, useMemo } from "react";
import * as THREE from "three";
import { Canvas } from "@react-three/fiber";

import Pointcloud from "@components/3d/PointcloudImage";
import Controls, { CONTROL_TYPE } from "@components/3d/Controls";
import { getConfig, getTransform } from "@rpc/http";

export interface Props {
  points?: Float32Array;
  imageUrl?: string;
  cameraName?: string;
  timestamp?: number;
  setCalibEnable?: any;
}

function CalibCheck({ points, imageUrl, cameraName, timestamp, setCalibEnable }: Props, ref: any) {
  // useWhyDidYouUpdate("why", [points, imageUrl, timestamp]);
  const [cameraConfig, setCameraConfig] = useState<{
    [index: string]: LSD.Config["camera"][0];
  }>();
  useEffect(() => {
    getConfig().then((config) => {
      console.log(config);
      const normConfig: typeof cameraConfig = {};
      for (let cfg of config.camera) {
        normConfig[cfg.name] = cfg;
      }
      setCameraConfig(normConfig);
    });
  }, []);

  const [transform, setTransform] = useState<THREE.Matrix4>();
  useEffect(() => {
    if (cameraConfig && cameraName) {
      const cfg = cameraConfig[cameraName];
      const pos = new THREE.Vector3(
        cfg.extrinsic_parameters[0],
        cfg.extrinsic_parameters[1],
        cfg.extrinsic_parameters[2]
      );
      const scale = new THREE.Vector3(1.0, 1.0, 1.0);
      const quat = new THREE.Quaternion();
      const euler = new THREE.Euler(
        cfg.extrinsic_parameters[3] * 0.01745329251994,
        cfg.extrinsic_parameters[4] * 0.01745329251994,
        cfg.extrinsic_parameters[5] * 0.01745329251994,
        "XYZ"
      );
      quat.setFromEuler(euler);
      quat.set(quat.y, quat.x, quat.z, quat.w);
      const T = new THREE.Matrix4();
      T.compose(pos, quat, scale);
      setTransform(T);
    }
  }, [cameraConfig, cameraName]);

  useEffect(() => {
    setCalibEnable(true);
  }, []);

  const [fx, fy, cx, cy] = useMemo(() => {
    if (cameraName === undefined || cameraConfig === undefined) {
      return [600, 600, 320, 240];
    } else {
      return cameraConfig[cameraName]?.intrinsic_parameters.slice(0, 4);
    }
  }, [cameraName, cameraConfig]);

  return (
    <>
      <div className="calibration-tab">
        <Canvas>
          <axesHelper />
          <Controls type={CONTROL_TYPE.NORMAL} />
          <gridHelper args={[100, 20, 0x222222, 0x222222]} visible={true} rotation-x={Math.PI / 2} />
          <Pointcloud
            maxNum={100e4}
            pointsize={3}
            transparent={false}
            points={points}
            imageUrl={imageUrl}
            cameraIntrinsic={{ cx, cy, fx, fy }}
            pointsProps={transform}
          />
        </Canvas>
      </div>
    </>
  );
}

export default React.forwardRef(CalibCheck);
