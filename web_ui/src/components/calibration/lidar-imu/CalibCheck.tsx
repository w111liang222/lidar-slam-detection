import React, { useState, useEffect } from "react";
import { Canvas } from "@react-three/fiber";
import { CONTROL_TYPE } from "@components/3d/Controls";

import { useImperativeHandle } from "react";
import {
  getLidarImuLidarPoses,
  getLidarImuImuPoses,
  setLidarImuExtrinsics,
} from "@rpc/http";
import Helper, { DEFAULT_CONFIG } from "@components/3d/Helper";
import TrajectoryView from "@components/preview/CarTrajectory";
import * as THREE from "three";

export interface Props {
  setCalibEnable?: any;
  setStatusString?: any;
  t?: (x: string | undefined) => string;
}

function CalibCheck({ setCalibEnable, setStatusString, t = (x) => x || "" }: Props, ref: any) {
  const config = JSON.parse(JSON.stringify(DEFAULT_CONFIG));
  config.polarGrid.visible = false;
  config.grid.visible = true;
  config.camera.type = CONTROL_TYPE.ORTHOGRAPHIC;

  const [lidarTrajectory, setLidarTrajectory] = useState<THREE.Vector3[]>([]);
  const [imuTrajectory, setImuTrajectory] = useState<THREE.Vector3[]>([]);

  useImperativeHandle(ref, () => ({
    calibrate: async () => {
      return await setLidarImuExtrinsics();
    },
  }));

  useEffect(() => {
    getLidarImuLidarPoses().then((vertex)=>{
      let ids = Object.keys(vertex);
      for (let i = 0; i < ids.length; i++) {
        let key = ids[i];
        lidarTrajectory[i] = new THREE.Vector3(vertex[key][3], vertex[key][7], vertex[key][11]);
      }
      setLidarTrajectory([...lidarTrajectory]);
    })

    getLidarImuImuPoses().then((vertex)=>{
      let ids = Object.keys(vertex);
      for (let i = 0; i < ids.length; i++) {
        let key = ids[i];
        imuTrajectory[i] = new THREE.Vector3(vertex[key][3], vertex[key][7], vertex[key][11]);
      }
      setImuTrajectory([...imuTrajectory]);
    })
    setCalibEnable(true);
    setStatusString("");
  }, []);

  return (
    <>
      <div className="calibration-tab" style={{ display: "flex" }}>
        <Canvas orthographic={true} style={{ position: "absolute" }}>
          <Helper config={config} />
          <TrajectoryView trajectory={lidarTrajectory} lineColor={0x00ff66}/>
          <TrajectoryView trajectory={imuTrajectory} lineColor={0xff0066}/>
        </Canvas>
      </div>
    </>
  );
}

export default React.forwardRef(CalibCheck);
