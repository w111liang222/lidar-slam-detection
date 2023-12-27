import React, { useState, useEffect } from "react";
import { Canvas } from "@react-three/fiber";
import { CONTROL_TYPE } from "@components/3d/Controls";

import { useImperativeHandle } from "react";
import { useRequest } from "ahooks";
import {
  getImuPositionPoints,
  getTransform,
  calibrateLidarImu,
  restartLidarImuCalibration,
} from "@rpc/http";
import Pointcloud from "@components/3d/Pointcloud";
import Helper, { DEFAULT_CONFIG } from "@components/3d/Helper";
import * as THREE from "three";

export interface Props {
  setCalibEnable?: any;
  setStatusString?: any;
  t?: (x: string | undefined) => string;
}

const kMAX_FRAME = 50;

function Calib({ setCalibEnable, setStatusString, t = (x) => x || "" }: Props, ref: any) {
  const config = JSON.parse(JSON.stringify(DEFAULT_CONFIG));
  config.polarGrid.visible = false;
  config.grid.visible = true;
  config.camera.type = CONTROL_TYPE.ORTHOGRAPHIC;

  useImperativeHandle(ref, () => ({
    calibrate: async () => {
      return await calibrateLidarImu();
    },
  }));

  const [pointsList, setPointsList] = useState<Float32Array[]>([]);
  const [positionList, setPositionList] = useState<number[][]>([]);

  useRequest(() => getImuPositionPoints(), {
    pollingInterval: 30,
    onSuccess: (data) => {
      if (data.is_reset && pointsList.length < kMAX_FRAME) {
        restartLidarImuCalibration().then(()=>{
          setPointsList([]);
          setPositionList([]);
        });
      } else if (data.result && pointsList.length < kMAX_FRAME) {
        setPointsList((points) => [...points, data.points]);
        setPositionList((position) => [...position, data.position]);
      }
    },
    onError: () => {
      setPointsList([]);
      setPositionList([]);
    },
    manual: false,
  });

  useEffect(() => {
    restartLidarImuCalibration().then(()=>{
      setPointsList([]);
      setPositionList([]);
    });
  }, []);

  useEffect(() => {
    if (pointsList.length >= kMAX_FRAME) {
      setCalibEnable(true);
    } else {
      setCalibEnable(false);
    }
    setStatusString(t("calibProgress") + ((pointsList.length / kMAX_FRAME) * 100).toFixed(1).toString() + " %");
  }, [pointsList]);

  return (
    <>
      <div className="calibration-tab" style={{ display: "flex" }}>
        <Canvas orthographic={true} style={{ position: "absolute" }}>
          <Helper config={config} />
          {pointsList.map((points, index) => {
            return (
              <PositionPoints
                points={points}
                pose={positionList[index]}
              />
            );
          })}
        </Canvas>
      </div>
    </>
  );
}

type IPointProp = {
  position: [number, number, number];
  rotation: [number, number, number, string];
};

type IProps = {
  points: Float32Array;
  pose?: number[];
};

function PositionPoints({ points, pose }: IProps) {
  const [transform, setTransform] = useState<THREE.Matrix4>();
  const [pointProp, setPointProp] = useState<IPointProp | undefined>(undefined);

  useEffect(() => {
    if (pose) {
      getTransform(pose).then((T) => {
        setTransform(T);
      });
    }
  }, [pose]);

  useEffect(() => {
    if (transform) {
      let T = transform.clone();
      const pos = new THREE.Vector3();
      const quat = new THREE.Quaternion();
      const scale = new THREE.Vector3();
      T.decompose(pos, quat, scale);
      const euler = new THREE.Euler();
      euler.setFromQuaternion(quat);
      setPointProp({
        position: pos.toArray() as [number, number, number],
        rotation: euler.toArray() as [number, number, number, string],
      });
    }
  }, [transform]);

  return (
    <Pointcloud
      maxNum={100e4}
      points={points}
      color={"height"}
      size={1.5}
      visible={pointProp != undefined}
      pointsProps={pointProp}
    />
  );
}

export default React.forwardRef(Calib);
