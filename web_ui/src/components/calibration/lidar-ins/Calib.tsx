import React, { useState, useEffect } from "react";
import { Canvas } from "@react-three/fiber";
import { CONTROL_TYPE } from "@components/3d/Controls";

import { useImperativeHandle } from "react";
import { useRequest } from "ahooks";
import {
  getPositionPoints,
  getTransform,
  calibrateLidarIns,
  getLidarInsCalibration,
  restartLidarInsCalibration,
  getLidarInsTransform,
  setLidarInsTransform,
} from "@rpc/http";
import Pointcloud from "@components/3d/Pointcloud";
import Helper, { DEFAULT_CONFIG } from "@components/3d/Helper";
import Slider from "../lidar/SliderCalib";
import * as THREE from "three";
import { Paper } from "@mui/material";

export interface Props {
  setCalibEnable?: any;
  setStatusString?: any;
  t?: (x: string | undefined) => string;
  isFinetune: boolean;
}

const kMAX_FRAME = 20;

function Calib({ setCalibEnable, setStatusString, t = (x) => x || "", isFinetune }: Props, ref: any) {
  const config = JSON.parse(JSON.stringify(DEFAULT_CONFIG));
  config.polarGrid.visible = false;
  config.grid.visible = true;
  config.camera.type = CONTROL_TYPE.ORTHOGRAPHIC;

  useImperativeHandle(ref, () => ({
    calibrate: async () => {
      cancel();
      if (!isFinetune) {
        return await calibrateLidarIns();
      } else {
        let transform = new THREE.Matrix4();
        transform.set(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
        if (staticTransform) {
          transform = transform.multiply(staticTransform);
        }
        if (finetuneTransform) {
          transform = transform.multiply(finetuneTransform);
        }
        return await setLidarInsTransform({ transform });
      }
    },
  }));

  const [pointsList, setPointsList] = useState<Float32Array[]>([]);
  const [positionList, setPositionList] = useState<number[][]>([]);
  const [staticTransform, setStaticTransform] = useState<THREE.Matrix4>();
  const [finetuneTransform, setFinetuneTransform] = useState<THREE.Matrix4>();

  const [rotation, setRotation] = useState<[number, number, number, string]>([0, 0, 0, "XYZ"]);
  const [translation, setTranslation] = useState<[number, number, number]>([0, 0, 0]);

  const { run, cancel } = useRequest(() => getPositionPoints(), {
    pollingInterval: 100,
    onSuccess: (data) => {
      if (data.result && pointsList.length < kMAX_FRAME) {
        setPointsList((points) => [...points, data.points]);
        setPositionList((position) => [...position, data.position]);
      }
    },
    onError: () => {
      setPointsList([]);
      setPositionList([]);
    },
    manual: true,
  });

  const {} = useRequest(() => getLidarInsTransform(), {
    pollingInterval: 200,
    onSuccess: (data) => {
      setStaticTransform(data);
    },
    manual: false,
  });

  const { data: progress } = useRequest(() => getLidarInsCalibration(), {
    pollingInterval: 200,
    manual: false,
  });

  useEffect(() => {
    if (!isFinetune) {
      setPointsList([]);
      setPositionList([]);
      setRotation([0, 0, 0, "XYZ"]);
      setTranslation([0, 0, 0]);
      restartLidarInsCalibration();
      run();
    } else {
      cancel();
    }
  }, [isFinetune]);

  useEffect(() => {
    if (rotation && translation) {
      let T = new THREE.Matrix4();
      const euler = new THREE.Euler();
      const rotationRadius = rotation.slice();
      rotationRadius[0] = ((rotationRadius[0] as number) * Math.PI) / 180;
      rotationRadius[1] = ((rotationRadius[1] as number) * Math.PI) / 180;
      rotationRadius[2] = ((rotationRadius[2] as number) * Math.PI) / 180;
      euler.fromArray(rotationRadius);
      T.makeRotationFromEuler(euler);
      T.setPosition(...(translation as [number, number, number]));
      setFinetuneTransform(T);
    }
  }, [rotation, translation]);

  useEffect(() => {
    if (!isFinetune) {
      if (pointsList.length >= kMAX_FRAME) {
        setCalibEnable(true);
      } else {
        setCalibEnable(false);
      }
    } else {
      setCalibEnable(progress ? progress.finish : false);
    }

    if (!isFinetune) {
      setStatusString(t("calibProgress") + ((pointsList.length / kMAX_FRAME) * 100).toFixed(1).toString() + " %");
    } else {
      let percent = progress ? progress.percent : 0;
      setStatusString(t("optProgress") + percent.toFixed(1).toString() + " %");
    }
  }, [pointsList, isFinetune, progress]);

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
                staticTransform={staticTransform}
                finetuneTransform={finetuneTransform}
              />
            );
          })}
        </Canvas>
      </div>
      {isFinetune && (
        <Paper className="finetune-panel">
          <Slider {...{ translation, rotation, setTranslation, setRotation }} />
        </Paper>
      )}
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
  staticTransform?: THREE.Matrix4;
  finetuneTransform?: THREE.Matrix4;
};

function PositionPoints({ points, pose, staticTransform, finetuneTransform }: IProps) {
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
      if (staticTransform) {
        T = T.multiply(staticTransform);
      }
      if (finetuneTransform) {
        T = T.multiply(finetuneTransform);
      }

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
  }, [transform, staticTransform, finetuneTransform]);

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
