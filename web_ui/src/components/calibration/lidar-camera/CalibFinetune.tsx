import React from "react";
import { useEffect, useState, useRef, useMemo } from "react";
import * as THREE from "three";
import { Canvas } from "@react-three/fiber";
import Pointcloud from "@components/3d/Pointcloud";
import { finetuneCamera, getConfig, getTransform } from "@rpc/http";
import { useTexture } from "@components/3d/ImagePicker";
import Slider from "../lidar/SliderCalib";
import Controls from "@components/3d/Controls";
import { useImperativeHandle } from "react";
import { useCtrlKey } from "@hooks/keyboard";
import { useSessionStorageState } from "ahooks";
import log from "loglevel";
import { Paper } from "@mui/material";

export interface Props {
  points?: Float32Array;
  imageUrl?: string;
  cameraName?: string;
  timestamp?: number;
  setCalibEnable?: any;
}

function eulerDeg2Rad(euler: any) {
  const ret = euler.slice();
  ret[0] = (euler[0] / 180) * Math.PI;
  ret[1] = (euler[1] / 180) * Math.PI;
  ret[2] = (euler[2] / 180) * Math.PI;
  return ret;
}

function CalibFinetune({ points, imageUrl, cameraName, timestamp, setCalibEnable }: Props, ref: any) {
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

  const [rotation, setRotation] = useState([0, 0, 0, "XYZ"]);
  const [translation, setTranslation] = useState([0, 0, 0]);
  const [deltaRotation, setDeltaRotation] = useState([0, 0, 0, "XYZ"]);
  const [deltaTranslation, setDeltaTranslation] = useState([0, 0, 0]);
  useEffect(() => {
    if (cameraConfig && cameraName) {
      const cfg = cameraConfig[cameraName];
      getTransform(cfg.extrinsic_parameters).then((T) => {
        // console.log(T.clone().transpose().toArray()); // !! inplace !!
        const pos = new THREE.Vector3();
        const quat = new THREE.Quaternion();
        const scale = new THREE.Vector3();
        T.decompose(pos, quat, scale);
        const euler = new THREE.Euler();
        euler.setFromQuaternion(quat);
        console.log(pos);
        setTranslation(pos.toArray());
        setRotation(euler.toArray());
      });
    }
  }, [cameraConfig, cameraName]);

  useEffect(() => {
    setCalibEnable(cameraName);
  }, [cameraName]);

  const [fx, fy, cx, cy] = useMemo(() => {
    if (cameraName === undefined || cameraConfig === undefined) {
      return [600, 600, 320, 240];
    } else {
      return cameraConfig[cameraName]?.intrinsic_parameters.slice(0, 4);
    }
  }, [cameraConfig, cameraName]);

  const { texture, width, height } = useTexture(imageUrl, () => {
    log.debug(`rendered image of ${timestamp} at ${performance.now()}`);
  });

  useImperativeHandle(ref, () => ({
    calibrate: async () => {
      let dT = new THREE.Matrix4();
      const euler = new THREE.Euler();
      euler.fromArray(eulerDeg2Rad(deltaRotation));
      dT.makeRotationFromEuler(euler);
      dT.setPosition(...(deltaTranslation as [number, number, number]));
      if (cameraName === undefined) {
        throw new Error("No target lidar index!");
      }
      // console.log(T);
      const T = await finetuneCamera({
        cameraName,
        transform: dT,
      });
      {
        const pos = new THREE.Vector3();
        const quat = new THREE.Quaternion();
        const scale = new THREE.Vector3();
        T.decompose(pos, quat, scale);
        const euler = new THREE.Euler();
        euler.setFromQuaternion(quat);
        setTranslation(pos.toArray());
        setRotation(euler.toArray());
      }
      setDeltaTranslation([0, 0, 0]);
      setDeltaRotation([0, 0, 0, "XYZ"]);
      return T; // transpose in place
    },
  }));

  const ctrlKeyPressed = useCtrlKey();
  return (
    <>
      <div className="calibration-tab"></div>
      <div
        style={{
          position: "absolute",
          width: "100vw",
          height: "45rem",
        }}>
        <Canvas
          camera={{
            fov: 60,
            position: [0, 0, 0],
          }}
          onCreated={({ camera }) => {
            console.log(camera);
            camera.rotateX(Math.PI);
          }}>
          <group
            position={deltaTranslation as any as THREE.Vector3}
            rotation={eulerDeg2Rad(deltaRotation as any as THREE.Euler)}>
            <Pointcloud
              size={2.0}
              maxNum={100e4}
              points={points}
              pointsProps={{
                position: translation as [number, number, number],
                rotation: rotation as [number, number, number, string],
              }}
              onPointsUpdate={() => {
                log.debug(`rendered pointcloud of ${timestamp} at ${performance.now()}`);
              }}
              color="depth"
            />
          </group>
          {/* camera image */}
          <mesh
            position={[(width / 2 - cx) / fx, (height / 2 - cy) / fy, 1]}
            rotation-x={Math.PI}
            scale={[width / fx, height / fy, 1]}>
            <planeBufferGeometry args={[1, 1]} />
            {texture ? (
              <meshBasicMaterial
                map={texture}
                side={THREE.DoubleSide}
                opacity={ctrlKeyPressed ? 0 : 1}
                // transparent={true}
                blending={THREE.AdditiveBlending}
              />
            ) : null}
          </mesh>
        </Canvas>
      </div>
      <Paper className="finetune-panel">
        <Slider
          {...{
            translation: deltaTranslation,
            setTranslation: setDeltaTranslation,
            rotation: deltaRotation,
            setRotation: setDeltaRotation,
          }}
        />
      </Paper>
    </>
  );
}

export default React.forwardRef(CalibFinetune);
