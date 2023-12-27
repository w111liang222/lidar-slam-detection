import React, { useRef } from "react";
import { useEffect, useState, useMemo } from "react";
import * as THREE from "three";

import Pointcloud from "@components/3d/PointcloudImage";

export interface Props {
  maxNum?: number;
  points?: Float32Array;
  imageData: LSD.MapKeyframe["images"];
  config?: any;
  boardConfig: LSD.Config;
  insExtrinic: THREE.Matrix4;
  visible: boolean;
}

export default function PointCloudRGB({ maxNum, points, imageData, config, boardConfig, insExtrinic, visible }: Props) {
  const [cameraConfig, setCameraConfig] = useState<{
    [index: string]: LSD.Config["camera"][0];
  }>();

  useEffect(() => {
    const c: typeof cameraConfig = {};
    for (let cfg of boardConfig.camera) {
      c[cfg.name] = cfg;
    }
    setCameraConfig(c);
  }, [boardConfig]);

  return (
    <>
      {imageData &&
        cameraConfig &&
        Object.keys(imageData)
          .sort()
          .map((key) => {
            if (!cameraConfig.hasOwnProperty(key)) {
              return;
            }
            return (
              <PointImageViewer
                key={key}
                maxNum={maxNum}
                cameraName={key}
                points={points}
                imageData={imageData[key]}
                cameraConfig={cameraConfig}
                insExtrinic={insExtrinic}
                visible={visible}
                config={config}
              />
            );
          })}
    </>
  );
}

type IProps = {
  maxNum?: number;
  cameraName?: string;
  points?: Float32Array;
  imageData: Uint8Array;
  cameraConfig?: { [index: string]: LSD.Config["camera"][0] };
  insExtrinic: THREE.Matrix4;
  visible: boolean;
  config?: any;
};

function PointImageViewer({
  maxNum,
  cameraName = "",
  points,
  imageData,
  cameraConfig,
  insExtrinic,
  visible,
  config,
}: IProps) {
  const ref = useRef({ lock: false });
  const [imageUrl, setImageUrl] = useState<string>();
  useEffect(() => {
    if (!imageData) return;
    if (ref.current.lock) return;

    ref.current.lock = true;
    const blob = new Blob([imageData.buffer], {
      type: "application/octet-stream",
    });
    setImageUrl(URL.createObjectURL(blob));
  }, [imageData]);

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
      setTransform(T.multiply(insExtrinic.clone().invert()));
    }
  }, [cameraConfig, cameraName]);

  const [fx, fy, cx, cy] = useMemo(() => {
    if (cameraName === undefined || cameraConfig === undefined) {
      return [600, 600, 320, 240];
    } else {
      return cameraConfig[cameraName]?.intrinsic_parameters.slice(0, 4);
    }
  }, [cameraName, cameraConfig]);

  return (
    <Pointcloud
      maxNum={maxNum}
      {...config}
      pointsize={3.0}
      transparent={true}
      points={points}
      imageUrl={imageUrl}
      onLoad={() => {
        if (imageUrl) {
          URL.revokeObjectURL(imageUrl);
        }
        ref.current.lock = false;
      }}
      cameraIntrinsic={{ cx, cy, fx, fy }}
      pointsProps={transform}
      visible={visible}
    />
  );
}
