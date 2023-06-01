import React, { useEffect, useState } from "react";
import * as THREE from "three";
import PointcloudIntensity from "@components/3d/PointcloudIntensity";
import Pointcloud from "@components/3d/Pointcloud";
import PointcloudCustomeColor from "@components/3d/PointcloudCustomeColor";
import PointCloudRGB from "./PointCloudRGB";

type IPointProp = {
  position: [number, number, number];
  rotation: [number, number, number, string];
};

type Props = {
  id: string;
  points?: Float32Array;
  images?: LSD.MapKeyframe["images"];
  color?: number;
  size: number;
  config?: any;
  boardConfig?: LSD.Config;
  insExtrinic?: THREE.Matrix4;
  isSelect?: boolean;
  visible?: boolean;
  vertexPose?: THREE.Matrix4;
};

export default function PointVertexView({
  id,
  points,
  images,
  color,
  size = 3,
  config,
  boardConfig,
  insExtrinic,
  isSelect,
  visible,
  vertexPose,
}: Props) {
  const [pointProp, setPointProp] = useState<IPointProp>();

  useEffect(() => {
    if (vertexPose) {
      const pos = new THREE.Vector3();
      const quat = new THREE.Quaternion();
      const scale = new THREE.Vector3();
      vertexPose.decompose(pos, quat, scale);
      const euler = new THREE.Euler();
      euler.setFromQuaternion(quat);
      if (pointProp) {
        let rotation = pointProp["rotation"];
        let translation = pointProp["position"];
        let eulerDiff =
          Math.abs((rotation[0] as number) - euler.x) +
          Math.abs((rotation[1] as number) - euler.y) +
          Math.abs((rotation[2] as number) - euler.z);
        let posDiff =
          Math.abs((translation[0] as number) - pos.x) +
          Math.abs((translation[1] as number) - pos.y) +
          Math.abs((translation[2] as number) - pos.z);
        if (eulerDiff < 1e-6 && posDiff < 1e-6) {
          return;
        }
      }
      setPointProp({
        position: pos.toArray() as [number, number, number],
        rotation: euler.toArray() as [number, number, number, string],
      });
    }
  }, [vertexPose]);

  return (
    <>
      {isSelect ? (
        <PointcloudCustomeColor
          maxNum={points ? points.length / 4 : 0}
          points={points}
          {...config}
          color={color}
          size={size}
          frustumCulled={config.frustumCulled}
          visible={(config.visible && visible) || isSelect}
          pointsProps={pointProp}
        />
      ) : config.color == "rgb" ? (
        <></>
      ) : // <group {...pointProp}>
      //   {boardConfig && insExtrinic && (
      //     <PointCloudRGB
      //       maxNum={points ? points.length / 4 : 0}
      //       points={points}
      //       imageData={images}
      //       config={config}
      //       boardConfig={boardConfig}
      //       insExtrinic={insExtrinic}
      //       visible={(config.visible && visible) || isSelect}
      //     />
      //   )}
      // </group>
      config.color == "intensity" ? (
        <PointcloudIntensity
          maxNum={points ? points.length / 4 : 0}
          points={points}
          {...config}
          frustumCulled={config.frustumCulled}
          visible={(config.visible && visible) || isSelect}
          pointsProps={pointProp}
        />
      ) : (
        <Pointcloud
          maxNum={points ? points.length / 4 : 0}
          points={points}
          {...config}
          frustumCulled={config.frustumCulled}
          visible={(config.visible && visible) || isSelect}
          pointsProps={pointProp}
        />
      )}
    </>
  );
}
