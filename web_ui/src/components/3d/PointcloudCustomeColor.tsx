import { PointsProps } from "@react-three/fiber";
import { useCreation } from "ahooks";
import React, { useEffect } from "react";
import * as THREE from "three";

// https://threejs.org/docs/#manual/en/introduction/How-to-update-things
const MAX_POINTS_NUM = 100e4;

export type Props = {
  maxNum?: number;
  points?: Float32Array;
  color?: number;
  onPointsUpdate?: () => void;
  size?: number;
  frustumCulled?: boolean;
  visible?: boolean;
  pointsProps?: PointsProps;
};

export default function PointcloudCustomeColor({
  maxNum = MAX_POINTS_NUM,
  points,
  color,
  onPointsUpdate,
  size = 1.0,
  frustumCulled = false,
  visible = true,
  pointsProps,
}: Props) {
  const positionAttr = useCreation(() => new THREE.BufferAttribute(new Float32Array(maxNum * 4), 4), [maxNum]);
  useEffect(() => {
    if (points && visible) {
      positionAttr.set(points);
      positionAttr.count = points.length / 4;
      positionAttr.needsUpdate = true;
    } else {
      positionAttr.count = 0;
      positionAttr.needsUpdate = true;
    }
    onPointsUpdate && onPointsUpdate();
  }, [points, visible]);

  return (
    <points frustumCulled={frustumCulled} {...pointsProps}>
      <bufferGeometry attributes={{ position: positionAttr }} />
      <pointsMaterial {...{ size: size, sizeAttenuation: false, color: color }} />
    </points>
  );
}
