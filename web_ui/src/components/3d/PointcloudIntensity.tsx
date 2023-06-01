import { PointsProps } from "@react-three/fiber";
import { useCounter, useCreation, useMount } from "ahooks";
import React, { useRef, useEffect } from "react";
import * as THREE from "three";

// https://threejs.org/docs/#manual/en/introduction/How-to-update-things
const MAX_POINTS_NUM = 100e4;

export type Props = {
  maxNum?: number;
  points?: Float32Array;
  onPointsUpdate?: () => void;
  color?: "gray" | "height" | "depth" | "intensity" | "rgb";
  maxIntensity?: number;
  size?: number;
  frustumCulled?: boolean;
  visible?: boolean;
  zRange?: any;
  clip?: any;
  pointsProps?: PointsProps;
};

export default function PointcloudIntensity({
  maxNum = MAX_POINTS_NUM,
  points,
  onPointsUpdate,
  color = "intensity",
  maxIntensity = 1.0,
  size = 1.0,
  frustumCulled = false,
  visible = true,
  zRange = { min: -10000.0, max: 10000.0 },
  clip,
  pointsProps,
}: Props) {
  const positionAttr = useCreation(() => new THREE.BufferAttribute(new Float32Array(maxNum * 4), 4), [maxNum]);
  const colorAttr = useCreation(() => new THREE.BufferAttribute(new Float32Array(maxNum * 3), 3), [maxNum]);
  useEffect(() => {
    if (points && visible) {
      let intensity = [];
      for (let i = 0, j = 0; i < points.length; i = i + 4, j = j + 3) {
        let h = Math.min(1.0, Math.max(0, points[i + 3] / maxIntensity)) * 4.0 + 1.0;
        let rank = Math.floor(h);
        let f = h - rank;
        if (rank & 1) f = 1 - f;
        let n = 1 - f;
        let r, g, b;

        if (rank <= 1) {
          r = 0;
          g = n;
          b = 1;
        } else if (rank == 2) {
          r = 0;
          g = 1;
          b = n;
        } else if (rank == 3) {
          r = n;
          g = 1;
          b = 0;
        } else {
          r = 1;
          g = n;
          b = 0;
        }
        intensity[j + 0] = r;
        intensity[j + 1] = g;
        intensity[j + 2] = b;
      }

      positionAttr.set(points);
      positionAttr.count = points.length / 4;
      colorAttr.set(intensity);
      colorAttr.count = intensity.length / 3;
      positionAttr.needsUpdate = true;
      colorAttr.needsUpdate = true;
    } else {
      positionAttr.count = 0;
      colorAttr.count = 0;
      positionAttr.needsUpdate = true;
      colorAttr.needsUpdate = true;
    }
    onPointsUpdate && onPointsUpdate();
  }, [points, visible, maxIntensity]);

  return (
    <points frustumCulled={frustumCulled} {...pointsProps}>
      <bufferGeometry attributes={{ position: positionAttr, color: colorAttr }} />
      <pointsMaterial
        {...{
          size: size,
          sizeAttenuation: false,
          vertexColors: true,
          clippingPlanes: [
            new THREE.Plane(new THREE.Vector3(0, 0, -1), clip ? clip.max / 10.0 : zRange.max / 10.0),
            new THREE.Plane(new THREE.Vector3(0, 0, 1), clip ? -clip.min / 10.0 : -zRange.min / 10.0),
          ],
        }}
      />
    </points>
  );
}
