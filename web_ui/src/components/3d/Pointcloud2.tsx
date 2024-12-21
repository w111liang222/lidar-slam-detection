import { PointsProps } from "@react-three/fiber";
import { useCounter, useCreation, useMount } from "ahooks";
import React, { useRef, useEffect } from "react";
import * as THREE from "three";
import vertexShader from "./shader/vertex_shader_points_base.glsl?raw";
import fragmentShaderDepth from "./shader/fragment_shader_points_color_depth.glsl?raw";
import fragmentShaderHeight from "./shader/fragment_shader_points_color_height.glsl?raw";

// https://threejs.org/docs/#manual/en/introduction/How-to-update-things
const MAX_POINTS_NUM = 100e4;

export type Props = {
  maxNum?: number;
  points?: LSD.PointCloud2;
  onPointsUpdate?: () => void;
  begin: number;
  end: number;
  color?: "gray" | "height" | "depth" | "intensity" | "rgb";
  size?: number;
  frustumCulled?: boolean;
  visible?: boolean;
  zRange?: any;
  pointsProps?: PointsProps;
};

export default function Pointcloud2({
  maxNum = MAX_POINTS_NUM,
  points,
  onPointsUpdate,
  begin,
  end,
  color = "gray",
  size = 1.0,
  frustumCulled = false,
  visible = true,
  zRange = { min: -10000.0, max: 10000.0 },
  pointsProps,
}: Props) {
  const positionAttr = useCreation(() => new THREE.BufferAttribute(new Float32Array(maxNum * 3), 3), [maxNum]);
  const colorAttr = useCreation(() => new THREE.BufferAttribute(new Float32Array(maxNum * 3), 3), [maxNum]);
  useEffect(() => {
    if (points && visible) {
      if (points.type == "intensity" && color == "intensity") {
        for (let i = 0; i < ((end - begin) / 3); i++) {
          let h = Math.min(1.0, Math.max(0, points.attr[i + (begin / 3)] / 1.0)) * 4.0 + 1.0;
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
          colorAttr.setXYZ(i, r, g, b);
        }
        colorAttr.count = (end - begin) / 3;
      } else if (points.type == "rgb" && color == "rgb") {
        for (let i = 0; i < ((end - begin) / 3); i++) {
          let r = points.attr[4 * i + 2] / 255.0;
          let g = points.attr[4 * i + 1] / 255.0;
          let b = points.attr[4 * i + 0] / 255.0;
          colorAttr.setXYZ(i, r, g, b);
        }
        colorAttr.count = (end - begin) / 3;
      }

      positionAttr.set(points.points.subarray(begin, end));
      positionAttr.count = (end - begin) / 3;
      positionAttr.needsUpdate = true;
      colorAttr.needsUpdate = true;
    } else {
      colorAttr.count = 0;
      positionAttr.count = 0;
      positionAttr.needsUpdate = true;
      colorAttr.needsUpdate = true;
    }
    onPointsUpdate && onPointsUpdate();
  }, [points, color, visible]);

  const material = useRef();
  const uniforms = useCreation(() => {
    const heightMax = zRange.max > 1000.0 ? 4.0 : zRange.max / 10.0;
    const heightMin = zRange.min < -1000.0 ? -2.0 : zRange.min / 10.0;
    return {
      pointSize: { value: size },
      heightColor: { value: [heightMax, heightMin] },
    };
  }, []);

  useEffect(() => {
    const shader = material.current as any;
    if (shader && shader.uniforms) {
      const heightMax = zRange.max > 1000.0 ? 4.0 : zRange.max / 10.0;
      const heightMin = zRange.min < -1000.0 ? -2.0 : zRange.min / 10.0;
      shader.uniforms.pointSize.value = size;
      shader.uniforms.heightColor.value = [heightMax, heightMin];
    }
  }, [size, zRange.min, zRange.max]);

  return (
    <points frustumCulled={frustumCulled} {...pointsProps}>
      <bufferGeometry attributes={{ position: positionAttr, color: colorAttr }} />
      {color !== "gray" && color !== "intensity" && color !== "rgb" ? (
        <shaderMaterial
          ref={material}
          {...{
            clippingPlanes: [
              new THREE.Plane(new THREE.Vector3(0, 0, -1), zRange.max / 10.0),
              new THREE.Plane(new THREE.Vector3(0, 0, 1), -zRange.min / 10.0),
            ],
            vertexShader,
            fragmentShader: color === "depth" ? fragmentShaderDepth : fragmentShaderHeight,
            uniforms,
          }}
          clipping={true}
          onUpdate={(self) => {
            self.needsUpdate = true;
          }}
        />
      ) : color === "gray" ? (
        <pointsMaterial
          key="gray"
          {...{
            size: size,
            sizeAttenuation: false,
            color: 0xc8f8f8f,
            clippingPlanes: [
              new THREE.Plane(new THREE.Vector3(0, 0, -1), zRange.max / 10.0),
              new THREE.Plane(new THREE.Vector3(0, 0, 1), -zRange.min / 10.0),
            ],
          }}
        />
      ) : (
        <pointsMaterial
          key="color"
          {...{
            size: size,
            sizeAttenuation: false,
            vertexColors: true,
            clippingPlanes: [
              new THREE.Plane(new THREE.Vector3(0, 0, -1), zRange.max / 10.0),
              new THREE.Plane(new THREE.Vector3(0, 0, 1), -zRange.min / 10.0),
            ],
          }}
        />
      )}
    </points>
  );
}
