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
  points?: Float32Array;
  onPointsUpdate?: () => void;
  color?: "gray" | "height" | "depth" | "intensity" | "rgb";
  size?: number;
  frustumCulled?: boolean;
  visible?: boolean;
  zRange?: any;
  clip?: any;
  pointsProps?: PointsProps;
};

export default function Pointcloud({
  maxNum = MAX_POINTS_NUM,
  points,
  onPointsUpdate,
  color = "gray",
  size = 1.0,
  frustumCulled = false,
  visible = true,
  zRange = { min: -10000.0, max: 10000.0 },
  clip,
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
      <bufferGeometry attributes={{ position: positionAttr }} />
      {color !== "gray" && color !== "rgb" ? (
        <shaderMaterial
          ref={material}
          {...{
            clippingPlanes: [
              new THREE.Plane(new THREE.Vector3(0, 0, -1), clip ? clip.max / 10.0 : zRange.max / 10.0),
              new THREE.Plane(new THREE.Vector3(0, 0, 1), clip ? -clip.min / 10.0 : -zRange.min / 10.0),
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
      ) : (
        <pointsMaterial
          {...{
            size: size,
            sizeAttenuation: false,
            color: 0xc8f8f8f,
            clippingPlanes: [
              new THREE.Plane(new THREE.Vector3(0, 0, -1), clip ? clip.max / 10.0 : zRange.max / 10.0),
              new THREE.Plane(new THREE.Vector3(0, 0, 1), clip ? -clip.min / 10.0 : -zRange.min / 10.0),
            ],
          }}
        />
      )}
    </points>
  );
}
