import React, { useRef, useEffect, useState, useMemo } from "react";
import * as THREE from "three";
import { useTexture } from "./ImagePicker";
import { useCreation, useTrackedEffect, useWhyDidYouUpdate } from "ahooks";

import vertexShader from "./shader/vertex_shader_points_image.glsl?raw";
import fragmentShader from "./shader/fragment_shader_points_texture.glsl?raw";
import fragmentShaderDiscard from "./shader/fragment_shader_points_texture_discard.glsl?raw";

const MAX_POINTS_NUM = 100e4;

type CameraIntrinsic = {
  cx: number;
  cy: number;
  fx: number;
  fy: number;
};

export type Props = {
  maxNum?: number;
  pointsize?: number;
  transparent?: boolean;
  points?: Float32Array;
  imageUrl?: string;
  onLoad?: any;
  cameraIntrinsic?: CameraIntrinsic;
  pointsProps?: THREE.Matrix4;
  visible?: boolean;
  zRange?: any;
};

export default function Pointcloud({
  maxNum = MAX_POINTS_NUM,
  pointsize = 1.5,
  transparent = false,
  points,
  imageUrl,
  onLoad,
  pointsProps,
  cameraIntrinsic,
  visible = true,
  zRange = { min: -10000.0, max: 10000.0 },
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
  }, [points, visible]);

  const { texture, width, height } = useTexture(imageUrl, () => {
    onLoad && onLoad();
  });
  const uniforms = useCreation(
    () => ({
      pointSize: { value: pointsize },
      width: new THREE.Uniform(width),
      height: new THREE.Uniform(height),
      textureImage: new THREE.Uniform(texture),
      cx: new THREE.Uniform(cameraIntrinsic?.cx || 320),
      cy: new THREE.Uniform(cameraIntrinsic?.cy || 240),
      fx: new THREE.Uniform(cameraIntrinsic?.fx || 600),
      fy: new THREE.Uniform(cameraIntrinsic?.fy || 600),
      extraT: new THREE.Uniform(pointsProps || new THREE.Matrix4().set(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1)),
    }),
    []
  );
  const materialRef = useRef();
  useEffect(() => {
    const shader = materialRef.current as any;
    uniforms.textureImage.value = texture;
    uniforms.width.value = width;
    uniforms.height.value = height;
    if (shader) {
      shader.uniforms.pointSize.value = pointsize;
    }
    if (cameraIntrinsic) {
      uniforms.cx.value = cameraIntrinsic.cx;
      uniforms.cy.value = cameraIntrinsic.cy;
      uniforms.fx.value = cameraIntrinsic.fx;
      uniforms.fy.value = cameraIntrinsic.fy;
    }
    if (pointsProps) {
      uniforms.extraT.value = pointsProps;
    }
  }, [cameraIntrinsic, pointsProps, texture, width, height, pointsize]);

  let geometry = useMemo(() => <bufferGeometry attributes={{ position: positionAttr }} />, [positionAttr]);
  let material = useMemo(
    () => (
      <shaderMaterial
        ref={materialRef}
        {...{
          uniforms,
          vertexShader,
          fragmentShader: transparent ? fragmentShaderDiscard : fragmentShader,
          clippingPlanes: [
            new THREE.Plane(new THREE.Vector3(0, 0, -1), zRange.max / 10.0),
            new THREE.Plane(new THREE.Vector3(0, 0, 1), -zRange.min / 10.0),
          ],
        }}
        transparent={transparent}
        clipping={true}
        onUpdate={(self) => {
          self.needsUpdate = true;
        }}
      />
    ),
    [uniforms, zRange.min, zRange.max]
  );
  return (
    <points frustumCulled={false}>
      {geometry}
      {material}
    </points>
  );
}
