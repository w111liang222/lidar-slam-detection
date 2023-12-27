import * as THREE from "three";
import React, { useEffect, useRef } from "react";
import { useThree } from "@react-three/fiber";
import vertexShader from "./shader/roi_vertex_shader.glsl?raw";
import fragmentShader from "./shader/roi_fragment_shader.glsl?raw";

const uniforms = {
  color1: {
    value: new THREE.Color("white"),
  },
  color2: {
    value: new THREE.Color("black"),
  },
};

const blackMat = new THREE.MeshBasicMaterial({
  color: 0x000000,
  transparent: true,
  opacity: 0.8,
  side: THREE.DoubleSide,
});
const grayMat = new THREE.MeshBasicMaterial({
  color: 0x343434,
  transparent: true,
  opacity: 0.8,
  side: THREE.DoubleSide,
});

export interface Props {
  roi: THREE.Vector3[];
  inside?: boolean;
  visible?: boolean;
}
// https://github.com/mrdoob/three.js/issues/11539
export default function ROI({ roi, inside = true, visible = true }: Props) {
  const roiMesh0 = useRef<THREE.Mesh>();
  const roiMesh1 = useRef<THREE.Mesh>();
  const roiUnderRef = useRef<THREE.Mesh>();
  const res = (
    <group visible={visible}>
      <mesh ref={roiUnderRef}>{/* <shapeGeometry /> */}</mesh>
      <mesh ref={roiMesh0}>
        <bufferGeometry />
        <shaderMaterial
          {...{
            uniforms,
            vertexShader,
            fragmentShader,
            transparent: true,
            opacity: 0.4,
            side: THREE.BackSide,
            renderOrder: 0,
          }}
        />
      </mesh>
      <mesh ref={roiMesh1}>
        <bufferGeometry />
        <shaderMaterial
          {...{
            uniforms,
            vertexShader,
            fragmentShader,
            transparent: true,
            opacity: 0.4,
            side: THREE.FrontSide,
            renderOrder: 1,
          }}
        />
      </mesh>
    </group>
  );

  const { scene } = useThree();
  useEffect(() => {
    if (!visible) {
      scene.background = new THREE.Color(0x000000);
      return;
    }

    const contour = roi;
    let vertices: number[] = [];
    let uvs: number[] = [];
    let vec2s = [];
    if (contour.length > 0) {
      for (const pt of contour) {
        vertices = vertices.concat([pt.x, pt.y, 0]);
        vertices = vertices.concat([pt.x, pt.y, 1]);
        uvs = uvs.concat([0, 0]);
        uvs = uvs.concat([0, 1]);
        vec2s.push(new THREE.Vector2(pt.x, pt.y));
      }
      vertices = vertices.concat([contour[0].x, contour[0].y, 0]);
      vertices = vertices.concat([contour[0].x, contour[0].y, 1]);
      uvs = uvs.concat([0, 0]);
      uvs = uvs.concat([0, 1]);
    }
    for (let roiMesh of [roiMesh0, roiMesh1]) {
      if (roiMesh.current) {
        roiMesh.current.geometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(vertices), 3));
        roiMesh.current.geometry.setAttribute("uv", new THREE.BufferAttribute(new Float32Array(uvs), 2));

        let indexs: number[] = [];
        for (let i = 0; i < contour.length; i++) {
          let j = i * 2;
          indexs = indexs.concat([j, j + 1, j + 2, j + 2, j + 1, j + 3]);
        }
        roiMesh.current.geometry.setIndex(indexs);
      }
    }

    const roiUnder = roiUnderRef.current;
    if (!roiUnder) return;
    if (vec2s.length > 0) {
      roiUnder.geometry = new THREE.ShapeGeometry(new THREE.Shape(vec2s));
      roiUnder.visible = true;
      if (inside) {
        scene.background = new THREE.Color(0x343434);
        roiUnder.material = blackMat;
      } else {
        scene.background = new THREE.Color(0x000000);
        roiUnder.material = grayMat;
      }
    } else {
      roiUnder.visible = false;
      scene.background = new THREE.Color(0x000000);
    }
  }, [visible, roi, inside]);

  return res;
}
