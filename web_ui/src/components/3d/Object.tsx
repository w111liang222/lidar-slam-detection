import React, { useRef, useEffect } from "react";
import * as THREE from "three";

const COLORS = [0x967fcb, 0x4e91f0, 0xffc6b1, 0x967fcb, 0xffffff];
const meshMatrials: THREE.MeshBasicMaterial[] = [];
for (let color of COLORS) {
  meshMatrials.push(
    new THREE.MeshBasicMaterial({
      color: color,
      wireframe: false,
      transparent: true,
      opacity: 0.4,
    })
  );
}
const lineMatrials: THREE.LineBasicMaterial[] = [];
for (let color of COLORS) {
  lineMatrials.push(
    new THREE.LineBasicMaterial({
      color: color,
      linewidth: 2,
    })
  );
}

export type Props = {
  object: LSD.IObject;
  children?: React.ReactNode[] | React.ReactNode;
  visible?: boolean;
  planeVisible?: boolean;
  showArrow?: boolean;
  colorType?: string;
};

export default function ObjectDetected({
  object,
  children,
  visible = true,
  planeVisible = true,
  showArrow = false,
  colorType = "objectType",
}: Props) {
  const boxRef = useRef<THREE.Mesh>();
  const edgeRef = useRef<THREE.Line>();
  const arrowRef = useRef<THREE.ArrowHelper>();
  const groupRef = useRef<THREE.Group>();
  const res = (
    <group ref={groupRef} visible={false}>
      {children}
      <mesh ref={boxRef} visible={false}>
        <boxGeometry args={[1, 1, 1]} />
      </mesh>
      <lineSegments ref={edgeRef} />
      <arrowHelper
        ref={arrowRef}
        args={[
          new THREE.Vector3(1, 0, 0), // dir
          new THREE.Vector3(0, 0, 0), // origin
          2,
          0xffff00,
        ]}
        visible={showArrow}
      />
    </group>
  );

  useEffect(() => {
    if (edgeRef.current && boxRef.current) {
      edgeRef.current.geometry = new THREE.EdgesGeometry(boxRef.current.geometry);
    }
  }, []);

  useEffect(() => {
    if (object && object.box) {
      const box = boxRef.current;
      const edge = edgeRef.current;
      const arrow = arrowRef.current;
      const group = groupRef.current;
      if (box && edge && arrow && group) {
        const classId = object.type;
        box.material = meshMatrials[classId];
        edge.material = lineMatrials[classId];
        box.visible = planeVisible;

        const { width, height, length } = object.box!;
        box.scale.set(length, width, height);
        edge.scale.set(length, width, height);

        arrow.setLength(Math.max(object.box.length / 2, 1));

        group.position.x = object.box.center.x;
        group.position.y = object.box.center.y;
        group.position.z = object.box.center.z;
        group.rotation.z = object.box.heading;
        group.visible = visible;
      }
    }
  }, [object]);

  return res;
}

export function ExcludeBox({ excludeConfig = [0, 0, 0, 0, 0, 0] }: { excludeConfig: number[] }) {
  const boxRef = useRef<THREE.Mesh>();
  const res = (
    <mesh ref={boxRef}>
      <boxGeometry args={[1, 1, 1]} />
    </mesh>
  );

  useEffect(() => {
    if (excludeConfig) {
      const box = boxRef.current;
      if (box) {
        box.material = new THREE.MeshBasicMaterial({
          color: 0x6699ff,
          wireframe: false,
          transparent: true,
          opacity: 0.4,
        });
        let lwhBox = [
          excludeConfig[3] - excludeConfig[0],
          excludeConfig[4] - excludeConfig[1],
          excludeConfig[5] - excludeConfig[2],
        ];
        box.scale.set(lwhBox[0], lwhBox[1], lwhBox[2]);
        box.position.x = (excludeConfig[3] + excludeConfig[0]) / 2;
        box.position.y = (excludeConfig[4] + excludeConfig[1]) / 2;
        box.position.z = (excludeConfig[5] + excludeConfig[2]) / 2;
      }
    }
  }, [excludeConfig]);
  return res;
}
