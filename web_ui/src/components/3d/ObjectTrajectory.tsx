import React, { useRef, useEffect } from "react";
import * as THREE from "three";

import * as proto from "@proto/detection";

export type Props = {
  trajectory?: proto.ITrajectory[];
  visible?: boolean;
};

export default function ObjectDetected({ trajectory = [], visible = true }: Props) {
  const meshRef = useRef<THREE.Mesh>();
  const res = (
    <mesh ref={meshRef} visible={visible}>
      <meshBasicMaterial {...{ color: 0xeeb04d, transparent: true, opacity: 0.8 }} />
    </mesh>
  );

  useEffect(() => {
    let traj = trajectory;
    if (visible && meshRef.current) {
      if (traj && traj.length > 0) {
        let dx = traj[1].x - traj[0].x;
        let dy = traj[1].y - traj[0].y;
        let lon = Math.atan2(dy, dx);
        let lat = lon + Math.PI / 2;
        dx = Math.cos(lat) * 0.3;
        dy = Math.sin(lat) * 0.3;
        let shape_points = [];
        for (let i = 0; i < traj.length; i++) shape_points.push(new THREE.Vector2(traj[i].x + dx, traj[i].y + dy));
        for (let i = 0; i < traj.length; i++)
          shape_points.push(new THREE.Vector2(traj[traj.length - 1 - i].x - dx, traj[traj.length - 1 - i].y - dy));

        meshRef.current.visible = true;
        meshRef.current.geometry.dispose();
        meshRef.current.geometry = new THREE.ShapeGeometry(new THREE.Shape(shape_points));
      } else {
        meshRef.current.visible = false;
      }
    }
  }, [trajectory]);

  return res;
}
