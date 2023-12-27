import React, { useEffect, useRef } from "react";
import * as THREE from "three";

type Props = {
  poses?: THREE.Vector3[];
  color: number;
};

export default function PoseView({ poses = [], color }: Props) {
  const pointRef = useRef<any>();

  useEffect(() => {
    const point = pointRef.current;
    if (point) {
      point.frustumCulled = false;
    }
  }, []);

  useEffect(() => {
    const point = pointRef.current;
    if (point && poses) {
      point.geometry.setFromPoints(poses);
    }
  }, [poses]);

  return (
    <points ref={pointRef}>
      <pointsMaterial color={color} size={0.5} precision="highp" />
    </points>
  );
}
