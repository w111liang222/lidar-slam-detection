import React from "react";
import { useEffect, useRef } from "react";

export type Props = {
  trajectory?: THREE.Vector3[];
  lineColor?: number;
};

export default function TrajectoryView({ trajectory = [], lineColor=0x00ff66 }: Props) {
  const trajectoryRef = useRef<any>();
  const trajectoryPointRef = useRef<any>();

  useEffect(() => {
    const line = trajectoryRef.current;
    const point = trajectoryPointRef.current;
    if (line) {
      line.frustumCulled = false;
    }
    if (point) {
      point.frustumCulled = false;
    }
  }, []);

  useEffect(() => {
    const line = trajectoryRef.current;
    const point = trajectoryPointRef.current;
    if (line && point && trajectory) {
      line?.geometry.setFromPoints(trajectory);
      point?.geometry.setFromPoints(trajectory);
    }
  }, [trajectory]);

  return (
    <>
      <line ref={trajectoryRef}>
        <lineBasicMaterial color={lineColor} linewidth={3} />
      </line>
      <points ref={trajectoryPointRef}>
        <pointsMaterial color={0xff0033} size={0.2} />
      </points>
    </>
  );
}
