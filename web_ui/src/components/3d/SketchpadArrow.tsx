import React, { useRef, useState } from "react";
import { ThreeEvent } from "@react-three/fiber";
import { useImperativeHandle } from "react";
import * as THREE from "three";
import { getEstimatePose } from "@rpc/http";

export type Props = {
  enabled?: boolean;
  state: string;
  setPose: (pose: number[]) => void;
  onStart?: () => void;
  onEnd?: (state: string, reset: boolean) => void;
};

export type Ref = {
  start: () => void;
};
function SketchpadArraw({ enabled = false, state, setPose, onStart, onEnd }: Props, ref: React.Ref<Ref | undefined>) {
  const pointRef = useRef<any>();
  const lockRef = useRef({ lock: false });
  const [lastPoint, setLastPoint] = useState(new THREE.Vector3(0, 0, 0));

  const addPoints = (e: ThreeEvent<MouseEvent>) => {
    if (enabled && state === "drawing") {
      onEnd && onEnd("idle", false);
    }
  };

  const moveLine = (e: ThreeEvent<MouseEvent>) => {
    const point = pointRef.current;
    const p = e.intersections[0].point;
    if (enabled && state === "drawing") {
      point.frustumCulled = false;
      point.renderOrder = 999;
      point.material.depthTest = false;
      point.material.depthWrite = false;
      point.geometry.setFromPoints([p]);

      if (!lockRef.current.lock && p.distanceTo(lastPoint) > 0.5) {
        lockRef.current.lock = true;
        getEstimatePose([
          [lastPoint.x, lastPoint.y],
          [p.x, p.y],
        ]).then((pose) => {
          if (pose[6] == 1) {
            onEnd && onEnd("idle", true);
          }
          setPose(pose);
          lockRef.current.lock = false;
        });
      }

      if (p.distanceTo(lastPoint) > 1.0) {
        setLastPoint(p);
      }
    }
  };

  useImperativeHandle(ref, () => ({
    start: () => {
      onStart && onStart();
    },
  }));

  return (
    <group>
      {state === "drawing" && (
        <points ref={pointRef}>
          <pointsMaterial color={0xff0000} size={8.0} sizeAttenuation={false} />
        </points>
      )}
      <mesh onClick={addPoints} onPointerMove={moveLine} visible={false}>
        <planeBufferGeometry args={[10000, 10000]} />
      </mesh>
    </group>
  );
}

export default React.forwardRef(SketchpadArraw);
