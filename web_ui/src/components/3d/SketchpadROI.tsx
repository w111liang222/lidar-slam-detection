import React, { useEffect, useRef } from "react";
import { ThreeEvent } from "@react-three/fiber";
import produce from "immer";
import { useState } from "react";
import { useImperativeHandle } from "react";

export type Props = {
  enabled?: boolean;
  roi: THREE.Vector3[];
  state: string;
  onChange: (pts: THREE.Vector3[]) => void;
  onStart?: () => void;
  onEnd?: (state: string) => void;
};

export type Ref = {
  start: () => void;
  stop: () => void;
};
function Sketchpad({ enabled = false, roi, state, onChange, onStart, onEnd }: Props, ref: React.Ref<Ref | undefined>) {
  const lineRef = useRef<any>();

  useEffect(() => {
    const line = lineRef.current;
    line && line.geometry.setFromPoints(roi);
  }, [roi]);

  useEffect(() => {
    const line = lineRef.current;
    if (line) {
      line.frustumCulled = false;
    }
  }, []);

  const addPoints = (e: ThreeEvent<MouseEvent>) => {
    if (enabled && state === "drawing") {
      onChange([...roi, e.intersections[0].point]);
    }
  };

  const closeROI = () => {
    if (enabled && state === "drawing") {
      if (roi.length > 2) {
        onChange([...roi, roi[0]]);
        onEnd && onEnd("idle");
      } else {
        onChange([]);
        onEnd && onEnd("drawing");
      }
    }
  };
  const moveLine = (e: ThreeEvent<MouseEvent>) => {
    if (enabled && state === "drawing") {
      const line = lineRef.current;
      line?.geometry.setFromPoints([...roi, e.intersections[0].point]);
    }
  };

  useImperativeHandle(ref, () => ({
    start: () => {
      onStart && onStart();
    },
    stop: closeROI,
    roi: roi,
  }));

  return (
    <group>
      <line ref={lineRef}>
        <lineBasicMaterial color={0xffffff} linewidth={2} />
      </line>
      <mesh onClick={addPoints} onPointerMove={moveLine} visible={false}>
        <planeBufferGeometry args={[10000, 10000]} />
      </mesh>
    </group>
  );
}

export default React.forwardRef(Sketchpad);
