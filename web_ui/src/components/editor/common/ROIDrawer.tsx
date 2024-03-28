import React, { useEffect, useRef, useState } from "react";
import { ThreeEvent, useThree } from "@react-three/fiber";
import { useImperativeHandle } from "react";
import * as THREE from "three";

export type Props = {
  state: string;
  setState: any;
  onFinish: any;
};

function ROIDrawer({ state, setState, onFinish }: Props, ref: any) {
  const { camera } = useThree();

  const [roi, setRoi] = useState<THREE.Vector3[]>([]);
  const lineRef = useRef<any>();
  const meshRef = useRef<THREE.Mesh>();

  useEffect(() => {
    const line = lineRef.current;
    line && line.geometry.setFromPoints(roi);
  }, [roi]);

  useEffect(() => {
    const mesh = meshRef.current;
    if (state == "drawing" && mesh) {
      mesh.applyMatrix4(camera.matrixWorld);
      const focalPoint = new THREE.Vector3(0, 0, -1);
      focalPoint.applyQuaternion(camera.quaternion).add(camera.position);
      mesh.position.set(focalPoint.x, focalPoint.y, focalPoint.z);
    }
  }, [state]);

  useEffect(() => {
    const line = lineRef.current;
    if (line) {
      line.renderOrder = 999;
      line.material.depthTest = false;
      line.material.depthWrite = false;
      line.frustumCulled = false;
    }
  }, []);

  const addPoints = (e: ThreeEvent<MouseEvent>) => {
    if (state.includes("drawing")) {
      setRoi([...roi, e.intersections[0].point]);
    }
  };

  const moveLine = (e: ThreeEvent<MouseEvent>) => {
    if (state.includes("drawing") && roi.length > 0) {
      const line = lineRef.current;
      line?.geometry.setFromPoints([...roi, e.intersections[0].point, roi[0]]);
    }
  };

  useImperativeHandle(ref, () => ({
    onContextClick: () => {
      if (roi.length == 0 && state == "idle") {
        return;
      }
      if (state == "idle") {
        onFinish([...roi], camera);
        setRoi([]);
        return;
      }

      if (roi.length <= 2) {
        onFinish(undefined, camera);
        setRoi([]);
      } else {
        setRoi([...roi, roi[0]]);
      }
      setState("idle");
    },
  }));

  return (
    <>
      <line ref={lineRef}>
        <lineBasicMaterial color={0xffffff} linewidth={2} />
      </line>
      {state.includes("drawing") && (
        <mesh ref={meshRef} onClick={addPoints} onPointerMove={moveLine} visible={false}>
          <planeBufferGeometry args={[10000, 10000]} />
        </mesh>
      )}
    </>
  );
}

export default React.memo(React.forwardRef(ROIDrawer));
