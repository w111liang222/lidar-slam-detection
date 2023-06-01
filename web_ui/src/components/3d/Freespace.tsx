import * as THREE from "three";
import React, { useEffect, useRef, useState } from "react";
import * as proto from "@proto/detection";

export type Props = {
  freespace?: proto.IFreespace | undefined;
  visible?: boolean;
};
export default function Freespace({ freespace, visible = false }: Props) {
  const meshRef = useRef<THREE.Mesh>();
  const res = (
    <mesh ref={meshRef}>
      <planeGeometry args={[1, 1]} />
    </mesh>
  );

  const ref = useRef<{
    canvas: HTMLCanvasElement | null;
    texture: THREE.Texture | null;
  }>({ canvas: null, texture: null });
  useEffect(() => {
    const canvas = document.createElement("canvas");
    const texture = new THREE.CanvasTexture(canvas);
    meshRef.current!.material = new THREE.MeshBasicMaterial({
      transparent: true,
      opacity: 0.8,
      map: texture,
    });
    ref.current = { canvas, texture };
  }, []);

  useEffect(() => {
    const mesh = meshRef.current;
    const { canvas, texture } = ref.current;
    if (canvas && texture && mesh && freespace && freespace.cells && freespace.info) {
      mesh.visible = visible;
      if (visible && freespace) {
        let w, h;
        w = freespace.info.xMax - freespace.info.xMin;
        h = freespace.info.yMax - freespace.info.yMin;
        mesh.scale.set(w, h, 1);
        mesh.position.x = w / 2 + freespace.info.xMin;
        mesh.position.y = h / 2 + freespace.info.yMin;
        mesh.position.z = freespace.info.zMin;

        let imageDataArray = new Uint8ClampedArray(freespace.info.yNum * freespace.info.xNum * 4);
        for (let i = 0; i < freespace.cells.length; i++) {
          if (freespace.cells[i] == 48) imageDataArray[4 * i + 1] = 255;
          imageDataArray[4 * i + 3] = 255;
        }
        let imageData = new ImageData(imageDataArray, freespace.info.xNum, freespace.info.yNum);

        canvas.width = freespace.info.xNum;
        canvas.height = freespace.info.yNum;
        const ctx = canvas.getContext("2d");
        ctx?.putImageData(imageData, 0, 0);

        texture.needsUpdate = true;
      }
    } else {
      if (mesh) mesh.visible = false;
    }
  });
  return res;
}
