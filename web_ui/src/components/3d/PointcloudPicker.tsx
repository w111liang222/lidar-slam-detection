import React, { useRef, useEffect, useState } from "react";
import * as THREE from "three";
import { Canvas, ThreeEvent } from "@react-three/fiber";
import produce from "immer";
import Pointcloud from "./Pointcloud";

export type Props = {
  maxNum?: number | undefined;
  points: Float32Array | undefined;
  onPointsUpdate?: () => void;
  color?: "gray" | "depth" | "height";
  pick?: boolean;
  remove?: boolean;
  pointsPicked?: THREE.Vector3[];
  onAdd?: (pt: THREE.Vector3) => void;
  onRemove?: (i: number) => void;
};

export default function PointcloudPicker({
  maxNum = 100e4,
  points,
  onPointsUpdate,
  color = "gray",
  pick = true,
  remove = false,
  pointsPicked = [],
  onAdd,
  onRemove,
}: Props) {
  const addPoint = (pt: THREE.Vector3) => {
    if (pick && !remove) {
      onAdd && onAdd(pt);
    }
  };

  const removePoint = (id: number) => {
    if (remove && pick) {
      onRemove && onRemove(id);
    }
  };

  const [canvasIndexes] = useState(new Array(10).fill(0).map((x, i) => document.createElement("canvas")));
  useEffect(() => {
    canvasIndexes.forEach((canvas, i) => {
      const context = canvas.getContext("2d");
      let fontsize = 32;
      canvas.width = fontsize * 10;
      canvas.height = fontsize;
      // draw text
      if (context) {
        context.fillStyle = "rgba(240, 0, 0, 1.0)";
        context.font = `Bold ${fontsize}px Arial`;
        context.fillText(i + 1 + "", 0, fontsize);
      }
    });
  }, []);

  return (
    <group>
      <Pointcloud
        maxNum={maxNum}
        points={points}
        onPointsUpdate={onPointsUpdate}
        color={color}
        size={2.0}
        pointsProps={{
          onClick: (e: ThreeEvent<MouseEvent>) => {
            e.stopPropagation();
            const intersections = e.intersections;
            if (intersections.length > 0 && points) {
              let closestIndex = 0;
              let minDist = intersections[0].distanceToRay;
              for (let i = 0; i < intersections.length; i++) {
                const intersect = intersections[i];
                if (minDist && intersect && intersect.distanceToRay && minDist > intersect.distanceToRay) {
                  minDist = intersections[i].distanceToRay;
                  closestIndex = i;
                }
              }

              let rawPoint = new THREE.Vector3();
              let closestIntersection = intersections[closestIndex];
              if (points && closestIndex && closestIntersection && closestIntersection.index) {
                rawPoint.setX(points[closestIntersection.index * 4]);
                rawPoint.setY(points[closestIntersection.index * 4 + 1]);
                rawPoint.setZ(points[closestIntersection.index * 4 + 2]);
                addPoint(rawPoint);
              }
            }
          },
        }}
      />
      {pointsPicked.map((pt, i) => {
        return (
          <group position={[pt.x, pt.y, pt.z]} key={i} onClick={() => removePoint(i)}>
            <mesh>
              <sphereGeometry args={[0.08, 32, 32]} />
              <meshBasicMaterial color={0xffffff} />
            </mesh>
            <sprite position-z={0.1} scale={[0.5, 0.05, 1]} center={[0, 0] as any as THREE.Vector2}>
              <spriteMaterial sizeAttenuation={false} depthTest={false}>
                <canvasTexture attach={"map"} args={[canvasIndexes[i]]} />
              </spriteMaterial>
            </sprite>
          </group>
        );
      })}
    </group>
  );
}
