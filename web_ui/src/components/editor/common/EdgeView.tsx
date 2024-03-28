import { ThreeEvent, useThree } from "@react-three/fiber";
import React, { useEffect, useRef, useState } from "react";
import * as THREE from "three";
import PoseView from "./PoseView";

export interface Props {
  vertexes: LSD.MapVertex;
  edges: LSD.MapEdge;
  enable: boolean;
  selectEdge?: string;
  onVertexClick: any;
  onCheckClickElement: any;
}

export const EdgeView = React.memo(
  ({ vertexes, edges, enable, selectEdge, onVertexClick, onCheckClickElement }: Props) => {
    const [poses, setPoses] = useState<THREE.Vector3[]>([]);
    const { camera } = useThree();

    useEffect(() => {
      const vertexesKeys = Object.keys(vertexes);
      let pose: THREE.Vector3[] = [];
      for (let i = 0; i < vertexesKeys.length; i++) {
        pose[i] = new THREE.Vector3(
          vertexes[vertexesKeys[i]][3],
          vertexes[vertexesKeys[i]][7],
          vertexes[vertexesKeys[i]][11]
        );
      }
      setPoses(pose);
    }, [vertexes]);

    const handleClick = (e: ThreeEvent<MouseEvent>) => {
      let vertexId = -1;
      let vertexDist = 10000;
      for (let i = 0; i < e.intersections.length; i++) {
        let distToRay = e.intersections[i].distanceToRay;
        let index = e.intersections[i].index;
        if (distToRay && index != undefined && distToRay < vertexDist) {
          vertexDist = distToRay;
          vertexId = index;
        }
      }
      onVertexClick(vertexId, e.button, e);
    };

    return (
      <>
        <group>
          {Object.keys(edges).length > 0 &&
            Object.keys(edges).map((id, idx) => {
              const prev = edges[id][0].toString();
              const next = edges[id][1].toString();
              if (vertexes[prev] == undefined || vertexes[next] == undefined) {
                return;
              }
              const prevPoint = new THREE.Vector3(vertexes[prev][3], vertexes[prev][7], vertexes[prev][11]);
              const nextPoint = new THREE.Vector3(vertexes[next][3], vertexes[next][7], vertexes[next][11]);
              return (
                <LineEdgeView
                  key={idx}
                  id={id}
                  edge={[prevPoint, nextPoint]}
                  color={selectEdge == id ? 0xff0000 : 0x00ff66}
                />
              );
            })}
        </group>
        <group
          {...{
            onClick: (e: ThreeEvent<MouseEvent>) => {
              if (enable) {
                e.stopPropagation();
                handleClick(e);
              }
            },
            onContextMenu: (e: ThreeEvent<MouseEvent>) => {
              if (enable) {
                e.stopPropagation();
                onCheckClickElement(e, camera);
              }
            },
            onPointerMissed: (e: ThreeEvent<MouseEvent>) => {
              if (enable) {
                e.stopPropagation();
                onCheckClickElement(e, camera);
              }
            },
          }}>
          <PoseView poses={poses} color={0x00ff00} />
        </group>
      </>
    );
  }
);

type IProps = {
  id: string;
  edge?: THREE.Vector3[];
  color: number;
};

function LineEdgeView({ id, edge = [], color }: IProps) {
  const lineRef = useRef<any>();

  useEffect(() => {
    const line = lineRef.current;
    if (line) {
      line.frustumCulled = false;
    }
  }, []);

  useEffect(() => {
    const line = lineRef.current;
    if (line && edge) {
      line.geometry.setFromPoints(edge);
    }
  }, [edge]);

  return (
    <line ref={lineRef} name={id}>
      <lineBasicMaterial color={color} linewidth={2} />
    </line>
  );
}
