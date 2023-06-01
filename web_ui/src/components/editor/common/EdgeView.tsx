import { ThreeEvent, useThree } from "@react-three/fiber";
import { pointToLineDistance } from "@utils/transform";
import React, { useEffect, useRef, useState } from "react";
import * as THREE from "three";
import PoseView from "./PoseView";

export interface Props {
  vertexes: LSD.MapVertex;
  edges: LSD.MapEdge;
  enable: boolean;
  selectEdge?: string;
  onVertexClick: any;
  onEdgeClick: any;
}

export const EdgeView = React.memo(({ vertexes, edges, enable, selectEdge, onVertexClick, onEdgeClick }: Props) => {
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

  const getPoseClient = (point: THREE.Vector3, ev: any) => {
    const poseClient = point.clone().project(camera);
    poseClient.x = ((poseClient.x + 1) / 2) * ev.target.offsetWidth;
    poseClient.y = (-(poseClient.y - 1) / 2) * ev.target.offsetHeight;
    poseClient.z = 0;
    return poseClient;
  };

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
          onPointerMissed: (e: ThreeEvent<MouseEvent>) => {
            console.log(e, "enable: " + enable);
            if (!enable) {
              return;
            }
            e.stopPropagation();

            let vertexDist = 100;
            if (e.type == "click") {
              let vertexId = -1;
              const poseClient: THREE.Vector3[] = [];
              for (let i = 0; i < poses.length; i++) {
                poseClient[i] = getPoseClient(poses[i], e);
              }
              for (let i = 0; i < poseClient.length; i++) {
                const a = poseClient[i].x - e.clientX;
                const b = poseClient[i].y - e.clientY;
                const dist = Math.sqrt(a * a + b * b);
                if (dist < vertexDist) {
                  vertexDist = dist;
                  vertexId = i;
                }
              }
              vertexId != -1 && onVertexClick(vertexId, e.button, e);
            } else if (e.type == "contextmenu") {
              let vertexId = "";
              const edgeIds = Object.keys(edges);
              for (let i = 0; i < edgeIds.length; i++) {
                const prev = edges[edgeIds[i]][0].toString();
                const next = edges[edgeIds[i]][1].toString();
                const A = new THREE.Vector3(e.clientX, e.clientY, 0);
                if (vertexes[prev] == undefined || vertexes[next] == undefined) {
                  continue;
                }
                const B = getPoseClient(new THREE.Vector3(vertexes[prev][3], vertexes[prev][7], vertexes[prev][11]), e);
                const C = getPoseClient(new THREE.Vector3(vertexes[next][3], vertexes[next][7], vertexes[next][11]), e);
                const dist = pointToLineDistance(A.x, A.y, B.x, B.y, C.x, C.y);
                if (dist < vertexDist) {
                  vertexDist = dist;
                  vertexId = edgeIds[i];
                }
              }
              vertexId != "" && onEdgeClick(vertexId, e);
            }
          },
        }}>
        <PoseView poses={poses} color={0x00ff00} />
      </group>
    </>
  );
});

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
