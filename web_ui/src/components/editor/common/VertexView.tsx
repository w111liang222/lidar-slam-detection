import { arrayToTransform, getTransform } from "@rpc/http";
import React, { useEffect, useState } from "react";
import { Config, MapFrame } from "../index";
import * as THREE from "three";
import PointVertexView from "./PointVertexView";
import PoseView from "./PoseView";
import ImageVertexView from "./ImageVertexView";

export interface Props {
  config: Config;
  boardConfig?: LSD.Config;
  vertexes?: LSD.MapVertex;
  mapFrame: MapFrame;
  selectedVertex: string[];
  selectedVertexColor: number[];
}

export const VertexView = React.memo(
  ({ config, boardConfig, vertexes, mapFrame, selectedVertex, selectedVertexColor }: Props) => {
    const [insExtrinic, setInsExtrinic] = useState<THREE.Matrix4>();

    useEffect(() => {
      if (boardConfig) {
        getTransform(boardConfig.ins.extrinsic_parameters).then((T) => {
          setInsExtrinic(T);
        });
      }
    }, [boardConfig]);

    return (
      <>
        {/* PointCloud */}
        {vertexes &&
          Object.keys(vertexes).map((id, idx) => {
            let visible = parseInt(id) % config.map.step == 0 ? true : false;
            const selectIndex = selectedVertex.findIndex((sid) => sid === id);
            const selectColor = selectedVertexColor[selectIndex] == 0 ? 0xff0000 : selectedVertexColor[selectIndex];
            return (
              <PointVertexView
                key={"PointView" + id}
                id={id}
                points={mapFrame[id]?.points}
                images={mapFrame[id]?.images}
                color={selectIndex != -1 ? selectColor : undefined}
                size={selectedVertexColor[selectIndex] == 0 ? 1.0 : 3.0}
                config={config.map}
                boardConfig={boardConfig}
                insExtrinic={insExtrinic}
                isSelect={selectIndex != -1 ? true : false}
                visible={visible}
                vertexPose={arrayToTransform(vertexes[id])}
              />
            );
          })}
        {/* Select Pose Node */}
        {vertexes &&
          selectedVertex.map((id, idx) => {
            if (!vertexes.hasOwnProperty(id)) {
              return;
            }
            return (
              <PoseView
                key={idx}
                poses={[new THREE.Vector3(vertexes[id][3], vertexes[id][7], vertexes[id][11])]}
                color={selectedVertexColor[idx] != 0 ? selectedVertexColor[idx] : 0xff0000}
              />
            );
          })}
      </>
    );
  }
);

interface IProps {
  config: Config;
  vertexes?: LSD.MapVertex;
  selectedVertex: string[];
  selectedVertexColor: number[];
}
export const VertexImageView = React.memo(({ config, vertexes, selectedVertex, selectedVertexColor }: IProps) => {
  return (
    <>
      {/* Image */}
      {vertexes &&
        Object.keys(vertexes).map((id, idx) => {
          let selectIndex = selectedVertex.findIndex((sid) => sid === id);
          const isSelect = selectIndex == 2 && selectedVertexColor[selectIndex] != 0;
          return <ImageVertexView key={idx} id={id} config={config.map} isSelect={isSelect} />;
        })}
    </>
  );
});
