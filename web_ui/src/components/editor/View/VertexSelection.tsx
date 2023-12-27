import { ListItemIcon, ListItemText, MenuItem } from "@mui/material";
import HighlightAltIcon from "@mui/icons-material/HighlightAlt";
import HubIcon from "@mui/icons-material/Hub";
import GrainIcon from "@mui/icons-material/Grain";
import React, { useState } from "react";
import { useTranslation } from "react-i18next";
import { insidePolygon } from "@utils/transform";
import * as THREE from "three";
import { NestedMenuItem } from "@components/general/mui-nested-menu/components/NestedMenuItem";
import { Config, MapFrame, MapFrameIndex } from "..";
import { arrayToTransform } from "@rpc/http";

export interface Props {
  config: Config;
  onEvent: any;
  onFinish: any;
  vertex?: LSD.MapVertex;
}

export default function useSelector({ config, onEvent, onFinish, vertex }: Props): [React.ReactNode, any] {
  const { t } = useTranslation();
  const [isVertexSelecting, setIsVertexSelecting] = useState(false);
  const [isPointsSelecting, setIsPointsSelecting] = useState(false);

  const onVertexSelect = () => {
    setIsVertexSelecting(true);
    onEvent("ClearSelection");
    onEvent("StartROI");
    onFinish();
  };

  const onPointsSelect = () => {
    setIsPointsSelecting(true);
    onEvent("ClearSelection");
    onEvent("StartROI");
    onFinish();
  };

  const onHanleRoi = (mapFrame: MapFrame, roi: THREE.Vector3[] | undefined, camera: THREE.Camera) => {
    if (isVertexSelecting) {
      const selectVertex: string[] = [];
      const selectedColor: number[] = [];
      setIsVertexSelecting(false);
      if (roi && vertex) {
        const polygon: THREE.Vector3[] = [];
        for (let i = 0; i < roi.length - 1; i++) {
          polygon[i] = roi[i].clone().project(camera);
        }
        const vertextIds = Object.keys(vertex);
        for (let i = 0; i < vertextIds.length; i++) {
          const vertexPose = vertex[vertextIds[i]];
          const vertexPosition = new THREE.Vector3(vertexPose[3], vertexPose[7], vertexPose[11]);
          const vertexProject = vertexPosition.clone().project(camera);
          if (insidePolygon(vertexProject, polygon)) {
            selectVertex.push(vertextIds[i]);
            selectedColor.push(0);
          }
        }
      }
      onEvent("SetSelectVertex", { selection: selectVertex, color: selectedColor });
    }
    if (isPointsSelecting) {
      let maxSelectNum = 0;
      for (const id of Object.keys(mapFrame)) {
        const frame = mapFrame[id];
        if (frame) {
          maxSelectNum += frame.points.length;
        }
      }
      const selectPoint = new Float32Array(maxSelectNum);
      const selectPointIndex: MapFrameIndex = {};
      let selectNum = 0;
      setIsPointsSelecting(false);
      if (roi && vertex) {
        const polygon: THREE.Vector3[] = [];
        for (let i = 0; i < roi.length - 1; i++) {
          polygon[i] = roi[i].clone().project(camera);
        }
        const vertexIds = Object.keys(vertex);
        for (let i = 0; i < vertexIds.length; i++) {
          if (parseInt(vertexIds[i]) % config.map.step != 0) {
            continue;
          }
          const pointIndex = [];
          const frame = mapFrame[vertexIds[i]];
          if (frame == undefined) {
            continue;
          }
          const points = frame.points;
          const T = arrayToTransform(vertex[vertexIds[i]]);
          for (let j = 0; j < points.length; j = j + 4) {
            const pointPosition = new THREE.Vector3(points[j], points[j + 1], points[j + 2]);
            pointPosition.applyMatrix4(T);
            if (pointPosition.z < config.map.zRange.min / 10.0 || pointPosition.z > config.map.zRange.max / 10.0) {
              continue;
            }
            const pointProject = pointPosition.clone().project(camera);
            if (insidePolygon(pointProject, polygon)) {
              pointIndex.push(j / 4);
              selectPoint[selectNum] = pointPosition.x;
              selectPoint[selectNum + 1] = pointPosition.y;
              selectPoint[selectNum + 2] = pointPosition.z;
              selectPoint[selectNum + 3] = points[j + 3];
              selectNum = selectNum + 4;
            }
          }
          if (pointIndex.length > 0) {
            selectPointIndex[vertexIds[i]] = pointIndex;
          }
        }
        onEvent("SetSelectPoint", { points: selectPoint.slice(0, selectNum), index: selectPointIndex });
      }
    }
  };

  return [
    <NestedMenuItem parentMenuOpen={true} leftIcon={<HighlightAltIcon fontSize="small" />} label={t("MapSelection")}>
      <MenuItem onClick={onVertexSelect} disabled={isVertexSelecting || isPointsSelecting}>
        <ListItemIcon>
          <HubIcon fontSize="small" />
        </ListItemIcon>
        <ListItemText>{t("VertexSelection")}</ListItemText>
      </MenuItem>
      <MenuItem onClick={onPointsSelect} disabled={isVertexSelecting || isPointsSelecting}>
        <ListItemIcon>
          <GrainIcon fontSize="small" />
        </ListItemIcon>
        <ListItemText>{t("PointsSelection")}</ListItemText>
      </MenuItem>
    </NestedMenuItem>,
    onHanleRoi,
  ];
}
