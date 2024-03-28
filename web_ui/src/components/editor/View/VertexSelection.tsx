import {
  Button,
  Dialog,
  DialogActions,
  DialogTitle,
  FormControl,
  Grid,
  InputLabel,
  ListItemIcon,
  ListItemText,
  MenuItem,
  Select,
  TextField,
} from "@mui/material";
import HighlightAltIcon from "@mui/icons-material/HighlightAlt";
import HubIcon from "@mui/icons-material/Hub";
import GrainIcon from "@mui/icons-material/Grain";
import SignalCellular0BarIcon from "@mui/icons-material/SignalCellular0Bar";
import React, { useState } from "react";
import { useTranslation } from "react-i18next";
import { insidePolygon } from "@utils/transform";
import * as THREE from "three";
import { NestedMenuItem } from "@components/general/mui-nested-menu/components/NestedMenuItem";
import { Config, MapFrame, MapFrameIndex } from "..";
import { arrayToTransform, addMapArea } from "@rpc/http";

type AreaType = {
  type: string | undefined;
  name: string;
  polygon: number[][];
};

export interface Props {
  config: Config;
  onEvent: any;
  onFinish: any;
  vertex?: LSD.MapVertex;
}

export default function useSelector({
  config,
  onEvent,
  onFinish,
  vertex,
}: Props): [React.ReactNode, any, React.ReactNode] {
  const { t } = useTranslation();
  const [open, setOpen] = useState(false);

  const [isVertexSelecting, setIsVertexSelecting] = useState(false);
  const [isPointsSelecting, setIsPointsSelecting] = useState(false);
  const [isAreaSelecting, setIsAreaSelecting] = useState(false);
  const [area, setArea] = useState<AreaType>();

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

  const onAreaSelect = () => {
    setIsAreaSelecting(true);
    onEvent("ClearSelection");
    onEvent("StartROI", "drawing_bev");
    onFinish();
  };

  const handleClose = () => {
    setOpen(false);
  };

  const onHanleDialog = async (confirm: boolean) => {
    if (confirm && area && area.type) {
      addMapArea(area).then(() => {
        onEvent("RefreshMap");
      });
    }
    setOpen(false);
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
    if (isAreaSelecting) {
      setIsAreaSelecting(false);
      if (roi) {
        let polygon = [];
        for (let i = 0; i < roi.length - 1; i++) {
          polygon.push([roi[i].x, roi[i].y, roi[i].z]);
        }
        setArea({
          type: undefined,
          name: "",
          polygon: polygon,
        });
        setOpen(true);
      }
    }
  };

  return [
    <>
      <NestedMenuItem parentMenuOpen={true} leftIcon={<HighlightAltIcon fontSize="small" />} label={t("MapSelection")}>
        <MenuItem onClick={onVertexSelect} disabled={isVertexSelecting || isPointsSelecting || isAreaSelecting}>
          <ListItemIcon>
            <HubIcon fontSize="small" />
          </ListItemIcon>
          <ListItemText>{t("VertexSelection")}</ListItemText>
        </MenuItem>
        <MenuItem onClick={onPointsSelect} disabled={isVertexSelecting || isPointsSelecting || isAreaSelecting}>
          <ListItemIcon>
            <GrainIcon fontSize="small" />
          </ListItemIcon>
          <ListItemText>{t("PointsSelection")}</ListItemText>
        </MenuItem>
        <MenuItem onClick={onAreaSelect} disabled={isVertexSelecting || isPointsSelecting || isAreaSelecting}>
          <ListItemIcon>
            <SignalCellular0BarIcon fontSize="small" />
          </ListItemIcon>
          <ListItemText>{t("AreaSelection")}</ListItemText>
        </MenuItem>
      </NestedMenuItem>
    </>,
    onHanleRoi,
    <>
      {area && (
        <Dialog open={open} onClose={handleClose}>
          <DialogTitle>{t("ConfirmAddArea")}</DialogTitle>
          <div style={{ minWidth: "450px" }}>
            <Grid style={{ paddingLeft: "1rem", paddingRight: "1rem" }} container>
              <Grid item md>
                <FormControl>
                  <InputLabel>{t("AreaType")}</InputLabel>
                  <Select
                    label={t("AreaType")}
                    value={area.type}
                    onChange={(event) => {
                      area.type = event.target.value;
                      setArea({ ...area });
                    }}>
                    <MenuItem value={"indoor"}>{t("Indoor")}</MenuItem>
                    <MenuItem value={"outdoor"}>{t("Outdoor")}</MenuItem>
                  </Select>
                </FormControl>
              </Grid>
            </Grid>
            <Grid style={{ paddingLeft: "1rem", paddingRight: "1rem" }} container>
              <Grid item sm>
                <TextField
                  label={t("AreaName")}
                  value={area.name}
                  onChange={(event) => {
                    area.name = event.target.value;
                    setArea({ ...area });
                  }}
                />
              </Grid>
            </Grid>
          </div>
          <DialogActions>
            <Button onClick={() => onHanleDialog(false)} color="primary">
              {t("False")}
            </Button>
            <Button onClick={() => onHanleDialog(true)} color="primary">
              {t("True")}
            </Button>
          </DialogActions>
        </Dialog>
      )}
    </>,
  ];
}
