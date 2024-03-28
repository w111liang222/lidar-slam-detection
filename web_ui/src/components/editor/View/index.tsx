import { Button, MenuList, Popover } from "@mui/material";
import React, { useImperativeHandle, useState } from "react";
import { useTranslation } from "react-i18next";
import GraphInfo, { GraphInfoMenu } from "./GraphInfo";
import { Config, MapFrame, MapFrameIndex } from "..";
import MapRefresh from "./MapRefresh";
import ClearSelection from "./ClearSelection";
import ReverseSelection from "./ReverseSelection";
import useSelector from "./VertexSelection";

export interface Props {
  onEvent: any;
  config: Config;
  vertex?: LSD.MapVertex;
  edge?: LSD.MapEdge;
  meta?: LSD.MapMeta;
  selectVertex: string[];
  selectColor: number[];
  selectPoints?: Float32Array;
  selectPointIndex: MapFrameIndex;
  selectArea?: string;
}

function MenuView(
  { onEvent, config, vertex, edge, meta, selectVertex, selectColor, selectPoints, selectPointIndex, selectArea }: Props,
  ref: any
) {
  const { t } = useTranslation();
  const [anchorEl, setAnchorEl] = useState(null);
  const [showGraphInfo, setShowGraphInfo] = useState(false);

  const handleButtonClick = (event: any) => {
    setAnchorEl(event.currentTarget);
  };
  const handleClose = () => {
    setAnchorEl(null);
  };

  const [selector, onHandleROI, selectorDialog] = useSelector({
    config: config,
    onEvent: onEvent,
    onFinish: handleClose,
    vertex: vertex,
  });

  useImperativeHandle(ref, () => ({
    onHanleRoi: (mapFrame: MapFrame, roi: THREE.Vector3[] | undefined, camera: THREE.Camera) => {
      onHandleROI(mapFrame, roi, camera);
    },
    onClear: () => {
      onHandleROI({}, undefined, undefined);
    },
  }));

  return (
    <>
      <Button onClick={handleButtonClick} color="inherit">
        {t("View")}
      </Button>
      <Popover
        anchorEl={anchorEl}
        anchorOrigin={{
          vertical: "bottom",
          horizontal: "center",
        }}
        transformOrigin={{
          vertical: "top",
          horizontal: "center",
        }}
        open={Boolean(anchorEl)}
        onClose={handleClose}>
        <MenuList>
          <MapRefresh onEvent={onEvent} onFinish={handleClose} />
          {selector}
          <ReverseSelection
            onEvent={onEvent}
            onFinish={handleClose}
            vertex={vertex}
            selectVertex={selectVertex}
            selectPointIndex={selectPointIndex}
          />
          <ClearSelection onEvent={onEvent} onFinish={handleClose} />
          <GraphInfoMenu
            onClick={() => {
              setShowGraphInfo(true);
              handleClose();
            }}
          />
        </MenuList>
      </Popover>
      {showGraphInfo && (
        <GraphInfo
          meta={meta}
          onEvent={onEvent}
          onClose={() => {
            setShowGraphInfo(false);
          }}
        />
      )}
      {selectorDialog}
    </>
  );
}

export default React.memo(React.forwardRef(MenuView));
