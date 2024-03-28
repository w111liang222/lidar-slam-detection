import { Button, MenuList, Popover } from "@mui/material";
import React, { useImperativeHandle, useState } from "react";
import { useTranslation } from "react-i18next";
import MapOptimization from "./MapOptimization";
import MapManualLoop from "./MapManualLoop";
import { Config, MapFrameIndex } from "..";
import SelectionDelete from "./SelectionDelete";

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

function MenuGraph(
  { onEvent, config, vertex, edge, meta, selectVertex, selectColor, selectPoints, selectPointIndex, selectArea }: Props,
  ref: any
) {
  const { t } = useTranslation();
  const [anchorEl, setAnchorEl] = useState(null);
  const handleButtonClick = (event: any) => {
    setAnchorEl(event.currentTarget);
  };
  const handleClose = () => {
    setAnchorEl(null);
  };

  useImperativeHandle(ref, () => ({
    onClear: () => {},
  }));

  return (
    <>
      <Button onClick={handleButtonClick} color="inherit">
        {t("Edit")}
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
          <MapManualLoop
            onEvent={onEvent}
            onFinish={handleClose}
            selectVertex={selectVertex}
            selectColor={selectColor}
          />
          <MapOptimization onEvent={onEvent} onFinish={handleClose} />
          <SelectionDelete
            onEvent={onEvent}
            onFinish={handleClose}
            selectVertex={selectVertex}
            selectPointIndex={selectPointIndex}
            selectArea={selectArea}
          />
        </MenuList>
      </Popover>
    </>
  );
}

export default React.memo(React.forwardRef(MenuGraph));
