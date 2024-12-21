import { Button, Divider, MenuList, Popover } from "@mui/material";
import { usePersistFn } from "ahooks";
import React, { useImperativeHandle, useState } from "react";
import { useTranslation } from "react-i18next";
import { Config, MapFrameIndex } from "..";
import MapClose from "./MapClose";
import MapExit from "./MapExit";
import MapExport from "./MapExport";
import MapMerge from "./MapMerge";
import MapOpen from "./MapOpen";
import MapSave from "./MapSave";

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

function MenuFile(
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

  const onFinish = usePersistFn(() => {
    handleClose();
  });

  useImperativeHandle(ref, () => ({
    onClear: () => {},
  }));

  return (
    <>
      <Button onClick={handleButtonClick} color="inherit">
        {t("File")}
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
          <MapOpen onFinish={onFinish} onEvent={onEvent} />
          <MapMerge onFinish={onFinish} onEvent={onEvent} />
          <MapSave onFinish={onFinish} onEvent={onEvent}/>
          <MapExport config={config} onFinish={onFinish} />
          <MapClose onFinish={onFinish} onEvent={onEvent} />
          <Divider />
          <MapExit onFinish={onFinish} />
        </MenuList>
      </Popover>
    </>
  );
}

export default React.memo(React.forwardRef(MenuFile));
