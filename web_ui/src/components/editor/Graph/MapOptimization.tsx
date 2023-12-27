import { ListItemIcon, ListItemText, MenuItem } from "@mui/material";
import DeviceHubIcon from "@mui/icons-material/DeviceHub";
import React from "react";
import { useTranslation } from "react-i18next";

export interface Props {
  onEvent: any;
  onFinish: any;
}

export default function MapOptimization({ onEvent, onFinish }: Props) {
  const { t } = useTranslation();

  const onClickMenu = () => {
    onEvent("OptimizeMap", false);
    onFinish();
  };

  return (
    <MenuItem onClick={onClickMenu}>
      <ListItemIcon>
        <DeviceHubIcon fontSize="small" />
      </ListItemIcon>
      <ListItemText>{t("MapOptimization")}</ListItemText>
    </MenuItem>
  );
}
