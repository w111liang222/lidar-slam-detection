import { ListItemIcon, ListItemText, MenuItem } from "@mui/material";
import RefreshIcon from "@mui/icons-material/Refresh";
import React from "react";
import { useTranslation } from "react-i18next";

export interface Props {
  onEvent: any;
  onFinish: any;
}

export default function MapRefresh({ onEvent, onFinish }: Props) {
  const { t } = useTranslation();

  const onClickRefresh = () => {
    onEvent("RefreshMap", true);
    onFinish();
  };

  return (
    <MenuItem onClick={onClickRefresh}>
      <ListItemIcon>
        <RefreshIcon fontSize="small" />
      </ListItemIcon>
      <ListItemText>{t("RefreashMap")}</ListItemText>
    </MenuItem>
  );
}
