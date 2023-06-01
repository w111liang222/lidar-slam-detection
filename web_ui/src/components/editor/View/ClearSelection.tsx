import { ListItemIcon, ListItemText, MenuItem } from "@mui/material";
import ClearAllIcon from "@mui/icons-material/ClearAll";
import React from "react";
import { useTranslation } from "react-i18next";

export interface Props {
  onEvent: any;
  onFinish: any;
}

export default function ClearSelection({ onEvent, onFinish }: Props) {
  const { t } = useTranslation();

  const onClickClearSelection = () => {
    onEvent("ClearSelection");
    onFinish();
  };

  return (
    <MenuItem onClick={onClickClearSelection}>
      <ListItemIcon>
        <ClearAllIcon fontSize="small" />
      </ListItemIcon>
      <ListItemText>{t("MapClearSelection")}</ListItemText>
    </MenuItem>
  );
}
