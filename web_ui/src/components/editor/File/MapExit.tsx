import { ListItemIcon, ListItemText, MenuItem } from "@mui/material";
import ExitToAppIcon from "@mui/icons-material/ExitToApp";
import React from "react";
import { useTranslation } from "react-i18next";
import { useNavigate } from "react-router-dom";

export interface Props {
  onFinish: any;
}

export default function MapExit({ onFinish }: Props) {
  const { t } = useTranslation();
  const navigate = useNavigate();

  const onClickMenu = () => {
    navigate("/preview", { replace: true });
  };

  return (
    <MenuItem onClick={onClickMenu}>
      <ListItemIcon>
        <ExitToAppIcon fontSize="small" />
      </ListItemIcon>
      <ListItemText>{t("MapExit")}</ListItemText>
    </MenuItem>
  );
}
