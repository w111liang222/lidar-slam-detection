import {
  Button,
  Dialog,
  DialogActions,
  DialogContent,
  DialogContentText,
  DialogTitle,
  ListItemIcon,
  ListItemText,
  MenuItem,
} from "@mui/material";
import CloseIcon from "@mui/icons-material/Close";
import React, { useState } from "react";
import { useTranslation } from "react-i18next";
import { openMapFile } from "@rpc/http";

export interface Props {
  onFinish: any;
  onEvent: any;
}

export default function MapClose({ onFinish, onEvent }: Props) {
  const { t } = useTranslation();
  const [open, setOpen] = useState(false);

  const onClickMenu = () => {
    setOpen(true);
  };

  const handleClose = (confirm: boolean) => {
    if (confirm) {
      onEvent("ShowMessage", { message: t("ClosingMap"), severity: "info", duration: 10000 });
      openMapFile("").then(() => {
        onEvent("RefreshMap", true);
        onEvent("ShowMessage", { message: t("finishClosingMap"), severity: "success", duration: 2000 });
      });
    }
    setOpen(false);
    onFinish();
  };

  return (
    <>
      <MenuItem onClick={onClickMenu}>
        <ListItemIcon>
          <CloseIcon fontSize="small" />
        </ListItemIcon>
        <ListItemText>{t("MapClose")}</ListItemText>
      </MenuItem>
      <Dialog open={open} onClose={handleClose}>
        <DialogTitle>{t("confirmCloseMap")}</DialogTitle>
        <DialogContent>
          <DialogContentText>{t("warnCloseMap")}</DialogContentText>
        </DialogContent>
        <DialogActions>
          <Button onClick={() => handleClose(false)} color="primary">
            {t("no")}
          </Button>
          <Button onClick={() => handleClose(true)} color="primary">
            {t("yes")}
          </Button>
        </DialogActions>
      </Dialog>
    </>
  );
}
