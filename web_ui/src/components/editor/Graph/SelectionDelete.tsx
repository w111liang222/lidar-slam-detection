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
import DeleteForeverIcon from "@mui/icons-material/DeleteForever";
import React, { useState } from "react";
import { useTranslation } from "react-i18next";
import { deleteMapPoints, delteMapVertex, delMapArea } from "@rpc/http";
import { MapFrameIndex } from "..";

export interface Props {
  onEvent: any;
  onFinish: any;
  selectVertex: string[];
  selectPointIndex: MapFrameIndex;
  selectArea?: string;
}

export default function SelectionDelete({ onEvent, onFinish, selectVertex, selectPointIndex, selectArea }: Props) {
  const { t } = useTranslation();
  const [open, setOpen] = useState(false);

  const onClickMenu = async () => {
    if (selectArea) {
      onFinish();
      await delMapArea(selectArea);
      onEvent("RefreshMap", false);
      onEvent("ShowMessage", { message: t("finishDeletingMap"), severity: "success", duration: 2000 });
      return;
    }
    if (selectVertex.length <= 0 && Object.keys(selectPointIndex).length <= 0) {
      onFinish();
      return;
    }
    if (Object.keys(selectPointIndex).length > 0) {
      onEvent("ShowMessage", { message: t("DeletingMap"), severity: "info", duration: 60000 });
      onEvent("ShowLoading", true);
      deleteMapPoints(selectPointIndex)
        .then(() => {
          onEvent("DeleteFramePoints");
        })
        .finally(() => {
          onEvent("ShowMessage", { message: t("finishDeletingMap"), severity: "success", duration: 2000 });
          onEvent("ShowLoading", false);
          onFinish();
        });
      return;
    }
    setOpen(true);
  };

  const handleClose = () => {
    setOpen(false);
    onFinish();
  };

  const onHanleDeletion = async (optimize: boolean) => {
    setOpen(false);
    onFinish();
    onEvent("ShowMessage", { message: t("DeletingMap"), severity: "info", duration: 10000 });
    const uniqueSelect = selectVertex.filter((v, i, a) => a.indexOf(v) === i);
    for (let i = 0; i < uniqueSelect.length; i++) {
      await delteMapVertex(uniqueSelect[i]);
    }
    onEvent("ClearSelection");
    if (optimize) {
      onEvent("OptimizeMap", false);
    } else {
      onEvent("RefreshMap", false);
    }
    onEvent("ShowMessage", { message: t("finishDeletingMap"), severity: "success", duration: 2000 });
  };

  return (
    <>
      <MenuItem onClick={onClickMenu}>
        <ListItemIcon>
          <DeleteForeverIcon fontSize="small" />
        </ListItemIcon>
        <ListItemText>{t("SelectionDelete")}</ListItemText>
      </MenuItem>
      <Dialog open={open} onClose={handleClose}>
        <DialogTitle>{t("confirmDeleteSelect")}</DialogTitle>
        <DialogContent style={{ minWidth: "350px", minHeight: "60px" }}>
          <DialogContentText>{t("needOptimize")}</DialogContentText>
        </DialogContent>
        <DialogActions>
          <Button onClick={() => onHanleDeletion(false)} color="primary">
            {t("False")}
          </Button>
          <Button onClick={() => onHanleDeletion(true)} color="primary">
            {t("True")}
          </Button>
        </DialogActions>
      </Dialog>
    </>
  );
}
