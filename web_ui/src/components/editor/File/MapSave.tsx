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
  TextField,
} from "@mui/material";
import SaveIcon from "@mui/icons-material/Save";
import React, { useState } from "react";
import { useTranslation } from "react-i18next";
import { saveMapping } from "@rpc/http";
import MapSavingProgress from "@components/preview/MapSavingProgress";

export interface Props {
  onFinish: any;
  onEvent: any;
}

export default function MapSave({ onFinish, onEvent }: Props) {
  const { t } = useTranslation();
  const [open, setOpen] = useState(false);
  const [mapName, setMapName] = useState(formatDate(new Date()));
  const [showMapSaving, setShowMapSaving] = useState(false);

  const handleClose = (confirm: boolean) => {
    if (confirm) {
      saveMapping(mapName).then((result) => {
        if (result == "ok") {
          setShowMapSaving(true);
        }
      });
    } else {
      onFinish();
    }
    setOpen(false);
  };

  const onClickMenu = () => {
    setOpen(true);
  };

  return (
    <>
      <Dialog open={open} onClose={() => handleClose(false)}>
        <DialogTitle>{t("MapSave")}</DialogTitle>
        <DialogContent>
          <DialogContentText style={{ minWidth: "400px" }}>{t("MapInputSaveName")}</DialogContentText>
          <TextField
            autoFocus
            margin="normal"
            id="name"
            defaultValue={mapName}
            onChange={(ev) => {
              setMapName(ev.target.value);
            }}
            fullWidth
            variant="standard"
          />
        </DialogContent>
        <DialogActions>
          <Button onClick={() => handleClose(false)}>{t("no")}</Button>
          <Button onClick={() => handleClose(true)}>{t("yes")}</Button>
        </DialogActions>
      </Dialog>
      {showMapSaving && (
        <MapSavingProgress
          setShowMapSaving={setShowMapSaving}
          setPause={() => {
            onFinish();
            setOpen(false);
            onEvent("RefreshMap", false);
          }}
        />
      )}
      <MenuItem onClick={onClickMenu}>
        <ListItemIcon>
          <SaveIcon fontSize="small" />
        </ListItemIcon>
        <ListItemText>{t("MapSave")}</ListItemText>
      </MenuItem>
    </>
  );
}

function formatDate(date: Date) {
  const mm = date.getMonth() + 1; // getMonth() is zero-based
  const dd = date.getDate();
  const h = date.getHours();
  const m = date.getMinutes();
  const s = date.getSeconds();

  return (
    [date.getFullYear(), (mm > 9 ? "" : "0") + mm, (dd > 9 ? "" : "0") + dd].join("-") +
    "-" +
    [(h > 9 ? "" : "0") + h, (m > 9 ? "" : "0") + m, (s > 9 ? "" : "0") + s].join("-")
  );
}
