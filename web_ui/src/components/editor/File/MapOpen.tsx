import {
  Button,
  Dialog,
  DialogActions,
  DialogContent,
  DialogTitle,
  ListItem,
  ListItemIcon,
  ListItemText,
  MenuItem,
  Theme,
} from "@mui/material";
import FolderOpenIcon from "@mui/icons-material/FolderOpen";
import FolderIcon from "@mui/icons-material/Folder";
import createStyles from "@mui/styles/createStyles";
import makeStyles from "@mui/styles/makeStyles";
import React, { useState } from "react";
import { useTranslation } from "react-i18next";
import Loading from "@components/general/Loading";
import { getMapFiles, openMapFile } from "@rpc/http";
import useTipsShow from "../common/TipsShow";

const useStyles = makeStyles((theme: Theme) =>
  createStyles({
    container: {
      display: "flex",
      flexWrap: "wrap",
      minWidth: 300,
      maxWidth: 500,
      maxHeight: 500,
    },
  })
);

export interface Props {
  onFinish: any;
  onEvent: any;
}

export default function MapOpen({ onFinish, onEvent }: Props) {
  const { t } = useTranslation();
  const classes = useStyles();

  const [open, setOpen] = useState(false);
  const [isLoading, setIsLoading] = useState<boolean>(false);
  const [tips, showMessage, closeMessage] = useTipsShow();
  const [selectIdx, setSelectIdx] = useState<string | undefined>(undefined);
  const [recordFile, setRecordFile] = useState<LSD.RecordFiles>({});

  const handleClickOpen = () => {
    getMapFiles().then((files) => {
      if (typeof files == "object") {
        setRecordFile(files);
      }
    });
    setOpen(true);
  };

  const handleDoubleClick = (idx: string) => {
    setSelectIdx(idx);
    onEvent("ClearMap");
    setIsLoading(true);
    showMessage(t("OpeningMaps"), "info", 10000);
    openMapFile(recordFile[idx][0])
      .then(() => {
        onEvent("RefreshMap");
      })
      .finally(() => {
        setIsLoading(false);
        closeMessage();
        setTimeout(() => {
          onFinish();
        }, 100);
      });
    setOpen(false);
  };

  const handleClose = (confirm: boolean) => {
    if (confirm && selectIdx) {
      setIsLoading(true);
      showMessage(t("OpeningMaps"), "info", 10000);
      onEvent("ClearMap");
      openMapFile(recordFile[selectIdx][0])
        .then(() => {
          onEvent("RefreshMap");
        })
        .finally(() => {
          setIsLoading(false);
          closeMessage();
          setTimeout(() => {
            onFinish();
          }, 100);
        });
    } else {
      onFinish();
    }
    setOpen(false);
  };

  return (
    <div>
      {isLoading && <Loading />}
      {tips}
      <MenuItem onClick={handleClickOpen}>
        <ListItemIcon>
          <FolderOpenIcon fontSize="small" />
        </ListItemIcon>
        <ListItemText>{t("MapOpen")}</ListItemText>
      </MenuItem>
      <Dialog open={open} onClose={() => handleClose(false)}>
        <DialogTitle>{t("selectFile")}</DialogTitle>
        <DialogContent id="map_file_div">
          <div className={classes.container}>
            {Object.keys(recordFile).map((key) => {
              let filePath = recordFile[key][0].split("/");
              return (
                <ListItem
                  key={key}
                  button
                  onClick={() => setSelectIdx(key)}
                  onDoubleClick={() => handleDoubleClick(key)}
                  selected={selectIdx == key ? true : false}
                  id={selectIdx == key ? "select_file_item" : undefined}>
                  <ListItemIcon>
                    <FolderIcon />
                  </ListItemIcon>
                  <ListItemText primary={filePath[filePath.length - 1]} />
                </ListItem>
              );
            })}
          </div>
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
    </div>
  );
}
