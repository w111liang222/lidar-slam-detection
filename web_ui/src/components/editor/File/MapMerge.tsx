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
import AddBoxIcon from "@mui/icons-material/AddBox";
import FolderIcon from "@mui/icons-material/Folder";
import createStyles from "@mui/styles/createStyles";
import makeStyles from "@mui/styles/makeStyles";
import React, { useEffect, useState } from "react";
import { useTranslation } from "react-i18next";
import Loading from "@components/general/Loading";
import { getConfig, getMapFiles, mergeMapFile } from "@rpc/http";

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

export default function MapMerge({ onFinish, onEvent }: Props) {
  const { t } = useTranslation();
  const classes = useStyles();

  const [open, setOpen] = useState(false);
  const [isLoading, setIsLoading] = useState<boolean>(false);
  const [selectIdx, setSelectIdx] = useState<string | undefined>(undefined);
  const [config, setConfig] = useState<LSD.Config | undefined>(undefined);
  const [recordFile, setRecordFile] = useState<LSD.RecordFiles>({});

  useEffect(() => {
    getConfig().then((c) => {
      setConfig(c);
    });
  }, []);

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
    setIsLoading(true);
    mergeMapFile(recordFile[idx][0])
      .then(() => {
        onEvent("RefreshMap");
      })
      .finally(() => {
        setIsLoading(false);
        onFinish();
      });
    setOpen(false);
  };

  const handleClose = (confirm: boolean) => {
    if (confirm && selectIdx) {
      setIsLoading(true);
      mergeMapFile(recordFile[selectIdx][0])
        .then(() => {
          onEvent("RefreshMap");
        })
        .finally(() => {
          setIsLoading(false);
          onFinish();
        });
    } else {
      onFinish();
    }
    setOpen(false);
  };

  return (
    <div>
      {config && config.slam && config.slam.mode == "localization" && (
        <>
          {isLoading && <Loading />}
          <MenuItem onClick={handleClickOpen}>
            <ListItemIcon>
              <AddBoxIcon fontSize="small" />
            </ListItemIcon>
            <ListItemText>{t("MapMerge")}</ListItemText>
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
        </>
      )}
    </div>
  );
}
