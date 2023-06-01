import {
  Button,
  Dialog,
  DialogActions,
  DialogContent,
  DialogTitle,
  FormControl,
  Grid,
  InputLabel,
  ListItem,
  ListItemIcon,
  ListItemText,
  MenuItem,
  Paper,
  Popover,
  Slider,
  Theme,
  Typography,
} from "@mui/material";
import createStyles from "@mui/styles/createStyles";
import makeStyles from "@mui/styles/makeStyles";
import withStyles from "@mui/styles/withStyles";
import React, { useRef, useState } from "react";
import { useEffect } from "react";
import FolderOpenIcon from "@mui/icons-material/FolderOpen";
import PlayArrowIcon from "@mui/icons-material/PlayArrow";
import PauseIcon from "@mui/icons-material/Pause";
import FolderIcon from "@mui/icons-material/Folder";
import Loading from "@components/general/Loading";
import { useRequest } from "ahooks";
import { useLeftKey, useRightKey } from "@hooks/keyboard";
import {
  getPlayerStatus,
  startPlayer,
  pausePlayer,
  seekPlayer,
  getRecordFiles,
  playRecordFile,
  setPlaybackRate,
  setPlayerStep,
} from "@rpc/http";
import { useTranslation } from "react-i18next";
import useInfoShow from "./InfoShow";

const PrettoSlider = withStyles({
  root: {
    height: 6,
  },
  thumb: {
    height: 16,
    width: 16,
    backgroundColor: "#fff",
    border: "3px solid currentColor",
    "&:focus, &:hover, &$active": {
      boxShadow: "inherit",
    },
  },
  active: {},
  track: {
    height: 8,
    borderRadius: 4,
  },
  rail: {
    height: 8,
    borderRadius: 4,
  },
})(Slider);

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

export type Props = {
  className: string;
  currentFile: string;
  pause: boolean;
  setPause: any;
  onPlay?: () => void;
};

export default function Player({ className, currentFile, pause, setPause, onPlay }: Props) {
  const { t } = useTranslation();
  const [isLoading, setIsLoading] = useState<boolean>(false);
  const [info, showMessage] = useInfoShow();
  const { data, loading, error } = useRequest(getPlayerStatus, {
    pollingInterval: 200,
  });

  const classes = useStyles();
  const [isFocus, setIsFocus] = useState<boolean>(false);
  const [doScroll, setDoScroll] = useState<boolean>(false);
  const [isControl, setIsControl] = useState<boolean>(false);
  const [percent, setPercent] = useState<number>(0);
  const [open, setOpen] = useState(false);
  const [recordFile, setRecordFile] = useState<LSD.RecordFiles>({});
  const [selectIdx, setSelectIdx] = useState<string | undefined>(undefined);
  const [playRate, setPlayRate] = useState(1.0);
  const leftKeyPressed = useLeftKey();
  const rightKeyPressed = useRightKey();

  let infoShow = React.useMemo(() => info, [info, showMessage]);

  useEffect(() => {
    if (leftKeyPressed) {
      setPlayerStep(-1).then((data) => {
        const pathes = data.split("/");
        showMessage(t("playbackPosition") + ": " + pathes[pathes.length - 1]);
      });
    }
    if (rightKeyPressed) {
      setPlayerStep(1).then((data) => {
        const pathes = data.split("/");
        showMessage(t("playbackPosition") + ": " + pathes[pathes.length - 1]);
      });
    }
  }, [leftKeyPressed, rightKeyPressed]);

  const [anchorEl, setAnchorEl] = useState(null);
  const handleButtonClick = (event: any) => {
    setAnchorEl(event.currentTarget);
  };
  const handItemClick = (rate: number) => {
    setPlayRate(rate);
    setPlaybackRate(rate);
    setAnchorEl(null);
  };
  const handleAnchorClose = () => {
    setAnchorEl(null);
  };

  useEffect(() => {
    if (data && !isControl) {
      setPercent(data.percent);
    }
    if (!data) {
      setPercent(0);
    }
  }, [data]);

  useEffect(() => {
    if (!pause) {
      startPlayer();
    } else {
      pausePlayer();
    }
  }, [pause]);

  useEffect(() => {
    setPlaybackRate(1.0);
    getRecordFiles().then((files) => {
      setRecordFile(files);
      Object.keys(files).map((key) => {
        if (files[key][0] == currentFile) {
          setSelectIdx(key);
        }
      });
    });
  }, []);

  const handleClickOpen = () => {
    getRecordFiles().then((files) => {
      setRecordFile(files);
    });
    setOpen(true);
    setDoScroll(true);
    // setSelectIdx(undefined);
  };

  if (doScroll) {
    if (open && selectIdx) {
      const selectFile = document.getElementById("select_file_item");
      const fileDiv = document.getElementById("files_div");
      if (selectFile && fileDiv) {
        fileDiv.scrollTop = Math.max(0, selectFile.offsetTop - 300);
        setDoScroll(false);
      }
    }
  }

  const handleDoubleClick = (idx: string) => {
    setSelectIdx(idx);
    setIsLoading(true);
    playRecordFile(recordFile[idx][0])
      .then(() => {
        onPlay && onPlay();
      })
      .finally(() => {
        setIsLoading(false);
      });
    setOpen(false);
  };

  const handleClose = (confirm: boolean) => {
    if (confirm && selectIdx) {
      setIsLoading(true);
      playRecordFile(recordFile[selectIdx][0])
        .then(() => {
          onPlay && onPlay();
        })
        .finally(() => {
          setIsLoading(false);
        });
    }
    setOpen(false);
  };

  return (
    <>
      {isLoading && <Loading />}
      {infoShow}
      <div className={className}>
        <Dialog open={open} onClose={() => handleClose(false)}>
          <DialogTitle>{t("selectFile")}</DialogTitle>
          <DialogContent id="files_div">
            <div className={classes.container}>
              {Object.keys(recordFile).map((key) => {
                let filePath = recordFile[key][0].split("/");
                let items = recordFile[key][1];
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
                    <ListItemText primary={filePath[filePath.length - 1]} secondary={items.toString() + " Files"} />
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
        <Paper
          onMouseEnter={() => {
            setIsFocus(true);
          }}
          onMouseLeave={() => {
            setIsFocus(false);
          }}
          square={false}
          elevation={1}
          style={
            isFocus
              ? {
                  margin: "0px",
                  width: "800px",
                  padding: "4px",
                  backgroundColor: "rgba(255, 255, 255, 0.8)",
                }
              : {
                  margin: "0px",
                  width: "800px",
                  padding: "4px",
                  backgroundColor: "rgba(255, 255, 255, 0.5)",
                }
          }>
          <Grid spacing={0} container style={{ marginTop: "0rem" }}>
            <Button
              style={{ left: "0px" }}
              size="small"
              color="primary"
              onClick={handleClickOpen}
              onKeyDown={(event) => {
                event.preventDefault();
              }}
              onKeyUp={(event) => {
                event.preventDefault();
              }}>
              <FolderOpenIcon style={{ fontSize: 30 }} />
            </Button>
            <Button
              style={{ left: "-10px" }}
              size="small"
              color="primary"
              onClick={() => {
                setPause(!pause);
              }}>
              {pause ? <PlayArrowIcon style={{ fontSize: 30 }} /> : <PauseIcon style={{ fontSize: 30 }} />}
            </Button>
            <div>
              <Button style={{ left: "-10px" }} color="primary" onClick={handleButtonClick}>
                {playRate.toFixed(1) + t("playbackRate")}
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
                onClose={handleAnchorClose}>
                <MenuItem onClick={() => handItemClick(8)}>{"8.0" + t("playbackRate")}</MenuItem>
                <MenuItem onClick={() => handItemClick(4)}>{"4.0" + t("playbackRate")}</MenuItem>
                <MenuItem onClick={() => handItemClick(2)}>{"2.0" + t("playbackRate")}</MenuItem>
                <MenuItem onClick={() => handItemClick(1)}>{"1.0" + t("playbackRate")}</MenuItem>
                <MenuItem onClick={() => handItemClick(0.5)}>{"0.5" + t("playbackRate")}</MenuItem>
                <MenuItem onClick={() => handItemClick(0.2)}>{"0.2" + t("playbackRate")}</MenuItem>
              </Popover>
            </div>
            <Grid xs={9} item>
              <Grid spacing={0} direction="row" container style={{ marginTop: "0rem" }}>
                <Grid xs={1} item style={{ marginRight: "1rem" }}>
                  <Typography>{data ? data.now_time : "00:00"}</Typography>
                </Grid>
                <Grid xs={9} item>
                  <PrettoSlider
                    onChange={(_, progress) => {
                      setIsControl(true);
                      setPercent(progress as any as number);
                      seekPlayer(progress as any as number).then((data) => {
                        const pathes = data.split("/");
                        showMessage(t("playbackPosition") + ": " + pathes[pathes.length - 1]);
                      });
                    }}
                    onChangeCommitted={(_, progress) => {
                      setPercent(progress as any as number);
                      seekPlayer(progress as any as number).then((data) => {
                        setIsControl(false);
                        const pathes = data.split("/");
                        showMessage(t("playbackPosition") + ": " + pathes[pathes.length - 1]);
                      });
                    }}
                    color="primary"
                    defaultValue={0}
                    step={0.001}
                    value={percent}
                    style={{ padding: "0rem", marginBottom: "0.1rem" }}
                  />
                </Grid>
                <Grid xs={1} item>
                  <Typography style={{ marginLeft: "1rem" }}>{data ? data.left_time : "00:00"}</Typography>
                </Grid>
              </Grid>
            </Grid>
          </Grid>
        </Paper>
      </div>
    </>
  );
}
