import { Paper, Typography, Box, Switch, Button, LinearProgress } from "@mui/material";
import makeStyles from "@mui/styles/makeStyles";
import withStyles from "@mui/styles/withStyles";
import React from "react";

import { startRecord, stopRecord } from "@rpc/http";
import { usePopover } from "@hooks/index";

function humanFileSize(bytes, si = false, dp = 1) {
  const thresh = si ? 1000 : 1024;
  if (Math.abs(bytes) < thresh) {
    return bytes + " B";
  }
  const units = si
    ? ["kB", "MB", "GB", "TB", "PB", "EB", "ZB", "YB"]
    : ["KiB", "MiB", "GiB", "TiB", "PiB", "EiB", "ZiB", "YiB"];
  let u = -1;
  const r = 10 ** dp;
  do {
    bytes /= thresh;
    ++u;
  } while (Math.round(Math.abs(bytes) * r) / r >= thresh && u < units.length - 1);

  return bytes.toFixed(dp) + units[u];
}

const useStyles = makeStyles((theme) => ({
  root: {
    paddingLeft: "2rem",
    paddingRight: "1rem",
    paddingTop: "0.5rem",
    paddingBottom: "0.5rem",
  },
  line: {
    display: "flex",
    alignItems: "center",
    paddingRight: "0.2rem",
  },
}));

const YellowTextTypography = withStyles({
  root: {
    color: "#ffcc00",
  },
})(Typography);
const GreenTextTypography = withStyles({
  root: {
    color: "#339900",
  },
})(Typography);
export default function Disk({
  name,
  usage,
  total,
  frame_success,
  frame_lost,
  frame_error,
  isRecording,
  t = (x) => x,
}) {
  const toggleRecordStatus = () => {
    if (isRecording) stopRecord();
    else startRecord();
  };

  const classes = useStyles();
  const [popover, showPopover, hidePopover] = usePopover();
  return (
    <>
      {popover}
      <Paper className={classes.root} elevation={3}>
        <div className={classes.line}>
          <Typography variant="h6">{t("disk")}</Typography>
          <Box flexGrow={1} />
          {isRecording && frame_success != undefined && (
            <>
              <GreenTextTypography variant="subtitle1">{frame_success}</GreenTextTypography>
              <span>&nbsp;</span>
              <Typography variant="subtitle1">/</Typography>
              <span>&nbsp;</span>
              <Typography color="error" variant="subtitle1">
                {frame_lost}
              </Typography>
              {/* <span>&nbsp;</span> / <span>&nbsp;</span>
            <Typography color="error" variant="subtitle1">{frame_error}</Typography> */}
            </>
          )}
          <Switch onChange={toggleRecordStatus} checked={isRecording} />
        </div>
        <div>
          <LinearProgressWithLabel
            value={usage}
            t={t}
            onMouseEnter={(ev) => {
              const msg =
                t("diskVolume") +
                humanFileSize(total * 1024, true) +
                " " +
                t("diskLeft") +
                humanFileSize((total * 1024 * (100 - usage)) / 100, true);
              showPopover(
                ev,
                msg,
                {
                  anchorOrigin: {
                    vertical: "top",
                    horizontal: "center",
                  },
                  transformOrigin: {
                    vertical: "top",
                    horizontal: "center",
                  },
                },
                name
              );
            }}
            onMouseLeave={hidePopover}
          />
        </div>
      </Paper>
    </>
  );
}

function LinearProgressWithLabel(props) {
  return (
    <Box display="flex" alignItems="center" {...props}>
      <Box width="100%" mr={1}>
        <LinearProgress variant="determinate" value={props.value} />
      </Box>
      <Box mr={1}>
        <Typography variant="body2" color="textSecondary">
          {props.value.toFixed(2) + "%"}
        </Typography>
      </Box>
      {/* <Box>
        <Button
          variant="contained"
          color="primary"
          onClick={() => {
            window.open(`http://${window.location.hostname}:8000/`);
          }}>
          {props.t("viewFile")}
        </Button>
      </Box> */}
    </Box>
  );
}
