import React, { useEffect, useState } from "react";
import makeStyles from "@mui/styles/makeStyles";
import withStyles from "@mui/styles/withStyles";
import { useLocalStorageState } from "ahooks";
import { useTranslation } from "react-i18next";
import { Box, LinearProgress, Typography } from "@mui/material";
import { useRequest } from "ahooks";

import { getUpgradeStatus, Status } from "@rpc/http-upgrade";
import { useRef } from "react";
import { useImperativeHandle } from "react";

const BorderLinearProgress = withStyles((theme) => ({
  root: {
    height: 12,
    borderRadius: 10,
  },
  colorPrimary: {
    backgroundColor: theme.palette.grey[400],
  },
  bar: {
    borderRadius: 5,
    backgroundColor: "#1a90ff",
  },
}))(LinearProgress);

function LinearProgressWithLabel(props: { value: number; stage: string }) {
  const { t } = useTranslation();
  return (
    <Box display="flex" alignItems="center">
      <Box minWidth={80}>
        <Typography variant="body1">
          <b>{t(props.stage)}</b>
        </Typography>
      </Box>
      <Box width="100%" mr={1}>
        <BorderLinearProgress variant="determinate" {...props} />
      </Box>
      <Box minWidth={35}>
        <Typography variant="body2" color="textSecondary">
          <b>{`${Math.round(props.value)}%`}</b>
        </Typography>
      </Box>
    </Box>
  );
}

function UpgradeLog(props: { content: string }) {
  const ref = useRef<HTMLDivElement>(null);
  useEffect(() => {
    if (ref.current) ref.current.scrollTop = ref.current.scrollHeight;
  }, [props.content]);
  return (
    <div
      style={{
        maxWidth: "1200px",
        whiteSpace: "pre-wrap",
        overflow: "auto",
        height: "30rem",
        border: "solid",
      }}
      ref={ref}>
      {props.content}
    </div>
  );
}

function UpgradeStatus(
  props: {
    isUpgradeDone: boolean;
    setIsUpgradeDone: any;
    percentage: number;
    setPercentage: any;
    stage: Status["stage"];
    setStage: any;
  },
  ref: any
) {
  const { data, error, loading } = useRequest(getUpgradeStatus, {
    pollingInterval: 500,
    loadingDelay: 1000,
  });

  const [logContent, setLogContent] = useState("");

  useImperativeHandle(ref, () => ({
    restart: () => {
      setLogContent("");
    },
  }));

  useEffect(() => {
    if (data) {
      setLogContent((x) => x + data.log);

      if (props.isUpgradeDone) {
        return;
      }

      if (data.stage == "failed") {
        props.setStage(data.stage);
        props.setPercentage(0);
        props.setIsUpgradeDone(true);
      }
      if (data.stage == "success") {
        data.stage = "restarting";
      }
      if (props.stage != "idle" && props.stage != "uploading" && props.stage != "success" && data.stage == "idle") {
        props.setStage("success");
        props.setIsUpgradeDone(true);
        return;
      }

      props.setStage(data.stage);
      props.setPercentage && props.setPercentage(data.percentage);
    }
  }, [data]);

  return (
    <>
      <LinearProgressWithLabel value={props.percentage} stage={props.stage as string} />
      <div style={{ marginTop: "1.0rem" }} />
      <UpgradeLog content={logContent} />
    </>
  );
}

export default React.memo(React.forwardRef(UpgradeStatus));
