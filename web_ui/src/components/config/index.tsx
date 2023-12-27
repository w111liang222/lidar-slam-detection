import React, { useEffect, useRef, useState } from "react";
import {
  Box,
  Container,
  Button,
  Accordion,
  AccordionSummary,
  AccordionDetails,
  Typography,
  Tabs,
  Tab,
  Theme,
} from "@mui/material";
import withStyles from "@mui/styles/withStyles";
import createStyles from "@mui/styles/createStyles";
import ExpandMoreIcon from "@mui/icons-material/ExpandMore";
import CloudUploadIcon from "@mui/icons-material/CloudUpload";
import { useTranslation } from "react-i18next";

import { getConfig, postConfig, reboot, getStatus } from "@rpc/http";
import { useDialog, useSnackbar } from "@hooks/misc";

// import Board, { Ref as BoardRef } from "./Board";
import Mode, { Ref as ModeRef } from "./Mode";
import LidarList, { Ref as LidarRef } from "./Lidar";
import CameraList, { Ref as CameraRef, CameraType } from "./Camera";
import RadarList, { Ref as RadarRef } from "./Radar";
import InsList, { Ref as InsRef } from "./Ins";
import Detection, { Ref as DetectionRef } from "./Detection";
import Detect, { Ref as DetectRef } from "./Detect";
import Slam, { Ref as SlamRef } from "./Slam";
import Output, { Ref as OutputRef } from "./Output";
import Advance, { Ref as AdvanceRef } from "./Advance";
import FloatButton from "@components/general/FloatButton";
import { useRequest } from "ahooks";
import Loading from "@components/general/Loading";
import "./index.css";
import { getAvfunsConfig } from "@components/store/avfunSlice";
import WEB_STORE from "@rpc/sample/webstore.json";
import { useSelector, useDispatch } from "react-redux";

const AntTabs = withStyles({
  root: {
    marginTop: "1rem",
  },
  indicator: {
    backgroundColor: "#1890ff",
  },
})(Tabs);

const AntTab = withStyles((theme: Theme) =>
  createStyles({
    root: {
      "&:hover": {
        color: "#40a9ff",
        opacity: 1,
      },
      "&$selected": {
        color: "#1890ff",
      },
      "&:focus": {
        color: "#40a9ff",
      },
    },
    selected: {},
  })
)((props: { label: any }) => <Tab {...props} />);

export default function Config() {
  const { t } = useTranslation();
  const [tableIndex, setTableIndex] = useState(0);

  // const boardRef = useRef<BoardRef>(null);
  const modeRef = useRef<ModeRef>(null);
  const lidarRef = useRef<LidarRef>(null);
  const cameraRef = useRef<CameraRef>(null);
  const radarRef = useRef<RadarRef>(null);
  const insRef = useRef<InsRef>(null);
  const detectRef = useRef<DetectRef>(null);
  const detectionRef = useRef<DetectionRef>(null);
  const slamRef = useRef<SlamRef>(null);
  const outputRef = useRef<OutputRef>(null);
  const advanceRef = useRef<AdvanceRef>(null);
  const [isLoading, setIsLoading] = useState<boolean>(false);

  const avfuns = useSelector((state) => (state as any).avfuns);
  const dispatch = useDispatch();
  const [show, setShow] = useState(WEB_STORE.avfuns.config);

  useEffect(() => {
    const action = getAvfunsConfig();
    dispatch(action as any);
  }, []);
  useEffect(() => {
    if (typeof avfuns == "object") {
      setShow(avfuns.config);
    }
  }, [avfuns]);

  const { data, error, loading, run, cancel } = useRequest(getConfig, {
    pollingInterval: 1000,
    loadingDelay: 1000,
    manual: true,
    onSuccess: () => {
      cancel();
      setIsLoading(false);
    },
  });
  const config = data;
  useEffect(() => {
    run();
  }, []);

  const statusData = useRequest(getStatus, {
    pollingInterval: 1000,
    loadingDelay: 1000,
  });
  const status = statusData.data;

  const handleUpload = async () => {
    try {
      if (!config) throw new Error("Empty config!");
      // if (boardRef.current && config?.board) {
      //   if (!boardRef.current.isValid) throw new Error();
      //   config.board = boardRef.current.validationSchema.cast(boardRef.current.values) as LSD.Config["board"];
      // }
      if (modeRef.current && config?.pipeline) {
        if (!modeRef.current.isValid) throw new Error();
        config.pipeline = modeRef.current.validationSchema.cast(modeRef.current.values) as LSD.Config["pipeline"];
      }
      if (lidarRef.current && config?.lidar) {
        if (!lidarRef.current.isValid) throw new Error();
        const lidar = lidarRef.current.validationSchema.cast(lidarRef.current.values);
        if (lidar) config.lidar = lidar;
      }

      if (radarRef.current && config?.radar) {
        if (!radarRef.current.isValid) throw new Error();
        const radar = radarRef.current.validationSchema.cast(radarRef.current.values);
        if (radar) config.radar = radar;
      }

      if (insRef.current && config?.ins) {
        if (!insRef.current.isValid) throw new Error();
        const ins = insRef.current.validationSchema.cast(insRef.current.values);
        if (ins) config.ins = ins;
      }

      if (cameraRef.current && config?.camera) {
        if (!cameraRef.current.isValid) throw new Error();
        config.camera = cameraRef.current.validationSchema.cast(cameraRef.current.values) as LSD.Config["camera"];
        config.camera = [...config.camera];
        let configCamera: CameraType[] = config.camera;
        configCamera.map(({ output_width, output_height }, index) => {
          let width = parseInt(output_width as string);
          let height = parseInt(output_height as string);
          if (Number.isNaN(width) || Number.isNaN(height)) {
            configCamera[index] = { ...configCamera[index] };
            delete configCamera[index]["output_width"];
            delete configCamera[index]["output_height"];
          } else {
            configCamera[index]["output_width"] = width;
            configCamera[index]["output_height"] = height;
          }
          config.camera[index] = { ...configCamera[index] };
        });
      }
      if (detectionRef.current && config?.detection) {
        if (!detectionRef.current.isValid) throw new Error();
          config.detection = detectionRef.current.validationSchema.cast(
          detectionRef.current.values
        ) as LSD.Config["detection"];
        config.detection = { ...config.detection };
      }
      if (detectRef.current && config?.output) {
        if (!detectRef.current.isValid) throw new Error();
        config.output = detectRef.current.validationSchema.cast(detectRef.current.values) as LSD.Config["output"];
        config.output = { ...config.output };
      }
      if (slamRef.current && config?.slam) {
        if (!slamRef.current.isValid) throw new Error();
        config.slam = slamRef.current.validationSchema.cast(slamRef.current.values) as LSD.Config["slam"];
        config.slam = { ...config.slam };
      }
      if (outputRef.current && config?.output) {
        if (!outputRef.current.isValid) throw new Error();
        const configOutput = outputRef.current.validationSchema.cast(
          outputRef.current.values
        ) as LSD.Config["output"];
        config.output.protocol = { ...configOutput.protocol };
        config.output.localization = { ...configOutput.localization };
      }
      if (advanceRef.current && config?.output?.point_cloud) {
        if (!advanceRef.current.isValid) throw new Error();
        const configAdvance = advanceRef.current.validationSchema.cast(advanceRef.current.values);
        if (configAdvance) {
          config.output.point_cloud = configAdvance.lidar;
          config.ins.relay = configAdvance.ins;
        }
      }
      setIsLoading(true);
      const configUpdateResponse = await postConfig(config);
      setIsLoading(false);
      const retStatus = configUpdateResponse.status;
      showMessage(t("configUpdated"), 2000);
      if (retStatus === "Reboot") {
        openDialog();
      }
    } catch (error) {
      console.log(error);
      showMessage(t("invalidConfig"), 1000);
    }
  };
  const handleRebootCancel = async () => {
    await reboot(false);
    await run();
  };
  const handleRebootConfirm = async () => {
    showMessage(t("doReboot"));
    const doRebootResponse = await reboot(true);
    if (doRebootResponse.hostname) {
      let href = `http://${doRebootResponse.hostname}`;
      const port = window.location.host.split(":")[1];
      href += port == null ? "" : `:${port}`;
      window.location.href = href;
    }
  };
  const handleTabChanged = (e: any, value: number) => {
    setTableIndex(value);
    run();
  };
  const [dialog, openDialog] = useDialog({
    content: t("needReboot"),
    onConfirm: handleRebootConfirm,
    onCancel: handleRebootCancel,
  });
  const [snackbar, showMessage] = useSnackbar();

  let res = null;
  let index = 0;
  if (loading || error || isLoading) {
    res = <Loading />;
  } else {
    const panels = {
      device: show.device && config?.board && config?.pipeline && (
        <>
          <Mode ref={modeRef} initialValues={config?.pipeline} t={t} />
          {/* <Board ref={boardRef} initialValues={config?.board} t={t} /> */}
        </>
      ),
      lidar: show.lidar && config?.lidar_all && config?.lidar && (
        <LidarList
          ref={lidarRef}
          lidarDefaultArray={config?.lidar_all}
          initialValues={config?.lidar}
          t={t}
          status={status}
        />
      ),
      camera: show.camera && config?.camera && (
        <CameraList ref={cameraRef} initialValues={config?.camera} t={t} status={status} />
      ),
      radar: show.radar && config?.radar_all && config?.radar && (
        <RadarList
          ref={radarRef}
          radarDefaultArray={config?.radar_all}
          initialValues={config?.radar}
          t={t}
          status={status}
        />
      ),
      ins: show.ins && config?.ins && <InsList ref={insRef} initialValues={config?.ins} t={t} />,
      detect: show.detect && config?.detection && config?.output && (
        <>
          <Detection ref={detectionRef} initialValues={config?.detection} config={config} t={t} />
          <Detect ref={detectRef} initialValues={config?.output} t={t} />
        </>
      ),
      slam: show.slam && config?.slam && <Slam ref={slamRef} initialValues={config?.slam} config={config} t={t} />,
      output: show.output && config?.output && <Output ref={outputRef} initialValues={config?.output} t={t} />,
      advance: show.advance && config?.output?.point_cloud && (
        <Advance ref={advanceRef} initialValues={{ lidar: config?.output?.point_cloud, ins: config.ins.relay }} t={t} />
      ),
    };

    res = (
      <div style={{ display: "flex", width: "100%" }}>
        <Accordion style={{ flex: "1", minHeight: window.innerHeight - 64 }}>
          <AntTabs orientation="vertical" variant="scrollable" value={tableIndex} onChange={handleTabChanged}>
            {show.device && <AntTab label={<Typography variant="subtitle1">{t("device")}</Typography>} />}
            {show.lidar && <AntTab label={<Typography variant="subtitle1">{t("lidar")}</Typography>} />}
            {show.camera && <AntTab label={<Typography variant="subtitle1">{t("camera")}</Typography>} />}
            {show.radar && <AntTab label={<Typography variant="subtitle1">{t("radar")}</Typography>} />}
            {show.ins && <AntTab label={<Typography variant="subtitle1">{t("ins")}</Typography>} />}
            {show.detect && <AntTab label={<Typography variant="subtitle1">{t("detect")}</Typography>} />}
            {show.slam && <AntTab label={<Typography variant="subtitle1">{t("slam")}</Typography>} />}
            {show.output && <AntTab label={<Typography variant="subtitle1">{t("output")}</Typography>} />}
            {show.advance && <AntTab label={<Typography variant="subtitle1">{t("advance")}</Typography>} />}
          </AntTabs>
        </Accordion>
        <div style={{ flex: "8" }}>
          <Container maxWidth="md" style={{ marginLeft: "0rem" }}>
            {Object.entries(panels).map((kv) => {
              const [name, component] = kv;
              if (component) {
                index++;
                return (
                  <div hidden={index - 1 != tableIndex} style={{ marginTop: "1rem" }}>
                    <Accordion defaultExpanded={true} key={name} elevation={3}>
                      <AccordionDetails>
                        <div>{component}</div>
                      </AccordionDetails>
                    </Accordion>
                  </div>
                );
              }
            })}
            <FloatButton onClick={handleUpload}>
              <CloudUploadIcon />
            </FloatButton>
            {dialog}
            {snackbar}
          </Container>
        </div>
      </div>
    );
  }
  return res;
}
