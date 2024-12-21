import React, { useEffect, useState } from "react";
import "./index.css";
import FloatButton from "@components/general/FloatButton";
import Chart, { DEFAULT_CHART_CONFIG } from "./Chart";
import createStyles from "@mui/styles/createStyles";
import makeStyles from "@mui/styles/makeStyles";
import AddIcon from "@mui/icons-material/Add";
import GrainIcon from "@mui/icons-material/Grain";
import { Canvas } from "@react-three/fiber";
import { startMessageSubscribe, stopMessageSubscribe, getMessageMeta, getStatus, getConfig } from "@rpc/http";
import {
  Autocomplete,
  Box,
  Button,
  Dialog,
  DialogActions,
  DialogContent,
  DialogTitle,
  Divider,
  Grid,
  IconButton,
  InputLabel,
  List,
  ListItem,
  ListItemIcon,
  ListItemText,
  MenuItem,
  Select,
  TextField,
  Theme,
} from "@mui/material";
import BasicMessage, { BasicDataProps } from "./message/BasicMessage";
import ImuMessage, { ImuDataProps } from "./message/ImuMessage";
import NavMessage, { NavDataProps } from "./message/NavMessage";
import OdometryMessage, {
  DEFAULT_ODOM_CONFIG,
  OdometryMessageConfig,
  OdometryDataProps,
} from "./message/OdometryMessage";
import PathMessage, { DEFAULT_PATH_CONFIG, PathMessageConfig, PathDataProps } from "./message/PathMessage";
import PointCloudMessage, {
  DEFAULT_CLOUD_CONFIG,
  PointCloudMessageConfig,
  PointCloudDataProps,
} from "./message/PointCloudMessage";
import CompressedImageMessage, { CompressedImageDataProps } from "./message/CompressedImageMessage";
import ImageMessage, { ImageDataProps } from "./message/ImageMessage";
import { useTranslation } from "react-i18next";
import Controls from "@components/3d/Controls";
import { useRequest } from "ahooks";
import { RemoveCircle } from "@mui/icons-material";
import Player from "@components/preview/Player";

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

export type Props = {};

export default function TViz({}: Props) {
  const { t } = useTranslation();
  const classes = useStyles();
  const [boardConfig, setBoardConfig] = useState<LSD.Config>();
  const [pause, setPause] = useState(false);
  const [open, setOpen] = useState(false);
  const [chartOpen, setChartOpen] = useState(false);
  const [messageMeta, setMessageMeta] = useState<LSD.MessageMeta>();
  const [selectMessage, setSelectMessage] = useState<number | undefined>(undefined);
  const [messageList, setMessageList] = useState<Map<string, string>>(new Map());
  const [messageConfig, setMessageConfig] = useState<Map<string, any>>(new Map());
  const [messageEnable, setMessageEnable] = useState<Map<string, boolean>>(new Map());
  const [messageDataProp, setMessageDataProp] = useState<Map<string, string[]>>(new Map());

  const [chartsDataList, setChartsDataList] = useState<Map<string, string[][]>>(new Map());
  const [chartsData, setChartsData] = useState<Map<string, any[]>>(new Map());
  const [chartsConfig, setChartsConfig] = useState<Map<string, typeof DEFAULT_CHART_CONFIG>>(new Map());
  const [selectChartMessage, setSelectChartMessage] = useState<string | undefined>(undefined);
  const [chartXData, setChartXData] = useState<string>("");
  const [chartYData, setChartYData] = useState<string>("");
  const [chartConfig, setChartConfig] = useState<typeof DEFAULT_CHART_CONFIG>(DEFAULT_CHART_CONFIG);
  const [chartDataList, setChartDataList] = useState<string[][]>([]);
  const [chartDataListStr, setChartDataListStr] = useState<string[]>([]);

  useRequest(getMessageMeta, {
    pollingInterval: 2000,
    onSuccess: (data) => {
      setMessageMeta(data);
    },
    onError: () => {
      setMessageMeta(undefined);
    },
  });

  const onClickMessageConfig = (message: string) => {
    const config = messageConfig.get(message);
    if (messageMeta && config) {
      config.open = true;
      messageConfig.set(message, config);
      setMessageConfig(new Map(messageConfig));
    }
  };

  const handleClose = (confirm: boolean) => {
    setOpen(false);
    if (confirm && messageMeta && selectMessage != undefined) {
      let config: any = undefined;
      if (messageMeta.types[selectMessage] == "Odometry") {
        config = JSON.parse(JSON.stringify(DEFAULT_ODOM_CONFIG));
      } else if (messageMeta.types[selectMessage] == "Path") {
        config = JSON.parse(JSON.stringify(DEFAULT_PATH_CONFIG));
      } else if (messageMeta.types[selectMessage] == "PointCloud") {
        config = JSON.parse(JSON.stringify(DEFAULT_CLOUD_CONFIG));
      }

      let data_prop: string[] = [];
      if (["Byte", "Float", "Int32", "Int64", "String"].includes(messageMeta.types[selectMessage])) {
        data_prop = BasicDataProps;
      } else if (messageMeta.types[selectMessage] == "Imu") {
        data_prop = ImuDataProps;
      } else if (messageMeta.types[selectMessage] == "NavSatFix") {
        data_prop = NavDataProps;
      } else if (messageMeta.types[selectMessage] == "Odometry") {
        data_prop = OdometryDataProps;
      } else if (messageMeta.types[selectMessage] == "Path") {
        data_prop = PathDataProps;
      } else if (messageMeta.types[selectMessage] == "PointCloud") {
        data_prop = PointCloudDataProps;
      } else if (messageMeta.types[selectMessage] == "CompressedImage") {
        data_prop = CompressedImageDataProps;
      } else if (messageMeta.types[selectMessage] == "Image") {
        data_prop = ImageDataProps;
      }

      messageList.set(messageMeta.channels[selectMessage], messageMeta.types[selectMessage]);
      setMessageList(new Map(messageList));
      messageEnable.set(messageMeta.channels[selectMessage], true);
      setMessageEnable(new Map(messageEnable));
      messageConfig.set(messageMeta.channels[selectMessage], config);
      setMessageConfig(new Map(messageConfig));
      messageDataProp.set(messageMeta.channels[selectMessage], data_prop);
      setMessageDataProp(new Map(messageDataProp));
    }
  };

  const HandleDelete = (message: string) => {
    messageList.delete(message);
    messageEnable.delete(message);
    messageConfig.delete(message);
    messageDataProp.delete(message);
    setMessageList(new Map(messageList));
    setMessageEnable(new Map(messageEnable));
    setMessageConfig(new Map(messageConfig));
    setMessageDataProp(new Map(messageDataProp));

    chartsDataList.delete(message);
    chartsData.delete(message);
    chartsConfig.delete(message);
    setChartsDataList(new Map(chartsDataList));
    setChartsData(new Map(chartsData));
    setChartsConfig(new Map(chartsConfig));
  };

  const handleAddMessage = () => {
    setSelectMessage(undefined);
    setOpen(true);
  };

  const handleChartMessageSelection = (message: string) => {
    let dataList = chartsDataList.get(message);
    let dataListStr = [];
    let config = chartsConfig.get(message);
    if (dataList == undefined || config == undefined) {
      dataList = [];
      config = JSON.parse(JSON.stringify(DEFAULT_CHART_CONFIG));
    }
    for (let i = 0; i < dataList.length; i++) {
      dataListStr[i] = "X: " + dataList[i][0] + "  Y: " + dataList[i][1];
    }
    setChartDataList(dataList);
    setChartDataListStr(dataListStr);
    setChartConfig(config as any);
  };

  const handleChartSettingClear = () => {
    setChartXData("");
    setChartYData("");
  };

  const handleChartDiagClose = (confirm: boolean) => {
    if (confirm && selectChartMessage && chartDataList.length > 0) {
      chartsDataList.set(selectChartMessage, chartDataList);
      if (chartsData.get(selectChartMessage) == undefined) {
        chartsData.set(selectChartMessage, []);
      }
      chartsConfig.set(selectChartMessage, JSON.parse(JSON.stringify(chartConfig)));
      setChartsDataList(new Map(chartsDataList));
      setChartsData(new Map(chartsData));
      setChartsConfig(new Map(chartsConfig));
    }
    setChartOpen(false);
    setSelectChartMessage(undefined);
    setChartDataList([]);
    setChartDataListStr([]);
    handleChartSettingClear();
  };

  const handleAddCharData = () => {
    if (selectChartMessage && chartXData !== "" && chartYData !== "") {
      if (chartDataList.length > 0 && chartDataList[0][0] != chartXData) {
        chartDataList.splice(0, chartDataList.length);
        chartDataListStr.splice(0, chartDataListStr.length);
      }
      const data = [chartXData, chartYData];
      const dataViz = "X: " + chartXData + "  Y: " + chartYData;
      setChartDataList([...chartDataList, data]);
      setChartDataListStr([...chartDataListStr, dataViz]);
      if (chartsData.get(selectChartMessage)) {
        chartsData.set(selectChartMessage, []);
      }
    }
  };

  let doUpdateConfig = () => {
    getStatus().then((status) => {
      if (status?.status == "Paused") {
        setPause(true);
      } else {
        setPause(false);
      }
    });
    getConfig()
      .then((config) => {
        setBoardConfig(config);
      })
      .catch(() => setBoardConfig(undefined));
  };

  window.onkeyup = (ev: KeyboardEvent) => {
    if (ev.key === " ") {
      setPause(!pause);
    }
  };

  useEffect(() => {
    doUpdateConfig();
    startMessageSubscribe();
    return () => {
      stopMessageSubscribe();
    };
  }, []);

  let player = React.useMemo(
    () =>
      boardConfig &&
      boardConfig.input.mode == "offline" && (
        <Player
          className="tviz-play"
          currentFile={boardConfig.input.data_path}
          pause={pause}
          setPause={setPause}
          onPlay={doUpdateConfig}
        />
      ),
    [boardConfig, pause, setPause]
  );

  let cameraIndex = 0;

  return (
    <>
      <div style={{ position: "relative" }}>
        <div className="tviz-right">
          <Canvas linear={true}>
            <gridHelper args={[10000, 1000, 0x444444, 0x444444]} visible={true} rotation-x={Math.PI / 2} />
            <Controls position={[0, 0, 0]} enable={true} />
            {Array.from(messageList).map(([message, type]) => {
              if (messageEnable.get(message) == false) {
                return;
              }
              if (type == "Odometry") {
                return (
                  <OdometryMessage
                    key={message}
                    name={message}
                    config={messageConfig.get(message)}
                    chartList={chartsDataList.get(message)}
                    chart={chartsData}
                    setChart={setChartsData}
                    chartConfig={chartsConfig.get(message)}
                  />
                );
              } else if (type == "Path") {
                return (
                  <PathMessage
                    key={message}
                    name={message}
                    config={messageConfig.get(message)}
                    chartList={chartsDataList.get(message)}
                    chart={chartsData}
                    setChart={setChartsData}
                    chartConfig={chartsConfig.get(message)}
                  />
                );
              } else if (type == "PointCloud") {
                return (
                  <PointCloudMessage
                    key={message}
                    name={message}
                    config={messageConfig.get(message)}
                    chartList={chartsDataList.get(message)}
                    chart={chartsData}
                    setChart={setChartsData}
                    chartConfig={chartsConfig.get(message)}
                  />
                );
              }
            })}
          </Canvas>
        </div>
      </div>
      <div className="tviz-left">
        <Grid item xs={12} md={6}>
          <List dense={false}>
            {Array.from(messageList).map(([message, type]) => {
              let component: any;
              if (["Byte", "Float", "Int32", "Int64", "String"].includes(type)) {
                component = (
                  <BasicMessage
                    name={message}
                    chartList={chartsDataList.get(message)}
                    chart={chartsData}
                    setChart={setChartsData}
                    chartConfig={chartsConfig.get(message)}
                  />
                );
              } else if (type == "Imu") {
                component = (
                  <ImuMessage
                    name={message}
                    chartList={chartsDataList.get(message)}
                    chart={chartsData}
                    setChart={setChartsData}
                    chartConfig={chartsConfig.get(message)}
                  />
                );
              } else if (type == "NavSatFix") {
                component = (
                  <NavMessage
                    name={message}
                    chartList={chartsDataList.get(message)}
                    chart={chartsData}
                    setChart={setChartsData}
                    chartConfig={chartsConfig.get(message)}
                  />
                );
              } else if (type == "Odometry") {
                component = (
                  <OdometryMessageConfig config={messageConfig} message={message} setConfig={setMessageConfig} />
                );
              } else if (type == "Path") {
                component = <PathMessageConfig config={messageConfig} message={message} setConfig={setMessageConfig} />;
              } else if (type == "PointCloud") {
                component = (
                  <PointCloudMessageConfig config={messageConfig} message={message} setConfig={setMessageConfig} />
                );
              } else if (type == "CompressedImage") {
                component = <div />;
              } else if (type == "Image") {
                component = <div />;
              } else {
                return;
              }

              return (
                <div key={message}>
                  <ListItem
                    style={{ marginBottom: "-0.5rem" }}
                    secondaryAction={
                      <IconButton edge="end" aria-label="delete" onClick={() => HandleDelete(message)}>
                        <RemoveCircle fontSize="large" color="primary" />
                      </IconButton>
                    }>
                    <ListItemText
                      primary={
                        <div style={{ width: "60%" }}>
                          <Button
                            onContextMenu={(event) => {
                              event.preventDefault();
                              if (messageEnable.get(message)) {
                                onClickMessageConfig(message);
                              }
                            }}
                            onDoubleClick={() => {
                              if (messageEnable.get(message) != undefined) {
                                messageEnable.set(message, !messageEnable.get(message));
                                setMessageEnable(new Map(messageEnable));
                              }
                            }}
                            variant="contained"
                            color={messageEnable.get(message) ? "primary" : "warning"}
                            style={{ justifyContent: "flex-start" }}
                            fullWidth
                            startIcon={<GrainIcon />}>
                            {"/" + message}
                          </Button>
                        </div>
                      }
                    />
                  </ListItem>
                  {messageEnable.get(message) && <ListItem>{component}</ListItem>}
                  <Divider style={{ marginTop: "0rem" }} />
                </div>
              );
            })}
          </List>
        </Grid>
      </div>
      <Dialog open={open} onClose={() => handleClose(false)}>
        <DialogTitle>{t("AddMessage")}</DialogTitle>
        <DialogContent>
          <div className={classes.container}>
            {messageMeta &&
              messageMeta.channels &&
              messageMeta.channels.map((value, index) => {
                if (messageList.has(value)) {
                  return;
                }
                return (
                  <ListItem
                    key={index}
                    button
                    onClick={() => setSelectMessage(index)}
                    selected={selectMessage == index ? true : false}>
                    <ListItemIcon>
                      <GrainIcon />
                    </ListItemIcon>
                    <ListItemText primary={value} secondary={messageMeta.types[index]} />
                  </ListItem>
                );
              })}
          </div>
        </DialogContent>
        <DialogActions>
          <Button
            onClick={() => {
              setChartOpen(true);
              handleClose(false);
            }}
            color="primary"
            variant="contained">
            {t("AddChart")}
          </Button>
          <Box flexGrow={1} />
          <Button onClick={() => handleClose(false)} color="primary" variant="contained">
            {t("no")}
          </Button>
          <Button onClick={() => handleClose(true)} color="primary" variant="contained">
            {t("yes")}
          </Button>
        </DialogActions>
      </Dialog>
      <Dialog open={chartOpen} onClose={() => handleChartDiagClose(false)}>
        <DialogTitle>{t("ChartConfig")}</DialogTitle>
        <DialogContent>
          <div style={{ display: "flex", flexWrap: "wrap", minWidth: "500px", maxWidth: "500px", maxHeight: "500px" }}>
            <InputLabel style={{ marginLeft: "6px" }}>{t("selectMessage")}</InputLabel>
            <Select
              fullWidth
              size={"small"}
              value={selectChartMessage}
              onChange={(event) => {
                setSelectChartMessage(event.target.value);
                handleChartMessageSelection(event.target.value);
                handleChartSettingClear();
              }}>
              {Array.from(messageList).map(([message, type]) => {
                return <MenuItem value={message}>{message}</MenuItem>;
              })}
            </Select>
            <InputLabel style={{ marginLeft: "6px", marginTop: "1rem" }}>{t("charXData")}</InputLabel>
            <Select
              fullWidth
              size={"small"}
              value={chartXData}
              onChange={(event) => {
                setChartXData(event.target.value);
              }}>
              {selectChartMessage &&
                messageDataProp.get(selectChartMessage)?.map((dataProp) => {
                  return <MenuItem value={dataProp}>{dataProp}</MenuItem>;
                })}
              {selectChartMessage && <MenuItem value={"Index"}>{"Index"}</MenuItem>}
            </Select>
            <InputLabel style={{ marginLeft: "6px", marginTop: "1rem" }}>{t("charYData")}</InputLabel>
            <Select
              fullWidth
              size={"small"}
              value={chartYData}
              onChange={(event) => {
                setChartYData(event.target.value);
              }}>
              {selectChartMessage &&
                messageDataProp.get(selectChartMessage)?.map((dataProp) => {
                  return <MenuItem value={dataProp}>{dataProp}</MenuItem>;
                })}
            </Select>
            <InputLabel style={{ marginLeft: "6px", marginTop: "1rem", marginBottom: "-0.5rem" }}>
              {t("charDataViz")}
            </InputLabel>
            <div style={{ width: "100%", marginTop: "0.5rem" }}>
              <Autocomplete
                multiple
                size={"small"}
                options={[]}
                value={chartDataListStr as any}
                readOnly
                renderInput={(params) => <TextField {...params} />}
              />
            </div>
            <InputLabel style={{ marginLeft: "6px", marginTop: "1rem" }}>{t("chartWidth")}</InputLabel>
            <TextField
              size={"small"}
              inputProps={{ inputMode: "numeric", pattern: "[0-9]*" }}
              value={chartConfig.maxNum}
              onChange={(event) => {
                chartConfig.maxNum = parseInt(event.target.value);
                setChartConfig({ ...chartConfig });
              }}
            />
            <InputLabel style={{ marginLeft: "6px", marginTop: "1rem", marginBottom: "-0.5rem" }}>
              {t("chartDomain")}
            </InputLabel>
            <Grid container>
              <Grid item md>
                <TextField
                  size={"small"}
                  label={"x Min"}
                  value={chartConfig.xDomain[0]}
                  onChange={(event) => {
                    let value = parseFloat(event.target.value);
                    chartConfig.xDomain[0] = isNaN(value) ? event.target.value : value;
                    setChartConfig({ ...chartConfig });
                  }}
                />
              </Grid>
              <Grid item md>
                <TextField
                  size={"small"}
                  label={"x Max"}
                  value={chartConfig.xDomain[1]}
                  onChange={(event) => {
                    let value = parseFloat(event.target.value);
                    chartConfig.xDomain[1] = isNaN(value) ? event.target.value : value;
                    setChartConfig({ ...chartConfig });
                  }}
                />
              </Grid>
              <Grid item md>
                <TextField
                  size={"small"}
                  label={"y Min"}
                  value={chartConfig.yDomain[0]}
                  onChange={(event) => {
                    let value = parseFloat(event.target.value);
                    chartConfig.yDomain[0] = isNaN(value) ? event.target.value : value;
                    setChartConfig({ ...chartConfig });
                  }}
                />
              </Grid>
              <Grid item md>
                <TextField
                  size={"small"}
                  label={"y Max"}
                  value={chartConfig.yDomain[1]}
                  onChange={(event) => {
                    let value = parseFloat(event.target.value);
                    chartConfig.yDomain[1] = isNaN(value) ? event.target.value : value;
                    setChartConfig({ ...chartConfig });
                  }}
                />
              </Grid>
            </Grid>
          </div>
        </DialogContent>
        <DialogActions>
          <Button onClick={handleAddCharData} color="primary" variant="contained">
            {t("AddAxisData")}
          </Button>
          <Box flexGrow={1} />
          <Button onClick={() => handleChartDiagClose(false)} color="primary" variant="contained">
            {t("no")}
          </Button>
          <Button onClick={() => handleChartDiagClose(true)} color="primary" variant="contained">
            {t("yes")}
          </Button>
        </DialogActions>
      </Dialog>
      {Array.from(chartsData).map(([message, data], index) => {
        return (
          <div key={message} style={{ position: "fixed", top: 300 * index + 100, left: "0" }}>
            <Chart
              title={message}
              config={chartsConfig.get(message)}
              data={data}
              names={chartsDataList.get(message)}
              onContextMenu={() => {
                setChartOpen(true);
                setSelectChartMessage(message);
                handleChartMessageSelection(message);
                handleChartSettingClear();
              }}
            />
          </div>
        );
      })}
      {Array.from(messageList).map(([message, type]) => {
        if (messageEnable.get(message) == false) {
          return;
        }
        if (type == "CompressedImage") {
          cameraIndex = cameraIndex + 1;
          return <CompressedImageMessage key={message} name={message} index={cameraIndex - 1} />;
        } else if (type == "Image") {
          cameraIndex = cameraIndex + 1;
          return <ImageMessage key={message} name={message} index={cameraIndex - 1} />;
        }
      })}
      {player}
      <FloatButton onClick={handleAddMessage}>
        <AddIcon />
      </FloatButton>
    </>
  );
}
