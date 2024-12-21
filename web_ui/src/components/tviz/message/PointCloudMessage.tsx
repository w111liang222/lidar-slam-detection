import { useRequest } from "ahooks";
import React, { useEffect, useState } from "react";
import { useTranslation } from "react-i18next";
import { getMessageData } from "@rpc/http";
import Pointcloud2 from "@components/3d/Pointcloud2";
import { Dialog, DialogContent, DialogTitle, InputLabel, MenuItem, Select, TextField } from "@mui/material";
import { DEFAULT_CHART_CONFIG } from "../Chart";

export const DEFAULT_CLOUD_CONFIG = {
  open: false,
  color: "height" as "gray" | "depth" | "height" | "intensity" | "rgb",
  size: 1.5,
  zRange: { min: -10000.0, max: 10000.0 },
};

export type Config = typeof DEFAULT_CLOUD_CONFIG;

export const PointCloudDataProps = ["Number"];

export type Props = {
  name: string;
  config: Config;
  chartList?: string[][];
  chart: Map<string, any[]>;
  setChart: any;
  chartConfig?: typeof DEFAULT_CHART_CONFIG;
};

function PointCloudMessage({ name, config, chartList, chart, setChart, chartConfig }: Props) {
  const { t } = useTranslation();
  const [maxNum, setMaxNum] = useState(1000e4);
  const [pointsSplition, setPointsSplition] = useState<number [][]>([]);
  const [points, setPoints] = useState<LSD.PointCloud2>();

  useRequest(() => getMessageData(name, "PointCloud"), {
    pollingInterval: 50,
    onSuccess: (data: LSD.PointCloud2 | undefined) => {
      if (data != undefined && data.points != undefined) {
        setPoints(data);

        let splition = [];
        for (let i = 0; i < data.points.length; i = i + (maxNum * 3)) {
          splition.push([i, Math.min(i + (maxNum * 3), data.points.length)]);
        }
        setPointsSplition(splition);

        // Chart Visualization
        let chartData = chart.get(name);
        if (chartList && chartList.length > 0 && chartData && chartConfig) {
          let item = {};
          const itemList = [];
          if (chartList[0][0] == "Index") {
            if (chartData.length == 0) {
              item = { ...item, Index: 0 };
            } else {
              item = { ...item, Index: chartData[chartData.length - 1].Index + 1 };
            }
          } else {
            itemList.push(chartList[0][0]);
          }
          for (let i = 0; i < chartList.length; i++) {
            itemList.push(chartList[i][1]);
          }

          for (let i = 0; i < itemList.length; i++) {
            if (itemList[i] == "Stamp") {
              item = { ...item, Stamp: 0 };
            } else if (itemList[i] == "Number") {
              item = { ...item, Number: data.points.length / 3 };
            }
          }

          while (chartData.length > chartConfig.maxNum) {
            chartData.shift();
          }
          chartData = [...chartData, item];
          chart.set(name, chartData);
          setChart(new Map(chart));
        }
      }
    },
  });

  return (
    <>
      {pointsSplition.map((splition, idx) => {
        return (
          <Pointcloud2 maxNum={maxNum} begin={splition[0]} end={splition[1]} color={config.color} size={config.size} zRange={config.zRange} points={points} />
        );
      })}
    </>
  );
}

export default React.memo(PointCloudMessage);

type ConfigProps = {
  config: Map<string, any>;
  message: string;
  setConfig: any;
};
export const PointCloudMessageConfig = React.memo(({ config, message, setConfig }: ConfigProps) => {
  const { t } = useTranslation();
  const handleClose = () => {
    const c = config.get(message);
    c.open = false;
    config.set(message, c);
    setConfig(new Map(config));
  };

  const handleColor = (data: string) => {
    const c = config.get(message);
    c.color = data;
    config.set(message, c);
    setConfig(new Map(config));
  };

  const handleSize = (data: string) => {
    const c = config.get(message);
    c.size = data;
    config.set(message, c);
    setConfig(new Map(config));
  };

  const handleZMin = (data: string) => {
    const c = config.get(message);
    c.zRange["min"] = data;
    config.set(message, c);
    setConfig(new Map(config));
  };

  const handleZMax = (data: string) => {
    const c = config.get(message);
    c.zRange["max"] = data;
    config.set(message, c);
    setConfig(new Map(config));
  };

  return (
    <>
      <Dialog open={config.get(message).open} onClose={() => handleClose()}>
        <DialogTitle>{t("SetCloudMessageConfig")}</DialogTitle>
        <DialogContent>
          <div style={{ display: "flex", flexWrap: "wrap", minWidth: "400px", maxWidth: "600px", maxHeight: "500px" }}>
            <InputLabel style={{ marginLeft: "6px" }}>{t("color")}</InputLabel>
            <Select
              fullWidth
              value={config.get(message).color}
              onChange={(event) => {
                handleColor(event.target.value);
              }}>
              <MenuItem value={"gray"}>{t("gray")}</MenuItem>
              <MenuItem value={"depth"}>{t("depth")}</MenuItem>
              <MenuItem value={"height"}>{t("height")}</MenuItem>
              <MenuItem value={"intensity"}>{t("intensity")}</MenuItem>
              <MenuItem value={"rgb"}>{t("rgb")}</MenuItem>
            </Select>
            <InputLabel style={{ marginLeft: "6px", marginTop: "1rem" }}>{t("size")}</InputLabel>
            <TextField
              inputProps={{ inputMode: "numeric", pattern: "[0-9]*" }}
              value={config.get(message).size}
              onChange={(event) => {
                handleSize(event.target.value);
              }}
            />
            <InputLabel style={{ marginLeft: "6px", marginTop: "1rem" }}>{t("zRange") + "Min"}</InputLabel>
            <TextField
              inputProps={{ inputMode: "numeric", pattern: "[0-9]*" }}
              value={config.get(message).zRange["min"]}
              onChange={(event) => {
                handleZMin(event.target.value);
              }}
            />
            <InputLabel style={{ marginLeft: "6px", marginTop: "1rem" }}>{t("zRange") + "Max"}</InputLabel>
            <TextField
              inputProps={{ inputMode: "numeric", pattern: "[0-9]*" }}
              value={config.get(message).zRange["max"]}
              onChange={(event) => {
                handleZMax(event.target.value);
              }}
            />
          </div>
        </DialogContent>
      </Dialog>
    </>
  );
});
