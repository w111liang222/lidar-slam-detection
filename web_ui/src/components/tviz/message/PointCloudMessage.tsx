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
  const [maxNum, setMaxNum] = useState(100e4);
  const [points, setPoints] = useState<TSARI.PointCloud2>();

  useRequest(() => getMessageData(name, "PointCloud"), {
    pollingInterval: 50,
    onSuccess: (data: TSARI.PointCloud2 | undefined) => {
      if (data != undefined && data.points != undefined) {
        if (data.points.length >= maxNum) {
          setMaxNum(maxNum * 10);
        }
        setPoints(data);

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

  return <Pointcloud2 maxNum={maxNum} color={config.color} size={config.size} points={points} />;
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
          </div>
        </DialogContent>
      </Dialog>
    </>
  );
});
