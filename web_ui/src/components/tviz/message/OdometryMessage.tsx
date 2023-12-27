import { useRequest } from "ahooks";
import React, { useEffect, useState } from "react";
import { useTranslation } from "react-i18next";
import { getMessageData } from "@rpc/http";
import * as THREE from "three";
import { Dialog, DialogContent, DialogTitle, InputLabel, TextField } from "@mui/material";
import { DEFAULT_CHART_CONFIG } from "../Chart";

export const DEFAULT_ODOM_CONFIG = {
  open: false,
  axis_size: 1,
  queue_size: 1,
};

export type Config = typeof DEFAULT_ODOM_CONFIG;

export const OdometryDataProps = [
  "Stamp",
  "Position[0]",
  "Position[1]",
  "Position[2]",
  "Rotation[0]",
  "Rotation[1]",
  "Rotation[2]",
];

type IPoseProp = {
  position: [number, number, number];
  rotation: [number, number, number, string];
};

export type Props = {
  name: string;
  config: Config;
  chartList?: string[][];
  chart: Map<string, any[]>;
  setChart: any;
  chartConfig?: typeof DEFAULT_CHART_CONFIG;
};

function OdometryMessage({ name, config, chartList, chart, setChart, chartConfig }: Props) {
  const { t } = useTranslation();
  const [message, setMessage] = useState<IPoseProp[]>([]);

  useRequest(() => getMessageData(name, "Odom"), {
    pollingInterval: 50,
    onSuccess: (data) => {
      if (data != undefined && data !== "") {
        let odom = data as any;
        const pos = new THREE.Vector3(odom.position[0], odom.position[1], odom.position[2]);
        const quat = new THREE.Quaternion(
          odom.orientation[0],
          odom.orientation[1],
          odom.orientation[2],
          odom.orientation[3]
        );
        const euler = new THREE.Euler();
        euler.setFromQuaternion(quat);

        while (message.length >= config.queue_size) {
          message.shift();
        }
        setMessage([
          ...message,
          {
            position: pos.toArray() as [number, number, number],
            rotation: euler.toArray() as [number, number, number, string],
          },
        ]);

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
              item = { ...item, Stamp: odom.stamp };
            } else if (itemList[i] == "Position[0]") {
              item = { ...item, "Position[0]": odom.position[0] };
            } else if (itemList[i] == "Position[1]") {
              item = { ...item, "Position[1]": odom.position[1] };
            } else if (itemList[i] == "Position[2]") {
              item = { ...item, "Position[2]": odom.position[2] };
            } else if (itemList[i] == "Rotation[0]") {
              item = { ...item, "Rotation[0]": euler.x };
            } else if (itemList[i] == "Rotation[1]") {
              item = { ...item, "Rotation[1]": euler.y };
            } else if (itemList[i] == "Rotation[2]") {
              item = { ...item, "Rotation[2]": euler.z };
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
      {message.map((pose) => {
        return (
          <group {...pose}>
            <axesHelper
              scale={new THREE.Vector3(config.axis_size, config.axis_size, config.axis_size)}
              material-linewidth={config.axis_size}
            />
          </group>
        );
      })}
    </>
  );
}

export default React.memo(OdometryMessage);

type ConfigProps = {
  config: Map<string, any>;
  message: string;
  setConfig: any;
};
export const OdometryMessageConfig = React.memo(({ config, message, setConfig }: ConfigProps) => {
  const { t } = useTranslation();
  const handleClose = () => {
    const c = config.get(message);
    c.open = false;
    config.set(message, c);
    setConfig(new Map(config));
  };

  const handleQueueSize = (data: string) => {
    const c = config.get(message);
    c.queue_size = data;
    config.set(message, c);
    setConfig(new Map(config));
  };

  const handleAxisSize = (data: string) => {
    const c = config.get(message);
    c.axis_size = data;
    config.set(message, c);
    setConfig(new Map(config));
  };

  return (
    <>
      <Dialog open={config.get(message).open} onClose={() => handleClose()}>
        <DialogTitle>{t("OdometryConfig")}</DialogTitle>
        <DialogContent>
          <div style={{ display: "flex", flexWrap: "wrap", minWidth: "400px", maxWidth: "600px", maxHeight: "500px" }}>
            <InputLabel style={{ marginLeft: "6px" }}>{t("OdometryQueue")}</InputLabel>
            <TextField
              inputProps={{ inputMode: "numeric", pattern: "[0-9]*" }}
              value={config.get(message).queue_size}
              onChange={(event) => {
                handleQueueSize(event.target.value);
              }}
            />
            <InputLabel style={{ marginLeft: "6px", marginTop: "1rem" }}>{t("OdometryAxisSize")}</InputLabel>
            <TextField
              inputProps={{ inputMode: "numeric", pattern: "[0-9]*" }}
              value={config.get(message).axis_size}
              onChange={(event) => {
                handleAxisSize(event.target.value);
              }}
            />
          </div>
        </DialogContent>
      </Dialog>
    </>
  );
});
