import { useRequest } from "ahooks";
import React, { useEffect, useRef, useState } from "react";
import { useTranslation } from "react-i18next";
import { getMessageData } from "@rpc/http";
import * as THREE from "three";
import { Dialog, DialogContent, DialogTitle, InputLabel, TextField } from "@mui/material";
import { DEFAULT_CHART_CONFIG } from "../Chart";

export const DEFAULT_PATH_CONFIG = {
  open: false,
  line_width: 3.0,
  point_size: 0.2,
};

export type Config = typeof DEFAULT_PATH_CONFIG;

export const PathDataProps = ["Stamp"];

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

function PathMessage({ name, config, chartList, chart, setChart, chartConfig }: Props) {
  const { t } = useTranslation();
  const [message, setMessage] = useState<IPoseProp[]>([]);

  const trajectoryRef = useRef<any>();
  const trajectoryPointRef = useRef<any>();

  useRequest(() => getMessageData(name, "Path"), {
    pollingInterval: 50,
    onSuccess: (data) => {
      if (data != undefined && data !== "") {
        let path = data as any;
        let pose = [];
        for (let i = 0; i < path.pose_stamp.length; i++) {
          const pos = new THREE.Vector3(path.position[i][0], path.position[i][1], path.position[i][2]);
          const quat = new THREE.Quaternion(
            path.orientation[i][0],
            path.orientation[i][1],
            path.orientation[i][2],
            path.orientation[i][3]
          );
          const euler = new THREE.Euler();
          euler.setFromQuaternion(quat);
          pose[i] = {
            position: pos.toArray() as [number, number, number],
            rotation: euler.toArray() as [number, number, number, string],
          };
        }
        setMessage(pose);

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
              item = { ...item, Stamp: path.stamp };
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

  useEffect(() => {
    const line = trajectoryRef.current;
    const point = trajectoryPointRef.current;
    if (line && point && message) {
      let shape_points = [];
      for (let i = 0; i < message.length; i++) {
        shape_points.push(new THREE.Vector3(message[i].position[0], message[i].position[1], message[i].position[2]));
      }

      line?.geometry.setFromPoints(shape_points);
      point?.geometry.setFromPoints(shape_points);
    }
  }, [message]);

  useEffect(() => {
    const line = trajectoryRef.current;
    const point = trajectoryPointRef.current;
    if (line) {
      line.frustumCulled = false;
    }
    if (point) {
      point.frustumCulled = false;
    }
  }, []);

  return (
    <>
      <line ref={trajectoryRef}>
        <lineBasicMaterial color={0x00ff66} linewidth={config.line_width} />
      </line>
      <points ref={trajectoryPointRef}>
        <pointsMaterial color={0xff0033} size={config.point_size} />
      </points>
    </>
  );
}

export default React.memo(PathMessage);

type ConfigProps = {
  config: Map<string, any>;
  message: string;
  setConfig: any;
};
export const PathMessageConfig = React.memo(({ config, message, setConfig }: ConfigProps) => {
  const { t } = useTranslation();
  const handleClose = () => {
    const c = config.get(message);
    c.open = false;
    config.set(message, c);
    setConfig(new Map(config));
  };

  const handleLineWidth = (data: string) => {
    const c = config.get(message);
    c.line_width = data;
    config.set(message, c);
    setConfig(new Map(config));
  };

  const handlePointSize = (data: string) => {
    const c = config.get(message);
    c.point_size = data;
    config.set(message, c);
    setConfig(new Map(config));
  };

  return (
    <>
      <Dialog open={config.get(message).open} onClose={() => handleClose()}>
        <DialogTitle>{t("PathConfig")}</DialogTitle>
        <DialogContent>
          <div style={{ display: "flex", flexWrap: "wrap", minWidth: "400px", maxWidth: "600px", maxHeight: "500px" }}>
            <InputLabel style={{ marginLeft: "6px" }}>{t("LineWidth")}</InputLabel>
            <TextField
              inputProps={{ inputMode: "numeric", pattern: "[0-9]*" }}
              value={config.get(message).line_width}
              onChange={(event) => {
                handleLineWidth(event.target.value);
              }}
            />
            <InputLabel style={{ marginLeft: "6px", marginTop: "1rem" }}>{t("PointSize")}</InputLabel>
            <TextField
              inputProps={{ inputMode: "numeric", pattern: "[0-9]*" }}
              value={config.get(message).point_size}
              onChange={(event) => {
                handlePointSize(event.target.value);
              }}
            />
          </div>
        </DialogContent>
      </Dialog>
    </>
  );
});
