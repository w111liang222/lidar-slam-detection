import { useRequest } from "ahooks";
import React, { useEffect, useState } from "react";
import { useTranslation } from "react-i18next";
import { getMessageData } from "@rpc/http";
import { FormControl, Grid, TextField } from "@mui/material";
import { DEFAULT_CHART_CONFIG } from "../Chart";

export type Props = {
  name: string;
  chartList?: string[][];
  chart: Map<string, any[]>;
  setChart: any;
  chartConfig?: typeof DEFAULT_CHART_CONFIG;
};

export const ImuDataProps = ["Stamp", "Gyro X", "Gyro Y", "Gyro Z", "Accel X", "Accel Y", "Accel Z"];

function ImuMessage({ name, chartList, chart, setChart, chartConfig }: Props) {
  const { t } = useTranslation();
  const [message, setMessage] = useState<any>();

  useRequest(() => getMessageData(name, "Imu"), {
    pollingInterval: 50,
    onSuccess: (data: any) => {
      if (data != undefined && data !== "") {
        setMessage(data);

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
              item = { ...item, Stamp: data.stamp };
            } else if (itemList[i] == "Gyro X") {
              item = { ...item, "Gyro X": data.gyro_x };
            } else if (itemList[i] == "Gyro Y") {
              item = { ...item, "Gyro Y": data.gyro_y };
            } else if (itemList[i] == "Gyro Z") {
              item = { ...item, "Gyro Z": data.gyro_z };
            } else if (itemList[i] == "Accel X") {
              item = { ...item, "Accel X": data.acc_x };
            } else if (itemList[i] == "Accel Y") {
              item = { ...item, "Accel Y": data.acc_y };
            } else if (itemList[i] == "Accel Z") {
              item = { ...item, "Accel Z": data.acc_z };
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
      {message && (
        <div style={{ width: "100%" }}>
          <FormControl fullWidth variant="standard">
            <TextField fullWidth label={"Stamp"} value={message.stamp} size={"small"} />
          </FormControl>
          <Grid container>
            <Grid item md>
              <TextField label={"Gyro X"} value={message.gyro_x} size={"small"} />
            </Grid>
            <Grid item md>
              <TextField label={"Gyro Y"} value={message.gyro_y} size={"small"} />
            </Grid>
            <Grid item md>
              <TextField label={"Gyro Z"} value={message.gyro_z} size={"small"} />
            </Grid>
          </Grid>
          <Grid container>
            <Grid item md>
              <TextField label={"Accel X"} value={message.acc_x} size={"small"} />
            </Grid>
            <Grid item md>
              <TextField label={"Accel Y"} value={message.acc_y} size={"small"} />
            </Grid>
            <Grid item md>
              <TextField label={"Accel Z"} value={message.acc_z} size={"small"} />
            </Grid>
          </Grid>
        </div>
      )}
    </>
  );
}

export default React.memo(ImuMessage);
