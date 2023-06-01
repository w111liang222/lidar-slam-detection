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

export const NavDataProps = ["Stamp", "Latitude", "Longitude", "Altitude"];

function NavMessage({ name, chartList, chart, setChart, chartConfig }: Props) {
  const { t } = useTranslation();
  const [message, setMessage] = useState<any>();

  useRequest(() => getMessageData(name, "NavSatFix"), {
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
            } else if (itemList[i] == "Latitude") {
              item = { ...item, Latitude: data.latitude };
            } else if (itemList[i] == "Longitude") {
              item = { ...item, Longitude: data.longitude };
            } else if (itemList[i] == "Altitude") {
              item = { ...item, Altitude: data.altitude };
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
              <TextField label={"Latitude"} value={message.latitude} size={"small"} />
            </Grid>
            <Grid item md>
              <TextField label={"Longitude"} value={message.longitude} size={"small"} />
            </Grid>
            <Grid item md>
              <TextField label={"Altitude"} value={message.altitude} size={"small"} />
            </Grid>
          </Grid>
        </div>
      )}
    </>
  );
}

export default React.memo(NavMessage);
