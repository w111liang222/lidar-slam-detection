import { useRequest } from "ahooks";
import React, { useEffect, useState } from "react";
import { useTranslation } from "react-i18next";
import { getMessageData } from "@rpc/http";
import { FormControl, OutlinedInput } from "@mui/material";
import { DEFAULT_CHART_CONFIG } from "../Chart";

export type Props = {
  name: string;
  chartList?: string[][];
  chart: Map<string, any[]>;
  setChart: any;
  chartConfig?: typeof DEFAULT_CHART_CONFIG;
};

export const BasicDataProps = ["Data"];

function BasicMessage({ name, chartList, chart, setChart, chartConfig }: Props) {
  const { t } = useTranslation();
  const [message, setMessage] = useState<String>();

  useRequest(() => getMessageData(name, ""), {
    pollingInterval: 50,
    onSuccess: (data) => {
      if (data != undefined && data !== "") {
        setMessage(data as String);

        let chartData = chart.get(name);
        if (chartList && chartList.length > 0 && chartData && chartConfig) {
          let item = {};
          if (chartList[0][0] == "Index") {
            if (chartData.length == 0) {
              item = { ...item, Index: 0 };
            } else {
              item = { ...item, Index: chartData[chartData.length - 1].Index + 1 };
            }
          }

          for (let i = 0; i < chartList.length; i++) {
            if (chartList[i][1] == "Data") {
              item = { ...item, Data: data };
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
      <FormControl fullWidth variant="standard">
        <OutlinedInput readOnly value={message} size={"small"} />
      </FormControl>
    </>
  );
}

export default React.memo(BasicMessage);
