import React, { useEffect, useState } from "react";
import Draggable from "react-draggable";
import ZoomChart from "./chart/zoomChart";

export const DEFAULT_CHART_CONFIG = {
  maxNum: 100,
  xDomain: ["dataMin", "dataMax"] as any[],
  yDomain: ["auto", "auto"] as any[],
};

export type Props = {
  title: string;
  config?: typeof DEFAULT_CHART_CONFIG;
  data: any[];
  names?: string[][];
  onContextMenu?: any;
};

function Chart({ title, config, data, names, onContextMenu }: Props) {
  const lineColor = ["#82ca9d", "#8884d8", "#ff7300", "#413ea0"];

  const [axisProp, setAxisProp] = useState<any>({});
  const [lineProp, setLineProp] = useState<any>([]);

  useEffect(() => {
    if (names && config) {
      setAxisProp({
        x: {
          dataKey: names[0][0],
          domain: config.xDomain,
          allowDataOverflow: true,
          type: "number",
        },
        y: {
          type: "number",
          domain: config.yDomain,
          allowDataOverflow: true,
        },
      });

      let line = [];
      for (let i = 0; i < names.length; i++) {
        line.push({
          dataKey: names[i][1],
          strokeWidth: 2,
          dot: { strokeWidth: 1 },
          stroke: lineColor[i],
        });
      }
      setLineProp(line);
    }
  }, [names, config]);

  return (
    <>
      {names && (
        <Draggable>
          <div
            style={{ width: "600px", height: "300px" }}
            onContextMenu={(event) => {
              event.preventDefault();
              onContextMenu && onContextMenu();
            }}>
            <ZoomChart
              data={data}
              lines={lineProp}
              axis={axisProp}
              title={title}
              tooltip={{
                allowEscapeViewBox: { x: true, y: true },
              }}
            />
          </div>
        </Draggable>
      )}
    </>
  );
}

export default React.memo(Chart);
