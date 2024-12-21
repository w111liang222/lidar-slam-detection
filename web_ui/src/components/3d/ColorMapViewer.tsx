import React from "react";
import { useState } from "react";
import { getColorMap } from "@rpc/http";
import { useRequest } from "ahooks";
import Pointcloud2 from "@components/3d/Pointcloud2";

export type IProps = {
  config?: any;
  mannual?: boolean;
};

export default function ColorMapViewer({ config, mannual }: IProps) {
  const [maxNum, setMaxNum] = useState(100e4);
  const [points, setPoints] = useState<LSD.PointCloud2>();

  const { run, cancel } = useRequest(() => getColorMap(), {
    pollingInterval: 3000,
    onSuccess: (data: LSD.PointCloud2 | undefined) => {
      if (data != undefined && data.points != undefined) {
        if (data.points.length > maxNum) {
          setMaxNum(Math.max(maxNum * 2, data.points.length + 100e4));
        }
        setPoints(data);
        if (mannual) {
          cancel();
        }
      }
    },
  });

  return (
    <Pointcloud2
      maxNum={maxNum}
      begin={0}
      end={points ? points.points.length : 0}
      color={config.color}
      size={config.size}
      points={points}
      zRange={config.zRange}
      visible={config.visible}
    />
  );
}
