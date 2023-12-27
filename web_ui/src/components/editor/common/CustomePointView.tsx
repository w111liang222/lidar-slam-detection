import React from "react";
import PointcloudCustomeColor from "@components/3d/PointcloudCustomeColor";

type Props = {
  points?: Float32Array;
  color: number;
};

export const CustomePointView = React.memo(({ points, color }: Props) => {
  return (
    <PointcloudCustomeColor
      maxNum={points ? points.length / 4 : 0}
      points={points}
      color={color}
      size={3}
      visible={true}
    />
  );
});
