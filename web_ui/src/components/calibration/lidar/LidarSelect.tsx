import React from "react";
import { Select, MenuItem } from "@mui/material";
import { STEP } from "./Stepper";

export interface Props {
  lidarArray: LSD.Config["lidar"];
  lidarIndex: number | undefined;
  setLidarIndex: any;
  activeStep: number | undefined;
  setActiveStep: any;
}
export default function lidarSelect({ lidarArray, lidarIndex, setLidarIndex, activeStep, setActiveStep }: Props) {
  return (
    <Select
      variant="standard"
      value={lidarIndex === undefined ? "" : lidarIndex}
      onChange={(ev) => {
        if (ev.target.value === "") {
          setLidarIndex(undefined);
        } else {
          setLidarIndex(ev.target.value);
          if (activeStep === STEP.BEGIN) {
            setActiveStep(activeStep + 1);
          }
        }
      }}
      onFocus={(ev) => {
        ev.target.blur();
      }}>
      <MenuItem value="" key="">
        <em>None</em>
      </MenuItem>
      {lidarArray.map((lidar, i) => {
        const lidarName = `${lidar.name}:${lidar.port}`;
        return (
          <MenuItem value={i} key={i}>
            {lidarName}
          </MenuItem>
        );
      })}
    </Select>
  );
}
