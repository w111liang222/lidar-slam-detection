import React from "react";

import { STEP } from "./Stepper";
import { Select, MenuItem } from "@mui/material";

export interface Props {
  radarArray: LSD.Config["radar"];
  radarIndex: number | undefined;
  setRadarIndex: any;
  setRadarPointcloud: any;
  activeStep: number | undefined;
  setActiveStep: any;
}
export default function radarSelect({
  radarArray,
  radarIndex,
  setRadarIndex,
  setRadarPointcloud,
  activeStep,
  setActiveStep,
}: Props) {
  return (
    <Select
      variant="standard"
      value={radarIndex === undefined ? "" : radarIndex}
      onChange={(ev) => {
        if (ev.target.value === "") {
          setRadarIndex(undefined);
          setRadarPointcloud(undefined);
        } else {
          setRadarIndex(ev.target.value);
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
      {radarArray.map((radar, i) => {
        const radarName = `${radar.name}`;
        return (
          <MenuItem value={i} key={i}>
            {radarName}
          </MenuItem>
        );
      })}
    </Select>
  );
}
