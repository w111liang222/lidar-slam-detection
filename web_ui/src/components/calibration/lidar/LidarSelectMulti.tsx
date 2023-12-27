import React from "react";
import { Select, MenuItem, Input, Checkbox, ListItemText, SelectChangeEvent } from "@mui/material";

export interface Props {
  lidarArray: LSD.Config["lidar"];
  lidarIndex: number[];
  setLidarIndex: any;
  activeStep: number | undefined;
  setActiveStep: any;
}
export default function lidarSelectMulti({ lidarArray, lidarIndex, setLidarIndex, activeStep, setActiveStep }: Props) {
  const handleChange = (event: SelectChangeEvent<unknown>) => {
    setLidarIndex(event.target.value as number[]);
  };

  return (
    <Select
      multiple
      value={lidarIndex}
      onChange={handleChange}
      variant="standard"
      renderValue={(selected: any) => {
        let render = "";
        for (let i = 0; i < selected.length; i++) {
          render = render + lidarArray[selected[i]].name + ":" + lidarArray[selected[i]].port.toString();
          if (i != selected.length - 1) {
            render = render + ", ";
          }
        }
        return render;
      }}>
      {lidarArray.map((lidar, i) => {
        const lidarName = `${lidar.name}:${lidar.port}`;
        return (
          <MenuItem value={i} key={i}>
            <Checkbox color="primary" checked={lidarIndex.indexOf(i) > -1} />
            <ListItemText primary={lidarName} />
          </MenuItem>
        );
      })}
    </Select>
  );
}
