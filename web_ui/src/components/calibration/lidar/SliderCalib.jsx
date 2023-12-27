import React, { useEffect, useState, useRef } from "react";
import PropTypes from "prop-types";
import withStyles from "@mui/styles/withStyles";
import makeStyles from "@mui/styles/makeStyles";
import Slider from "@mui/material/Slider";
import Typography from "@mui/material/Typography";
import Tooltip from "@mui/material/Tooltip";
import Grid from "@mui/material/Grid";
import produce from "immer";
import { Divider } from "@mui/material";

function ValueLabelComponent(props) {
  const { children, open, value } = props;

  return (
    <Tooltip open={open} enterTouchDelay={0} placement="top" title={value}>
      {children}
    </Tooltip>
  );
}

ValueLabelComponent.propTypes = {
  children: PropTypes.element.isRequired,
  open: PropTypes.bool.isRequired,
  value: PropTypes.number.isRequired,
};

function Row({ label, value, onChange, min, max, step }) {
  const ref = useRef();
  return (
    <>
      <Typography
        style={{ width: "2.8rem", cursor: "pointer" }}
        onClick={() => {
          ref.current.getElementsByClassName("MuiSlider-thumb")[0].focus();
        }}>
        {label}
      </Typography>
      <div style={{ flexGrow: 1.5 }}>
        <Slider
          ref={ref}
          components={{ ValueLabel: ValueLabelComponent }}
          size="small"
          valueLabelDisplay="auto"
          aria-label="x"
          value={value}
          onChange={onChange}
          min={min}
          max={max}
          step={step}
        />
      </div>
    </>
  );
}

export default function CustomizedSlider({ rotation, translation, setRotation, setTranslation }) {
  return (
    <>
      {["X", "Y", "Z"].map((label, i) => (
        <div
          style={{
            display: "flex",
            alignItems: "center",
            marginTop: "0.1rem",
          }}>
          <Row
            {...{
              label,
              value: translation[i],
              onChange: (ev, val) => {
                setTranslation(
                  produce((x) => {
                    x[i] = val;
                  })
                );
              },
              min: -5,
              max: 5,
              step: 0.01,
            }}
          />
        </div>
      ))}
      {["Roll", "Pitch", "Yaw"].map((label, i) => (
        <div
          style={{
            display: "flex",
            alignItems: "center",
            marginTop: "0.1rem",
          }}>
          <Row
            {...{
              label,
              value: rotation[i],
              onChange: (ev, val) => {
                setRotation(
                  produce((x) => {
                    x[i] = val;
                  })
                );
              },
              min: -30,
              max: 30,
              step: 0.05,
            }}
          />
        </div>
      ))}
    </>
  );
}

export function ExcludeSlider({ excludeConfig, setExcludeConfig }) {
  const order = [0, 3, 1, 4, 2, 5];
  return (
    <>
      {["Xmin", "Xmax", "Ymin", "Ymax", "Zmin", "Zmax"].map((label, i) => (
        <div
          style={{
            display: "flex",
            alignItems: "center",
            marginTop: "0.1rem",
          }}>
          <Row
            {...{
              label,
              value: excludeConfig[order[i]],
              onChange: (ev, val) => {
                excludeConfig[order[i]] = val;
                setExcludeConfig([...excludeConfig]);
              },
              min: -5,
              max: 5,
              step: 0.01,
            }}
          />
        </div>
      ))}
    </>
  );
}
