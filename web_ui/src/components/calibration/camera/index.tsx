import React, { useState, useEffect, useRef } from "react";
import { useTranslation } from "react-i18next";

import { getSourceData } from "@rpc/http";
import Stepper, { STEP } from "./Stepper";
import Calib, { Config, DEFAULT_CONFIG } from "./Calib";
import CalibCheck from "./CalibCheck";
import { MenuItem, Select, Input, Typography, InputAdornment } from "@mui/material";

import { useInterval, useKeyPress, useLockFn, useSetState, useToggle } from "ahooks";

type Frame = {
  imageUrl?: string;
  images?: LSD.Detection["images"];
};

export default function CameraCalibrator({}) {
  const { t } = useTranslation();

  const [calibEnable, setCalibEnable] = useState<boolean>(false);
  const [activeCameraName, setActiveCameraName] = useState<string>();
  const [frame, setFrame] = useSetState<Frame>({});
  const [config, setConfig] = useState<Config>(DEFAULT_CONFIG);
  const [paused, { toggle }] = useToggle(false);
  const [doDistort, setDoDistort] = useState<boolean>(false);
  useKeyPress("space", (ev) => {
    toggle();
  });

  useInterval(
    useLockFn(async () => {
      if (!paused) {
        const frameData = await getSourceData(doDistort);
        let url;
        if (activeCameraName) {
          const blob = new Blob([frameData.images[activeCameraName].buffer], {
            type: "application/octet-stream",
          });
          url = URL.createObjectURL(blob);
        }
        setFrame({
          imageUrl: url,
          ...frameData,
        });
      }
    }),
    100
  );

  useEffect(() => {
    if (activeCameraName && frame.images && frame.images.hasOwnProperty(activeCameraName)) {
      const blob = new Blob([frame.images[activeCameraName].buffer], {
        type: "application/octet-stream",
      });
      const url = URL.createObjectURL(blob);
      setFrame({
        imageUrl: url,
        ...frame,
      });
    }
  }, [activeCameraName]);

  const [activeStep, setActiveStep] = useState(0);
  let canvas = null;
  const ref = useRef<any>();
  if (activeStep === STEP.BEGIN) {
    canvas = (
      <Calib
        imageUrl={frame.imageUrl}
        images={frame.images}
        cameraName={activeCameraName}
        paused={false}
        setCalibEnable={setCalibEnable}
        setDoDistort={setDoDistort}
        config={config}
      />
    );
  } else if (activeStep === STEP.CALIB_CAMERA) {
    canvas = (
      <Calib
        ref={ref}
        imageUrl={frame.imageUrl}
        images={frame.images}
        cameraName={activeCameraName}
        paused={paused}
        setCalibEnable={setCalibEnable}
        setDoDistort={setDoDistort}
        config={config}
      />
    );
  } else {
    ref.current = null;
    canvas = (
      <CalibCheck ref={ref} imageUrl={frame.imageUrl} setCalibEnable={setCalibEnable} setDoDistort={setDoDistort} />
    );
  }

  const calibrate = async () => {
    if (ref.current) {
      setCalibEnable(false);
      const T = await ref.current.calibrate();
      setCalibEnable(true);
    }
  };

  const belowComponent = activeStep === STEP.CALIB_CAMERA && (
    <div style={{ display: "flex", alignItems: "center", paddingLeft: "2rem", whiteSpace: "pre-wrap" }}>
      <Typography>{t("boardRow")}</Typography>
      <span> </span>
      <Input
        style={{ maxWidth: "35px" }}
        inputProps={{ min: 0, style: { textAlign: "center" } }}
        type="number"
        value={config?.chessboardRow.toFixed(0)}
        onChange={(event) => {
          if (config) {
            config.chessboardRow = Number(event.target.value);
            setConfig({ ...config });
          }
        }}
      />
      <span> </span>
      <Typography>{t("boardCol")}</Typography>
      <span> </span>
      <Input
        style={{ maxWidth: "35px" }}
        inputProps={{ min: 0, style: { textAlign: "center" } }}
        type="number"
        value={config?.chessboardCol.toFixed(0)}
        onChange={(event) => {
          if (config) {
            config.chessboardCol = Number(event.target.value);
            setConfig({ ...config });
          }
        }}
      />
      <span> </span>
      <Typography>{t("boardSize")}</Typography>
      <span> </span>
      <Input
        style={{ maxWidth: "60px" }}
        inputProps={{ min: 0, style: { textAlign: "center" } }}
        type="number"
        value={config?.chessboardSize.toFixed(0)}
        endAdornment={<InputAdornment position="end">mm</InputAdornment>}
        onChange={(event) => {
          if (config) {
            config.chessboardSize = Number(event.target.value);
            setConfig({ ...config });
          }
        }}
      />
    </div>
  );

  return (
    <>
      <Stepper {...{ activeStep, setActiveStep, onNext: calibrate, t, calibEnable, belowComponent }}>
        {activeStep === STEP.BEGIN && (
          <Select
            variant="standard"
            value={activeCameraName || ""}
            onChange={(ev) => {
              setActiveCameraName(ev.target.value as string);
              if (activeStep === STEP.BEGIN) {
                setActiveStep(activeStep + 1);
              }
            }}>
            {Object.keys(frame.images || {})
              .sort()
              .map((cameraName, key) => {
                return (
                  <MenuItem value={cameraName} key={cameraName}>
                    {cameraName}
                  </MenuItem>
                );
              })}
          </Select>
        )}
      </Stepper>
      {canvas}
    </>
  );
}
