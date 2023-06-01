import React, { useState, useRef, useEffect } from "react";
import { useTranslation } from "react-i18next";

import { getSourceData } from "@rpc/http";
import Stepper, { STEP } from "./Stepper";
import Calib from "./Calib";
import CalibCheck from "./CalibCheck";
import { Checkbox, Input, ListItemText, MenuItem, Select, Typography, SelectChangeEvent } from "@mui/material";

import { useInterval, useKeyPress, useLockFn, useSetState, useToggle } from "ahooks";

type Frame = {
  images: LSD.Detection["images"];
};

export default function PanoramaCameraCalibrator({}) {
  const { t } = useTranslation();

  const [calibEnable, setCalibEnable] = useState<boolean>(false);
  const [activeCameraName, setActiveCameraName] = useState<string[]>([]);
  const [frame, setFrame] = useSetState<Frame>({ images: {} });
  const [activeStep, setActiveStep] = useState(0);
  const [slideCameraIndex, setSlideCameraIndex] = useState(1);
  const [paused, { toggle }] = useToggle(false);

  useKeyPress("space", (ev) => {
    toggle();
  });

  useInterval(
    useLockFn(async () => {
      if (!paused && activeStep != STEP.CHECK) {
        const frameData = await getSourceData(false);
        setFrame({ ...frameData });
      }
    }),
    100
  );

  useEffect(() => {
    if (activeStep == STEP.CALIB_CAMERA) {
      setSlideCameraIndex(1);
    }
  }, [activeStep]);

  let canvas = null;
  const ref = useRef<any>();
  if (activeStep === STEP.BEGIN) {
    if (paused) {
      toggle(false);
    }
    canvas = (
      <Calib
        images={frame.images}
        cameraName={activeCameraName}
        paused={false}
        setCalibEnable={setCalibEnable}
        slideCameraIndex={slideCameraIndex}
        setSlideCameraIndex={setSlideCameraIndex}
      />
    );
  } else if (activeStep === STEP.CALIB_CAMERA) {
    canvas = (
      <Calib
        ref={ref}
        images={frame.images}
        cameraName={activeCameraName}
        paused={paused}
        setCalibEnable={setCalibEnable}
        slideCameraIndex={slideCameraIndex}
        setSlideCameraIndex={setSlideCameraIndex}
      />
    );
  } else {
    ref.current = null;
    canvas = <CalibCheck ref={ref} setCalibEnable={setCalibEnable} />;
  }

  const calibrate = async () => {
    if (ref.current) {
      setCalibEnable(false);
      const T = await ref.current.calibrate();
      setCalibEnable(true);
    }
  };

  const handleActiveCameraChange = (event: SelectChangeEvent<unknown>) => {
    setActiveCameraName(event.target.value as string[]);
  };

  const belowComponent = activeStep === STEP.CALIB_CAMERA && slideCameraIndex < activeCameraName.length && (
    <div style={{ display: "flex", alignItems: "center", paddingLeft: "2rem", whiteSpace: "pre-wrap" }}>
      <Typography>{t("CurrentStitchCamera")}</Typography>
      <span> </span>
      {activeCameraName.length >= 2 && (
        <Typography color="secondary">{t("Camera") + "-" + activeCameraName[slideCameraIndex - 1]}</Typography>
      )}
      <span> </span>
      {activeCameraName.length >= 2 && (
        <Typography color="secondary">{t("Camera") + "-" + activeCameraName[slideCameraIndex]}</Typography>
      )}
    </div>
  );

  return (
    <>
      <Stepper {...{ activeStep, setActiveStep, onNext: calibrate, t, calibEnable, belowComponent }}>
        {activeStep === STEP.BEGIN && (
          <Select
            multiple
            value={activeCameraName}
            onChange={handleActiveCameraChange}
            variant="standard"
            renderValue={(selected: any) => {
              let render = "";
              for (let i = 0; i < selected.length; i++) {
                render = render + t("Camera") + ": " + selected[i];
                if (i != selected.length - 1) {
                  render = render + ", ";
                }
              }
              return render;
            }}>
            {Object.keys(frame.images)
              .sort()
              .map((cameraName) => {
                return (
                  <MenuItem value={cameraName} key={cameraName}>
                    <Checkbox color="primary" checked={activeCameraName.indexOf(cameraName) > -1} />
                    <ListItemText primary={t("Camera") + ":" + cameraName} />
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
