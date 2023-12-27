import React, { useState, useEffect, useMemo, useRef } from "react";
import { useTranslation } from "react-i18next";

import CalibPnP from "./CalibPnP";
import { getDetection } from "@rpc/http";
import Stepper, { STEP } from "./Stepper";
import { MenuItem, Select } from "@mui/material";
import CalibFinetune from "./CalibFinetune";
import { useCounter, useInterval, useKeyPress, useLockFn, useMount, useSetState, useToggle } from "ahooks";
import CalibCheck from "./CalibCheck";

type Frame = {
  timestamp?: number;
  imageUrl?: string;
  points?: LSD.Detection["points"];
  images?: LSD.Detection["images"];
};

export default function LidarCameraCalibrator({}) {
  const { t } = useTranslation();

  const [calibEnable, setCalibEnable] = useState<boolean>(false);
  const [activeCameraName, setActiveCameraName] = useState<string>();
  const [frame, setFrame] = useSetState<Frame>({});

  const [paused, { toggle }] = useToggle(false);
  useKeyPress("space", (ev) => {
    toggle();
  });
  const [cnt, { inc }] = useCounter(0);
  useInterval(
    useLockFn(async () => {
      if (!paused) {
        const frameData = await getDetection();
        let url;
        if (activeCameraName) {
          const blob = new Blob([frameData.images[activeCameraName].buffer], {
            type: "application/octet-stream",
          });
          url = URL.createObjectURL(blob); //possibly `webkitURL` or another vendor prefix for old browsers.
        }
        setFrame({
          timestamp: cnt,
          imageUrl: url,
          ...frameData,
        });
        inc();
      }
    }),
    100
  );

  useEffect(() => {
    if (activeCameraName && frame.images && frame.images.hasOwnProperty(activeCameraName)) {
      const blob = new Blob([frame.images[activeCameraName].buffer], {
        type: "application/octet-stream",
      });
      const url = URL.createObjectURL(blob); //possibly `webkitURL` or another vendor prefix for old browsers.
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
      <CalibPnP
        points={frame.points}
        timestamp={frame.timestamp}
        imageUrl={frame.imageUrl}
        images={frame.images}
        cameraName={activeCameraName}
        paused={false}
        setCalibEnable={setCalibEnable}
      />
    );
  } else if (activeStep === STEP.CALIB_LIDAR_CAMERA) {
    canvas = (
      <CalibPnP
        ref={ref}
        points={frame.points}
        timestamp={frame.timestamp}
        imageUrl={frame.imageUrl}
        images={frame.images}
        cameraName={activeCameraName}
        paused={paused}
        setCalibEnable={setCalibEnable}
      />
    );
  } else if (activeStep === STEP.FINETUNE) {
    canvas = (
      <CalibFinetune
        ref={ref}
        timestamp={frame.timestamp}
        points={frame.points}
        imageUrl={frame.imageUrl}
        cameraName={activeCameraName}
        setCalibEnable={setCalibEnable}
      />
    );
  } else {
    ref.current = null;
    canvas = (
      <CalibCheck
        points={frame.points}
        imageUrl={frame.imageUrl}
        cameraName={activeCameraName}
        setCalibEnable={setCalibEnable}
      />
    );
  }

  const calibrate = async () => {
    if (ref.current) {
      setCalibEnable(false);
      const T = await ref.current.calibrate();
      setCalibEnable(true);
    }
  };

  return (
    <>
      <Stepper {...{ activeStep, setActiveStep, onNext: calibrate, t, calibEnable }}>
        {activeStep === STEP.BEGIN && (
          <Select
            variant="standard"
            value={activeCameraName || ""}
            onChange={(ev) => {
              setActiveCameraName(ev.target.value as string);
              if (activeStep === STEP.BEGIN) {
                setActiveStep(activeStep + 1);
              }
            }}
            // onClose={loseFocus}
          >
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
