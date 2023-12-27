import React, { useState, useEffect, useRef } from "react";
import { useTranslation } from "react-i18next";
import { startPlayer } from "@rpc/http";
import Stepper, { STEP } from "./Stepper";
import Calib from "./Calib";
import CalibBegin from "./CalibBegin";

export default function InsCalibrator({}) {
  const { t } = useTranslation();

  const [calibEnable, setCalibEnable] = useState<boolean>(false);
  const [statusString, setStatusString] = useState<string>("");

  useEffect(() => {
    startPlayer();
  }, []);

  const [activeStep, setActiveStep] = useState(0);
  let canvas = null;
  const ref = useRef<any>();
  if (activeStep === STEP.BEGIN) {
    canvas = <CalibBegin ref={ref} setCalibEnable={setCalibEnable} setStatusString={setStatusString} t={t} />;
  } else if (activeStep === STEP.CALIB) {
    canvas = (
      <Calib ref={ref} setCalibEnable={setCalibEnable} setStatusString={setStatusString} t={t} isFinetune={false} />
    );
  } else {
    canvas = (
      <Calib ref={ref} setCalibEnable={setCalibEnable} setStatusString={setStatusString} t={t} isFinetune={true} />
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
      <Stepper {...{ activeStep, setActiveStep, onNext: calibrate, t, calibEnable, statusString }}></Stepper>
      {canvas}
    </>
  );
}
