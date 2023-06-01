import yup from "@plugins/yup-extended";
import { getLidarPointcloud, calibrateGround, calibrateHeading, getConfig, finetune } from "@rpc/http";
import { transformPoints } from "@utils/transform";
import { setIntervalBlock } from "@utils/timer";

import { useState, useEffect, useRef, useCallback } from "react";
import { useTranslation } from "react-i18next";
import React from "react";
import produce from "immer";
import Stepper, { STEP } from "./Stepper";
import LidarSelect from "./LidarSelect";
import LidarSelectMulti from "./LidarSelectMulti";
import * as THREE from "three";
import CalibrateGround, { Ref as CalibrateGroundRef } from "./CalibrateGround";
import Finetune, { Ref as FinetuneRef } from "./Finetune";
import CalibratePosition from "./CalibratePosition";
import { MenuItem, Select, Typography } from "@mui/material";

export default function Calibration() {
  const { t } = useTranslation();

  const [lidarArray, setLidarArray] = useState<LSD.Config["lidar"]>([]);
  useEffect(() => {
    getConfig().then((config) => setLidarArray(config.lidar));
  }, []);

  const [lidarPointcloud, setLidarPointcloud] = useState<Float32Array[]>([]);
  useEffect(() => {
    let paused = false;
    window.onkeyup = (ev: any) => {
      if (ev.key === " ") {
        paused = !paused;
      }
    };

    const exit = setIntervalBlock(async () => {
      if (paused) return;
      try {
        const lpcs = await getLidarPointcloud();
        setLidarPointcloud(lpcs);
      } catch (err) {
        console.log(err);
      }
    }, 1000 / 10);

    return () => {
      window.onkeyup = null;
      exit();
    };
  }, []);

  const [calibEnable, setCalibEnable] = useState<boolean>(false);
  const [coordinate, setCoordinate] = useState("Local");
  const [targetLidarIndex, setTargetLidarIndex] = useState<number>();
  const [referenceLidarIndex, setReferenceLidarIndex] = useState<number[]>([]);
  const [activeStep, setActiveStep] = useState(0);
  const calibRef = useRef<any>(null);

  const calibrate = async () => {
    let T: THREE.Matrix4 | undefined;
    setCalibEnable(false);
    T = await calibRef.current?.calibrate();
    setCalibEnable(true);
    setLidarPointcloud(
      produce((lpcs) => {
        console.log(targetLidarIndex, lpcs, T);
        if (targetLidarIndex !== undefined && lpcs[targetLidarIndex] && T) {
          lpcs[targetLidarIndex] = transformPoints(lpcs[targetLidarIndex], T);
        }
      })
    );
  };

  let stepperHelper = null,
    canvas = null;
  let targetLidarSelect = (
    <LidarSelect
      lidarArray={lidarArray}
      lidarIndex={targetLidarIndex}
      setLidarIndex={setTargetLidarIndex}
      activeStep={activeStep}
      setActiveStep={setActiveStep}
    />
  );
  let referenceLidarSelect = (
    <LidarSelectMulti
      lidarArray={lidarArray}
      lidarIndex={referenceLidarIndex}
      setLidarIndex={setReferenceLidarIndex}
      activeStep={activeStep}
      setActiveStep={setActiveStep}
    />
  );

  if (activeStep === STEP.BEGIN) {
    stepperHelper = targetLidarSelect;
    canvas = (
      <CalibrateGround
        points={lidarPointcloud[targetLidarIndex!]}
        lidarIndex={targetLidarIndex}
        setCalibEnable={setCalibEnable}
        disable={true}
      />
    );
  } else if (activeStep === STEP.CALIB_GROUND) {
    stepperHelper = null;
    canvas = (
      <CalibrateGround
        ref={calibRef}
        points={lidarPointcloud[targetLidarIndex!]}
        lidarIndex={targetLidarIndex}
        setCalibEnable={setCalibEnable}
        disable={false}
      />
    );
  } else if (activeStep === STEP.CALIB_POSITION) {
    stepperHelper = (
      <>
        <Typography variant="body1">{t("useCoordinate")}</Typography>
        <Select
          variant="standard"
          value={coordinate}
          onChange={(ev: any) => {
            setCoordinate(ev.target.value);
          }}
          onFocus={(ev) => {
            ev.target.blur();
          }}>
          <MenuItem value={"Local"}>{t("local")}</MenuItem>
          <MenuItem value={"Global"}>{t("global")}</MenuItem>
        </Select>
        <Typography variant="body1">{t("Coordinate")}</Typography>
      </>
    );
    canvas = (
      <CalibratePosition
        ref={calibRef}
        points={lidarPointcloud[targetLidarIndex!]}
        lidarIndex={targetLidarIndex}
        coordinate={coordinate}
        setCalibEnable={setCalibEnable}
      />
    );
  } else if (activeStep === STEP.FINETUNE || activeStep === STEP.RECALIB) {
    stepperHelper = (
      <>
        {t("target")}: {targetLidarSelect}
        {t("reference")}: {referenceLidarSelect}
      </>
    );
    canvas = (
      <Finetune
        ref={calibRef}
        pointsArray={lidarPointcloud}
        targetLidarIndex={targetLidarIndex}
        referenceLidarIndex={referenceLidarIndex}
        setCalibEnable={setCalibEnable}
      />
    );
  }

  return (
    <>
      <div style={{ position: "relative", width: "100vw", marginTop: "-0.2rem" }}>
        <Stepper {...{ activeStep, setActiveStep, onNext: calibrate, t, calibEnable }}>{stepperHelper}</Stepper>
      </div>
      {canvas}
    </>
  );
}
