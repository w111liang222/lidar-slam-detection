import { useState, useEffect, useRef } from "react";
import React from "react";
import produce from "immer";
import * as THREE from "three";
import { useRequest } from "ahooks";

import { useTranslation } from "react-i18next";
import Stepper, { STEP } from "./Stepper";
import RadarSlider from "./RadarSlider";
import Finetune from "./Finetune";
import RadarSelect from "./RadarSelect";
import { IObject } from "@proto/detection";
import { getConfig, getDetection } from "@rpc/http";

export default function LidarRadarCalibration(dt = 100) {
  const { t } = useTranslation();
  const [lidarPointcloud, setLidarPointcloud] = useState<Float32Array>();
  const [radarPointcloud, setRadarPointcloud] = useState<IObject[]>();
  const { data, error, loading, run, cancel } = useRequest(() => getDetection(false), {
    pollingInterval: dt,
    manual: true,
  });

  const [radarArray, setRadarArray] = useState<LSD.Config["radar"]>([]);
  useEffect(() => {
    getConfig().then((config) => setRadarArray(config.radar));
  }, []);

  const [pause, setPause] = useState(false);
  window.onkeyup = (ev: KeyboardEvent) => {
    if (ev.key === " ") {
      setPause(!pause);
    }
  };
  useEffect(() => {
    pause ? cancel() : run();
  }, [pause]);

  const [calibEnable, setCalibEnable] = useState<boolean>(false);
  const [targetRadarIndex, setTargetRadarIndex] = useState<number>();
  const [activeStep, setActiveStep] = useState(0);
  const calibRef = useRef<any>(null);

  useEffect(() => {
    const lpcs = data?.points;
    const rpcs = data?.radar;
    let rpc = [];
    let rpcn = rpcs ? Object.keys(rpcs) : undefined;
    if (rpcs != undefined) {
      for (let rp of rpcn || []) {
        // for (let i = 0; i < rpcs[rp].length; i++) {
        //   if (rpcs[rp][i].type !=0){
        //     console.log(rpcs[rp])
        //   }
        // }
        rpc.push(rpcs[rp]);
      }
    }
    setLidarPointcloud(lpcs);
    if (targetRadarIndex != undefined) {
      setRadarPointcloud(rpc[targetRadarIndex]);
    }
  }, [data, targetRadarIndex]);

  const calibrate = async () => {
    let T: THREE.Matrix4 | undefined;
    setCalibEnable(false);
    T = await calibRef.current?.calibrate();
    setCalibEnable(true);
    setRadarPointcloud(
      produce((rpcs) => {
        console.log(targetRadarIndex, rpcs, T);
        if (targetRadarIndex !== undefined && rpcs && T) {
          // rpcs[targetRadarIndex] = transformPoints(rpcs[targetRadarIndex], T);
        }
      })
    );
  };

  let stepperHelper = null,
    canvas = null;
  let targetRadarSelect = (
    <RadarSlider
      radarArray={radarArray}
      radarIndex={targetRadarIndex}
      setRadarIndex={setTargetRadarIndex}
      setRadarPointcloud={setRadarPointcloud}
      activeStep={activeStep}
      setActiveStep={setActiveStep}
    />
  );

  if (activeStep === STEP.BEGIN) {
    stepperHelper = targetRadarSelect;
    canvas = (
      <RadarSelect
        ref={calibRef}
        points={lidarPointcloud}
        radarPoints={radarPointcloud}
        radarIndex={targetRadarIndex}
        setCalibEnable={setCalibEnable}
      />
    );
  } else if (activeStep === STEP.FINETUNE || activeStep === STEP.RECALIB) {
    stepperHelper = (
      <>
        {t("target")}: {targetRadarSelect}
      </>
    );
    canvas = (
      <Finetune
        ref={calibRef}
        points={lidarPointcloud}
        radarPoints={radarPointcloud}
        radarIndex={targetRadarIndex}
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
