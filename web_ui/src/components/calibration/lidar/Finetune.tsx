import React, { useState, useEffect, useMemo } from "react";
import { useImperativeHandle } from "react";
import { Canvas } from "@react-three/fiber";
import * as THREE from "three";

import Pointcloud from "@components/3d/Pointcloud";
import { finetune, getConfig, postConfig } from "@rpc/http";
import { useCtrlKey } from "@hooks/keyboard";
import Helper, { Config, DEFAULT_CONFIG } from "@components/3d/Helper";
import Controls, { CONTROL_TYPE } from "@components/3d/Controls";
import Slider, { ExcludeSlider } from "./SliderCalib";
import { Paper } from "@mui/material";
import { ExcludeBox } from "@components/3d/Object";
import "./index.css";

import ExpandMoreIcon from "@mui/icons-material/ExpandMore";
import {
  Box,
  Container,
  Button,
  Accordion,
  AccordionSummary,
  AccordionDetails,
  Typography,
  Tabs,
  Tab,
  Theme,
} from "@mui/material";

import withStyles from "@mui/styles/withStyles";
import createStyles from "@mui/styles/createStyles";

export type Props = {
  pointsArray: Float32Array[];
  targetLidarIndex?: number;
  referenceLidarIndex: number[];
  setCalibEnable?: any;
};
export type Ref = {
  calibrate: () => Promise<THREE.Matrix4>;
};

function eulerDeg2Rad(euler: any) {
  const ret = euler.slice();
  ret[0] = (euler[0] / 180) * Math.PI;
  ret[1] = (euler[1] / 180) * Math.PI;
  ret[2] = (euler[2] / 180) * Math.PI;
  return ret;
}

function Finetune({ pointsArray, targetLidarIndex, referenceLidarIndex, setCalibEnable }: Props, ref: React.Ref<Ref>) {
  const [excludeConfig, setExcludeConfig] = useState<[number, number, number, number, number, number]>([
    0, 0, 0, 0, 0, 0,
  ]);
  const [rotation, setRotation] = useState<[number, number, number, string]>([0, 0, 0, "XYZ"]);
  const [translation, setTranslation] = useState<[number, number, number]>([0, 0, 0]);

  const [configdata, setConfigdata] = useState<LSD.Config>();
  useEffect(() => {
    getConfig().then((config) => {
      if (config.lidar[0].exclude == undefined) {
        for (var i = 0; i <= config.lidar.length; i++) {
          if (config.lidar[i] != undefined) {
            config.lidar[i].exclude = [0, 0, 0, 0, 0, 0];
          }
        }
      }
      setConfigdata(config);
      if (targetLidarIndex != undefined) {
        setExcludeConfig(config.lidar[targetLidarIndex].exclude as [number, number, number, number, number, number]);
      } else {
        setExcludeConfig([0, 0, 0, 0, 0, 0]);
      }
    });
  }, []);
  useEffect(() => {
    setCalibEnable(targetLidarIndex != undefined);
    if (
      targetLidarIndex != undefined &&
      configdata != undefined &&
      configdata.lidar[targetLidarIndex].exclude != undefined
    ) {
      setExcludeConfig(configdata.lidar[targetLidarIndex].exclude as [number, number, number, number, number, number]);
    } else {
      setExcludeConfig([0, 0, 0, 0, 0, 0]);
    }
  }, [targetLidarIndex]);

  const config = JSON.parse(JSON.stringify(DEFAULT_CONFIG));
  config.polarGrid.visible = false;
  config.grid.visible = true;
  config.grid.divisions = 50;
  config.camera.type = CONTROL_TYPE.ORTHOGRAPHIC;

  useImperativeHandle(ref, () => ({
    calibrate: async () => {
      let T = new THREE.Matrix4();
      const euler = new THREE.Euler();
      const rotationRadius = rotation.slice();
      rotationRadius[0] = ((rotationRadius[0] as number) * Math.PI) / 180;
      rotationRadius[1] = ((rotationRadius[1] as number) * Math.PI) / 180;
      rotationRadius[2] = ((rotationRadius[2] as number) * Math.PI) / 180;
      euler.fromArray(rotationRadius);
      T.makeRotationFromEuler(euler);
      T.setPosition(...(translation as [number, number, number]));
      const transform = T.transpose().toArray();
      if (targetLidarIndex === undefined) {
        throw new Error("No target lidar index!");
      }
      if (configdata === undefined) {
        throw new Error("No config obtained!");
      }

      configdata.lidar[targetLidarIndex].exclude = excludeConfig;
      setConfigdata({ ...configdata });
      try {
        await postConfig(configdata);
        T = await finetune({
          lidarIndex: targetLidarIndex,
          transform,
        });
      } catch (e) {
        console.log(e);
      }

      setTranslation([0, 0, 0]);
      setRotation([0, 0, 0, "XYZ"]);
      return T; // transpose in place
    },
  }));

  return (
    <>
      <div className="calibration-tab">
        <Canvas orthographic={true} style={{ position: "absolute" }}>
          <Helper config={config} />
          <Pointcloud
            size={2.0}
            points={targetLidarIndex !== undefined ? pointsArray[targetLidarIndex] : undefined}
            pointsProps={{
              position: translation,
              rotation: eulerDeg2Rad(rotation),
            }}
            color={"height"}
          />
          {referenceLidarIndex.map((referenceLidar, i) => {
            return (
              <Pointcloud size={2.5} points={referenceLidar !== undefined ? pointsArray[referenceLidar] : undefined} />
            );
          })}
          <ExcludeBox {...{ excludeConfig }} />
        </Canvas>
      </div>

      <div className="finetune-panel">
        <Accordion defaultExpanded={true} style={{ marginBottom: "1px" }}>
          <AccordionSummary expandIcon={<ExpandMoreIcon />} id="finetune-panel0-header">
            {/* <Typography></Typography> */}
          </AccordionSummary>
          <AccordionDetails style={{ paddingTop: "0px" }}>
            <div>
              <Slider {...{ translation, rotation, setTranslation, setRotation }} />
            </div>
          </AccordionDetails>
        </Accordion>
        <Accordion style={{ marginTop: "1px" }}>
          <AccordionSummary expandIcon={<ExpandMoreIcon />} id="finetune-panel1-header">
            {/* <Typography ></Typography> */}
          </AccordionSummary>
          <AccordionDetails style={{ paddingTop: "0px" }}>
            <div>
              <ExcludeSlider {...{ excludeConfig, setExcludeConfig }} />
            </div>
          </AccordionDetails>
        </Accordion>
      </div>
    </>
  );
}

export default React.forwardRef(Finetune);
