import React, { useState, useEffect, useMemo } from "react";
import { useImperativeHandle } from "react";
import { Canvas } from "@react-three/fiber";
import * as THREE from "three";

import Pointcloud from "@components/3d/Pointcloud";
import { getConfig, postConfig } from "@rpc/http";

import Helper, { DEFAULT_CONFIG } from "@components/3d/Helper";
import { CONTROL_TYPE } from "@components/3d/Controls";
import Slider from "./SliderCalib";
import { IObject } from "@proto/detection";
import Object from "@components/3d/Object";

import ExpandMoreIcon from "@mui/icons-material/ExpandMore";
import { getTransformFromVec, getVecFromTransform } from "@utils/transform";
import { Accordion, AccordionSummary, AccordionDetails, Typography } from "@mui/material";
import "./index.css";

export type Props = {
  points: Float32Array | undefined;
  radarPoints: IObject[] | undefined;
  radarIndex?: number;
  setCalibEnable?: any;
};
export type Ref = {
  calibrate: () => Promise<THREE.Matrix4>;
};

type IObjectProp = {
  position: [number, number, number];
  rotation: [number, number, number, string];
};

function eulerDeg2Rad(euler: any) {
  const ret = euler.slice();
  ret[0] = (euler[0] / 180) * Math.PI;
  ret[1] = (euler[1] / 180) * Math.PI;
  ret[2] = (euler[2] / 180) * Math.PI;
  return ret;
}

function Finetune({ points, radarPoints, radarIndex, setCalibEnable }: Props, ref: React.Ref<Ref>) {
  const [rotation, setRotation] = useState<[number, number, number, string]>([0, 0, 0, "XYZ"]);
  const [translation, setTranslation] = useState<[number, number, number]>([0, 0, 0]);
  const [objectProp, setObjectProp] = useState<IObjectProp>();
  const [configdata, setConfigdata] = useState<LSD.Config>();

  useEffect(() => {
    getConfig().then((conf) => setConfigdata(conf));
  }, []);

  useEffect(() => {
    setCalibEnable(radarIndex != undefined);
  }, [radarIndex]);

  const config = JSON.parse(JSON.stringify(DEFAULT_CONFIG));
  config.polarGrid.visible = false;
  config.grid.visible = true;
  config.grid.divisions = 50;

  config.camera.type = CONTROL_TYPE.ORTHOGRAPHIC;
  // console.log(radarPoints)

  useImperativeHandle(ref, () => ({
    calibrate: async () => {
      let T = new THREE.Matrix4();
      if (radarIndex === undefined) {
        throw new Error("No target lidar index!");
      }
      if (configdata != undefined && radarIndex != undefined) {
        let matrixConfig = getTransformFromVec(configdata.radar[radarIndex].extrinsic_parameters);
        let matrixObjectProp = getTransformFromVec(translation.concat(rotation.slice(0, 3) as number[]));
        T.multiplyMatrices(matrixObjectProp, matrixConfig);
        configdata.radar[radarIndex].extrinsic_parameters = getVecFromTransform(T);
        setConfigdata({ ...configdata });
        postConfig(configdata);

        let _rotasion = configdata.radar[radarIndex].extrinsic_parameters.slice(3, 6) as any[];
        _rotasion.push("XYZ");
        setObjectProp({
          position: configdata.radar[radarIndex].extrinsic_parameters.slice(0, 3) as [number, number, number],
          rotation: _rotasion as [number, number, number, string],
        });
      }
      setTranslation([0, 0, 0]);
      setRotation([0, 0, 0, "XYZ"]);
      return T; // transpose in place
    },
  }));

  useEffect(() => {
    setObjectProp({
      position: translation,
      rotation: eulerDeg2Rad(rotation),
    });
  }, [translation, rotation]);

  // console.log(radarPoints)
  let radarView = useMemo(
    () => (
      <Canvas orthographic={true} style={{ position: "absolute" }}>
        <Helper config={config} />
        <Pointcloud points={points} color={"height"} size={1.5} />
        <group {...objectProp}>
          {radarPoints?.map((object) => (
            <Object
              object={object}
              planeVisible={true}
              visible={true}
              showArrow={false}
              colorType={"objectType"}></Object>
          ))}
        </group>
      </Canvas>
    ),
    [radarPoints, points, objectProp, config]
  );

  return (
    <>
      <div className="calibration-tab">{radarView}</div>

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
      </div>
    </>
  );
}

export default React.forwardRef(Finetune);
