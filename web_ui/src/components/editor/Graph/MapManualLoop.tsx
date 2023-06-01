import { Box, Button, Dialog, DialogActions, ListItemIcon, ListItemText, MenuItem } from "@mui/material";
import AllInclusiveIcon from "@mui/icons-material/AllInclusive";
import React, { useEffect, useRef, useState } from "react";
import "../index.css";
import { useTranslation } from "react-i18next";
import { Canvas } from "@react-three/fiber";
import { addMapEdge, arrayToTransform, getMapVertex, getVertexData, KeyframeAlign } from "@rpc/http";
import PointVertexView from "../common/PointVertexView";
import Controls from "@components/3d/Controls";
import SliderBar from "./SliderBar";
import * as THREE from "three";
import Loading from "@components/general/Loading";

export interface Props {
  onEvent: any;
  onFinish: any;
  selectVertex: string[];
  selectColor: number[];
}

function eulerDeg2Rad(euler: any) {
  const ret = euler.slice();
  ret[0] = (euler[0] / 180) * Math.PI;
  ret[1] = (euler[1] / 180) * Math.PI;
  ret[2] = (euler[2] / 180) * Math.PI;
  return ret;
}

export default function MapManualLoop({ onEvent, onFinish, selectVertex, selectColor }: Props) {
  const { t } = useTranslation();
  const paneRef = useRef<any>();
  const [open, setOpen] = useState(false);
  const [isLoading, setIsLoading] = useState<boolean>(false);

  const [point0, setPoint0] = useState<Float32Array>();
  const [point1, setPoint1] = useState<Float32Array>();
  const [vertex, setVertex] = useState<LSD.MapVertex>();
  const [rotation, setRotation] = useState<[number, number, number, string]>([0, 0, 0, "XYZ"]);
  const [translation, setTranslation] = useState<[number, number, number]>([0, 0, 0]);
  const [estimate, setEstimate] = useState<THREE.Matrix4>(new THREE.Matrix4().identity());

  const onClickMenu = () => {
    setOpen(true);
    getMapVertex().then((data) => {
      setVertex(data);
    });
  };

  useEffect(() => {
    if (vertex && vertex.hasOwnProperty(selectVertex[0]) && vertex.hasOwnProperty(selectVertex[1])) {
      if (selectColor[0] == 0 || selectColor[1] == 0) {
        return;
      }
      const T = arrayToTransform(vertex[selectVertex[0]]).invert().multiply(arrayToTransform(vertex[selectVertex[1]]));
      getVertexData(selectVertex[0], "p").then((kf) => {
        setPoint0(kf.points);
      });
      getVertexData(selectVertex[1], "p").then((kf) => {
        setPoint1(kf.points);
      });
      setEstimate(T);
    }
  }, [vertex, selectVertex, selectColor]);

  const getRelativeTransform = () => {
    let delta = new THREE.Matrix4();
    const euler = new THREE.Euler();
    const rotationRadius = rotation.slice();
    rotationRadius[0] = ((rotationRadius[0] as number) * Math.PI) / 180;
    rotationRadius[1] = ((rotationRadius[1] as number) * Math.PI) / 180;
    rotationRadius[2] = ((rotationRadius[2] as number) * Math.PI) / 180;
    euler.fromArray(rotationRadius);
    delta.makeRotationFromEuler(euler);
    delta.setPosition(...(translation as [number, number, number]));
    return delta.multiply(estimate);
  };

  const resetMenu = () => {
    onFinish();
    setOpen(false);
    setVertex({});
    setPoint0(undefined);
    setPoint1(undefined);
    setRotation([0, 0, 0, "XYZ"]);
    setTranslation([0, 0, 0]);
    setEstimate(new THREE.Matrix4().identity());
  };

  const handleClose = (confirm: boolean) => {
    if (selectColor[0] == 0 || selectColor[1] == 0) {
      resetMenu();
      return;
    }
    if (confirm && vertex && vertex.hasOwnProperty(selectVertex[0]) && vertex.hasOwnProperty(selectVertex[1])) {
      const relative = getRelativeTransform();
      addMapEdge(selectVertex[0], selectVertex[1], relative)
        .then(() => {
          onEvent("ClearSelection");
          onEvent("OptimizeMap", false);
        })
        .finally(() => {
          resetMenu();
        });
    } else {
      resetMenu();
    }
  };

  const onAutoAlign = () => {
    if (vertex && vertex.hasOwnProperty(selectVertex[0]) && vertex.hasOwnProperty(selectVertex[1])) {
      if (selectColor[0] == 0 || selectColor[1] == 0) {
        return;
      }
      setIsLoading(true);
      const guess = getRelativeTransform();
      KeyframeAlign(selectVertex[1], selectVertex[0], guess)
        .then((T) => {
          setEstimate(T);
          setRotation([0, 0, 0, "XYZ"]);
          setTranslation([0, 0, 0]);
          paneRef.current.onClear();
        })
        .finally(() => {
          setIsLoading(false);
        });
    }
  };

  const onReset = () => {
    if (vertex && vertex.hasOwnProperty(selectVertex[0]) && vertex.hasOwnProperty(selectVertex[1])) {
      const T = arrayToTransform(vertex[selectVertex[0]]).invert().multiply(arrayToTransform(vertex[selectVertex[1]]));
      setEstimate(T);
      setRotation([0, 0, 0, "XYZ"]);
      setTranslation([0, 0, 0]);
      paneRef.current.onClear();
    }
  };

  return (
    <div>
      <MenuItem onClick={onClickMenu}>
        <ListItemIcon>
          <AllInclusiveIcon fontSize="small" />
        </ListItemIcon>
        <ListItemText>{t("MapManualLoop")}</ListItemText>
      </MenuItem>
      <Dialog maxWidth="lg" open={open}>
        {isLoading && <Loading />}
        <div className="align-scene">
          <Canvas orthographic={true}>
            <gridHelper args={[100, 10, 0x111111, 0x111111]} visible={true} rotation-x={Math.PI / 2} />
            <Controls type={4} position={[0, 0, 0]} />
            {vertex && vertex.hasOwnProperty(selectVertex[0]) && vertex.hasOwnProperty(selectVertex[1]) && (
              <>
                <PointVertexView
                  id={selectVertex[0]}
                  points={point0}
                  color={0xff00ff}
                  size={3}
                  config={{}}
                  isSelect={true}
                  visible={true}
                  vertexPose={new THREE.Matrix4().identity()}
                  insExtrinic={new THREE.Matrix4().identity()}
                />
                <group
                  {...{
                    position: translation,
                    rotation: eulerDeg2Rad(rotation),
                  }}>
                  <PointVertexView
                    id={selectVertex[1]}
                    points={point1}
                    color={0xffff00}
                    size={3}
                    config={{}}
                    isSelect={true}
                    visible={true}
                    vertexPose={estimate}
                    insExtrinic={new THREE.Matrix4().identity()}
                  />
                </group>
              </>
            )}
          </Canvas>
        </div>
        <div className="editor-finetune-panel">
          <SliderBar {...{ translation, rotation, setTranslation, setRotation }} ref={paneRef} />
        </div>
        <DialogActions style={{ padding: "4px", backgroundColor: "#111" }}>
          <Button onClick={() => onAutoAlign()} color="primary" variant="contained">
            {t("AutoAlign")}
          </Button>
          <Button onClick={() => onReset()} color="primary" variant="contained">
            {t("Reset")}
          </Button>
          <Box flexGrow={1} />
          <Button onClick={() => handleClose(false)} color="primary" variant="contained">
            {t("no")}
          </Button>
          <Button onClick={() => handleClose(true)} color="primary" variant="contained">
            {t("yes")}
          </Button>
        </DialogActions>
      </Dialog>
    </div>
  );
}
