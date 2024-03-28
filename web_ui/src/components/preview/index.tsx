import * as React from "react";
import { useState, useEffect, useRef } from "react";
import { useTranslation } from "react-i18next";
import produce from "immer";

import "./index.css";
import {
  getStatus,
  getDetection,
  getConfig,
  getTransform,
  postRoi,
  getRoi,
  restartMapping,
  saveMapping,
  setInitPose,
  rotateGroundConstraint,
} from "@rpc/http";

import Scene, { DEFAULT_CONFIG, Config } from "./Scene";
import Sketchpad, { Ref as SketchpadRef } from "@components/3d/SketchpadROI";
import SketchpadArrow, { Ref as SketchpadArrawRef } from "@components/3d/SketchpadArrow";
import ContextMenu from "@components/general/ContextMenu";
// import { getDetection, getConfig } from "@rpc/http";
import { FPSCounter, Queue, setIntervalBlock } from "@utils/index";
import { transformPoints } from "@utils/transform";
import { useCtrlKey } from "@hooks/keyboard";
import DatGui from "./DatGui";
import ROI from "@components/3d/ROI";
import { ImageList } from "./ImageViewer";
import { useRequest, useCounter, useCreation, useLocalStorageState } from "ahooks";
import Pointcloud from "@components/3d/Pointcloud";
import PointcloudIntensity from "@components/3d/PointcloudIntensity";
import * as THREE from "three";
import Player from "./Player";
import { IObject } from "@proto/detection";
import ObjectDetected from "@components/3d/Object";
import MapSavingProgress from "./MapSavingProgress";
import { CONTROL_TYPE } from "@components/3d/Controls";
import { useNavigate } from "react-router-dom";
import useInfoShow from "./InfoShow";
import AimButton from "@components/general/AimButton";
import FlareIcon from "@mui/icons-material/Flare";

type IPointProp = {
  position: [number, number, number];
  rotation: [number, number, number, string];
};

export type Props = {
  dev?: boolean;
  dt?: number;
};

export default function Preview({ dev = false, dt = 10 }: Props) {
  const { t } = useTranslation();
  const navigate = useNavigate();

  const [dataFpsCounter, displayFpsCounter] = useCreation(() => {
    return [new FPSCounter("data"), new FPSCounter("display")];
  }, []);

  const [pause, setPause] = useState(false);
  const [connect, setConnect] = useState(false);
  const [frameData, setFrameData] = useState<LSD.Detection>();
  const [insExtrinic, setInsExtrinic] = useState<THREE.Matrix4>();
  const [pointProp, setPointProp] = useState<IPointProp>();
  const [invPointProp, setInvPointProp] = useState<IPointProp>();
  const [radarPointcloud, setRadarPointcloud] = useState<IObject[]>([]);
  const [showMapSaving, setShowMapSaving] = useState(false);
  const [info, showMessage] = useInfoShow();

  let onDataSuccess = async (data: LSD.Detection) => {
    if (!connect) {
      setConnect(true);
      onFirstSuccess();
    }
    dataFpsCounter.hit();
    if (data && data.pose) {
      if (drawerState == "drawing") {
        data.pose.x = estPose[0];
        data.pose.y = estPose[1];
        data.pose.z = estPose[2];
        data.pose.roll = estPose[3];
        data.pose.pitch = estPose[4];
        data.pose.heading = estPose[5];
      }
      const pos = new THREE.Vector3(data.pose.x, data.pose.y, data.pose.z);
      const quat = new THREE.Quaternion();
      const scale = new THREE.Vector3(1.0, 1.0, 1.0);
      const euler = new THREE.Euler(
        data.pose.roll * 0.01745329251994,
        data.pose.pitch * 0.01745329251994,
        -data.pose.heading * 0.01745329251994,
        "XYZ"
      );
      quat.setFromEuler(euler);
      quat.set(quat.y, quat.x, quat.z, quat.w);
      const T = new THREE.Matrix4();
      T.compose(pos, quat, scale);
      insExtrinic && T.multiply(insExtrinic);
      const invT = T.clone().invert();

      // invT
      invT.decompose(pos, quat, scale);
      euler.setFromQuaternion(quat);
      setInvPointProp({
        position: pos.toArray() as [number, number, number],
        rotation: euler.toArray() as [number, number, number, string],
      });

      // T
      if (config.camera.type != CONTROL_TYPE.SELF) {
        data.points = transformPoints(data.points, T);
      }
      T.decompose(pos, quat, scale);
      euler.setFromQuaternion(quat);
      setPointProp({
        position: pos.toArray() as [number, number, number],
        rotation: euler.toArray() as [number, number, number, string],
      });
    }
    if (data.header?.timestamp) {
      setFrameData(data);
    }
  };

  let doUpdateConfig = () => {
    setPointProp(undefined);
    setInvPointProp(undefined);
    getStatus().then((status) => {
      if (status?.status == "Paused") {
        setPause(true);
      } else {
        setPause(false);
      }
    });
    getConfig()
      .then((config) => {
        getTransform(config.ins.extrinsic_parameters).then((T) => {
          setInsExtrinic(T);
        });
        setBoardConfig(config);
        setMode("Detect");
        for (let i = 0; i < config.pipeline.length; i++) {
          for (let j = 0; j < config.pipeline[i].length; j++) {
            if (config.pipeline[i][j] == "SLAM") {
              setMode("SLAM");
              break;
            }
          }
        }
      })
      .catch(() => setBoardConfig(undefined));
  };

  useRequest(() => getDetection(config.object.showOnImage, config.pointcloud.sampleStep), {
    pollingInterval: dt,
    manual: false,
    onSuccess: onDataSuccess,
    onError: () => {
      setConnect(false);
      setPointProp(undefined);
      setInvPointProp(undefined);
    },
  });

  window.onkeyup = (ev: KeyboardEvent) => {
    if (ev.key === " ") {
      setPause(!pause);
    } else if (ev.key === "f" && mode == "SLAM") {
      rotateGroundConstraint().then((result) => {
        if (result == "enable") {
          showMessage(t("ActiveFloorContraint"));
        } else {
          showMessage(t("DeactiveFloorContraint"));
        }
      });
    } else if (ev.key === "Escape") {
      setDrawerState("idle");
    }
  };

  let onFirstSuccess = () => {
    doUpdateConfig();
    getRoi().then(({ roi, inside }) => {
      setRoi(roi.map((pt) => ({ x: pt[0], y: pt[1], z: 0 })));
      if (roi.length != 0) {
        setRoiSide(inside ? "in" : "out");
      } else {
        setRoiSide(null);
        setPadState("drawing");
        padRef.current?.start();
      }
    });
  };

  const [roi, setRoi] = useState<{ x: number; y: number; z: number }[]>([]);
  const [roiSide, setRoiSide] = useState<"in" | "out" | null>(null);
  const [padState, setPadState] = useState("idle");
  const [config, setConfig] = useLocalStorageState(
    (import.meta.env.VITE_COMMIT_HASH as string) + "-config",
    DEFAULT_CONFIG
  );
  const [boardConfig, setBoardConfig] = useState<LSD.Config>();
  const [mode, setMode] = useState("Detect");

  const padRef = useRef<SketchpadRef>();
  const ctrlKeyPressed = useCtrlKey();
  let sketchpad = React.useMemo(
    () => (
      <Sketchpad
        enabled={ctrlKeyPressed && mode == "Detect"}
        roi={roi as THREE.Vector3[]}
        state={padState}
        onChange={setRoi}
        onStart={() => {
          setPadState("drawing");
          setRoiSide(null);
        }}
        onEnd={(state: string) => setPadState(state)}
        ref={padRef}
      />
    ),
    [ctrlKeyPressed, roi, padState, padRef, mode]
  );

  let boundary = React.useMemo(
    () => (
      <ROI roi={roi as THREE.Vector3[]} inside={roiSide === "in"} visible={padState === "idle" && roiSide !== null} />
    ),
    [roi, roiSide, padState]
  );

  const [estPose, setEstPose] = useState([0, 0, 0, 0, 0, 0, 0]);
  const [drawerState, setDrawerState] = useState("idle");
  const drawerRef = useRef<SketchpadArrawRef>();
  let rangeDrawer = React.useMemo(
    () => (
      <SketchpadArrow
        enabled={mode == "SLAM"}
        state={drawerState}
        setPose={setEstPose}
        onStart={() => {
          setDrawerState("drawing");
        }}
        onEnd={(state: string, reset: boolean) => {
          setDrawerState(state);
          showMessage(t("setInitPoseOk"));
          if (!reset) setInitPose([estPose[0], estPose[1], estPose[2], -estPose[5], estPose[4], estPose[3]]);
        }}
        ref={drawerRef}
      />
    ),
    [estPose, drawerState, drawerRef, mode]
  );

  let detectMenuItems = {
    redraw: () => {
      padRef.current?.start();
      setRoi([]);
      postRoi([], true);
    },
    roiSegIn: () => {
      setRoiSide("in");
      postRoi(
        roi.map((pt) => [pt.x, pt.y]),
        true
      );
    },
    roiSegOut: () => {
      setRoiSide("out");
      postRoi(
        roi.map((pt) => [pt.x, pt.y]),
        false
      );
    },
  };
  let slamMenuItems: any;
  if (boardConfig && boardConfig.slam.mode == "mapping") {
    slamMenuItems = {
      restartMapping: () => {
        restartMapping().then(() => {
          doUpdateConfig();
        });
      },
      saveMap: () => {
        saveMapping().then((result) => {
          if (result == "ok") {
            setPause(true);
            setShowMapSaving(true);
          }
        });
      },
    };
  } else {
    slamMenuItems = {};
  }
  slamMenuItems["mapEditor"] = () => {
    navigate("/editor", { replace: true });
  };

  let onAimButtomClick = () => {
    showMessage(t("startEstimationInitPose"));
    drawerRef.current?.start();
  };

  let pointView = React.useMemo(() => {
    if (config.pointcloud.color == "intensity") {
      return (
        <PointcloudIntensity
          maxNum={100e4}
          points={frameData?.points}
          {...(config && config.pointcloud)}
          onPointsUpdate={() => displayFpsCounter.hit()}
        />
      );
    } else {
      return (
        <Pointcloud
          maxNum={100e4}
          points={frameData?.points}
          {...(config && config.pointcloud)}
          onPointsUpdate={() => displayFpsCounter.hit()}
        />
      );
    }
  }, [frameData, config]);

  useEffect(() => {
    const rpcs = frameData?.radar;
    let rpc = [] as IObject[];
    if (rpcs != undefined) {
      rpc = Object.values(rpcs).flat();
    }
    setRadarPointcloud(rpc);
  }, [frameData]);

  let radarView = React.useMemo(
    () => (
      <>
        {radarPointcloud.map((object) => (
          <ObjectDetected
            object={object}
            planeVisible={true}
            visible={config.radar.visible}
            showArrow={false}
            colorType={"objectType"}></ObjectDetected>
        ))}
      </>
    ),
    [frameData, config]
  );

  let sceneView = React.useMemo(
    () => (
      <Scene
        frameData={frameData}
        rangeDrawer={rangeDrawer}
        config={config}
        boardConfig={boardConfig}
        props={pointProp}
        invProps={invPointProp}
        pointView={pointView}
        showMessage={showMessage}>
        {sketchpad}
        {boundary}
        {radarView}
      </Scene>
    ),
    [frameData, config, boardConfig, sketchpad, rangeDrawer, boundary]
  );

  let dataGuiView = React.useMemo(() => <DatGui className="config-panel" {...{ config, setConfig, t }} />, [config, t]);

  let imagetView = React.useMemo(() => frameData?.images && <ImageList imageData={frameData?.images} />, [frameData]);

  let contextMenu = React.useMemo(
    () =>
      mode == "Detect" ? (
        <ContextMenu
          enabled={ctrlKeyPressed && padState === "idle"}
          menuItems={detectMenuItems}
          onStart={
            ctrlKeyPressed && padState === "drawing"
              ? () => padRef.current?.stop() // be careful about old data
              : undefined
          }
        />
      ) : (
        <ContextMenu enabled={ctrlKeyPressed} menuItems={slamMenuItems} />
      ),
    [ctrlKeyPressed, padState, padRef, mode]
  );

  let player = React.useMemo(
    () =>
      boardConfig &&
      boardConfig.input.mode == "offline" && (
        <Player
          className="play"
          currentFile={boardConfig.input.data_path}
          pause={pause}
          setPause={setPause}
          onPlay={doUpdateConfig}
        />
      ),
    [boardConfig, pause, setPause]
  );

  let infoShow = React.useMemo(() => info, [info, showMessage]);

  // useWhyDidYouUpdate("why update", [cnt, frameData?.points, config]);
  let insView = React.useMemo(
    () => (
      <div>
        <div style={{ color: "white", position: "fixed", fontSize: "14px", bottom: 0, right: 0 }}>
          {boardConfig && boardConfig.input.mode == "offline" && `${formatDate(frameData?.header?.timestamp)} `}
          {`FPS:${format(dataFpsCounter.fps, 2)} / ${format(frameData?.header?.fps, 2)}`}
        </div>
        <div style={{ color: "white", position: "fixed", fontSize: "14px", bottom: 0, left: 0 }}>
          {`Lat:${format(frameData?.pose?.latitude, 6)} Lon:${format(frameData?.pose?.longitude, 6)}
              Yaw:${format(frameData?.pose?.heading, 2)} Status:${frameData?.pose?.state}(${frameData?.pose?.status})`}
        </div>
      </div>
    ),
    [dataFpsCounter.fps, frameData]
  );

  let mapSavingProgress = React.useMemo(
    () => showMapSaving && <MapSavingProgress setShowMapSaving={setShowMapSaving} setPause={setPause} />,
    [showMapSaving, setShowMapSaving, setPause]
  );

  let aimButtom = React.useMemo(
    () =>
      mode == "SLAM" &&
      boardConfig &&
      boardConfig.slam.mode == "localization" && (
        <AimButton onClick={onAimButtomClick} disable={drawerState == "drawing"}>
          <FlareIcon />
        </AimButton>
      ),
    [mode, boardConfig, drawerState]
  );

  return (
    <div style={{ position: "relative" }}>
      <div className="scene">{sceneView}</div>
      {dataGuiView}
      {imagetView}
      {contextMenu}
      {insView}
      {player}
      {infoShow}
      {mapSavingProgress}
      {aimButtom}
    </div>
  );
}

function format(x: number | null | undefined, n: number) {
  if (x == undefined || x == null) {
    return "NaN";
  }
  let s = x.toFixed(n);
  return s.padStart(5, "0");
}

function formatDate(x: number | Long.Long | null | undefined) {
  if (x == undefined || x == null) {
    return "";
  }
  const date = new Date((x as number) / 1000);

  const mm = date.getMonth() + 1; // getMonth() is zero-based
  const dd = date.getDate();
  const h = date.getHours();
  const m = date.getMinutes();
  const s = date.getSeconds();
  const ms = date.getMilliseconds();

  return (
    [date.getFullYear(), (mm > 9 ? "" : "0") + mm, (dd > 9 ? "" : "0") + dd].join("-") +
    " " +
    [(h > 9 ? "" : "0") + h, (m > 9 ? "" : "0") + m, (s > 9 ? "" : "0") + s].join(":") +
    ("." + ms.toString().padStart(3, "0"))
  );
}
