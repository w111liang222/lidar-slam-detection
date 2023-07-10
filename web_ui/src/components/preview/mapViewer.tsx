import React, { useImperativeHandle, useRef } from "react";
import { useEffect, useState, useMemo } from "react";
import * as THREE from "three";

import Pointcloud from "@components/3d/Pointcloud";
import PointcloudIntensity from "@components/3d/PointcloudIntensity";
import { getMapVertex, getMapStatus, getVertexData, arrayToTransform } from "@rpc/http";
import { useInterval, useLockFn, useRequest } from "ahooks";
import { DEFAULT_CONFIG, Config } from "./Scene";
import TrajectoryView from "./CarTrajectory";
import { useTranslation } from "react-i18next";

export interface Props {
  config: Config;
  boardConfig?: LSD.Config;
  showMessage: any;
}

export default function MapViewer({ config = DEFAULT_CONFIG, showMessage, boardConfig }: Props) {
  const { t } = useTranslation();
  const [dispVertex, setDispVertex] = useState<string[]>([]);
  const [component, setComponent] = useState<any[]>([]);

  const [trajectory, setTrajectory] = useState<THREE.Vector3[]>([]);

  const { data } = useRequest(getMapVertex, {
    pollingInterval: 1000,
    onError: () => {
      setDispVertex([]);
      setComponent([]);
      setTrajectory([]);
    },
  });

  useInterval(
    useLockFn(async () => {
      if (!data) {
        return;
      }
      let vertexChanged = false;
      let ids = Object.keys(data);
      if (ids.length < dispVertex.length) {
        setDispVertex(dispVertex.slice(0, ids.length));
        setComponent(component.slice(1, ids.length + 1));
        setTrajectory(trajectory.slice(0, ids.length));
        return;
      }

      for (let i = 0; i < dispVertex.length; i++) {
        let key = ids[i];
        if (dispVertex[i] != key) {
          vertexChanged = true;
          dispVertex[i] = key;
          let visible = parseInt(key) % config.map.step == 0 ? true : false;
          component[i + 1] = (
            <MapVertexViewer
              key={key}
              id={key}
              config={config.map}
              visible={visible}
              vertexPose={arrayToTransform(data[key])}
            />
          );
          trajectory[i] = new THREE.Vector3(data[key][3], data[key][7], data[key][11]);
        }
      }

      let appendNum = Math.min(10, ids.length - dispVertex.length);
      if (appendNum > 0) {
        vertexChanged = true;
        let offset = dispVertex.length;
        for (let i = offset; i < appendNum + offset; i++) {
          let key = ids[i];
          let visible = parseInt(key) % config.map.step == 0 ? true : false;
          dispVertex[i] = key;
          component[i + 1] = (
            <MapVertexViewer
              key={key}
              id={key}
              config={config.map}
              visible={visible}
              vertexPose={arrayToTransform(data[key])}
            />
          );
          trajectory[i] = new THREE.Vector3(data[key][3], data[key][7], data[key][11]);
        }
      }
      if (vertexChanged) {
        component[0] = (
          <TrajectoryView
            key={-1}
            trajectory={trajectory}
            disable={boardConfig && boardConfig.slam.mode == "localization"}
          />
        );
        setDispVertex(dispVertex);
        setComponent(component);
        setTrajectory([...trajectory]);
      }
    }),
    200
  );

  // update odoms every 3s
  useInterval(
    useLockFn(async () => {
      if (!data) {
        return;
      }
      getMapStatus().then((status) => {
        if (status.loop_detected) {
          showMessage(t("loopDetected"), 5000);
        }
      });
      for (let i = 1; i < component.length; i++) {
        let key = dispVertex[i - 1];
        let visible = parseInt(key) % config.map.step == 0 ? true : false;
        component[i] = (
          <MapVertexViewer
            key={key}
            id={key}
            config={config.map}
            visible={visible}
            vertexPose={arrayToTransform(data[key])}
          />
        );
        trajectory[i - 1] = new THREE.Vector3(data[key][3], data[key][7], data[key][11]);
      }
      component[0] = (
        <TrajectoryView
          key={-1}
          trajectory={trajectory}
          disable={boardConfig && boardConfig.slam.mode == "localization"}
        />
      );
      setTrajectory([...trajectory]);
    }),
    3000
  );

  useEffect(() => {
    if (!data) {
      return;
    }
    for (let i = 1; i < component.length; i++) {
      let key = dispVertex[i - 1];
      let visible = parseInt(key) % config.map.step == 0 ? true : false;
      component[i] = (
        <MapVertexViewer
          key={key}
          id={key}
          config={config.map}
          visible={visible}
          vertexPose={arrayToTransform(data[key])}
        />
      );
    }
  }, [config]);

  return <>{component}</>;
}

type IPointProp = {
  position: [number, number, number];
  rotation: [number, number, number, string];
};

type IProps = {
  id: string;
  config?: any;
  visible?: boolean;
  vertexPose?: THREE.Matrix4;
};

function MapVertexViewer({ id, config, visible, vertexPose }: IProps) {
  const [vetexId, setVetexId] = useState<string>();
  const [points, setPoints] = useState<Float32Array>();

  useEffect(() => {
    if (id != vetexId && visible) {
      getVertexData(id).then((kf) => {
        setPoints(kf.points);
      });
      setVetexId(id);
    }
  }, [id]);

  useEffect(() => {
    if (visible && (points == undefined || points.length == 0 || id != vetexId)) {
      getVertexData(id).then((kf) => {
        setPoints(kf.points);
      });
      setVetexId(id);
    }
  }, [visible]);

  const [pointProp, setPointProp] = useState<IPointProp>();

  useEffect(() => {
    if (vertexPose) {
      const pos = new THREE.Vector3();
      const quat = new THREE.Quaternion();
      const scale = new THREE.Vector3();
      vertexPose.decompose(pos, quat, scale);
      const euler = new THREE.Euler();
      euler.setFromQuaternion(quat);
      if (pointProp) {
        let rotation = pointProp["rotation"];
        let translation = pointProp["position"];
        let eulerDiff =
          Math.abs((rotation[0] as number) - euler.x) +
          Math.abs((rotation[1] as number) - euler.y) +
          Math.abs((rotation[2] as number) - euler.z);
        let posDiff =
          Math.abs((translation[0] as number) - pos.x) +
          Math.abs((translation[1] as number) - pos.y) +
          Math.abs((translation[2] as number) - pos.z);
        if (eulerDiff < 1e-1 && posDiff < 1e-1) {
          return;
        }
      }
      setPointProp({
        position: pos.toArray() as [number, number, number],
        rotation: euler.toArray() as [number, number, number, string],
      });
    }
  }, [vertexPose]);

  return (
    <>
      {config.color == "intensity" ? (
        <PointcloudIntensity
          maxNum={points ? points.length / 4 : 0}
          points={points}
          {...config}
          clip={{ min: -10000.0, max: 10000.0 }}
          maxIntensity={0.75}
          size={1.5}
          frustumCulled={config.frustumCulled}
          visible={config.visible && visible}
          pointsProps={pointProp}
        />
      ) : config.color != "rgb" ? (
        <Pointcloud
          maxNum={points ? points.length / 4 : 0}
          points={points}
          {...config}
          clip={{ min: -10000.0, max: 10000.0 }}
          size={1.5}
          frustumCulled={config.frustumCulled}
          visible={config.visible && visible}
          pointsProps={pointProp}
        />
      ) : (
        <></>
      )}
    </>
  );
}
