import Pointcloud from "@components/3d/Pointcloud";
import Sketchpad, { Ref as SketchpadRef } from "@components/3d/SketchpadROI";
import { Canvas } from "@react-three/fiber";
import React, { useState, useCallback, useEffect } from "react";
import { useImperativeHandle } from "react";
import { calibrateGround, calibrateHeading, getProjectionForward, getProjectionBackward } from "@rpc/http";
import { useRef } from "react";
import ContextMenu from "@components/general/ContextMenu";
import { useCtrlKey, useShiftKey } from "@hooks/keyboard";
import Helper, { Config, DEFAULT_CONFIG } from "@components/3d/Helper";
import Controls, { CONTROL_TYPE } from "@components/3d/Controls";
import * as THREE from "three";
import PointcloudPicker from "@components/3d/PointcloudPicker";
import PointsForm from "./PointsForm";
import produce from "immer";

export type Props = {
  points: Float32Array | undefined;
  lidarIndex?: number;
  coordinate: string;
  setCalibEnable: any;
};
export type Ref = {
  calibrate: () => Promise<THREE.Matrix4>;
};

function CalibratePosition({ points, lidarIndex, coordinate, setCalibEnable }: Props, ref: React.Ref<Ref>) {
  const [pointsLidar, setPointsLidar] = useState<THREE.Vector3[]>([]);
  const [pointsWorld, setPointsWorld] = useState<[number, number][]>([]);

  const config = JSON.parse(JSON.stringify(DEFAULT_CONFIG));
  config.polarGrid.visible = false;
  config.grid.visible = true;

  useImperativeHandle(ref, () => ({
    calibrate: async () => {
      if (lidarIndex === undefined || pointsLidar.length < 2) {
        return new THREE.Matrix4().identity();
      }

      let pointSource: [number, number][] = pointsLidar.map((pt) => [pt.x, pt.y]);
      let pointTarget = [...pointsWorld];
      if (coordinate == "Global") {
        for (let i = 1; i < pointTarget.length; i++) {
          pointTarget[i] = await getProjectionForward(
            pointTarget[0][0],
            pointTarget[0][1],
            pointTarget[i][0],
            pointTarget[i][1]
          );
        }
        pointSource = pointSource.slice(1);
        pointTarget = pointTarget.slice(1);
      }

      return await calibrateHeading({
        source: pointSource,
        target: pointTarget,
        key: lidarIndex,
      });
    },
  }));

  useEffect(() => {
    if (coordinate == "Global" && pointsLidar.length <= 0) {
      setPointsLidar([new THREE.Vector3(0, 0, 0)]);
      setPointsWorld([[0, 0]]);
    }

    let pointLimit = coordinate == "Local" ? 2 : 3;
    if (lidarIndex === undefined || pointsLidar.length < pointLimit) {
      setCalibEnable(false);
    } else {
      if (coordinate == "Local") {
        setCalibEnable(true);
      } else {
        let pointValid = true;
        for (let i = 0; i < pointsWorld.length; i++) {
          if (pointsWorld[i][0] == undefined || pointsWorld[i][1] == undefined) {
            pointValid = false;
            break;
          }
        }
        setCalibEnable(pointValid);
      }
    }
  }, [lidarIndex, pointsLidar, pointsWorld]);

  useEffect(() => {
    if (coordinate == "Local") {
      setPointsLidar([]);
      setPointsWorld([]);
    } else {
      setPointsLidar([new THREE.Vector3(0, 0, 0)]);
      setPointsWorld([[0, 0]]);
    }
  }, [coordinate]);

  const pick = useCtrlKey();
  const remove = useShiftKey();

  return (
    <>
      <div className="calibration-tab">
        <Canvas raycaster={{ params: { Points: { threshold: 1.0 } } }}>
          <Helper config={config} />
          <PointcloudPicker
            {...{
              points: points,
              color: "height",
              pick,
              remove,
              pointsPicked: pointsLidar,
              onAdd: (pt) => {
                if (coordinate == "Local") {
                  setPointsWorld((pts) => [...pts, [pt.x, pt.y]]);
                } else {
                  setPointsWorld((pts) => [...pts, [undefined, undefined] as any]);
                }
                setPointsLidar((pts) => [...pts, pt]);
              },
              onRemove: (i) => {
                setPointsLidar(
                  produce((pts) => {
                    pts.splice(i, 1);
                  })
                );
                setPointsWorld(
                  produce((pts) => {
                    pts.splice(i, 1);
                  })
                );
              },
            }}
          />
        </Canvas>
      </div>
      <div className="form-panel">
        <PointsForm
          source={pointsLidar}
          target={pointsWorld}
          isLocal={coordinate == "Local"}
          onChange={useCallback(
            (source, target) => {
              setPointsLidar(source);
              setPointsWorld(target);
            },
            [setPointsLidar, setPointsWorld]
          )}
        />
      </div>
    </>
  );
}

export default React.forwardRef(CalibratePosition);
