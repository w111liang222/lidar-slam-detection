import React, { useState, useEffect } from "react";
import PointcloudPicker from "@components/3d/PointcloudPicker";
import ImagePicker from "@components/3d/ImagePicker";
import { useCtrlKey, useShiftKey } from "@hooks/keyboard";
import useContextMenu from "@hooks/context-menu";
import ContextMenu from "@components/general/ContextMenu";
import * as THREE from "three";
import { Canvas } from "@react-three/fiber";
import Controls, { CONTROL_TYPE } from "@components/3d/Controls";
import { calibrateLidarCamera } from "@rpc/http";
import { useImperativeHandle } from "react";
import produce from "immer";
import log from "loglevel";
import { useSessionStorageState } from "ahooks";
export interface Props {
  points?: Float32Array;
  imageUrl?: string;
  images?: LSD.Detection["images"];
  cameraName?: string;
  timestamp?: number;
  paused: boolean;
  setCalibEnable?: any;
}

function CalibPnP({ points, imageUrl, images, cameraName, timestamp, paused, setCalibEnable }: Props, ref: any) {
  const [pointsLidarPicked, setPointsLidarPicked] = useState<THREE.Vector3[]>([]);
  const [pointsCameraPicked, setPointsCameraPicked] = useState<THREE.Vector3[]>([]);

  const [currentFrameIndex, setCurrentFrameIndex] = useState<number>(0);
  const [allPointsLidarPicked, setAllPointsLidarPicked] = useState<THREE.Vector3[][]>([]);
  const [allPointsCameraPicked, setAllPointsCameraPicked] = useState<THREE.Vector3[][]>([]);
  const [pointsList, setPointsList] = useState<Float32Array[]>([]);
  const [imagesList, setImagesList] = useState<string[]>([]);

  const pick = useCtrlKey();
  const remove = useShiftKey();

  useImperativeHandle(ref, () => ({
    calibrate: async () => {
      let pickedLidarPoints: THREE.Vector3[] = [];
      let pickedImagePoints: THREE.Vector3[] = [];
      for (var i = 0; i < pointsList.length; i++) {
        pickedLidarPoints = [...pickedLidarPoints, ...allPointsLidarPicked[i]];
        pickedImagePoints = [...pickedImagePoints, ...allPointsCameraPicked[i]];
      }
      console.log(pickedLidarPoints, pickedImagePoints, cameraName);
      if (cameraName)
        return await calibrateLidarCamera({
          pointsLidar: pickedLidarPoints,
          pointsCamera: pickedImagePoints,
          cameraName: cameraName,
        });
    },
  }));

  useEffect(() => {
    if (paused) {
      if (points && images && cameraName) {
        const blob = new Blob([images[cameraName].buffer], { type: "application/octet-stream" });
        var url = URL.createObjectURL(blob);

        setPointsLidarPicked([]);
        setPointsCameraPicked([]);
        setAllPointsLidarPicked((pts) => [[], ...(pts || [])]);
        setAllPointsCameraPicked((pts) => [[], ...(pts || [])]);
        setPointsList((pts) => [points, ...(pts || [])]);
        setImagesList((img) => [url, ...(img || [])]);
        setCurrentFrameIndex(0);
      }
    } else {
      var index = 0;
      while (index < pointsList.length) {
        if (allPointsLidarPicked[index].length <= 0 && allPointsCameraPicked[index].length <= 0) {
          pointsList.splice(index, 1);
          imagesList.splice(index, 1);
          setPointsList(pointsList);
          setImagesList(imagesList);
          allPointsLidarPicked.splice(index, 1);
          allPointsCameraPicked.splice(index, 1);
          setAllPointsLidarPicked(allPointsLidarPicked);
          setAllPointsCameraPicked(allPointsCameraPicked);
        } else {
          index++;
        }
      }
    }
  }, [paused]);

  useEffect(() => {
    if (ref) {
      for (var i = 0; i < pointsList.length; i++) {
        if (allPointsLidarPicked[i].length != allPointsCameraPicked[i].length) {
          setCalibEnable(false);
          return;
        }
      }
      let pickedLidarPoints: THREE.Vector3[] = [];
      let pickedImagePoints: THREE.Vector3[] = [];
      for (var i = 0; i < pointsList.length; i++) {
        pickedLidarPoints = [...pickedLidarPoints, ...allPointsLidarPicked[i]];
        pickedImagePoints = [...pickedImagePoints, ...allPointsCameraPicked[i]];
      }

      if (pickedLidarPoints.length >= 4) {
        setCalibEnable(true);
      } else {
        setCalibEnable(false);
      }
    } else {
      setCalibEnable(cameraName);
    }
  }, [ref, cameraName, pointsLidarPicked, pointsCameraPicked]);

  let addLidarPoints = (pt: THREE.Vector3) => {
    setPointsLidarPicked([...pointsLidarPicked, pt]);
    allPointsLidarPicked[currentFrameIndex] = [...pointsLidarPicked, pt];
    setAllPointsLidarPicked(allPointsLidarPicked);
  };

  let delLidarPoints = (i: number) => {
    pointsLidarPicked.splice(i, 1);
    allPointsLidarPicked[currentFrameIndex].splice(i, 1);
    setPointsLidarPicked(pointsLidarPicked);
    setAllPointsLidarPicked(allPointsLidarPicked);
  };

  let onImagePointsChanged = (pts: THREE.Vector3[]) => {
    setPointsCameraPicked(pts);
    allPointsCameraPicked[currentFrameIndex] = pts;
    setAllPointsCameraPicked(allPointsCameraPicked);
  };

  let contextMenu = (
    <ContextMenu
      enabled={pick && paused}
      menuItems={{
        previousFrame: () => {
          if (currentFrameIndex < pointsList.length - 1) {
            setPointsLidarPicked(allPointsLidarPicked[currentFrameIndex + 1]);
            setPointsCameraPicked(allPointsCameraPicked[currentFrameIndex + 1]);
            setCurrentFrameIndex(currentFrameIndex + 1);
          }
        },
        nextFrame: () => {
          if (currentFrameIndex > 0) {
            setPointsLidarPicked(allPointsLidarPicked[currentFrameIndex - 1]);
            setPointsCameraPicked(allPointsCameraPicked[currentFrameIndex - 1]);
            setCurrentFrameIndex(currentFrameIndex - 1);
          }
        },
        clearPoints: () => {
          setPointsLidarPicked([]);
          setPointsCameraPicked([]);
          allPointsLidarPicked[currentFrameIndex] = [];
          allPointsCameraPicked[currentFrameIndex] = [];
          setAllPointsLidarPicked(allPointsLidarPicked);
          setAllPointsCameraPicked(allPointsCameraPicked);
        },
      }}
      onStart={undefined}
    />
  );

  return (
    <div className="calibration-tab" style={{ display: "flex" }}>
      <div
        style={{
          width: "50%",
          borderRight: "2px solid",
          borderRightColor: "#ffffff",
        }}>
        <Canvas raycaster={{ params: { Points: { threshold: 1.0 } } }}>
          <Controls />
          <PointcloudPicker
            {...{
              maxNum: 100e4,
              points: !paused ? points : pointsList[currentFrameIndex],
              onPointsUpdate: () => {
                log.debug(`rendered pointcloud of ${timestamp} at ${performance.now()}`);
              },
              color: "height",
              pick: pick && paused,
              remove,
              pointsPicked: !paused ? [] : pointsLidarPicked,
              onAdd: addLidarPoints,
              onRemove: delLidarPoints,
            }}
          />
        </Canvas>
      </div>
      <div
        style={{
          flexGrow: 1,
        }}>
        <Canvas>
          <Controls type={CONTROL_TYPE.IMAGE} height={1.2} />
          <ImagePicker
            src={!paused ? imageUrl : imagesList[currentFrameIndex]}
            onTextureLoad={() => {
              log.debug(`rendered image of ${timestamp} at ${performance.now()}`);
            }}
            {...{
              pointsPicked: !paused ? [] : pointsCameraPicked,
              onChange: onImagePointsChanged,
              enabled: pick && paused,
              removing: remove,
            }}
          />
        </Canvas>
        {contextMenu}
      </div>
    </div>
  );
}

export default React.forwardRef(CalibPnP);
