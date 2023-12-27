import React, { useState, useEffect } from "react";
import ImagePicker from "@components/3d/ImagePicker";
import { Canvas } from "@react-three/fiber";
import Controls, { CONTROL_TYPE } from "@components/3d/Controls";
import { findCorners, calibrateCamera } from "@rpc/http";
import { useImperativeHandle } from "react";
import FrameForm from "./frameForm";

export const DEFAULT_CONFIG = {
  chessboardRow: 9,
  chessboardCol: 6,
  chessboardSize: 30,
};

export type Config = typeof DEFAULT_CONFIG;

export interface Props {
  imageUrl?: string;
  images?: LSD.Detection["images"];
  cameraName?: string;
  paused: boolean;
  setCalibEnable?: any;
  setDoDistort?: any;
  config?: object;
}

function Calib(
  { imageUrl, images, cameraName, paused, setCalibEnable, setDoDistort, config = DEFAULT_CONFIG }: Props,
  ref: any
) {
  const [pauseImage, setPauseImage] = useState<boolean>(false);
  const [currentFrameIndex, setCurrentFrameIndex] = useState<number>(0);
  const [cornerStatus, setCornerStatus] = useState<boolean[]>([]);
  const [allCorners, setAllCorners] = useState<[][][]>([]);
  const [imagesList, setImagesList] = useState<string[]>([]);

  useEffect(() => {
    setDoDistort(false);
  }, []);

  useImperativeHandle(ref, () => ({
    calibrate: async () => {
      if (cameraName) {
        return await calibrateCamera({
          pointsCamera: allCorners,
          cameraName: cameraName,
          config: config,
        });
      }
    },
  }));

  useEffect(() => {
    if (paused) {
      if (images && cameraName) {
        findCorners({ imageData: images[cameraName], cameraName, config: config }).then((data) => {
          const blob = new Blob([data.images.buffer], { type: "application/octet-stream" });
          var url = URL.createObjectURL(blob);

          setImagesList((img) => [url, ...img]);
          setAllCorners((corner) => [data.corners, ...corner]);
          setCornerStatus((status) => [data.result, ...status]);
          setCurrentFrameIndex(0);
          setPauseImage(true);
        });
      }
    } else {
      setPauseImage(false);
    }
  }, [paused]);

  useEffect(() => {
    if (ref) {
      for (let i = 0; i < cornerStatus.length; i++) {
        if (cornerStatus[i] == false) {
          setCalibEnable(false);
          return;
        }
      }
      if (cornerStatus.length >= 10) {
        setCalibEnable(true);
      } else {
        setCalibEnable(false);
      }
    } else {
      setCalibEnable(cameraName);
    }
  }, [ref, cameraName, cornerStatus]);

  let onDelete = (i: number, images: string[], source: boolean[]) => {
    setImagesList(images);
    setCornerStatus(source);
    allCorners.splice(i, 1);
    setAllCorners(allCorners);
    if (currentFrameIndex >= images.length) {
      setCurrentFrameIndex(images.length - 1);
    }
  };

  return (
    <>
      <div className="calibration-tab" style={{ display: "flex" }}>
        <Canvas>
          <Controls type={CONTROL_TYPE.IMAGE} height={1.2} />
          <ImagePicker
            src={!paused ? imageUrl : pauseImage ? imagesList[currentFrameIndex] : undefined}
            onTextureLoad={() => {}}
            {...{
              pointsPicked: [],
              onChange: () => {},
              enabled: false,
              removing: false,
            }}
          />
        </Canvas>
      </div>
      <div className="frameform-panel">
        {ref && (
          <FrameForm
            source={cornerStatus}
            images={imagesList}
            onChange={(i: number) => {
              setCurrentFrameIndex(i);
            }}
            onDelete={onDelete}
          />
        )}
      </div>
    </>
  );
}

export default React.forwardRef(Calib);
