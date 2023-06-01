import React, { useState, useEffect } from "react";
import ImageViewer from "@components/preview/ImageViewer";
import Loading from "@components/general/Loading";
import { useImperativeHandle } from "react";
import { getHomography } from "@rpc/http";
import { Canvas } from "@react-three/fiber";
import Controls from "@components/3d/Controls";
import ImagePicker from "@components/3d/ImagePicker";
import { useCtrlKey, useShiftKey } from "@hooks/keyboard";

export interface Props {
  images: LSD.Detection["images"];
  cameraName: string[];
  paused: boolean;
  setCalibEnable: any;
  slideCameraIndex: number;
  setSlideCameraIndex: any;
}

function Calib({ images, cameraName, paused, setCalibEnable, slideCameraIndex, setSlideCameraIndex }: Props, ref: any) {
  const [frame, setFrame] = useState<LSD.Detection["images"]>({});
  const [isLoading, setIsLoading] = useState<boolean>(false);
  const [stitchImg, setStitchImg] = useState<string | undefined>(undefined);
  const [imageLeft, setImageLeft] = useState<string | undefined>(undefined);
  const [imageRight, setImageRight] = useState<string | undefined>(undefined);

  const [pointsLeft, setPointsLeft] = useState<THREE.Vector3[]>([]);
  const [pointsRight, setPointsRight] = useState<THREE.Vector3[]>([]);

  const pick = useCtrlKey();
  const remove = useShiftKey();

  useImperativeHandle(ref, () => ({
    calibrate: async () => {},
  }));

  let onLeftImagePointsChanged = (pts: THREE.Vector3[]) => {
    setPointsLeft(pts);
    console.log(pts);
  };
  let onRightImagePointsChanged = (pts: THREE.Vector3[]) => {
    setPointsRight(pts);
  };

  useEffect(() => {
    setFrame(images);
    if (
      ref &&
      images &&
      cameraName.length >= 2 &&
      slideCameraIndex < cameraName.length &&
      frame.hasOwnProperty(cameraName[slideCameraIndex - 1]) &&
      frame.hasOwnProperty(cameraName[slideCameraIndex])
    ) {
      const blob0 = new Blob([frame[cameraName[slideCameraIndex - 1]].buffer], { type: "application/octet-stream" });
      var url0 = URL.createObjectURL(blob0);
      const blob1 = new Blob([frame[cameraName[slideCameraIndex]].buffer], { type: "application/octet-stream" });
      var url1 = URL.createObjectURL(blob1);
      setImageLeft(url0);
      setImageRight(url1);
    }
  }, [images]);

  useEffect(() => {
    if (
      ref &&
      paused &&
      cameraName.length >= 2 &&
      slideCameraIndex < cameraName.length &&
      frame.hasOwnProperty(cameraName[slideCameraIndex - 1]) &&
      frame.hasOwnProperty(cameraName[slideCameraIndex])
    ) {
      const name0 = cameraName[slideCameraIndex - 1];
      const name1 = cameraName[slideCameraIndex];
      const centerIdx = Math.floor(cameraName.length / 2);
      const order = slideCameraIndex > centerIdx ? "right" : "left";
      setIsLoading(true);
      getHomography(cameraName, name0, name1, frame[name0], frame[name1], pointsLeft, pointsRight, order)
        .then((result) => {
          if (result.result) {
            const blob = new Blob([result.images.buffer], { type: "application/octet-stream" });
            var url = URL.createObjectURL(blob);
            setStitchImg(url);
            setSlideCameraIndex(slideCameraIndex + 1);
          }
        })
        .finally(() => {
          setIsLoading(false);
        });
    }
  }, [paused]);

  useEffect(() => {
    if (ref) {
      if (cameraName.length >= 2 && slideCameraIndex >= cameraName.length) {
        setCalibEnable(true);
      } else {
        setCalibEnable(false);
      }
    } else {
      if (cameraName.length >= 2 && cameraName.length <= 3) {
        setCalibEnable(true);
      } else {
        setCalibEnable(false);
      }
    }
  }, [ref, cameraName, slideCameraIndex]);

  return (
    <div className="calibration-tab" style={{ display: "flex" }}>
      {isLoading ? (
        <Loading />
      ) : paused ? (
        <Canvas>
          <Controls type={3} height={1.0} />
          <ImagePicker
            src={stitchImg}
            onTextureLoad={() => {}}
            {...{
              pointsPicked: [],
              onChange: () => {},
              enabled: false,
              removing: false,
            }}
          />
        </Canvas>
      ) : ref ? (
        cameraName.length >= 2 &&
        slideCameraIndex < cameraName.length && (
          <>
            <div style={{ width: "50%", borderRight: "2px solid", borderRightColor: "#ffffff" }}>
              <Canvas>
                <Controls type={3} height={1.2} />
                <ImagePicker
                  src={imageLeft}
                  onTextureLoad={() => {}}
                  {...{
                    pointsPicked: pointsLeft,
                    onChange: onLeftImagePointsChanged,
                    enabled: pick,
                    removing: remove,
                  }}
                />
              </Canvas>
            </div>
            <div style={{ flexGrow: 1 }}>
              <Canvas>
                <Controls type={3} height={1.2} />
                <ImagePicker
                  src={imageRight}
                  onTextureLoad={() => {}}
                  {...{
                    pointsPicked: pointsRight,
                    onChange: onRightImagePointsChanged,
                    enabled: pick,
                    removing: remove,
                  }}
                />
              </Canvas>
            </div>
          </>
        )
      ) : (
        <div style={{ top: "40%", left: `${50 - cameraName.length * 15}%`, position: "absolute" }}>
          {cameraName.map((value, i) => {
            const name = cameraName[i];
            if (!images.hasOwnProperty(name)) {
              return;
            }
            return <ImageViewer name={name} imageData={images[name]} numCamera={cameraName.length} />;
          })}
        </div>
      )}
    </div>
  );
}

export default React.forwardRef(Calib);
