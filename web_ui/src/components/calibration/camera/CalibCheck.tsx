import React, { useState, useEffect } from "react";
import ImagePicker from "@components/3d/ImagePicker";
import { Canvas } from "@react-three/fiber";
import Controls, { CONTROL_TYPE } from "@components/3d/Controls";
import { useCtrlKey } from "@hooks/keyboard";

export interface Props {
  imageUrl?: string;
  setCalibEnable?: any;
  setDoDistort?: any;
}

function CalibCheck({ imageUrl, setCalibEnable, setDoDistort }: Props, ref: any) {
  const ctrlKeyPressed = useCtrlKey();

  useEffect(() => {
    setCalibEnable(true);
  }, []);

  useEffect(() => {
    setDoDistort(ctrlKeyPressed);
  }, [ctrlKeyPressed]);

  return (
    <>
      <div className="calibration-tab" style={{ display: "flex" }}>
        <Canvas>
          <Controls type={CONTROL_TYPE.IMAGE} height={1.2} />
          <ImagePicker
            src={imageUrl}
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
    </>
  );
}

export default React.forwardRef(CalibCheck);
