import React, { useState, useEffect, useImperativeHandle } from "react";
import ImagePicker from "@components/3d/ImagePicker";
import { Canvas } from "@react-three/fiber";
import Controls, { CONTROL_TYPE } from "@components/3d/Controls";
import Loading from "@components/general/Loading";
import { getPanorama, setPanoramaConfig } from "@rpc/http";
import { useRequest } from "ahooks";

export interface Props {
  setCalibEnable?: any;
}

function CalibCheck({ setCalibEnable }: Props, ref: any) {
  const [isLoading, setIsLoading] = useState<boolean>(true);
  const [panoramaImg, setPanoramaImg] = useState<string | undefined>(undefined);
  useRequest(() => getPanorama(), {
    pollingInterval: 100,
    onSuccess: (data) => {
      const blob = new Blob([data.buffer], { type: "application/octet-stream" });
      var url = URL.createObjectURL(blob);
      setPanoramaImg(url);
      setIsLoading(false);
    },
    manual: false,
  });

  useImperativeHandle(ref, () => ({
    calibrate: async () => {
      await setPanoramaConfig();
    },
  }));

  useEffect(() => {
    setCalibEnable(true);
  }, []);

  return (
    <>
      <div className="calibration-tab" style={{ display: "flex" }}>
        {isLoading ? (
          <Loading />
        ) : (
          <Canvas>
            <Controls type={CONTROL_TYPE.IMAGE} height={1.0} />
            <ImagePicker
              src={panoramaImg}
              onTextureLoad={() => {}}
              {...{
                pointsPicked: [],
                onChange: () => {},
                enabled: false,
                removing: false,
              }}
            />
          </Canvas>
        )}
      </div>
    </>
  );
}

export default React.forwardRef(CalibCheck);
