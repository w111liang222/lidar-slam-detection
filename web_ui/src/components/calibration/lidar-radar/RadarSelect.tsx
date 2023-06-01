import Pointcloud from "@components/3d/Pointcloud";
import { Canvas } from "@react-three/fiber";
import React, { useEffect } from "react";
import Helper, { DEFAULT_CONFIG } from "@components/3d/Helper";
import * as THREE from "three";
import { IObject } from "@proto/detection";
import Object from "@components/3d/Object";

export type Props = {
  points: Float32Array | undefined;
  radarPoints: IObject[] | undefined;
  radarIndex?: number;
  setCalibEnable: any;
};
export type Ref = {
  calibrate: () => Promise<THREE.Matrix4>;
};

function RadarSelect({ points, radarPoints, radarIndex, setCalibEnable }: Props, ref: React.Ref<Ref>) {
  const config = JSON.parse(JSON.stringify(DEFAULT_CONFIG));
  config.polarGrid.visible = false;
  config.grid.visible = true;

  useEffect(() => {
    if (radarIndex === undefined) {
      setCalibEnable(false);
    } else {
      setCalibEnable(true);
    }
  }, [radarIndex]);

  return (
    <>
      <div className="calibration-tab">
        <Canvas raycaster={{ params: { Points: { threshold: 1.0 } } }}>
          <Helper config={config} />
          <Pointcloud points={points} color={"height"} size={1.5} />
          {radarPoints &&
            radarPoints.map((object) => (
              <>
                <Object
                  object={object}
                  planeVisible={true}
                  visible={true}
                  showArrow={false}
                  colorType={"objectType"}></Object>
              </>
            ))}
        </Canvas>
      </div>
    </>
  );
}

export default React.forwardRef(RadarSelect);
