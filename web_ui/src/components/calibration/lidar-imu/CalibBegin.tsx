import React, { useState, useEffect } from "react";
import { getDetection, getStatus } from "@rpc/http";
import { useImperativeHandle } from "react";
import { useInterval, useLockFn, useRequest } from "ahooks";
import { Canvas } from "@react-three/fiber";
import Pointcloud from "@components/3d/Pointcloud";
import Helper, { DEFAULT_CONFIG } from "@components/3d/Helper";
import Controls, { CONTROL_TYPE } from "@components/3d/Controls";

export interface Props {
  setCalibEnable?: any;
  setStatusString?: any;
  t?: (x: string | undefined) => string;
}

function CalibBegin({ setCalibEnable, setStatusString, t = (x) => x || "" }: Props, ref: any) {
  const [status, setStatus] = useState<LSD.Status>();

  const config = JSON.parse(JSON.stringify(DEFAULT_CONFIG));
  config.polarGrid.visible = false;
  config.grid.visible = true;
  config.camera.type = CONTROL_TYPE.UP;

  useInterval(
    useLockFn(async () => {
      getStatus()
        .then((data) => {
          setStatus(data);
        })
        .catch(() => {
          setStatus(undefined);
        });
    }),
    200
  );

  const { data } = useRequest(() => getDetection(false), {
    pollingInterval: 100,
    manual: false,
  });
  const frameData = data;

  useEffect(() => {
    if (status) {
      let lidarValid = false;
      let insValid = status.ins.valid;
      let lidars = Object.keys(status.lidar);
      for (let i = 0; i < lidars.length; i++) {
        if (status.lidar[lidars[i]].valid) {
          lidarValid = true;
          break;
        }
      }
      if (lidarValid && insValid) {
        setStatusString(t("LidarInsConnect"));
        setCalibEnable(true);
      } else {
        if (lidarValid && !insValid) {
          setStatusString(t("InsDisconnect"));
        } else if (!lidarValid && insValid) {
          setStatusString(t("LidarDisconnect"));
        } else {
          setStatusString(t("LidarInsDisconnect"));
        }
        setCalibEnable(false);
      }
    } else {
      setStatusString(t("notConnected"));
      setCalibEnable(false);
    }
  }, [status]);

  useImperativeHandle(ref, () => ({
    calibrate: async () => {},
  }));

  return (
    <>
      <div className="calibration-tab" style={{ display: "flex" }}>
        <Canvas>
          <Helper config={config} />
          <Pointcloud maxNum={100e4} color={"height"} size={1.5} points={frameData?.points} />
        </Canvas>
        <div style={{ color: "white", position: "fixed", bottom: 0, right: 0 }}>
          {`Lat: ${status?.ins.latitude}, Lon: ${status?.ins.longitude}`}
        </div>
      </div>
    </>
  );
}

export default React.forwardRef(CalibBegin);
