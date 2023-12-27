import React, { useEffect, useState } from "react";
import { Canvas } from "@react-three/fiber";

import Object from "@components/3d/Object";
import ObjectLabel from "@components/3d/ObjectLabel";
import Freespace from "@components/3d/Freespace";
import Controls, { CONTROL_TYPE } from "@components/3d/Controls";
import ObjectTrajectory from "@components/3d/ObjectTrajectory";
import ObjObject from "@components/3d/ObjObject";
import PointCloudImages from "@components/3d/PointCloudImages";
import MapViewer from "./mapViewer";
import ColorMapViewer from "@components/3d/ColorMapViewer";
import * as THREE from "three";

export const DEFAULT_CONFIG = {
  axis: { size: 2, visible: true },
  objObject: { visible: false },
  polarGrid: {
    visible: true,
    radius: 50,
    radials: 16,
    circles: 5,
    divisions: 64,
    color: 0x444444,
  },
  pointcloud: {
    color: "gray" as "gray" | "depth" | "height" | "intensity" | "rgb",
    size: 1.5,
    maxIntensity: 1.0,
    sampleStep: 0,
  },
  freespace: { visible: false },
  object: {
    visible: true,
    color: "objectType" as "objectType",
    showOnImage: false,
    showInfo: false,
    showTrajectory: false,
    showArrow: false,
    labelScale: 1,
  },
  map: {
    visible: true,
    frustumCulled: false,
    color: "height" as "height" | "intensity" | "gray" | "rgb",
    zRange: { min: -20.0, max: 40.0 },
    step: 5,
  },
  camera: { type: CONTROL_TYPE.NORMAL },
  radar: { visible: false },
};

export type Config = typeof DEFAULT_CONFIG;

export type Props = {
  frameData?: LSD.Detection;
  rangeDrawer: any;
  config?: Config;
  boardConfig?: LSD.Config;
  props?: any;
  invProps?: any;
  pointView: any;
  showMessage: any;
  children: React.ReactChild | React.ReactChild[];
};

export default function Scene({
  frameData,
  rangeDrawer,
  config = DEFAULT_CONFIG,
  boardConfig,
  props,
  invProps,
  pointView,
  showMessage,
  children,
}: Props) {
  const axisScale = new Array(3).fill(config ? config.axis.size : 1);
  const { visible, radials, radius, circles, divisions, color } = config.polarGrid;

  let mapView = React.useMemo(
    () => <MapViewer config={config} boardConfig={boardConfig} showMessage={showMessage} />,
    [config, boardConfig]
  );
  let colorMapView = React.useMemo(() => <ColorMapViewer config={config.map} mannual={false} />, [config]);

  let control = React.useMemo(
    () => <Controls {...(config && config.camera)} position={props ? props["position"] : [0, 0, 0]} />,
    [config, props]
  );

  let axis = React.useMemo(
    () => (
      <axesHelper
        {...(config && config.axis)}
        scale={axisScale as any as THREE.Vector3}
        material-linewidth={config.axis.size}
      />
    ),
    [config]
  );

  return (
    <Canvas linear={true}>
      <polarGridHelper
        args={[radius, radials, circles, divisions, color, color]}
        visible={visible}
        rotation-x={Math.PI / 2}
      />
      <ObjObject
        {...{ visible: config.objObject.visible }}
        props={config.camera.type != CONTROL_TYPE.SELF ? props : undefined}
      />
      <group
        frustumCulled={config.map.frustumCulled}
        {...(config.camera.type == CONTROL_TYPE.SELF ? { ...invProps } : {})}>
        {mapView}
        {config.map.color == "rgb" && colorMapView}
      </group>
      {control}
      {rangeDrawer}
      {pointView}
      {config.camera.type == CONTROL_TYPE.SELF && config.pointcloud.color == "rgb" && frameData && frameData.images && (
        <PointCloudImages points={frameData.points} imageData={frameData.images} pointSize={config.pointcloud.size} />
      )}
      <group {...(config.camera.type != CONTROL_TYPE.SELF ? { ...props } : {})}>
        {axis}
        {children}
        {frameData &&
          frameData.objects.map((object) => (
            <>
              <Object
                object={object}
                planeVisible={config.pointcloud.color != "rgb" ? true : false}
                visible={config && config.object.visible}
                showArrow={config && config.object.showArrow}
                colorType={config.object.color}>
                <ObjectLabel
                  object={object}
                  showInfo={config && config.object.showInfo}
                  scale={config && config.object.labelScale}
                />
              </Object>
              <ObjectTrajectory trajectory={object.trajectory!} visible={config && config.object.showTrajectory} />
            </>
          ))}
        {frameData && <Freespace freespace={frameData.freespace} {...(config && config.freespace)} />}
      </group>
    </Canvas>
  );
}
