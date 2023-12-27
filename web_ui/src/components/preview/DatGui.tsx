import { FolderApi, Pane } from "tweakpane";
import * as EssentialsPlugin from "@tweakpane/plugin-essentials";
import React, { useRef, useEffect } from "react";
import { CONTROL_TYPE } from "@components/3d/Controls";
import { DEFAULT_CONFIG, Config } from "./Scene";

export type Props = {
  config: Config;
  setConfig: any;
  t: any;
  className: string;
};

export default function PreviewDatGui({ config = DEFAULT_CONFIG, setConfig, t = (x: string) => x, className }: Props) {
  const ref = useRef<any>();
  useEffect(() => {
    const pane = new Pane({ container: ref.current });
    pane.registerPlugin(EssentialsPlugin);
    pane.on("change", (ev) => {
      setConfig({ ...config });
      for (let i = 0; i < pane.children.length; i++) {
        let folder = pane.children[i] as FolderApi;
        if (folder.title == t("pointcloud")) {
          // maxIntensity
          folder.children[1].hidden = config.pointcloud.color == "intensity" ? false : true;
        }
      }
      // setTimeout(() => {
      //   (document.activeElement as HTMLElement)?.blur();
      // }, 0);
    });

    let folder;
    folder = pane.addFolder({ title: t("axis"), expanded: false });
    folder.addInput(config.axis, "size", {
      min: 1,
      max: 10,
      step: 1,
      label: t("size"),
    });
    folder.addInput(config.axis, "visible", { label: t("visible") });
    folder = pane.addFolder({ title: t("objObject"), expanded: false });
    folder.addInput(config.objObject, "visible", { label: t("visible") });

    folder = pane.addFolder({ title: t("polarGrid"), expanded: false });
    folder.addInput(config.polarGrid, "visible", { label: t("visible") });
    folder.addInput(config.polarGrid, "radius", {
      min: 10,
      max: 100,
      step: 1,
      label: t("radius"),
    });
    folder.addInput(config.polarGrid, "radials", {
      min: 8,
      max: 36,
      step: 1,
      label: t("radials"),
    });
    folder.addInput(config.polarGrid, "circles", {
      min: 2,
      max: 16,
      step: 1,
      label: t("circles"),
    });
    folder.addInput(config.polarGrid, "color", {
      view: "color",
      label: t("color"),
    });
    folder = pane.addFolder({ title: t("camera"), expanded: false });
    folder.addInput(config.camera, "type", {
      options: Object.fromEntries(
        Object.entries(CONTROL_TYPE)
          .filter(([k, v]) => {
            if (v <= 2 || v == 6) return [k, v];
          })
          .map(([k, v]) => [t(k), v])
      ),
      label: t("type"),
    });
    folder = pane.addFolder({ title: t("pointcloud"), expanded: false });
    folder.addInput(config.pointcloud, "color", {
      label: t("color"),
      options: Object.fromEntries(["gray", "depth", "height", "intensity", "rgb"].map((x) => [t(x), x])),
    });
    folder.addInput(config.pointcloud, "maxIntensity", {
      min: 0.1,
      max: 10.0,
      step: 0.1,
      label: t("maxIntensity"),
      hidden: config.pointcloud.color == "intensity" ? false : true,
    });
    folder.addInput(config.pointcloud, "size", {
      min: 1.0,
      max: 5.0,
      step: 0.5,
      label: t("size"),
    });
    folder.addInput(config.pointcloud, "sampleStep", {
      min: 0,
      max: 2.0,
      step: 0.1,
      label: t("sampleStep"),
    });
    folder = pane.addFolder({ title: t("object"), expanded: false });
    folder.addInput(config.object, "visible", { label: t("objectVisible") });
    folder.addInput(config.object, "showOnImage", { label: t("showOnImage") });
    folder.addInput(config.object, "showArrow", { label: t("direction") });
    folder.addInput(config.object, "showInfo", { label: t("details") });
    folder.addInput(config.object, "labelScale", {
      label: t("labelScale"),
      min: 1,
      max: 4,
    });
    folder.addInput(config.object, "showTrajectory", {
      label: t("trajectory"),
    });
    folder.addInput(config.freespace, "visible", { label: t("freespaceSub") });
    folder.addInput(config.radar, "visible", { label: t("radarDisplay") });
    folder = pane.addFolder({ title: t("map"), expanded: false });
    folder.addInput(config.map, "visible", { label: t("visible") });
    folder.addInput(config.map, "frustumCulled", { label: t("frustumCulled") });
    folder.addInput(config.map, "color", {
      label: t("color"),
      options: Object.fromEntries(["height", "intensity", "gray", "rgb"].map((x) => [t(x), x])),
    });
    folder.addInput(config.map, "step", {
      min: 1,
      max: 20,
      step: 1,
      label: t("step"),
    });
    folder.addInput(config.map, "zRange", {
      min: -200.0,
      max: 200.0,
      step: 0.1,
      label: t("zRange"),
    });
    return () => {
      pane.dispose();
    };
  }, [t]);
  return <div className={className} ref={ref}></div>;
}
