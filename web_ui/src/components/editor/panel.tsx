import { FolderApi, Pane } from "tweakpane";
import * as EssentialsPlugin from "@tweakpane/plugin-essentials";
import React, { useRef, useEffect } from "react";
import { DEFAULT_PANEL_CONFIG, Config } from "./index";

export type Props = {
  config: Config;
  setConfig: any;
  t: any;
  className: string;
};

export const PanelGui = React.memo(
  ({ config = DEFAULT_PANEL_CONFIG, setConfig, t = (x: string) => x, className }: Props) => {
    const ref = useRef<any>();
    useEffect(() => {
      const pane = new Pane({ container: ref.current });
      pane.registerPlugin(EssentialsPlugin);
      pane.on("change", (ev) => {
        setConfig({ ...config });
        for (let i = 0; i < pane.children.length; i++) {
          let folder = pane.children[i] as FolderApi;
          if (folder.title == t("map")) {
            // maxIntensity
            folder.children[4].hidden = config.map.color == "intensity" ? false : true;
          }
        }
      });

      let folder;
      folder = pane.addFolder({ title: t("camera"), expanded: false });
      folder.addInput(config.camera, "height", {
        min: -200,
        max: 200,
        step: 1.0,
        label: t("targetHeight"),
      });
      folder = pane.addFolder({ title: t("map"), expanded: false });
      folder.addInput(config.map, "visible", { label: t("visible") });
      folder.addInput(config.map, "frustumCulled", { label: t("frustumCulled") });
      folder.addInput(config.map, "pose", { label: t("mapEdge") });
      folder.addInput(config.map, "color", {
        label: t("color"),
        options: Object.fromEntries(["height", "intensity", "gray", "rgb"].map((x) => [t(x), x])),
      });
      folder.addInput(config.map, "maxIntensity", {
        min: 0.1,
        max: 10.0,
        step: 0.05,
        label: t("maxIntensity"),
        hidden: config.map.color == "intensity" ? false : true,
      });
      folder.addInput(config.map, "size", {
        min: 1.0,
        max: 5.0,
        step: 0.5,
        label: t("size"),
      });
      folder.addInput(config.map, "step", {
        min: 1,
        max: 20,
        step: 1,
        label: t("step"),
      });
      folder.addInput(config.map, "zRange", {
        min: -5000.0,
        max: 5000.0,
        step: 0.1,
        label: t("zRange"),
      });
      return () => {
        pane.dispose();
      };
    }, [t]);
    return <div className={className} ref={ref}></div>;
  }
);
