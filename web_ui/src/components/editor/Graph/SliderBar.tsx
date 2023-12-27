import { Pane } from "tweakpane";
import React, { useRef, useEffect, useState, useImperativeHandle } from "react";

export type Props = {
  translation: [number, number, number];
  rotation: [number, number, number, string];
  setTranslation: any;
  setRotation: any;
};

type Config = {
  x: number;
  y: number;
  z: number;
  R: number;
  P: number;
  Y: number;
};

function SliderBar({ translation, rotation, setTranslation, setRotation }: Props, ref: any) {
  const [config, setConfig] = useState<Config>({ x: 0, y: 0, z: 0, R: 0, P: 0, Y: 0 });
  const [panel, setPanel] = useState<Pane>();

  useImperativeHandle(ref, () => ({
    onClear: () => {
      panel?.importPreset({
        x: 0,
        y: 0,
        z: 0,
        R: 0,
        P: 0,
        Y: 0,
      });
    },
  }));

  const divRef = useRef<any>();
  useEffect(() => {
    const pane = new Pane({ container: divRef.current });
    pane.on("change", (ev) => {
      setTranslation([config.x, config.y, config.z]);
      setRotation([config.R, config.P, config.Y, "XYZ"]);
    });

    pane.addInput(config, "x", {
      min: -50,
      max: 50,
      step: 0.01,
    });
    pane.addInput(config, "y", {
      min: -50,
      max: 50,
      step: 0.01,
    });
    pane.addInput(config, "z", {
      min: -50,
      max: 50,
      step: 0.01,
    });
    pane.addInput(config, "R", {
      min: -180,
      max: 180,
      step: 0.1,
    });
    pane.addInput(config, "P", {
      min: -180,
      max: 180,
      step: 0.1,
    });
    pane.addInput(config, "Y", {
      min: -360,
      max: 360,
      step: 0.1,
    });
    setPanel(pane);
    return () => {
      pane.dispose();
    };
  }, []);
  return <div ref={divRef}></div>;
}

export default React.forwardRef(SliderBar);
