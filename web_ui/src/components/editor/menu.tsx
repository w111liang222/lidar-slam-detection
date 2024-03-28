import React, { useImperativeHandle, useRef, useState } from "react";
import { useTranslation } from "react-i18next";
import { DEFAULT_PANEL_CONFIG, MapFrame, MapFrameIndex } from ".";

import MenuFile from "./File";
import MenuGraph from "./Graph";
import MenuView from "./View";

export interface Props {
  onEvent: any;
}

const MenuBars = {
  menuFile: MenuFile,
  menuGraph: MenuGraph,
  menuView: MenuView,
  // menuHelp: MenuHelp,
};

function MapMenu({ onEvent }: Props, ref: any) {
  const { t } = useTranslation();
  const fileRef = useRef<any>();
  const graphRef = useRef<any>();
  const viewRef = useRef<any>();
  const helpRef = useRef<any>();
  const refs = [fileRef, graphRef, viewRef, helpRef];

  const [config, setConfig] = useState(DEFAULT_PANEL_CONFIG);
  const [vertex, setVertex] = useState<LSD.MapVertex>();
  const [edge, setEdge] = useState<LSD.MapEdge>();
  const [meta, setMeta] = useState<LSD.MapMeta>();

  const [selectVertex, setSelectVertex] = useState<string[]>([]);
  const [selectColor, setSelectColor] = useState<number[]>([]);
  const [selectPoint, setSelectPoint] = useState<Float32Array>();
  const [selectArea, setSelectArea] = useState<string>();
  const [selectPointIndex, setSelectPointIndex] = useState<MapFrameIndex>({});

  useImperativeHandle(ref, () => ({
    onSelectVertex: (select: string[], color: number[]) => {
      setSelectVertex(select);
      setSelectColor(color);
    },
    onSelectPoints: (points: Float32Array | undefined, index: MapFrameIndex) => {
      setSelectPoint(points);
      setSelectPointIndex(index);
    },
    onSelectArea: (select: string) => {
      setSelectArea(select);
    },
    onConfig: (config: any) => {
      setConfig(config);
    },
    onMapChanged: (vertex: LSD.MapVertex, edge: LSD.MapEdge, meta: LSD.MapMeta) => {
      setVertex(vertex);
      setEdge(edge);
      setMeta(meta);
    },
    onHanleRoi: (mapFrame: MapFrame, roi: THREE.Vector3[] | undefined, camera: THREE.Camera) => {
      viewRef.current?.onHanleRoi(mapFrame, roi, camera);
    },
    onClear: () => {
      for (let i = 0; i < refs.length; i++) {
        refs[i].current?.onClear();
      }
    },
  }));

  return (
    <>
      {Object.entries(MenuBars).map((kv, idx) => {
        const [name, Menu] = kv;
        return (
          <Menu
            onEvent={onEvent}
            config={config}
            vertex={vertex}
            edge={edge}
            meta={meta}
            selectVertex={selectVertex}
            selectColor={selectColor}
            selectPoints={selectPoint}
            selectPointIndex={selectPointIndex}
            selectArea={selectArea}
            ref={refs[idx]}
          />
        );
      })}
    </>
  );
}

export default React.memo(React.forwardRef(MapMenu));
