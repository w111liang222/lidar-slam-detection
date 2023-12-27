import Controls from "@components/3d/Controls";
import { Canvas } from "@react-three/fiber";
import {
  getConfig,
  getMapEdge,
  getMapMeta,
  getMapVertex,
  getVertexData,
  mapOptimization,
  pausePlayer,
} from "@rpc/http";
import { useLocalStorageState, usePersistFn } from "ahooks";
import React, { useEffect, useImperativeHandle, useRef, useState } from "react";
import { useTranslation } from "react-i18next";
import "./index.css";
import { PanelGui } from "./panel";
import { VertexView, VertexImageView } from "./common/VertexView";
import ROIDrawer from "./common/ROIDrawer";
import { EdgeView } from "./common/EdgeView";
import MapMenu from "./menu";
import { useCtrlKey } from "@hooks/keyboard";
import VertexInfo from "./VertexInfo";
import EdgeInfo from "./EdgeInfo";
import Loading from "@components/general/Loading";
import { CustomePointView } from "./common/CustomePointView";
import ColorMapViewer from "@components/3d/ColorMapViewer";
import useTipsShow from "./common/TipsShow";
import { goodBinarySearch } from "@utils/index";

export const DEFAULT_PANEL_CONFIG = {
  map: {
    visible: true,
    frustumCulled: false,
    pose: true,
    color: "height" as "height" | "intensity" | "gray" | "rgb",
    maxIntensity: 0.75,
    size: 1.5,
    step: 5,
    zRange: { min: -200.0, max: 200.0 },
  },
};

export type Config = typeof DEFAULT_PANEL_CONFIG;

export type MapFrame = {
  [id: string]:
    | {
        points: Float32Array;
        images: { [key: string]: Uint8Array } | undefined;
      }
    | undefined;
};
export type MapFrameIndex = {
  [id: string]: number[];
};

function MapEditor({}, ref: any) {
  const { t } = useTranslation();
  const [isLoading, setIsLoading] = useState<boolean>(false);
  const [tips, showMessage, closeMessage] = useTipsShow();
  const [controlEnable, setControlEnable] = useState<boolean>(true);
  const [config, setConfig] = useLocalStorageState(
    (import.meta.env.VITE_COMMIT_HASH as string) + "-editor-config",
    DEFAULT_PANEL_CONFIG
  );
  const [mapStep, setMapStep] = useState(config.map.step);
  const [boardConfig, setBoardConfig] = useState<LSD.Config>();
  const menuRef = useRef<any>();
  const vertexInfoRef = useRef<any>();
  const edgeInfoRef = useRef<any>();
  const roiRef = useRef<any>();

  const [vertex, setVertex] = useState<LSD.MapVertex>({});
  const [edge, setEdge] = useState<LSD.MapEdge>({});
  const [meta, setMeta] = useState<LSD.MapMeta>({ vertex: {}, edge: {} });
  const [mapFrame, setMapFrame] = useState<MapFrame>({});
  const [selectVertex, setSelectVertex] = useState<string[]>([]);
  const [selectColor, setSelectColor] = useState<number[]>([]);
  const [selectEdge, setSelectEdge] = useState<string | undefined>(undefined);
  const [selectPoint, setSelectPoint] = useState<Float32Array>();
  const [selectPointIndex, setSelectPointIndex] = useState<MapFrameIndex>({});
  const [roiState, setRoiState] = useState("idle");

  const ctrlKeyPressed = useCtrlKey();

  const refreshMap = usePersistFn((reload: boolean) => {
    if (reload) {
      onHandleClearMap();
    }
    pausePlayer().then(() => {
      getConfig().then((config) => {
        setBoardConfig(config);
      });
      setTimeout(() => {
        getMapVertex().then((data) => {
          data && setVertex(data);
        });
        getMapEdge().then((data) => {
          data && setEdge(data);
        });
        getMapMeta().then((data) => {
          data && setMeta(data);
        });
      }, 100);
    });
  });

  const checkConsistent = usePersistFn(() => {
    const frameIds = Object.keys(mapFrame);
    for (let i = 0; i < frameIds.length; i++) {
      if (!vertex.hasOwnProperty(frameIds[i])) {
        mapFrame[frameIds[i]] = undefined;
      }
    }
  });

  const acquireFrame = usePersistFn(async (acqureIds: string[], step: number) => {
    checkConsistent();
    const loadIds: string[] = [];
    for (let i = 0; i < acqureIds.length; i++) {
      if (mapFrame[acqureIds[i]] == undefined && vertex.hasOwnProperty(acqureIds[i])) {
        loadIds.push(acqureIds[i]);
      }
    }
    const showLoading = loadIds.length >= 10 ? true : false;
    showLoading && setIsLoading(true);
    showLoading && showMessage(t("loadingMaps"), "info", 10000);
    for (let i = 0; i < loadIds.length; i++) {
      const kf = await getVertexData(loadIds[i], "pi");
      mapFrame[loadIds[i]] = {
        points: kf.points,
        images: kf.images,
      };
    }
    showLoading && setIsLoading(false);
    showLoading && showMessage(t("finishLoadingMaps"), "success", 2000);
    setMapFrame({ ...mapFrame });
    step != mapStep && setMapStep(step);
    return mapFrame;
  });

  const onHandleClearMap = usePersistFn(() => {
    setVertex({});
    setEdge({});
    setMeta({ vertex: {}, edge: {} });
    onHandleClearSelection();
  });

  const onHandleClearSelection = usePersistFn(() => {
    onHandleVertexSelection([], []);
    onHanlePointSelection(undefined, {});
    setSelectEdge(undefined);
    setRoiState("idle");
    setControlEnable(true);
    menuRef.current?.onClear();
  });

  const onHandleVertexSelection = usePersistFn((selection: string[], color: number[]) => {
    acquireFrame(selection, config.map.step);
    setSelectVertex(selection);
    setSelectColor(color);
    menuRef.current?.onSelectVertex(selection, color);
  });

  const onHanlePointSelection = usePersistFn((points: Float32Array | undefined, index: MapFrameIndex) => {
    setSelectPoint(points);
    setSelectPointIndex(index);
    menuRef.current?.onSelectPoints(points, index);
  });

  const onHanleDeletePoints = usePersistFn(() => {
    for (const id of Object.keys(selectPointIndex)) {
      const frame = mapFrame[id];
      if (frame == undefined) {
        continue;
      }
      frame.points = frame.points.filter((v, idx, a) => {
        const pointIdx = Math.floor(idx / 4);
        return goodBinarySearch(selectPointIndex[id], pointIdx) == -1;
      });
    }
    setMapFrame({ ...mapFrame });
    onHandleClearSelection();
  });

  const onEvent = usePersistFn((event: string, data: any) => {
    if (event == "ClearMap") {
      onHandleClearMap();
    } else if (event == "RefreshMap") {
      refreshMap(data ? data : false);
    } else if (event == "OptimizeMap") {
      mapOptimization().then(() => {
        refreshMap(data);
      });
    } else if (event == "StartROI") {
      setRoiState("drawing");
      setControlEnable(false);
    } else if (event == "ShowMessage") {
      data != undefined ? showMessage(data.message, data.severity, data.duration) : closeMessage();
    } else if (event == "ShowLoading") {
      setIsLoading(data);
    } else if (event == "ClearSelection") {
      onHandleClearSelection();
    } else if (event == "SetSelectVertex") {
      onHandleVertexSelection(data.selection, data.color);
    } else if (event == "SetSelectPoint") {
      onHanlePointSelection(data.points, data.index);
    } else if (event == "DeleteFramePoints") {
      onHanleDeletePoints();
    }
  });

  const onVertexClick = usePersistFn((index: number, button: number, event: any) => {
    const selectVertexId = Object.keys(vertex)[index];
    // reset loop begin/end vertex
    if (selectColor[0] == 0 || selectColor[1] == 0) {
      selectColor[0] = 0xff00ff;
      selectVertex[0] = "";
      selectColor[1] = 0xffff00;
      selectVertex[1] = "";
    }
    if (selectPoint) {
      onHanlePointSelection(undefined, {});
    }
    selectVertex[2] = selectVertexId;
    selectColor[2] = 0xff0000;
    onHandleVertexSelection(selectVertex.slice(0, 3), selectColor.slice(0, 3));
    vertexInfoRef.current?.onShow(selectVertexId, event);
  });

  const onEdgeClick = usePersistFn((id: string, event: any) => {
    setSelectEdge(id);
    edgeInfoRef.current?.onShow(id, event);
  });

  const onHanleRoi = usePersistFn((roi: THREE.Vector3[] | undefined, camera: THREE.Camera) => {
    menuRef.current?.onHanleRoi(mapFrame, roi, camera);
    setControlEnable(true);
  });

  const getLoadingFrames = usePersistFn((vertexIds: string[], step: number) => {
    const loadFrames: string[] = [];
    for (let i = 0; i < vertexIds.length; i++) {
      if (parseInt(vertexIds[i]) % step == 0) {
        loadFrames.push(vertexIds[i]);
      }
    }
    return loadFrames;
  });

  const onConextMenu = usePersistFn((ev) => {
    roiRef.current?.onContextClick();
  });

  useEffect(() => {
    refreshMap(false);
    document.body.oncontextmenu = onConextMenu;
    return () => {
      document.body.oncontextmenu = null;
    };
  }, []);

  useEffect(() => {
    if (isLoading) {
      return;
    }
    const vertexIds = Object.keys(vertex);
    if (vertexIds.length <= 0) {
      setMapFrame({});
      return;
    }
    const loadFrames = getLoadingFrames(vertexIds, config.map.step);
    acquireFrame(loadFrames, config.map.step);
  }, [vertex, config.map.step]);

  useEffect(() => {
    if (mapStep != config.map.step) {
      const loadFrames = getLoadingFrames(Object.keys(vertex), config.map.step);
      acquireFrame(loadFrames, config.map.step);
    }
  }, [mapFrame]);

  useEffect(() => {
    menuRef.current?.onConfig(config);
  }, [config, menuRef.current]);

  useEffect(() => {
    menuRef.current?.onMapChanged(vertex, edge, meta);
  }, [vertex, edge, meta, menuRef.current]);

  useImperativeHandle(ref, () => ({
    menuBar: () => <MapMenu onEvent={onEvent} ref={menuRef} />,
  }));

  return (
    <div style={{ position: "relative" }}>
      {isLoading && <Loading />}
      {tips}
      <div className="scene">
        <Canvas linear={true} raycaster={{ params: { Points: { threshold: 1.0 }, Line: undefined } }}>
          <gridHelper args={[5000, 500, 0x111111, 0x111111]} visible={true} rotation-x={Math.PI / 2} />
          <Controls position={[0, 0, 0]} enable={controlEnable} />
          <ROIDrawer state={roiState} setState={setRoiState} onFinish={onHanleRoi} ref={roiRef} />
          <>
            {config.map.pose && (
              <EdgeView
                vertexes={vertex}
                edges={edge}
                enable={ctrlKeyPressed && roiState == "idle"}
                selectEdge={selectEdge}
                onVertexClick={onVertexClick}
                onEdgeClick={onEdgeClick}
              />
            )}
            <VertexView
              config={config}
              boardConfig={boardConfig}
              vertexes={vertex}
              mapFrame={mapFrame}
              selectedVertex={selectVertex}
              selectedVertexColor={selectColor}
            />
            {config.map.color == "rgb" && <ColorMapViewer config={config.map} mannual={true} />}
            <CustomePointView points={selectPoint} color={0xff0000} />
          </>
        </Canvas>
      </div>
      <PanelGui className="config-panel" {...{ config, setConfig, t }} />;
      <VertexImageView
        config={config}
        vertexes={vertex}
        selectedVertex={selectVertex}
        selectedVertexColor={selectColor}
      />
      <VertexInfo
        onEvent={onEvent}
        vertex={vertex}
        vertexMeta={meta.vertex}
        selectVertex={selectVertex}
        selectedVertexColor={selectColor}
        ref={vertexInfoRef}
      />
      <EdgeInfo onEvent={onEvent} ref={edgeInfoRef} />
    </div>
  );
}

export default React.forwardRef(MapEditor);
