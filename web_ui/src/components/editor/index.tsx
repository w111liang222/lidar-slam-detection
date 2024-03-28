import * as THREE from "three";
import Controls from "@components/3d/Controls";
import { Canvas, ThreeEvent } from "@react-three/fiber";
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
import { AreaView } from "./common/AreaView";
import MapMenu from "./menu";
import { useCtrlKey } from "@hooks/keyboard";
import VertexInfo from "./VertexInfo";
import EdgeInfo from "./EdgeInfo";
import Loading from "@components/general/Loading";
import { CustomePointView } from "./common/CustomePointView";
import ColorMapViewer from "@components/3d/ColorMapViewer";
import useTipsShow from "./common/TipsShow";
import { goodBinarySearch } from "@utils/index";
import { getPoseClient, pointToLineDistance } from "@utils/transform";

export const DEFAULT_PANEL_CONFIG = {
  camera: {
    height: 0,
  },
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
  const [meta, setMeta] = useState<LSD.MapMeta>({ vertex: {}, edge: {}, area: {} });
  const [mapFrame, setMapFrame] = useState<MapFrame>({});
  const [selectVertex, setSelectVertex] = useState<string[]>([]);
  const [selectColor, setSelectColor] = useState<number[]>([]);
  const [selectEdge, setSelectEdge] = useState<string | undefined>(undefined);
  const [selectArea, setSelectArea] = useState<string | undefined>(undefined);
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
    let promise = [];
    for (let i = 0; i < loadIds.length; i++) {
      promise.push(getVertexData(loadIds[i], "pi"));
      if (i % 128 == 0 || i == loadIds.length - 1) {
        const frames = await Promise.all(promise);
        for (let j = 0; j < frames.length; j++) {
          mapFrame[loadIds[j + i + 1 - frames.length]] = {
            points: frames[j].points,
            images: frames[j].images,
          };
        }
        promise = [];
      }
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
    setMeta({ vertex: {}, edge: {}, area: {} });
    onHandleClearSelection();
  });

  const onHandleClearSelection = usePersistFn(() => {
    onHandleVertexSelection([], []);
    onHanlePointSelection(undefined, {});
    setSelectEdge(undefined);
    setSelectArea(undefined);
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
      setRoiState(data ? data : "drawing");
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
    setSelectEdge(undefined);
    setSelectArea(undefined);
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
    onHandleClearSelection();
    setSelectEdge(id);
    edgeInfoRef.current?.onShow(id, event);
  });

  const onAreaClick = usePersistFn((id: string, event: any) => {
    onHandleClearSelection();
    setSelectArea(id);
    menuRef.current?.onSelectArea(id);
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

  window.onkeyup = (ev: KeyboardEvent) => {
    if (ev.key === "Escape") {
      onHandleClearSelection();
    }
  };

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

  const onCheckClickElement = usePersistFn((e: ThreeEvent<MouseEvent>, camera: THREE.Camera) => {
    let vertexDist = 100;
    if (e.type == "contextmenu") {
      let vertexId = "";
      const edgeIds = Object.keys(edge);
      for (let i = 0; i < edgeIds.length; i++) {
        const prev = edge[edgeIds[i]][0].toString();
        const next = edge[edgeIds[i]][1].toString();
        const A = new THREE.Vector3(e.clientX, e.clientY, 0);
        if (vertex[prev] == undefined || vertex[next] == undefined) {
          continue;
        }
        const B = getPoseClient(new THREE.Vector3(vertex[prev][3], vertex[prev][7], vertex[prev][11]), e, camera);
        const C = getPoseClient(new THREE.Vector3(vertex[next][3], vertex[next][7], vertex[next][11]), e, camera);
        const dist = pointToLineDistance(A.x, A.y, B.x, B.y, C.x, C.y);
        if (dist < vertexDist) {
          vertexDist = dist;
          vertexId = edgeIds[i];
        }
      }
      vertexId != "" && onEdgeClick(vertexId, e);
    }
  });

  return (
    <div style={{ position: "relative" }}>
      {isLoading && <Loading />}
      {tips}
      <div className="scene">
        <Canvas
          linear={true}
          camera={{ near: 0.1, far: 10000 }}
          raycaster={{ params: { Points: { threshold: 1.0 }, Line: undefined } }}>
          <gridHelper args={[10000, 1000, 0x222222, 0x222222]} visible={true} rotation-x={Math.PI / 2} />
          <Controls position={[0, 0, 0]} target={[0, 0, config.camera.height]} enable={controlEnable} />
          <ROIDrawer state={roiState} setState={setRoiState} onFinish={onHanleRoi} ref={roiRef} />
          <>
            {config.map.pose && (
              <EdgeView
                vertexes={vertex}
                edges={edge}
                enable={ctrlKeyPressed && roiState == "idle"}
                selectEdge={selectEdge}
                onVertexClick={onVertexClick}
                onCheckClickElement={onCheckClickElement}
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
            <AreaView
              meta={meta}
              enable={ctrlKeyPressed && roiState == "idle"}
              selectArea={selectArea}
              onAreaClick={onAreaClick}
              onCheckClickElement={onCheckClickElement}
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
