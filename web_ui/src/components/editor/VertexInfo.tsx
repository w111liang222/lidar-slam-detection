import { Popover } from "@mui/material";
import "./index.css";
import React, { useEffect, useImperativeHandle, useRef, useState } from "react";
import { useTranslation } from "react-i18next";
import { Pane } from "tweakpane";
import { delteMapVertex, setMapVertexFix } from "@rpc/http";

type InfoProp = {
  id: string;
  pos: string;
  stamp: string;
  fix: boolean;
};

export interface Props {
  onEvent: any;
  vertex: LSD.MapVertex;
  vertexMeta: LSD.MapMeta["vertex"];
  selectVertex: string[];
  selectedVertexColor: number[];
}

function VertexInfo({ onEvent, vertex, vertexMeta, selectVertex, selectedVertexColor }: Props, ref: any) {
  const [menuPosition, setMenuPosition] = useState({ left: 0, top: 0 });
  const [menuVisibility, setMenuVisibility] = useState(false);
  const [vertexId, setVertexId] = useState<string | undefined>(undefined);
  const [vertexPos, setVertexPos] = useState({ x: 0, y: 0, z: 0 });

  const handleButtonClick = (event: any) => {
    setMenuPosition({
      left: event.clientX,
      top: event.clientY,
    });
    setMenuVisibility(true);
  };
  const handleClose = () => {
    // setVertexId(undefined);
    setMenuVisibility(false);
  };

  useImperativeHandle(ref, () => ({
    onShow: (index: string, event: any) => {
      const vertexPose = vertex[index];
      setVertexId(index);
      setVertexPos({ x: vertexPose[3], y: vertexPose[7], z: vertexPose[11] });
      handleButtonClick(event);
    },
  }));

  const onVertexEvent = (event: string, data: any) => {
    if (vertexId) {
      if (event == "Delete") {
        delteMapVertex(vertexId).then(() => {
          onEvent("ClearSelection");
          onEvent("OptimizeMap", false);
        });
      } else if (event == "LoopBegin") {
        selectVertex[0] = vertexId;
        selectedVertexColor[0] = 0xff00ff;
        onEvent("SetSelectVertex", { selection: [...selectVertex], color: [...selectedVertexColor] });
      } else if (event == "LoopEnd") {
        selectVertex[1] = vertexId;
        selectedVertexColor[1] = 0xffff00;
        onEvent("SetSelectVertex", { selection: [...selectVertex], color: [...selectedVertexColor] });
      } else if (event == "fix") {
        setMapVertexFix(vertexId, data).then(() => {
          onEvent("RefreshMap");
        });
        return;
      }
    }
    setMenuVisibility(false);
  };

  return (
    <>
      <Popover
        anchorReference="anchorPosition"
        anchorPosition={menuPosition}
        anchorOrigin={{
          vertical: "bottom",
          horizontal: "center",
        }}
        transformOrigin={{
          vertical: "top",
          horizontal: "center",
        }}
        open={menuVisibility}
        onClose={handleClose}>
        {vertexId && vertexMeta[vertexId] && (
          <PanelInfo
            info={{
              id: vertexId,
              pos: PosToString(vertexPos),
              stamp: vertexMeta[vertexId].stamps.toString(),
              fix: vertexMeta[vertexId].fix,
            }}
            onEvent={onVertexEvent}
          />
        )}
      </Popover>
    </>
  );
}

export default React.memo(React.forwardRef(VertexInfo));

type PaneProps = {
  info: InfoProp;
  setConfig?: any;
  onEvent: any;
};

function PanelInfo({ info, setConfig, onEvent }: PaneProps) {
  const { t } = useTranslation();
  const ref = useRef<any>();
  useEffect(() => {
    const pane = new Pane({ container: ref.current });
    pane.on("change", (ev) => {
      if (ev.presetKey == "fix") {
        onEvent("fix", ev.value);
      }
    });

    const folder = pane.addFolder({ title: t("Vertex"), expanded: true });
    folder.addMonitor(info, "id", { label: t("id") });
    folder.addMonitor(info, "pos", { label: t("pos") });
    folder.addMonitor(info, "stamp", { label: t("stamp") });
    folder.addInput(info, "fix", { label: t("fix") });
    let btn = folder.addButton({ title: t("Delete") });
    btn.on("click", () => {
      onEvent("Delete");
    });
    btn = folder.addButton({ title: t("LoopBegin") });
    btn.on("click", () => {
      onEvent("LoopBegin");
    });
    btn = folder.addButton({ title: t("LoopEnd") });
    btn.on("click", () => {
      onEvent("LoopEnd");
    });

    return () => {
      pane.dispose();
    };
  }, [t]);
  return <div className="info-panel" ref={ref}></div>;
}

function PosToString(pos: { x: number; y: number; z: number }) {
  return "[" + pos.x.toFixed(2) + ", " + pos.y.toFixed(2) + ", " + pos.z.toFixed(2) + "]";
}
