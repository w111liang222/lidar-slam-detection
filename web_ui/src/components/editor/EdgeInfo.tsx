import { Popover } from "@mui/material";
import "./index.css";
import React, { useEffect, useImperativeHandle, useRef, useState } from "react";
import { useTranslation } from "react-i18next";
import { Pane } from "tweakpane";
import { delteMapEdge } from "@rpc/http";

type InfoProp = {
  id: string;
};

export interface Props {
  onEvent: any;
}

function EdgeInfo({ onEvent }: Props, ref: any) {
  const [menuPosition, setMenuPosition] = useState({ left: 0, top: 0 });
  const [menuVisibility, setMenuVisibility] = useState(false);
  const [EdgeId, setEdgeId] = useState<string | undefined>(undefined);

  const handleButtonClick = (event: any) => {
    setMenuPosition({
      left: event.clientX,
      top: event.clientY,
    });
    setMenuVisibility(true);
  };
  const handleClose = () => {
    setEdgeId(undefined);
    setMenuVisibility(false);
  };

  useImperativeHandle(ref, () => ({
    onShow: (index: string, event: any) => {
      setEdgeId(index);
      handleButtonClick(event);
    },
  }));

  const onEdgeEvent = (event: string) => {
    setMenuVisibility(false);
    if (EdgeId && event == "Delete") {
      delteMapEdge(EdgeId).then(() => {
        onEvent("OptimizeMap");
      });
    }
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
        {<PanelInfo info={{ id: EdgeId ? EdgeId : "" }} onEvent={onEdgeEvent} />}
      </Popover>
    </>
  );
}

export default React.memo(React.forwardRef(EdgeInfo));

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
      // setConfig({ ...info });
    });

    const folder = pane.addFolder({ title: t("Edge"), expanded: true });
    folder.addMonitor(info, "id", { label: t("id") });
    const btn = folder.addButton({ title: t("Delete") });
    btn.on("click", () => {
      onEvent("Delete");
    });

    return () => {
      pane.dispose();
    };
  }, [t]);
  return <div className="info-panel" ref={ref}></div>;
}
