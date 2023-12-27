import { ListItemIcon, ListItemText, MenuItem } from "@mui/material";
import BorderStyleIcon from "@mui/icons-material/BorderStyle";
import React from "react";
import { useTranslation } from "react-i18next";
import { MapFrameIndex } from "..";

export interface Props {
  onEvent: any;
  onFinish: any;
  vertex?: LSD.MapVertex;
  selectVertex: string[];
  selectPointIndex: MapFrameIndex;
}

export default function ReverseSelection({ onEvent, onFinish, vertex, selectVertex, selectPointIndex }: Props) {
  const { t } = useTranslation();

  const onReverseSelect = () => {
    if (vertex && Object.keys(selectPointIndex).length == 0) {
      const vertextIds = Object.keys(vertex);
      const reverseVertex = vertextIds.filter(function (item, pos) {
        return selectVertex.indexOf(item) == -1;
      });
      onEvent("SetSelectVertex", { selection: reverseVertex, color: new Array(reverseVertex.length).fill(0) });
    } else {
      onEvent("ShowMessage", { message: t("NotSupportReversePoints"), severity: "warning", duration: 5000 });
    }
    onFinish();
  };

  return (
    <MenuItem onClick={onReverseSelect}>
      <ListItemIcon>
        <BorderStyleIcon fontSize="small" />
      </ListItemIcon>
      <ListItemText>{t("ReverseSelection")}</ListItemText>
    </MenuItem>
  );
}
