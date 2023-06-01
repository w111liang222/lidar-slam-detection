import {
  Box,
  Button,
  DialogContent,
  DialogTitle,
  Divider,
  IconButton,
  ListItemIcon,
  ListItemText,
  MenuItem,
  Typography,
} from "@mui/material";
import InfoIcon from "@mui/icons-material/Info";
import CloseIcon from "@mui/icons-material/Close";
import React, { useEffect, useState } from "react";
import { useTranslation } from "react-i18next";
import Draggable from "react-draggable";
import { setMapVertexFix } from "@rpc/http";

export interface Props {
  meta?: LSD.MapMeta;
  onEvent: any;
  onClose: any;
}

export default function GraphInfo({ meta, onEvent, onClose }: Props) {
  const { t } = useTranslation();

  const [vertexNum, setVertexNum] = useState(0);
  const [edgeNum, setEdgeNum] = useState(0);

  useEffect(() => {
    if (meta) {
      setVertexNum(Object.keys(meta.vertex).length);
      setEdgeNum(Object.keys(meta.edge).length);
    }
  }, [meta]);

  const onHandleUnfix = (vid: string) => {
    setMapVertexFix(vid, false).then(() => {
      onEvent("RefreshMap");
    });
  };

  return (
    <Draggable>
      <div style={{ position: "fixed", left: "0%", top: "8%", backgroundColor: "#1976d2", borderRadius: "5px" }}>
        <DialogTitle style={{ padding: "0px", width: "300px" }}>
          <Box display="flex" alignItems="center" height="32px">
            <Box flexGrow={1} style={{ marginLeft: "6px", marginTop: "6px" }}>
              {t("MapGraphInfo")}
            </Box>
            <Box>
              <IconButton onClick={onClose} style={{ paddingRight: "8px", fontSize: "24px" }}>
                <CloseIcon fontSize="inherit" />
              </IconButton>
            </Box>
          </Box>
          <DialogContent
            dividers
            style={{
              backgroundColor: "#aaaaaa",
              padding: "8px",
              marginLeft: "8px",
              marginRight: "8px",
              marginBottom: "8px",
            }}>
            <Typography variant="body1" style={{ marginBottom: "4px", color: "black" }}>
              {t("MapStatistics")}
            </Typography>
            <Divider style={{ marginBottom: "8px" }} />
            <Box display="flex" alignItems="center">
              <Box flexGrow={1}>
                <Typography variant="body2" style={{ marginBottom: "4px", color: "black" }}>
                  {t("MapVertexNum")}
                </Typography>
              </Box>
              <Box>
                <Typography variant="body2" style={{ marginBottom: "4px", color: "blue" }}>
                  {vertexNum}
                </Typography>
              </Box>
            </Box>
            <Box display="flex" alignItems="center">
              <Box flexGrow={1}>
                <Typography variant="body2" style={{ marginBottom: "4px", color: "black" }}>
                  {t("MapEdgeNum")}
                </Typography>
              </Box>
              <Box>
                <Typography variant="body2" style={{ marginBottom: "4px", color: "blue" }}>
                  {edgeNum}
                </Typography>
              </Box>
            </Box>
            <Divider style={{ marginBottom: "8px" }} />
            <Typography variant="body1" style={{ marginBottom: "4px", color: "black" }}>
              {t("FixedVertex")}
            </Typography>
            <Divider style={{ marginBottom: "8px" }} />
            {meta &&
              Object.entries(meta.vertex).map(([vid, { stamps, fix, edge_num }]) => {
                if (!fix) {
                  return;
                }
                return (
                  <Box display="flex" style={{ marginTop: "4px" }}>
                    <Box flexGrow={1} alignItems="center">
                      <Typography variant="body2" style={{ marginTop: "4px", color: "black" }}>
                        {t("id") + ": " + vid}
                      </Typography>
                    </Box>
                    <Box>
                      <Button
                        size="small"
                        variant="contained"
                        onClick={() => {
                          onHandleUnfix(vid);
                        }}>
                        {t("UnFix")}
                      </Button>
                    </Box>
                  </Box>
                );
              })}
          </DialogContent>
        </DialogTitle>
      </div>
    </Draggable>
  );
}

export function GraphInfoMenu({ onClick }: { onClick: any }) {
  const { t } = useTranslation();
  return (
    <MenuItem onClick={onClick}>
      <ListItemIcon>
        <InfoIcon fontSize="small" />
      </ListItemIcon>
      <ListItemText>{t("MapGraphInfo")}</ListItemText>
    </MenuItem>
  );
}
