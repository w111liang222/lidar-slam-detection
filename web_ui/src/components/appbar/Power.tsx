import React, { useState } from "react";
import { Button, Popover, MenuItem } from "@mui/material";
import { PowerSettingsNew, RefreshOutlined } from "@mui/icons-material";
import { useTranslation } from "react-i18next";
import { useDialog } from "@hooks/misc";
import { usePersistFn } from "ahooks";
import { PowerAction } from "@rpc/http-upgrade";
import { useNavigate } from "react-router-dom";

export interface Props {}

export default function Power({}: Props) {
  const { t } = useTranslation();
  const navigate = useNavigate();
  const [action, setAction] = useState<string>("");

  const onConfirm = usePersistFn(() => {
    PowerAction(action);
    navigate("/home", { replace: true });
  });

  const [dialog, openDialog] = useDialog({
    content: t("Confirm" + action),
    onConfirm: onConfirm,
    onCancel: () => {},
  });
  const [anchorEl, setAnchorEl] = useState(null);
  const handleButtonClick = (event: any) => {
    setAnchorEl(event.currentTarget);
  };
  const handItemClick = (key: string) => {
    setAction(key);
    openDialog();
    setAnchorEl(null);
  };
  const handleClose = () => {
    setAnchorEl(null);
  };

  return (
    <div>
      <Button onClick={handleButtonClick} color="inherit">
        <PowerSettingsNew />
      </Button>
      <Popover
        anchorEl={anchorEl}
        anchorOrigin={{
          vertical: "bottom",
          horizontal: "center",
        }}
        transformOrigin={{
          vertical: "top",
          horizontal: "center",
        }}
        open={Boolean(anchorEl)}
        onClose={handleClose}>
        <MenuItem onClick={() => handItemClick("PowerOff")}>
          <PowerSettingsNew />
          {t("PowerOff")}
        </MenuItem>
        <MenuItem onClick={() => handItemClick("Reboot")}>
          <RefreshOutlined />
          {t("Reboot")}
        </MenuItem>
      </Popover>
      {dialog}
    </div>
  );
}
