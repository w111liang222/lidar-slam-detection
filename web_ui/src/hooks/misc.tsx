import React, { useEffect, useState } from "react";
import { Dialog, DialogContent, DialogActions, Button, Snackbar, Popover, Typography } from "@mui/material";
import makeStyles from "@mui/styles/makeStyles";
import { useTranslation } from "react-i18next";

const useStyles = makeStyles((theme) => ({
  popover: {
    pointerEvents: "none",
  },
  typography: {
    padding: theme.spacing(1),
  },
}));

export function useDialog({
  content = "",
  onConfirm,
  onCancel,
}: {
  content: string;
  onConfirm: () => void;
  onCancel: () => void;
}): [React.ReactNode, () => void] {
  const [open, setOpen] = useState(false);
  const { t } = useTranslation();

  const showDialog = () => {
    setOpen(true);
  };
  return [
    <div>
      <Dialog open={open}>
        <DialogContent>{content}</DialogContent>
        <DialogActions>
          <Button
            onClick={() => {
              setOpen(false);
              onCancel && onCancel();
            }}>
            {t("no")}
          </Button>
          <Button
            onClick={() => {
              setOpen(false);
              onConfirm && onConfirm();
            }}>
            {t("yes")}
          </Button>
        </DialogActions>
      </Dialog>
    </div>,
    showDialog,
  ];
}

export function useSnackbar(): [React.ReactNode, (message: string, duration?: number) => void] {
  const [open, setOpen] = useState(false);
  const [message, setMessage] = useState("");
  const [duration, setDuration] = useState<number>();

  const show = (message: string, duration = 2000) => {
    setOpen(true);
    setMessage(message);
    setDuration(duration);
  };
  return [
    <Snackbar
      open={open}
      message={message}
      autoHideDuration={duration}
      onClose={() => setOpen(false)}
      anchorOrigin={{ vertical: "bottom", horizontal: "left" }}
    />,
    show,
  ];
}

export function usePopover(): [React.ReactNode, (ev: MouseEvent, msg: string, option: any) => void, () => void] {
  const [message, setMessage] = useState("Popover content placeholder");
  const [header, setHeader] = useState<string | undefined>(undefined);
  const [anchorEl, setAnchorEl] = useState<HTMLElement>();
  const [popoverProps, setPopoverProps] = useState({});
  const show = (
    ev: MouseEvent,
    msg: string,
    option: any = {
      anchorOrigin: {
        vertical: "top",
        horizontal: "left",
      },
      transformOrigin: {
        vertical: "top",
        horizontal: "left",
      },
    },
    head: string | undefined = undefined
  ) => {
    setAnchorEl(ev.currentTarget as HTMLElement);
    setMessage(msg);
    setPopoverProps(option);
    setHeader(head);
  };
  const hide = () => {
    setAnchorEl(undefined);
  };
  const classes = useStyles();
  return [
    <Popover
      open={Boolean(anchorEl)}
      anchorEl={anchorEl}
      anchorOrigin={{
        vertical: "bottom",
        horizontal: "left",
      }}
      transformOrigin={{
        vertical: "top",
        horizontal: "left",
      }}
      marginThreshold={0}
      onClose={hide}
      className={classes.popover}
      {...popoverProps}>
      {header && (
        <Typography
          style={{ display: "inline-block", fontWeight: "bold" }}
          className={classes.typography}
          variant="body2">
          {header}
        </Typography>
      )}
      <Typography style={{ display: "inline-block" }} className={classes.typography} variant="body2">
        {message}
      </Typography>
    </Popover>,
    show,
    hide,
  ];
}
