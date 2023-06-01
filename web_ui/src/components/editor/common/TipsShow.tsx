import React from "react";
import Snackbar from "@mui/material/Snackbar";
import MuiAlert, { AlertColor, AlertProps } from "@mui/material/Alert";
import { useState } from "react";

const Alert = React.forwardRef<HTMLDivElement, AlertProps>(function Alert(props, ref) {
  return <MuiAlert elevation={6} ref={ref} variant="filled" {...props} />;
});

export default function useTipsShow(): [
  React.ReactNode,
  (message: string, severity: AlertColor, duration?: number) => void,
  () => void
] {
  const [open, setOpen] = useState(false);
  const [severity, setSeverity] = useState<AlertColor>("success");
  const [message, setMessage] = useState("");
  const [duration, setDuration] = useState<number>();

  const show = (message: string, severity: AlertColor, duration = 2000) => {
    setOpen(true);
    setMessage(message);
    setSeverity(severity);
    setDuration(duration);
  };

  const close = () => {
    setOpen(false);
  };

  return [
    <Snackbar
      open={open}
      autoHideDuration={duration}
      onClose={() => {
        setOpen(false);
      }}
      anchorOrigin={{ vertical: "top", horizontal: "center" }}>
      <Alert
        onClose={() => {
          setOpen(false);
        }}
        severity={severity}
        sx={{ width: "100%" }}>
        {message}
      </Alert>
    </Snackbar>,
    show,
    close,
  ];
}
