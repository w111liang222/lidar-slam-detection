import * as React from "react";
import Snackbar from "@mui/material/Snackbar";
import MuiAlert, { AlertProps } from "@mui/material/Alert";
import { useState } from "react";

const Alert = React.forwardRef<HTMLDivElement, AlertProps>(function Alert(props, ref) {
  return <MuiAlert elevation={6} ref={ref} variant="filled" {...props} />;
});

export default function useInfoShow(): [React.ReactNode, (message: string, duration?: number) => void] {
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
      autoHideDuration={duration}
      onClose={() => {
        setOpen(false);
      }}
      anchorOrigin={{ vertical: "top", horizontal: "center" }}>
      <Alert
        onClose={() => {
          setOpen(false);
        }}
        severity="success"
        sx={{ width: "100%" }}>
        {message}
      </Alert>
    </Snackbar>,
    show,
  ];
}
