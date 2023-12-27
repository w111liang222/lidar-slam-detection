import React from "react";
import { Backdrop } from "@mui/material";
import makeStyles from "@mui/styles/makeStyles";
import CircularProgress from "@mui/material/CircularProgress";

const useStyles = makeStyles((theme) => ({
  backdrop: {
    zIndex: theme.zIndex.appBar - 1,
    color: "#fff",
  },
}));

export default function Loading() {
  const classes = useStyles();

  return (
    <Backdrop open className={classes.backdrop}>
      <CircularProgress color="inherit" />
    </Backdrop>
  );
}
