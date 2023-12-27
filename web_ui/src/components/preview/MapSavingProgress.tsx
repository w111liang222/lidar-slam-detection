import { withStyles } from "@mui/styles";
import React from "react";
import { Backdrop, Slider } from "@mui/material";
import makeStyles from "@mui/styles/makeStyles";
import { useRequest } from "ahooks";
import { getSavingProgress } from "@rpc/http";

const PrettoSlider = withStyles({
  root: {
    height: 0,
  },
  thumb: {
    height: 0,
    width: 0,
  },
  active: {},
  track: {
    height: 14,
    borderRadius: 6,
    backgroundColor: "#fff",
  },
  rail: {
    height: 14,
    borderRadius: 6,
    backgroundColor: "#888",
  },
})(Slider);

const useStyles = makeStyles((theme) => ({
  backdrop: {
    zIndex: theme.zIndex.appBar,
  },
  paper: {
    position: "absolute",
    width: 600,
    backgroundColor: "#0000",
    opacity: 1,
  },
  root: {
    width: "100%",
    "& > * + *": {
      marginTop: theme.spacing(2),
    },
  },
}));

function getModalStyle() {
  const top = 50;
  const left = 50;

  return {
    top: `${top}%`,
    left: `${left}%`,
    transform: `translate(-${top}%, -${left}%)`,
  };
}

type Props = {
  setShowMapSaving: any;
  setPause: any;
};

export default function MapSavingProgress({ setShowMapSaving, setPause }: Props) {
  const classes = useStyles();
  const [modalStyle] = React.useState(getModalStyle);
  const [percentage, setPercentage] = React.useState(0);

  useRequest(getSavingProgress, {
    pollingInterval: 200,
    loadingDelay: 1000,
    onSuccess: (data) => {
      let percent = parseFloat(data);
      if (percent >= 100) {
        setPause(false);
        setShowMapSaving(false);
      } else {
        setPercentage(percent);
      }
    },
    onError: () => {
      setPause(false);
      setShowMapSaving(false);
    },
  });

  return (
    <Backdrop open className={classes.backdrop}>
      <div style={modalStyle} className={classes.paper}>
        <div className={classes.root}>
          <PrettoSlider color="primary" defaultValue={0} step={0.1} value={percentage} />
        </div>
      </div>
    </Backdrop>
  );
}
