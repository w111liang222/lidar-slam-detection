import React, { useState } from "react";
import { Backdrop, Button, Collapse, FormControl, Grid, IconButton, InputLabel, OutlinedInput } from "@mui/material";
import makeStyles from "@mui/styles/makeStyles";
import ErrorOutlineRoundedIcon from "@mui/icons-material/ErrorOutlineRounded";
import CloseIcon from "@mui/icons-material/Close";
import { useTranslation } from "react-i18next";
import { licenceRegister } from "@rpc/http";
import { Alert } from "@mui/material";
import { useNavigate } from "react-router-dom";
import { isPermit } from "@rpc/http";
import { useRequest } from "ahooks";

function sleep(time: number) {
  return new Promise((resolve) => setTimeout(resolve, time));
}

function secondsToDhms(seconds: number, t: any) {
  if (seconds == -1) {
    return t("Permanent");
  }
  var d = Math.floor(seconds / (3600 * 24));
  var h = Math.floor((seconds % (3600 * 24)) / 3600);
  var m = Math.floor((seconds % 3600) / 60);
  var s = Math.floor(seconds % 60);

  var dDisplay = d > 0 ? d + t("Day") : "";
  var hDisplay = h > 0 ? h + t("Hour") : "";
  var mDisplay = m > 0 ? m + t("Minute") : "";
  var sDisplay = s > 0 ? s + t("Second") : "";
  return dDisplay + hDisplay + mDisplay + sDisplay;
}

const useStyles = makeStyles((theme) => ({
  backdrop: {
    zIndex: theme.zIndex.appBar + 1,
  },
  paper: {
    position: "absolute",
    width: 340,
    backgroundColor: theme.palette.background.paper,
    boxShadow: theme.shadows[5],
    padding: theme.spacing(2, 4, 3),
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

export default function AskMoney() {
  const classes = useStyles();
  const navigate = useNavigate();
  const [isLicenced, setIsLicenced] = React.useState(false);
  const [licenceTime, setLicenceTime] = React.useState(0);
  const [openSuccess, setOpenSuccess] = React.useState(false);
  const [openError, setOpenError] = React.useState(false);
  const [modalStyle] = React.useState(getModalStyle);
  const { t } = useTranslation();
  const [validNumber, setValidNumber] = useState("");

  const { data, run, cancel } = useRequest(isPermit, {
    pollingInterval: 1000,
    loadingDelay: 1000,
    onSuccess: (data) => {
      if (data.licence != "error") {
        setIsLicenced(true);
        setLicenceTime(data.time);
      }
      cancel();
    },
  });

  const onSubmit = async () => {
    const result = await licenceRegister(validNumber);
    if (result == "ok") {
      setOpenSuccess(true);
      sleep(2000).then(() => {
        navigate("/home", { replace: true });
      });
    } else {
      setOpenError(true);
      sleep(2000).then(() => {
        setOpenError(false);
      });
    }
  };

  const body = (
    <div style={modalStyle} className={classes.paper}>
      <div className={classes.root}>
        <Collapse in={openSuccess}>
          <Alert
            action={
              <IconButton
                aria-label="close"
                color="inherit"
                size="small"
                onClick={() => {
                  setOpenSuccess(false);
                }}>
                <CloseIcon fontSize="inherit" />
              </IconButton>
            }>
            {t("RegisterSuccess")}
          </Alert>
        </Collapse>
        <Collapse in={openError}>
          <Alert
            severity="error"
            action={
              <IconButton
                aria-label="close"
                color="inherit"
                size="small"
                onClick={() => {
                  setOpenError(false);
                }}>
                <CloseIcon fontSize="inherit" />
              </IconButton>
            }>
            {t("RegisterError")}
          </Alert>
        </Collapse>
      </div>
      <Grid container>
        <Grid item xs={1}></Grid>
        <Grid item xs={2}>
          <ErrorOutlineRoundedIcon color={isLicenced ? "primary" : "error"} fontSize="large" />
        </Grid>
        <Grid item xs={8}>
          <h1>{isLicenced ? t("Permit") : t("NoPermit")}</h1>
        </Grid>
      </Grid>
      <p>{isLicenced ? t("LicenceUage") + secondsToDhms(licenceTime, t) : t("NoPermitDescription")}</p>
      {!isLicenced && (
        <FormControl fullWidth variant="outlined">
          <InputLabel>{t("ValidNumber")}</InputLabel>
          <OutlinedInput
            value={validNumber}
            onChange={(value) => {
              setValidNumber(value.target.value);
            }}
            // labelWidth={60}
          />
          <Button style={{ marginTop: "1rem" }} variant="outlined" color="primary" onClick={onSubmit}>
            {t("Register")}
          </Button>
        </FormControl>
      )}
    </div>
  );

  return (
    <Backdrop open className={classes.backdrop}>
      {body}
    </Backdrop>
  );
}
