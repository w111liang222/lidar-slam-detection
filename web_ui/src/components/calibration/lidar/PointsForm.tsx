import React, { memo, useEffect } from "react";
import { Grid, Paper, IconButton, Input, Divider, Typography, InputAdornment } from "@mui/material";
import DeleteIcon from "@mui/icons-material/Delete";
import makeStyles from "@mui/styles/makeStyles";
import produce from "immer";
import { useTranslation } from "react-i18next";

const useStyles = makeStyles((theme) => ({
  header: {
    marginBottom: "0.5rem",
  },
  container: {
    marginTop: "0rem",
    fontSize: "14px",
  },
}));

type Point = [number, number];
interface Props {
  source: THREE.Vector3[];
  target: Point[];
  isLocal: boolean;
  onChange?: (source: THREE.Vector3[], target: Point[]) => void;
}

function ConvertDMSToDD(degrees: number, minutes: number, seconds: number) {
  const d = isNaN(degrees) ? 0 : degrees;
  const m = isNaN(minutes) ? 0 : minutes;
  const s = isNaN(seconds) ? 0 : seconds;
  const dd = d + m / 60 + s / (60 * 60);
  return dd;
}

function PointsForm({ source = [], target = [], isLocal, onChange }: Props) {
  const { t } = useTranslation();

  const handleChange = (i: number, j: number, value: number | undefined) => {
    let targetValue: number = 0;
    if (value != undefined) {
      targetValue = value;
    } else {
      const dd = document.getElementById("DD" + i.toString() + "-" + j.toString()) as any;
      const mm = document.getElementById("MM" + i.toString() + "-" + j.toString()) as any;
      const ss = document.getElementById("SS" + i.toString() + "-" + j.toString()) as any;
      if (dd && mm && ss) {
        targetValue = ConvertDMSToDD(parseFloat(dd.value), parseFloat(mm.value), parseFloat(ss.value));
        console.log("gps coordinate:", targetValue);
      }
    }
    onChange &&
      onChange(
        source,
        produce(target, (t) => {
          t[i][j] = targetValue;
        })
      );
  };

  const handleDeletePoint = (i: number) => {
    if (!isLocal && i == 0) {
      return;
    }
    onChange &&
      onChange(
        produce(source, (t) => {
          t.splice(i, 1);
        }),
        produce(target, (t) => {
          t.splice(i, 1);
        })
      );
  };

  const classes = useStyles();
  let lines = [];
  for (let i = 0; i < Math.min(source.length, target.length); i++) {
    lines.push(
      <Grid container spacing={1} className={classes.container}>
        <Grid item md style={{ marginLeft: "1.5rem", marginTop: "0rem" }}>
          <Typography variant="subtitle1">{i + 1}</Typography>
        </Grid>
        <Grid item md>
          <Input value={(source[i].x as number).toFixed(2)} disabled />
        </Grid>
        <Grid item md>
          <Input value={(source[i].y as number).toFixed(2)} disabled />
        </Grid>
        <Grid item md>
          {isLocal ? (
            <Input
              type="number"
              defaultValue={target[i][0].toFixed(2)}
              onChange={(ev) => {
                handleChange(i, 0, Number(ev.target.value));
              }}
            />
          ) : (
            <>
              <Input
                id={"DD" + i.toString() + "-" + "0"}
                type="number"
                margin="dense"
                endAdornment={<InputAdornment position="end">°</InputAdornment>}
                onChange={(ev) => {
                  handleChange(i, 0, undefined);
                }}
              />
              <Input
                id={"MM" + i.toString() + "-" + "0"}
                type="number"
                margin="dense"
                endAdornment={<InputAdornment position="end">′</InputAdornment>}
                onChange={(ev) => {
                  handleChange(i, 0, undefined);
                }}
              />
              <Input
                id={"SS" + i.toString() + "-" + "0"}
                type="number"
                margin="dense"
                endAdornment={<InputAdornment position="end">″</InputAdornment>}
                onChange={(ev) => {
                  handleChange(i, 0, undefined);
                }}
              />
            </>
          )}
        </Grid>
        <Grid item md>
          {isLocal ? (
            <Input
              type="number"
              defaultValue={target[i][1].toFixed(2)}
              onChange={(ev) => {
                handleChange(i, 1, Number(ev.target.value));
              }}
            />
          ) : (
            <>
              <Input
                id={"DD" + i.toString() + "-" + "1"}
                type="number"
                margin="dense"
                endAdornment={<InputAdornment position="end">°</InputAdornment>}
                onChange={(ev) => {
                  handleChange(i, 1, undefined);
                }}
              />
              <Input
                id={"MM" + i.toString() + "-" + "1"}
                type="number"
                margin="dense"
                endAdornment={<InputAdornment position="end">′</InputAdornment>}
                onChange={(ev) => {
                  handleChange(i, 1, undefined);
                }}
              />
              <Input
                id={"SS" + i.toString() + "-" + "1"}
                type="number"
                margin="dense"
                endAdornment={<InputAdornment position="end">″</InputAdornment>}
                onChange={(ev) => {
                  handleChange(i, 1, undefined);
                }}
              />
            </>
          )}
        </Grid>
        <Grid item md container justifyContent="center" style={{ marginTop: "0rem" }}>
          <IconButton size="small" onClick={() => handleDeletePoint(i)}>
            <DeleteIcon fontSize="small" />
          </IconButton>
        </Grid>
      </Grid>
    );
  }
  return (
    <Paper className={classes.container} style={{ paddingBottom: "0.5rem" }}>
      <Grid container spacing={1} className={lines.length > 0 ? classes.header : ""}>
        <Grid item md style={{ marginLeft: "1rem" }}>
          {t("index")}
        </Grid>
        <Grid item md style={{ marginLeft: "2.5rem" }}>
          {t("xLidar")}
        </Grid>
        <Grid item md>
          {t("yLidar")}
        </Grid>
        <Grid item md>
          {isLocal ? t("xCar") : t("LatCar")}
        </Grid>
        <Grid item md>
          {isLocal ? t("yCar") : t("LonCar")}
        </Grid>
        <Grid item md>
          {t("action")}
        </Grid>
      </Grid>
      {lines.length > 0 ? <Divider /> : null}
      {lines}
    </Paper>
  );
}

export default memo(PointsForm);
