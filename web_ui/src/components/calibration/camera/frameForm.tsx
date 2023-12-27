import React, { memo } from "react";
import { Grid, Paper, IconButton, Divider, Typography, Checkbox } from "@mui/material";
import DeleteIcon from "@mui/icons-material/Delete";
import makeStyles from "@mui/styles/makeStyles";
import produce from "immer";
import { useTranslation } from "react-i18next";

const useStyles = makeStyles((theme) => ({
  header: {
    marginBottom: "0.3rem",
    marginTop: "0rem",
  },
  container: {
    padding: "0.5rem",
    marginTop: "0rem",
  },
}));

interface Props {
  source: boolean[];
  images: string[];
  onChange?: (i: number) => void;
  onDelete?: (i: number, images: string[], source: boolean[]) => void;
}

function FrameForm({ source = [], images = [], onChange, onDelete }: Props) {
  const { t } = useTranslation();

  const handleDeletePoint = (i: number) => {
    onDelete &&
      onDelete(
        i,
        produce(images, (t) => {
          t.splice(i, 1);
        }),
        produce(source, (t) => {
          t.splice(i, 1);
        })
      );
  };

  const classes = useStyles();
  let lines = [];
  for (let i = 0; i < source.length; i++) {
    lines.push(
      <Grid container spacing={0} alignItems="center" style={{ marginTop: "0rem" }}>
        <Grid item md justifyContent="center">
          <Typography style={{ marginLeft: "10px" }} variant="subtitle1">
            {("0" + (i + 1)).slice(-2)}
          </Typography>
        </Grid>
        <Grid item md>
          <img
            src={images[i]}
            width="40"
            height="30"
            onClick={(ev) => {
              onChange && onChange(i);
            }}
          />
        </Grid>
        <Grid item md>
          <Checkbox checked={source[i]} color="primary" style={{ pointerEvents: "none" }} />
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
    <Paper className={classes.container}>
      <Grid container spacing={0} className={lines.length > 0 ? classes.header : ""} style={{ marginTop: "0rem" }}>
        <Grid item xs={3}>
          {t("index")}
        </Grid>
        <Grid item xs={3}>
          {t("calibPreview")}
        </Grid>
        <Grid item xs={4}>
          {t("calibStatus")}
        </Grid>
        <Grid item xs={2}>
          {t("action")}
        </Grid>
      </Grid>
      {lines.length > 0 ? <Divider /> : null}
      {lines}
    </Paper>
  );
}

export default memo(FrameForm);
