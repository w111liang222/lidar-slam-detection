import {
  FormControl,
  FormControlLabel,
  Grid,
  InputLabel,
  MenuItem,
  Select,
  Switch,
  TextField,
  Typography,
} from "@mui/material";
import yup from "@plugins/yup-extended";
import { FormikErrors, useFormik } from "formik";
import { produce } from "immer";
import * as React from "react";
import { useEffect, useImperativeHandle } from "react";

const validationSchema = yup.array().of(yup.array().of(yup.string()));

type IPIPE = LSD.Config["pipeline"];

export interface Props {
  initialValues: IPIPE;
  t?: (x: string) => string;
}

export interface Ref {
  values: IPIPE;
  isValid: boolean;
  validationSchema: typeof validationSchema;
}

export default React.memo(
  React.forwardRef(({ initialValues, t = (x) => x }: Props, ref: React.Ref<Ref>) => {
    const formik = useFormik({
      initialValues,
      validationSchema,
      onSubmit: (values) => {},
    });

    useImperativeHandle(ref, () => ({
      ...formik,
      validationSchema,
    }));

    useEffect(() => {
      formik.setValues(initialValues);
    }, [initialValues]);

    let collection = [["Source", "Sink"]];
    let detect = [["Source", "Detect", "Sink"]];
    let slam = [["Source", "SLAM", "Sink"]];
    let detect_slam = [
      ["Source", "Split", "Detect", "Merge", "Sink"],
      ["Source", "Split", "SLAM", "Merge", "Sink"],
    ];

    let getMode = (pipe: IPIPE) => {
      if (pipe.length == 1) {
        for (let i = 0; i < pipe[0].length; i++) {
          if (pipe[0][i] == "Detect") {
            return detect;
          } else if (pipe[0][i] == "SLAM") {
            return slam;
          }
        }
        return collection;
      } else {
        return detect_slam;
      }

      return [];
    };

    const { values, handleChange, touched, errors, setValues } = formik;

    return (
      <>
        <Typography style={{ marginTop: "0rem" }} variant="subtitle2" className="subtitle-no-mt">
          {t("workmode")}
        </Typography>
        <Grid style={{ marginTop: "0.5rem" }} container>
          <Grid item md>
            <FormControl>
              <InputLabel>{t("Mode")}</InputLabel>
              <Select
                label={t("mode")}
                name={``}
                value={getMode(values)}
                onChange={(ev) => {
                  setValues(ev.target.value as IPIPE);
                }}>
                <MenuItem value={collection as any}>{t("collection")}</MenuItem>
                <MenuItem value={detect as any}>{t("detect")}</MenuItem>
                <MenuItem value={slam as any}>{t("slam")}</MenuItem>
                <MenuItem value={detect_slam as any}>{t("detect_slam")}</MenuItem>
              </Select>
            </FormControl>
          </Grid>
          <Grid item md />
          <Grid item md />
        </Grid>
      </>
    );
  })
);
