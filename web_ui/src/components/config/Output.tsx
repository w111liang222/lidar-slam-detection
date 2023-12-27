import {
  Box,
  Chip,
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
import { Autocomplete } from "@mui/material";
import yup from "@plugins/yup-extended";
import { useFormik } from "formik";
import produce from "immer";
import * as React from "react";
import { useEffect, useImperativeHandle } from "react";

const validationSchema = yup.object({
  protocol: yup.object({
    UDP: yup.object({
      destination: (yup.string().required("invalidAddress") as any).ipv4("invalidAddress"),
      coordinate: yup.string(),
      port: yup
        .number()
        .typeError("invalidNumber")
        .required("invalidNumber")
        .min(1024, "invalidPort")
        .max(49151, "invalidPort"),
    }),
    CAN: yup.object({
      device: yup.string().required("invalidName"),
    }),
  }),
  localization: yup.object({
    UDP: yup.object({
      destination: (yup.string().required("invalidAddress") as any).ipv4("invalidAddress"),
      port: yup
        .number()
        .typeError("invalidNumber")
        .required("invalidNumber")
        .min(1024, "invalidPort")
        .max(49151, "invalidPort"),
    }),
  }),
});

type Value = LSD.Config["output"];
export interface Props {
  initialValues: Value;
  t?: (x: string | undefined) => string;
}

export interface Ref {
  values: Value;
  isValid: boolean;
  validationSchema: typeof validationSchema;
}

function Output({ initialValues, t = (x) => x || "" }: Props, ref: React.Ref<Ref>) {
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

  const { values, setValues, handleChange, touched, errors } = formik;

  return (
    <>
      <Typography variant="subtitle2" className="subtitle-no-mt">
        {t("detect")}
      </Typography>
      {/* UDP */}
      <Grid container>
        <Grid item md>
          <FormControlLabel
            control={<Switch checked={values.protocol.UDP.use} name="protocol.UDP.use" onChange={handleChange} />}
            label={t("UDP")}
          />
        </Grid>
        <Grid item md>
          <div hidden={!values.protocol.UDP.use}>
            <TextField
              label={t("destination")}
              value={values.protocol.UDP.destination}
              onChange={handleChange}
              name="protocol.UDP.destination"
              error={Boolean(errors?.protocol?.UDP?.destination)}
              helperText={t(errors?.protocol?.UDP?.destination)}
            />
          </div>
        </Grid>
        <Grid item md>
          <div hidden={!values.protocol.UDP.use}>
            <TextField
              label={t("port")}
              value={values.protocol.UDP.port}
              onChange={handleChange}
              name="protocol.UDP.port"
              error={Boolean(errors?.protocol?.UDP?.port)}
              helperText={t(errors?.protocol?.UDP?.port)}
            />
          </div>
        </Grid>
        <Grid item md />
      </Grid>

      {/* CAN */}
      <Grid container>
        <Grid item md>
          <FormControlLabel
            control={<Switch checked={values.protocol.CAN.use} onChange={handleChange} name="protocol.CAN.use" />}
            label={t("CAN")}
          />
        </Grid>
        <Grid item sm>
          <div hidden={!values.protocol.CAN.use}>
            <TextField
              label={t("name")}
              name="protocol.CAN.device"
              value={values.protocol.CAN.device}
              onChange={handleChange}
              error={Boolean(errors?.protocol?.CAN?.device)}
              helperText={t(errors?.protocol?.CAN?.device)}
            />
          </div>
        </Grid>
        <Grid item md>
          <div hidden={!values.protocol.CAN.use}>
            <FormControl>
              <InputLabel>{t("baud")}</InputLabel>
              <Select
                value={values.protocol.CAN.baud}
                onChange={handleChange}
                name="protocol.CAN.baud"
                label={t("baud")}>
                {[250000, 500000, 1000000].map((x) => (
                  <MenuItem value={x} key={x}>
                    {x}
                  </MenuItem>
                ))}
              </Select>
            </FormControl>
          </div>
        </Grid>
        <Grid item md />
      </Grid>

      <Typography style={{ marginTop: "1rem" }} variant="subtitle2" className="subtitle-no-mt">
        {t("slam")}
      </Typography>
      {/* UDP */}
      <Grid container>
        <Grid item md>
          <FormControlLabel
            control={
              <Switch checked={values.localization.UDP.use} name="localization.UDP.use" onChange={handleChange} />
            }
            label={t("UDP")}
          />
        </Grid>
        <Grid item md>
          <div hidden={!values.localization.UDP.use}>
            <TextField
              label={t("destination")}
              value={values.localization.UDP.destination}
              onChange={handleChange}
              name="localization.UDP.destination"
              error={Boolean(errors?.localization?.UDP?.destination)}
              helperText={t(errors?.localization?.UDP?.destination)}
            />
          </div>
        </Grid>
        <Grid item md>
          <div hidden={!values.localization.UDP.use}>
            <TextField
              label={t("port")}
              value={values.localization.UDP.port}
              onChange={handleChange}
              name="localization.UDP.port"
              error={Boolean(errors?.localization?.UDP?.port)}
              helperText={t(errors?.localization?.UDP?.port)}
            />
          </div>
        </Grid>
        <Grid item md />
      </Grid>
    </>
  );
}

export default React.memo(React.forwardRef(Output));
