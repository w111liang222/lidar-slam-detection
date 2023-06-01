import {
  FormControl,
  Grid,
  InputAdornment,
  InputLabel,
  MenuItem,
  Select,
  Stack,
  Switch,
  TextField,
  Typography,
} from "@mui/material";
import yup from "@plugins/yup-extended";
import { useFormik } from "formik";
import React, { useEffect, useImperativeHandle } from "react";

const validationSchema = yup.object({
  device: yup.string().required("invalidName"),
  port: yup
    .number()
    .typeError("invalidNumber")
    .required("invalidNumber")
    .min(1024, "invalidPort")
    .max(49151, "invalidPort"),
  extrinsic_parameters: yup.array().of(yup.number().typeError("invalidNumber").required("invalidNumber")),
  imu_extrinsic_parameters: yup.array().of(yup.number().typeError("invalidNumber").required("invalidNumber")),
  ins_fix: yup.object({
    use: yup.boolean(),
    status: yup.number(),
    stable_time: yup.number().min(0, "invalidNumber").required("invalidNumber"),
    precision: yup.number().min(0.1, "invalidNumber").required("invalidNumber"),
  }),
  ins_float: yup.object({
    use: yup.boolean(),
    status: yup.number(),
    stable_time: yup.number().min(0, "invalidNumber").required("invalidNumber"),
    precision: yup.number().min(0.1, "invalidNumber").required("invalidNumber"),
  }),
  ins_normal: yup.object({
    use: yup.boolean(),
    status: yup.number(),
    stable_time: yup.number().min(0, "invalidNumber").required("invalidNumber"),
    precision: yup.number().min(0.1, "invalidNumber").required("invalidNumber"),
  }),
}) as any;

type IIns = LSD.Config["ins"];

export interface Props {
  initialValues: IIns;
  t?: (x: string | undefined) => string;
}

export interface Ref {
  values: IIns;
  isValid: boolean;
  validationSchema: typeof validationSchema;
}

function InsList({ initialValues, t = (x) => x || "" }: Props, ref: React.Ref<Ref>) {
  const formik = useFormik({
    initialValues,
    validationSchema,
    onSubmit: (values) => {},
  });

  useImperativeHandle(ref, () => ({
    ...formik,
    validationSchema,
  }));

  // useEffect(() => {
  //   formik.setValues(initialValues);
  // }, [initialValues]);

  const { values, setValues, handleChange, touched, errors } = formik;

  return (
    <>
      <Grid container>
        <Grid item sm>
          <TextField
            label={t("inputDevice")}
            name={`device`}
            value={values?.device}
            onChange={handleChange}
            error={Boolean(errors?.device)}
            helperText={t(errors?.device)}
          />
        </Grid>
        <Grid item sm>
          <TextField
            label={t("port")}
            name={`port`}
            value={values?.port}
            onChange={handleChange}
            error={Boolean(errors?.port)}
            helperText={t(errors?.port)}
          />
        </Grid>
        <Grid item md>
          <FormControl>
            <InputLabel>{t("insType")}</InputLabel>
            <Select label={t("insType")} name={`ins_type`} value={values?.ins_type} onChange={handleChange}>
              <MenuItem value={"2D"}>{t("2D")}</MenuItem>
              <MenuItem value={"3D"}>{t("3D")}</MenuItem>
              <MenuItem value={"6D"}>{t("6D")}</MenuItem>
            </Select>
          </FormControl>
        </Grid>
      </Grid>

      <Typography variant="subtitle2" style={{ marginTop: "1rem" }} className="subtitle-no-mt">
        {t("insExtrinsic")}
      </Typography>
      <Grid container>
        {["x", "y", "z", "roll", "pitch", "yaw"].map((key, parameterIndex) => (
          <Grid item md key={parameterIndex}>
            <TextField
              label={key}
              name={`extrinsic_parameters.${parameterIndex}`}
              value={values?.extrinsic_parameters[parameterIndex].toString().slice(0, 6)}
              InputProps={{
                endAdornment: <InputAdornment position="end"> {parameterIndex < 3 ? "m" : "deg"}</InputAdornment>,
              }}
              onChange={handleChange}
              error={Boolean(errors?.extrinsic_parameters?.[parameterIndex])}
              helperText={t(errors?.extrinsic_parameters?.[parameterIndex])}
            />
          </Grid>
        ))}
      </Grid>

      <Typography variant="subtitle2" style={{ marginTop: "1rem" }} className="subtitle-no-mt">
        {t("imuExtrinsic")}
      </Typography>
      <Grid container>
        {["x", "y", "z", "roll", "pitch", "yaw"].map((key, parameterIndex) => (
          <Grid item md key={parameterIndex}>
            <TextField
              label={key}
              name={`imu_extrinsic_parameters.${parameterIndex}`}
              value={values?.imu_extrinsic_parameters[parameterIndex].toString().slice(0, 6)}
              InputProps={{
                endAdornment: <InputAdornment position="end"> {parameterIndex < 3 ? "m" : "deg"}</InputAdornment>,
              }}
              onChange={handleChange}
              error={Boolean(errors?.imu_extrinsic_parameters?.[parameterIndex])}
              helperText={t(errors?.imu_extrinsic_parameters?.[parameterIndex])}
            />
          </Grid>
        ))}
      </Grid>

      <Typography variant="subtitle2" style={{ marginTop: "1rem" }} className="subtitle-no-mt">
        {t("insStatusTable")}
      </Typography>
      {/* RTK Fix */}
      <Grid container>
        <Grid item md key={"fixUse"}>
          <Stack direction="row" spacing={1} alignItems="center">
            <Switch color="primary" name="ins_fix.use" checked={values?.ins_fix.use} onChange={handleChange} />
            <Typography>{t("rtkFix")}</Typography>
          </Stack>
        </Grid>
        <Grid item md key={"fixStatus"} hidden={!values.ins_fix.use}>
          <TextField
            label={t("rtkStatus")}
            name={"ins_fix.status"}
            value={values?.ins_fix.status}
            onChange={handleChange}
            error={Boolean(errors?.ins_fix?.status)}
            helperText={t(errors?.ins_fix?.status)}
          />
        </Grid>
        <Grid item md key={"fixStableTime"} hidden={!values.ins_fix.use}>
          <TextField
            label={t("rtkStableTime")}
            name={"ins_fix.stable_time"}
            value={values?.ins_fix.stable_time}
            InputProps={{
              endAdornment: <InputAdornment position="end">s</InputAdornment>,
            }}
            onChange={handleChange}
            error={Boolean(errors?.ins_fix?.stable_time)}
            helperText={t(errors?.ins_fix?.stable_time)}
          />
        </Grid>
        <Grid item md key={"fixPrecision"} hidden={!values.ins_fix.use}>
          <TextField
            label={t("rtkPrecision")}
            name={"ins_fix.precision"}
            value={values?.ins_fix.precision}
            InputProps={{
              endAdornment: <InputAdornment position="end">cm</InputAdornment>,
            }}
            onChange={handleChange}
            error={Boolean(errors?.ins_fix?.precision)}
            helperText={t(errors?.ins_fix?.precision)}
          />
        </Grid>
      </Grid>
      {/* RTK Float */}
      <Grid container>
        <Grid item md key={"floatUse"}>
          <Stack direction="row" spacing={1} alignItems="center">
            <Switch color="primary" name="ins_float.use" checked={values?.ins_float.use} onChange={handleChange} />
            <Typography>{t("rtkFloat")}</Typography>
          </Stack>
        </Grid>
        <Grid item md key={"fixStatus"} hidden={!values.ins_float.use}>
          <TextField
            label={t("rtkStatus")}
            name={"ins_float.status"}
            value={values?.ins_float.status}
            onChange={handleChange}
            error={Boolean(errors?.ins_float?.status)}
            helperText={t(errors?.ins_float?.status)}
          />
        </Grid>
        <Grid item md key={"fixStableTime"} hidden={!values.ins_float.use}>
          <TextField
            label={t("rtkStableTime")}
            name={"ins_float.stable_time"}
            value={values?.ins_float.stable_time}
            InputProps={{
              endAdornment: <InputAdornment position="end">s</InputAdornment>,
            }}
            onChange={handleChange}
            error={Boolean(errors?.ins_float?.stable_time)}
            helperText={t(errors?.ins_float?.stable_time)}
          />
        </Grid>
        <Grid item md key={"fixPrecision"} hidden={!values.ins_float.use}>
          <TextField
            label={t("rtkPrecision")}
            name={"ins_float.precision"}
            value={values?.ins_float.precision}
            InputProps={{
              endAdornment: <InputAdornment position="end">cm</InputAdornment>,
            }}
            onChange={handleChange}
            error={Boolean(errors?.ins_float?.precision)}
            helperText={t(errors?.ins_float?.precision)}
          />
        </Grid>
      </Grid>
      {/* RTK Normal */}
      <Grid container>
        <Grid item md key={"NormalUse"}>
          <Stack direction="row" spacing={1} alignItems="center">
            <Switch color="primary" name="ins_normal.use" checked={values?.ins_normal.use} onChange={handleChange} />
            <Typography>{t("rtkNormal")}</Typography>
          </Stack>
        </Grid>
        <Grid item md key={"fixStatus"} hidden={!values.ins_normal.use}>
          <TextField
            label={t("rtkStatus")}
            name={"ins_normal.status"}
            value={values?.ins_normal.status}
            onChange={handleChange}
            error={Boolean(errors?.ins_normal?.status)}
            helperText={t(errors?.ins_normal?.status)}
          />
        </Grid>
        <Grid item md key={"fixStableTime"} hidden={!values.ins_normal.use}>
          <TextField
            label={t("rtkStableTime")}
            name={"ins_normal.stable_time"}
            value={values?.ins_normal.stable_time}
            InputProps={{
              endAdornment: <InputAdornment position="end">s</InputAdornment>,
            }}
            onChange={handleChange}
            error={Boolean(errors?.ins_normal?.stable_time)}
            helperText={t(errors?.ins_normal?.stable_time)}
          />
        </Grid>
        <Grid item md key={"fixPrecision"} hidden={!values.ins_normal.use}>
          <TextField
            label={t("rtkPrecision")}
            name={"ins_normal.precision"}
            value={values?.ins_normal.precision}
            InputProps={{
              endAdornment: <InputAdornment position="end">cm</InputAdornment>,
            }}
            onChange={handleChange}
            error={Boolean(errors?.ins_normal?.precision)}
            helperText={t(errors?.ins_normal?.precision)}
          />
        </Grid>
      </Grid>
    </>
  );
}

export default React.memo(React.forwardRef(InsList));
