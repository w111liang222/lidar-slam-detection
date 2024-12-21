import {
  Box,
  Chip,
  FormControl,
  FormControlLabel,
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
import { Autocomplete } from "@mui/material";
import yup from "@plugins/yup-extended";
import { FormikErrors, useFormik } from "formik";
import { produce } from "immer";
import * as React from "react";
import { useEffect, useImperativeHandle } from "react";

const validationSchema = yup.object({
  mapping: yup.object({
    key_frames_interval: yup
      .array()
      .of(
        yup
          .number()
          .typeError("invalidNumber")
          .required("invalidNumber")
          .min(0, "invalidInterval")
          .max(100, "invalidInterval")
      ),
    map_resolution: yup
      .number()
      .typeError("invalidNumber")
      .required("invalidNumber")
      .min(0.1, "invalidMapResolution")
      .max(1, "invalidMapResolution"),
    key_frames_range: yup
      .number()
      .typeError("invalidNumber")
      .required("invalidNumber")
      .min(0, "invalidKeyFrameRange")
      .max(1000, "invalidKeyFrameRange"),
    sensor_input: yup.array().of(yup.string()).min(0, "emptySensorInput"),
    ground_constraint: yup.boolean(),
    gravity_constraint: yup.boolean(),
    loop_closure: yup.boolean(),
  }),
  localization: yup.object({
    key_frames_interval: yup
      .array()
      .of(
        yup
          .number()
          .typeError("invalidNumber")
          .required("invalidNumber")
          .min(0, "invalidInterval")
          .max(100, "invalidInterval")
      ),
    map_resolution: yup
      .number()
      .typeError("invalidNumber")
      .required("invalidNumber")
      .min(0, "invalidLocResolution")
      .max(1, "invalidLocResolution"),
    map_path: yup.string().required("invalidName"),
    sensor_input: yup.array().of(yup.string()).min(0, "emptySensorInput"),
    colouration: yup.boolean(),
  }),
  origin: yup.object({
    use: yup.boolean(),
    latitude: yup.number().typeError("invalidNumber").required("invalidNumber"),
    longitude: yup.number().typeError("invalidNumber").required("invalidNumber"),
    altitude: yup.number().typeError("invalidNumber").required("invalidNumber"),
  }),
});

type ISLAM = LSD.Config["slam"];

export interface Props {
  initialValues: ISLAM;
  config: LSD.Config;
  t?: (x: string) => string;
}

export interface Ref {
  values: ISLAM;
  isValid: boolean;
  validationSchema: typeof validationSchema;
}

export default React.memo(
  React.forwardRef(({ initialValues, config, t = (x) => x }: Props, ref: React.Ref<Ref>) => {
    const [sensorOptions, setSensorOptions] = React.useState<string[]>([]);
    const [cameraSensorOptions, setCameraSensorOptions] = React.useState<string[]>([]);
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
      let sensors: string[] = [];
      for (let i = 0; i < config.lidar.length; i++) {
        sensors[i] = i.toString() + "-" + config.lidar[i].name;
      }
      let cameraSensors: string[] = [];
      for (let i = 0; i < config.camera.length; i++) {
        cameraSensors[i] = config.camera[i].name;
      }
      setSensorOptions(sensors);
      setCameraSensorOptions(cameraSensors);
    }, [initialValues, config]);

    const { values, handleChange, touched, errors, setValues } = formik;

    if (typeof Array.prototype.subsetTo !== "function") {
      Array.prototype.subsetTo = function (arr) {
        return this.filter((v) => arr.includes(v));
      };
    }

    return (
      <>
        <Typography style={{ marginTop: "0rem" }} variant="subtitle2" className="subtitle-no-mt">
          {t("workmode")}
        </Typography>
        <Grid style={{ marginTop: "0.5rem" }} container>
          <Grid item md>
            <FormControl>
              <InputLabel>{t("Mode")}</InputLabel>
              <Select label={t("mode")} name={`mode`} value={values.mode} onChange={handleChange}>
                <MenuItem value={"mapping"}>{t("mapping")}</MenuItem>
                <MenuItem value={"localization"}>{t("localization")}</MenuItem>
              </Select>
            </FormControl>
          </Grid>
          <Grid item md />
          <Grid item md />
        </Grid>
        {/* Mapping */}
        <div hidden={values.mode != "mapping"}>
          <Typography style={{ marginTop: "1rem" }} variant="subtitle2" className="subtitle-no-mt">
            {t("slamMethod")}
          </Typography>
          <Grid style={{ marginTop: "0.5rem" }} container>
            <Grid item md>
              <FormControl>
                <InputLabel>{t("method")}</InputLabel>
                <Select label={t("method")} name={`method`} value={values.method || "RTKM"} onChange={handleChange}>
                  <MenuItem value={"RTKM"}>{t("RTKM")}</MenuItem>
                  <MenuItem value={"FastLIO"}>{t("FastLIO")}</MenuItem>
                </Select>
              </FormControl>
            </Grid>
            <Grid item md />
            <Grid item md />
          </Grid>
          <Typography style={{ marginTop: "1rem" }} variant="subtitle2" className="subtitle-no-mt">
            {t("sensor")}
          </Typography>
          <Box mb="1rem">
            <Autocomplete
              options={[...sensorOptions, ...cameraSensorOptions, "RTK", "IMU"]}
              value={values.mapping.sensor_input.subsetTo([...sensorOptions, ...cameraSensorOptions, "RTK", "IMU"])}
              onChange={(e, items) => {
                setValues(
                  produce((values) => {
                    values.mapping.sensor_input = items;
                  })
                );
              }}
              multiple
              filterSelectedOptions
              renderTags={(value, getTagProps) =>
                value.map((optionValue, index) => (
                  <Chip
                    variant="outlined"
                    label={
                      cameraSensorOptions.includes(optionValue) ? t("Camera") + "-" + t(optionValue) : t(optionValue)
                    }
                    {...getTagProps({ index })}
                  />
                ))
              }
              renderInput={(params) => (
                <TextField
                  {...params}
                  placeholder=""
                  error={Boolean(errors.mapping?.sensor_input)}
                  helperText={t((errors.mapping?.sensor_input as string) || "")}
                />
              )}
              renderOption={(props, option) => (
                <Box component="li" {...props}>
                  {cameraSensorOptions.includes(option) ? t("Camera") + "-" + t(option) : t(option)}
                </Box>
              )}
            />
          </Box>
          <Typography style={{ marginTop: "1rem" }} variant="subtitle2" className="subtitle-no-mt">
            {t("mapParam")}
          </Typography>
          <Grid container>
            {["distance", "rotation"].map((key, parameterIndex) => (
              <Grid item md key={parameterIndex}>
                <TextField
                  label={t(key)}
                  name={`mapping.key_frames_interval.${parameterIndex}`}
                  value={values.mapping.key_frames_interval[parameterIndex]}
                  InputProps={{
                    endAdornment: <InputAdornment position="end">{key == "distance" ? "m" : "deg"}</InputAdornment>,
                  }}
                  onChange={handleChange}
                  error={Boolean(errors.mapping?.key_frames_interval?.[parameterIndex])}
                  helperText={t(errors.mapping?.key_frames_interval?.[parameterIndex] || "")}
                />
              </Grid>
            ))}
            <Grid item md>
              <TextField
                label={t("mapResolution")}
                name={`mapping.map_resolution`}
                value={values.mapping.map_resolution}
                InputProps={{
                  endAdornment: <InputAdornment position="end">m</InputAdornment>,
                }}
                onChange={handleChange}
                error={Boolean(errors.mapping?.map_resolution)}
                helperText={t(errors.mapping?.map_resolution || "")}
              />
            </Grid>
            <Grid item md>
              <TextField
                label={t("mapKeyFrameRange")}
                name={`mapping.key_frames_range`}
                value={values.mapping.key_frames_range}
                InputProps={{
                  endAdornment: <InputAdornment position="end">m</InputAdornment>,
                }}
                onChange={handleChange}
                error={Boolean(errors.mapping?.key_frames_range)}
                helperText={t(errors.mapping?.key_frames_range || "")}
              />
            </Grid>
          </Grid>
          <Grid container>
            <Grid item md>
              <TextField
                label={t("originLat")}
                name={"origin.latitude"}
                value={values.origin.latitude}
                onChange={handleChange}
                disabled={values.origin.use}
                error={Boolean(errors?.origin?.latitude)}
                helperText={t(errors?.origin?.latitude || "")}
              />
            </Grid>
            <Grid item md>
              <TextField
                label={t("originLon")}
                name={"origin.longitude"}
                value={values.origin.longitude}
                onChange={handleChange}
                disabled={values.origin.use}
                error={Boolean(errors?.origin?.longitude)}
                helperText={t(errors?.origin?.longitude || "")}
              />
            </Grid>
            <Grid item md>
              <TextField
                label={t("originAlt")}
                name={"origin.altitude"}
                value={values.origin.altitude}
                onChange={handleChange}
                disabled={values.origin.use}
                error={Boolean(errors?.origin?.altitude)}
                helperText={t(errors?.origin?.altitude || "")}
              />
            </Grid>
            <Grid item md>
              <Stack direction="row" spacing={1} alignItems="center">
                <Switch color="primary" name="origin.use" checked={values.origin.use} onChange={handleChange} />
                <Typography>{values.origin.use ? t("originAuto") : t("originMannual")}</Typography>
              </Stack>
            </Grid>
          </Grid>
          <Typography style={{ marginTop: "1rem" }} variant="subtitle2" className="subtitle-no-mt">
            {t("otherParam")}
          </Typography>
          <Grid container>
            <Grid item md>
              <FormControl>
                <InputLabel>{t("groundConstraint")}</InputLabel>
                <Select
                  label={t("groundConstraint")}
                  name={`mapping.ground_constraint`}
                  value={values.mapping.ground_constraint}
                  onChange={handleChange}>
                  <MenuItem value={"true"}>{t("True")}</MenuItem>
                  <MenuItem value={"false"}>{t("False")}</MenuItem>
                </Select>
              </FormControl>
            </Grid>
            <Grid item md>
              <FormControl>
                <InputLabel>{t("loopClosure")}</InputLabel>
                <Select
                  label={t("loopClosure")}
                  name={`mapping.loop_closure`}
                  value={values.mapping.loop_closure}
                  onChange={handleChange}>
                  <MenuItem value={"true"}>{t("True")}</MenuItem>
                  <MenuItem value={"false"}>{t("False")}</MenuItem>
                </Select>
              </FormControl>
            </Grid>
            <Grid item md>
              <FormControl>
                <InputLabel>{t("gravityConstraint")}</InputLabel>
                <Select
                  label={t("gravityConstraint")}
                  name={`mapping.gravity_constraint`}
                  value={values.mapping.gravity_constraint}
                  onChange={handleChange}>
                  <MenuItem value={"true"}>{t("True")}</MenuItem>
                  <MenuItem value={"false"}>{t("False")}</MenuItem>
                </Select>
              </FormControl>
            </Grid>
          </Grid>
        </div>
        {/* Localization */}
        <div hidden={values.mode != "localization"}>
          <Typography style={{ marginTop: "1rem" }} variant="subtitle2" className="subtitle-no-mt">
            {t("sensor")}
          </Typography>
          <Box mb="1rem">
            <Autocomplete
              options={[...sensorOptions, ...cameraSensorOptions, "RTK", "IMU"]}
              value={values.localization.sensor_input.subsetTo([
                ...sensorOptions,
                ...cameraSensorOptions,
                "RTK",
                ,
                "IMU",
              ])}
              onChange={(e, items) => {
                setValues(
                  produce((values) => {
                    values.localization.sensor_input = items;
                  })
                );
              }}
              multiple
              filterSelectedOptions
              renderTags={(value, getTagProps) =>
                value.map((optionValue, index) => (
                  <Chip
                    variant="outlined"
                    label={
                      cameraSensorOptions.includes(optionValue) ? t("Camera") + "-" + t(optionValue) : t(optionValue)
                    }
                    {...getTagProps({ index })}
                  />
                ))
              }
              renderInput={(params) => (
                <TextField
                  {...params}
                  placeholder=""
                  error={Boolean(errors.localization?.sensor_input)}
                  helperText={t((errors.localization?.sensor_input as string) || "")}
                />
              )}
              renderOption={(props, option) => (
                <Box component="li" {...props}>
                  {cameraSensorOptions.includes(option) ? t("Camera") + "-" + t(option) : t(option)}
                </Box>
              )}
            />
          </Box>
          <Typography style={{ marginTop: "1rem" }} variant="subtitle2" className="subtitle-no-mt">
            {t("locParam")}
          </Typography>
          <Grid container>
            {["LocDistance", "LocRotation"].map((key, parameterIndex) => (
              <Grid item md key={parameterIndex}>
                <TextField
                  label={t(key)}
                  name={`localization.key_frames_interval.${parameterIndex}`}
                  value={values.localization.key_frames_interval[parameterIndex]}
                  InputProps={{
                    endAdornment: <InputAdornment position="end">{key == "LocDistance" ? "m" : "deg"}</InputAdornment>,
                  }}
                  onChange={handleChange}
                  error={Boolean(errors.localization?.key_frames_interval?.[parameterIndex])}
                  helperText={t(errors.localization?.key_frames_interval?.[parameterIndex] || "")}
                />
              </Grid>
            ))}
            <Grid item md>
              <TextField
                label={t("locResolution")}
                name={`localization.map_resolution`}
                value={values.localization.map_resolution}
                InputProps={{
                  endAdornment: <InputAdornment position="end">m</InputAdornment>,
                }}
                onChange={handleChange}
                error={Boolean(errors.localization?.map_resolution)}
                helperText={t(errors.localization?.map_resolution || "")}
              />
            </Grid>
          </Grid>
          <Grid container>
            <Grid item md>
              <TextField
                label={t("mapPath")}
                name={`localization.map_path`}
                value={values.localization.map_path}
                onChange={handleChange}
                error={Boolean(errors.localization?.map_path)}
                helperText={t(errors.localization?.map_path || "")}
              />
            </Grid>
          </Grid>
          <Typography style={{ marginTop: "1rem" }} variant="subtitle2" className="subtitle-no-mt">
            {t("otherParam")}
          </Typography>
          <Grid container>
            <Grid item md>
              <FormControl>
                <InputLabel>{t("mapColouration")}</InputLabel>
                <Select
                  label={t("mapColouration")}
                  name={`localization.colouration`}
                  value={values.localization.colouration}
                  onChange={handleChange}>
                  <MenuItem value={"true"}>{t("True")}</MenuItem>
                  <MenuItem value={"false"}>{t("False")}</MenuItem>
                </Select>
              </FormControl>
            </Grid>
            <Grid item md />
            <Grid item md />
          </Grid>
        </div>
      </>
    );
  })
);
