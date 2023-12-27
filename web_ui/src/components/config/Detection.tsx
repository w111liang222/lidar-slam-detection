import { Box, Chip, Grid, InputAdornment, TextField, Typography } from "@mui/material";
import { Autocomplete } from "@mui/material";
import yup from "@plugins/yup-extended";
import { useFormik } from "formik";
import produce from "immer";
import * as React from "react";
import { useEffect, useImperativeHandle } from "react";

const validationSchema = yup.object({
  sensor_input: yup.array().of(yup.string()).min(0, "emptySensorInput"),
});

type Value = LSD.Config["detection"];
export interface Props {
  initialValues: Value;
  config: LSD.Config;
  t?: (x: string | undefined) => string;
}

export interface Ref {
  values: Value;
  isValid: boolean;
  validationSchema: typeof validationSchema;
}

function Detection({ initialValues, config, t = (x) => x || "" }: Props, ref: React.Ref<Ref>) {
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

  const { values, setValues, handleChange, touched, errors } = formik;

  if (typeof Array.prototype.subsetTo !== "function") {
    Array.prototype.subsetTo = function (arr) {
      return this.filter((v) => arr.includes(v));
    };
  }

  return (
    <>
      <Typography style={{ marginTop: "1rem" }} variant="subtitle2" className="subtitle-no-mt">
        {t("sensor")}
      </Typography>
      <Box mb="1rem">
        <Autocomplete
          options={[...sensorOptions, ...cameraSensorOptions]}
          value={values.sensor_input.subsetTo([...sensorOptions, ...cameraSensorOptions])}
          onChange={(e, items) => {
            setValues(
              produce((values) => {
                values.sensor_input = items;
              })
            );
          }}
          multiple
          filterSelectedOptions
          renderTags={(value, getTagProps) =>
            value.map((optionValue, index) => (
              <Chip
                variant="outlined"
                label={cameraSensorOptions.includes(optionValue) ? t("Camera") + "-" + t(optionValue) : t(optionValue)}
                {...getTagProps({ index })}
              />
            ))
          }
          renderInput={(params) => (
            <TextField
              {...params}
              placeholder=""
              error={Boolean(errors?.sensor_input)}
              helperText={t((errors?.sensor_input as string) || "")}
            />
          )}
          renderOption={(props, option) => (
            <Box component="li" {...props}>
              {cameraSensorOptions.includes(option) ? t("Camera") + "-" + t(option) : t(option)}
            </Box>
          )}
        />
      </Box>
    </>
  );
}

export default React.memo(React.forwardRef(Detection));
