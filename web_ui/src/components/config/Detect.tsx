import { Box, Chip, Grid, InputAdornment, TextField, Typography } from "@mui/material";
import { Autocomplete } from "@mui/material";
import yup from "@plugins/yup-extended";
import { useFormik } from "formik";
import produce from "immer";
import * as React from "react";
import { useEffect, useImperativeHandle } from "react";

const validationSchema = yup.object({});

type Value = LSD.Config["output"];
type ObjectKey = keyof Value["object"];
export interface Props {
  initialValues: Value;
  t?: (x: string | undefined) => string;
}

export interface Ref {
  values: Value;
  isValid: boolean;
  validationSchema: typeof validationSchema;
}

function Detect({ initialValues, t = (x) => x || "" }: Props, ref: React.Ref<Ref>) {
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

  const classCandidateArray = [];
  const classActiveArray = [];
  for (let key of [...Object.keys(values.object)]) {
    if (key === "use") continue;
    classCandidateArray.push(key);
    if (values.object[key as ObjectKey]) {
      classActiveArray.push(key);
    }
  }

  return (
    <Box flexGrow={1}>
      <Typography style={{ marginTop: "1rem" }} variant="subtitle2" className="subtitle-no-mt">
        {t("objects")}
      </Typography>
      <Box mb="1rem">
        <Autocomplete
          options={classCandidateArray}
          value={classActiveArray}
          onChange={(e, items) => {
            setValues(
              produce((output) => {
                for (let key in values.object) {
                  if (key !== "use") output.object[key as ObjectKey] = items.includes(key);
                }
              })
            );
          }}
          multiple
          filterSelectedOptions
          renderTags={(value, getTagProps) =>
            value.map((optionValue, index) => (
              <Chip variant="outlined" label={t(optionValue)} {...getTagProps({ index })} />
            ))
          }
          renderInput={(params) => <TextField {...params} placeholder="" />}
          renderOption={(props, option) => (
            <Box component="li" {...props}>
              {t(option)}
            </Box>
          )}
        />
      </Box>
    </Box>
  );
}

export default React.memo(React.forwardRef(Detect));
