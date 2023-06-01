import * as React from "react";
import { TextField, Grid, Box, FormControl, InputLabel, Select, MenuItem, Button, InputAdornment } from "@mui/material";
import { AddCircle, RemoveCircle, Link, LinkOff } from "@mui/icons-material";
import { useFormik } from "formik";
import produce from "immer";
import yup from "@plugins/yup-extended";
import { useImperativeHandle } from "react";
import { useEffect, useState } from "react";
import { usePopover } from "@hooks/index";

const validationSchema = yup.array().of(
  yup.object({
    name: yup.string().required("invalidName"),
    device: yup.string().required("invalidName"),
    baud: yup.number().typeError("invalidNumber").required("invalidNumber"),
    extrinsic_parameters: yup.array().of(yup.number().typeError("invalidNumber").required("invalidNumber")),
  }) as any
);

type IRadar = LSD.Config["radar"][0];

export interface Props {
  radarDefaultArray: IRadar[];
  initialValues: IRadar[];
  t?: (x: string) => string;
  status?: LSD.Status;
}

export interface Ref {
  values: Props["initialValues"];
  isValid: boolean;
  validationSchema: typeof validationSchema;
}

function RadarList({ radarDefaultArray, initialValues, t = (x) => x, status }: Props, ref: React.Ref<Ref>) {
  const formik = useFormik({
    initialValues,
    validationSchema,
    onSubmit: (values) => {},
  });
  useImperativeHandle(ref, () => ({
    ...formik,
    validationSchema,
  }));

  const [showLink, setShowLink] = useState<boolean>(true);

  useEffect(() => {
    formik.setValues(initialValues);
    setShowLink(true);
  }, [initialValues]);

  const { values, handleChange, errors, touched, setValues } = formik;
  const [popover, showPopover, hidePopover] = usePopover();

  const add = () => {
    setShowLink(false);
    setValues(
      produce((radarArray) => {
        radarArray.push({
          name: "",
          device: "",
          baud: 0,
          extrinsic_parameters: [0, 0, 0, 0, 0, 0],
        } as IRadar);
      })
    );
  };
  // todo: add id and normalize radar array;
  const remove = (index: number) => {
    setShowLink(false);
    setValues(
      produce((radarArray) => {
        radarArray.splice(index, 1);
      })
    );
  };
  const update = (index: number, name: string) => {
    setShowLink(false);
    setValues(
      produce((radarArray) => {
        const radar = radarDefaultArray.find((radar) => radar.name === name);
        if (radar) radarArray[index] = radar;
      })
    );
  };

  return (
    <>
      {popover}
      {values.map(({ name, device, baud, extrinsic_parameters }, index) => (
        <div key={index}>
          <Grid container alignItems="center">
            <Grid item sm>
              <FormControl>
                <InputLabel>{t("name")}</InputLabel>
                <Select
                  label={t("name")}
                  name={`${index}.name`}
                  value={name}
                  onChange={(e: any) => {
                    update(index, e.target.value);
                  }}>
                  {radarDefaultArray.map((x, i) => (
                    <MenuItem value={x.name} key={x.name}>
                      {x.name}
                    </MenuItem>
                  ))}
                </Select>
              </FormControl>
            </Grid>

            <Grid item sm>
              <TextField
                label={t("radarDevice")}
                name={`${index}.device`}
                value={device}
                onChange={handleChange}
                error={Boolean(errors[index]?.device)}
                helperText={t(errors[index]?.device!)}
              />
            </Grid>

            <Grid item sm>
              <FormControl>
                <InputLabel>{t("baud")}</InputLabel>
                <Select label={t("baud")} name={`${index}.baud`} value={baud} onChange={handleChange}>
                  {[100000, 200000, 500000, 1000000].map((x) => (
                    <MenuItem value={x} key={x}>
                      {x}
                    </MenuItem>
                  ))}
                </Select>
              </FormControl>
            </Grid>

            <Grid item sm>
              <div
                style={{ width: "24px" }}
                onMouseEnter={(ev) => {
                  if (
                    showLink &&
                    status?.radar?.hasOwnProperty(`${index}-${name}`) &&
                    status?.radar[`${index}-${name}`].valid
                  ) {
                    let info = status?.radar[`${index}-${name}`].num.toString();
                    showPopover(ev as any as MouseEvent, info, {
                      anchorOrigin: {
                        vertical: "top",
                        horizontal: "left",
                      },
                      transformOrigin: {
                        vertical: "bottom",
                        horizontal: "left",
                      },
                    });
                  }
                }}
                onMouseLeave={hidePopover}>
                {showLink ? (
                  status?.radar?.hasOwnProperty(`${index}-${name}`) && status?.radar[`${index}-${name}`].valid ? (
                    <Link fontSize="medium" color="primary" />
                  ) : (
                    <LinkOff fontSize="medium" />
                  )
                ) : undefined}
              </div>
            </Grid>

            <Grid item sm>
              <Box display="flex">
                <Box flexGrow={1} />
                <Button color="primary" onClick={() => remove(index)} className="remove-button">
                  {<RemoveCircle fontSize="large" />}
                </Button>
              </Box>
            </Grid>
          </Grid>

          {/* extrinsic */}
          <Grid container>
            {["x", "y", "z", "roll", "pitch", "yaw"].map((key, parameterIndex) => (
              <Grid item md key={parameterIndex}>
                <TextField
                  label={key}
                  name={`${index}.extrinsic_parameters.${parameterIndex}`}
                  value={extrinsic_parameters[parameterIndex].toString().slice(0, 6)}
                  InputProps={{
                    endAdornment: <InputAdornment position="end"> {parameterIndex < 3 ? "m" : "deg"}</InputAdornment>,
                  }}
                  onChange={handleChange}
                  error={Boolean(errors[index]?.extrinsic_parameters?.[parameterIndex])}
                  helperText={t(errors[index]?.extrinsic_parameters?.[parameterIndex] || "")}
                />
              </Grid>
            ))}
          </Grid>

          <div style={{ marginBottom: "0.5rem" }}></div>
        </div>
      ))}
      <Box display="flex">
        <Box flexGrow={1} />
        <Button onClick={add} className="float-right">
          {values.length < 4 && <AddCircle fontSize="large" />}
        </Button>
      </Box>
    </>
  );
}

export default React.memo(React.forwardRef(RadarList));
