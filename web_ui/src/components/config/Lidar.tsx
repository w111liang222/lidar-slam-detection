import * as React from "react";
import { TextField, Grid, Box, FormControl, InputLabel, Select, MenuItem, Button, InputAdornment } from "@mui/material";
import { AddCircle, RemoveCircle, Link, LinkOff } from "@mui/icons-material";
import { useFormik } from "formik";
import produce from "immer";
import yup from "@plugins/yup-extended";
import { useImperativeHandle } from "react";
import { useEffect, useState } from "react";
import { usePopover } from "@hooks/index";
import "./index.css";

const validationSchema = yup.array().of(
  (
    yup.object({
      name: yup.string().required("invalidName"),
      port: yup
        .number()
        .typeError("invalidNumber")
        .required("invalidNumber")
        .min(1024, "invalidPort")
        .max(49151, "invalidPort"),
      extrinsic_parameters: yup.array().of(yup.number().typeError("invalidNumber").required("invalidNumber")),
      range: yup.array().of(yup.number().typeError("invalidNumber").required("invalidNumber")),
    }) as any
  ).uniqueProperty("port", "duplicatedPorts")
);

type ILidar = LSD.Config["lidar"][0];

export interface Props {
  lidarDefaultArray: ILidar[];
  initialValues: ILidar[];
  t?: (x: string) => string;
  status?: LSD.Status;
}

export interface Ref {
  values: Props["initialValues"];
  isValid: boolean;
  validationSchema: typeof validationSchema;
}

function LidarList({ lidarDefaultArray, initialValues, t = (x) => x, status }: Props, ref: React.Ref<Ref>) {
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
      produce((lidarArray) => {
        lidarArray.push({
          name: "",
          port: 0,
          extrinsic_parameters: [0, 0, 0, 0, 0, 0],
          range: [0, 0, 0, 0, 0, 0],
        } as ILidar);
      })
    );
  };
  // todo: add id and normalize lidar array;
  const remove = (index: number) => {
    setShowLink(false);
    setValues(
      produce((lidarArray) => {
        lidarArray.splice(index, 1);
      })
    );
  };
  const update = (index: number, name: string) => {
    setShowLink(false);
    setValues(
      produce((lidarArray) => {
        const lidar = lidarDefaultArray.find((lidar) => lidar.name === name);
        if (lidar) lidarArray[index] = lidar;
      })
    );
  };
  const updateRoi = (lidarIndex: number, event: any) => {
    if (event.target.value < 0) {
      event.target.value = 0;
    }
    setValues(
      produce((lidarArray) => {
        lidarArray[lidarIndex].range[0] = -event.target.value as number;
        lidarArray[lidarIndex].range[1] = -event.target.value as number;
        lidarArray[lidarIndex].range[3] = event.target.value as number;
        lidarArray[lidarIndex].range[4] = event.target.value as number;
      })
    );
  };

  return (
    <>
      {popover}
      {values.map(({ name, port, extrinsic_parameters, range }, index) => (
        <div key={index}>
          <Grid container alignItems="center">
            <Grid item sm={4}>
              <FormControl>
                <InputLabel>{t("name")}</InputLabel>
                <Select
                  label={t("name")}
                  name={`${index}.name`}
                  value={name}
                  onChange={(e: any) => {
                    update(index, e.target.value);
                  }}>
                  {lidarDefaultArray.map((x, i) => (
                    <MenuItem value={x.name} key={x.name}>
                      {x.name}
                    </MenuItem>
                  ))}
                </Select>
              </FormControl>
            </Grid>
            <Grid item sm>
              <TextField
                label={t("port")}
                name={`${index}.port`}
                value={port}
                onChange={handleChange}
                error={Boolean(errors[index]?.port)}
                helperText={t(errors[index]?.port!)}
              />
            </Grid>
            <Grid item sm>
              <TextField
                label={t("roi")}
                value={Math.max(-range[0], -range[1], range[3], range[4])}
                InputProps={{
                  endAdornment: <InputAdornment position="end">m</InputAdornment>,
                }}
                onChange={(e: any) => {
                  updateRoi(index, e);
                }}
                error={Boolean(errors[index]?.range?.[3])}
                helperText={t(errors[index]?.range?.[3]!)}
              />
            </Grid>
            <Grid item sm>
              <div
                style={{ width: "24px" }}
                onMouseEnter={(ev) => {
                  if (
                    showLink &&
                    status?.lidar?.hasOwnProperty(`${index}-${name}`) &&
                    status?.lidar[`${index}-${name}`].valid
                  ) {
                    let info =
                      status?.lidar[`${index}-${name}`].num[1].toString() +
                      "x" +
                      status?.lidar[`${index}-${name}`].num[0].toString();
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
                  status?.lidar?.hasOwnProperty(`${index}-${name}`) && status?.lidar[`${index}-${name}`].valid ? (
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
        <div hidden={values.length < 4 ? false : true}>
          <Button onClick={add} className="float-right">
            <AddCircle fontSize="large" />
          </Button>
        </div>
      </Box>
    </>
  );
}

export default React.memo(React.forwardRef(LidarList));
