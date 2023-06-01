import * as React from "react";
import { TextField, Grid, Box, FormControl, InputLabel, Select, MenuItem, Button, InputAdornment } from "@mui/material";
import { AddCircle, RemoveCircle, Link, LinkOff, Margin } from "@mui/icons-material";
import { useFormik } from "formik";
import produce from "immer";
import yup from "@plugins/yup-extended";
import { useImperativeHandle } from "react";
import { useEffect, useState } from "react";
import { usePopover } from "@hooks/index";

const validationSchema = yup.array().of(
  (
    yup.object({
      name: yup.string().required("invalidName"),
      output_width: yup.string(),
      output_height: yup.string(),
      undistortion: yup.boolean(),
      extrinsic_parameters: yup.array().of(yup.number().typeError("invalidNumber").required("invalidNumber")),
      intrinsic_parameters: yup.array().of(yup.number().typeError("invalidNumber").required("invalidNumber")),
      stream: yup.object({
        sink: yup.string(),
        host: yup.string().when("sink", {
          is: "udp",
          then: (yup.string().required("invalidAddress") as any).ipv4("invalidAddress"),
          otherwise: yup.string(),
        }),
        port: yup.number().when("sink", {
          is: "none",
          then: yup.number(),
          otherwise: yup.number().typeError("invalidNumber").required("invalidNumber"),
        }),
      }),
    }) as any
  ).uniqueProperty("name", "duplicatedName")
);

export type CameraType = {
  extrinsic_parameters: number[];
  intrinsic_parameters: number[];
  name: string;
  output_width?: number | string;
  output_height?: number | string;
  undistortion: boolean;
  stream: {
    sink: string;
    host: string;
    port: any;
  };
};

export interface Props {
  initialValues: CameraType[];
  t?: (x: string) => string;
  status?: LSD.Status;
}

export interface Ref {
  values: CameraType[];
  isValid: boolean;
  validationSchema: typeof validationSchema;
}

function CameraList({ initialValues, t = (x) => x, status }: Props, ref: React.Ref<Ref>) {
  for (let i = 0; i < initialValues.length; i++) {
    if (initialValues[i].stream == undefined) {
      initialValues[i].stream = {
        sink: "none",
        host: "",
        port: undefined,
      };
    }
    if (initialValues[i].undistortion == undefined) {
      initialValues[i].undistortion = false;
    }
  }
  const formik = useFormik({
    initialValues,
    validationSchema,
    onSubmit: () => {},
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
      produce((cameraArray) => {
        cameraArray.push({
          name: "",
          intrinsic_parameters: new Array(8).fill(0),
          extrinsic_parameters: [0, 0, 0, 0, 0, 0],
          output_width: "",
          output_height: "",
          undistortion: false,
          stream: {
            sink: "none",
            host: "",
            port: undefined,
          },
        } as CameraType);
      })
    );
  };
  const remove = (index: number) => {
    setShowLink(false);
    setValues(
      produce((cameraArray) => {
        cameraArray.splice(index, 1);
      })
    );
  };
  return (
    <>
      {popover}
      {values.map(
        (
          { name, output_width, output_height, undistortion, intrinsic_parameters, extrinsic_parameters, stream },
          index
        ) => (
          <div key={index}>
            <Grid container alignItems="center">
              <Grid item sm>
                <TextField
                  label={t("name")}
                  name={`${index}.name`}
                  value={name}
                  onChange={handleChange}
                  error={Boolean(errors[index]?.name)}
                  helperText={t(errors[index]?.name as string)}
                />
              </Grid>
              <Grid item sm>
                <TextField
                  label={t("imageWidth")}
                  name={`${index}.output_width`}
                  value={output_width ? output_width : ""}
                  onChange={handleChange}
                />
              </Grid>
              <Grid item sm>
                <TextField
                  label={t("imageHeight")}
                  name={`${index}.output_height`}
                  value={output_height ? output_height : ""}
                  onChange={handleChange}
                />
              </Grid>
              <Grid item sm>
                <FormControl>
                  <InputLabel>{t("undistortion")}</InputLabel>
                  <Select
                    label={t("undistortion")}
                    name={`${index}.undistortion`}
                    value={undistortion}
                    onChange={handleChange}>
                    <MenuItem value={"true"}>{t("True")}</MenuItem>
                    <MenuItem value={"false"}>{t("False")}</MenuItem>
                  </Select>
                </FormControl>
              </Grid>
              <Grid item sm>
                <FormControl>
                  <InputLabel>{t("stream")}</InputLabel>
                  <Select label={t("stream")} name={`${index}.stream.sink`} value={stream.sink} onChange={handleChange}>
                    <MenuItem value={"none"}>{t("False")}</MenuItem>
                    {(stream.sink == "udp" || stream.sink == "none") && <MenuItem value={"udp"}>{t("True")}</MenuItem>}
                    {stream.sink == "rtsp" && <MenuItem value={"rtsp"}>{t("True")}</MenuItem>}
                  </Select>
                </FormControl>
              </Grid>
              <div
                style={{ width: "24px" }}
                onMouseEnter={(ev) => {
                  if (showLink && status?.camera?.hasOwnProperty(name) && status?.camera[name].valid) {
                    let info = (status?.camera[name].in_w).toString() + "x" + (status?.camera[name].in_h).toString();
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
                  status?.camera?.hasOwnProperty(name) && status?.camera[name].valid ? (
                    <Link fontSize="medium" color="primary" />
                  ) : (
                    <LinkOff fontSize="medium" />
                  )
                ) : undefined}
              </div>
              <Grid item sm>
                <Box display="flex">
                  <Box flexGrow={1} />
                  <Button color="primary" onClick={() => remove(index)} className="remove-button">
                    <RemoveCircle fontSize="large" />
                  </Button>
                </Box>
              </Grid>
            </Grid>

            <div hidden={stream.sink == "none"}>
              <Grid container>
                <Grid item md>
                  <FormControl>
                    <InputLabel>{t("streamProtocol")}</InputLabel>
                    <Select
                      label={t("streamProtocol")}
                      name={`${index}.stream.sink`}
                      value={stream.sink}
                      onChange={handleChange}>
                      <MenuItem value={"udp"}>{t("UDP")}</MenuItem>
                      <MenuItem value={"rtsp"}>{t("RTSP")}</MenuItem>
                      {stream.sink == "none" && <MenuItem value={"none"}>{t("NONE")}</MenuItem>}
                    </Select>
                  </FormControl>
                </Grid>
                <Grid hidden={stream.sink != "udp"} item md>
                  <TextField
                    label={t("destination")}
                    value={stream.host}
                    onChange={handleChange}
                    name={`${index}.stream.host`}
                    error={Boolean(errors[index]?.stream?.host)}
                    helperText={t(errors[index]?.stream?.host as string)}
                  />
                </Grid>
                <Grid item md>
                  <TextField
                    label={t("port")}
                    value={stream.port}
                    onChange={handleChange}
                    name={`${index}.stream.port`}
                    error={Boolean(errors[index]?.stream?.port)}
                    helperText={t(errors[index]?.stream?.port as string)}
                  />
                </Grid>
                <Grid hidden={!(stream.sink == "rtsp")} item md />

                <div style={{ width: "24px" }}></div>
              </Grid>
            </div>

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
              <div style={{ width: "24px" }}></div>
            </Grid>
            {/* intrinsic */}
            <Grid container>
              {["fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2"].map((key, parameterIndex) => (
                <Grid item md key={parameterIndex}>
                  <TextField
                    label={key}
                    name={`${index}.intrinsic_parameters.${parameterIndex}`}
                    value={intrinsic_parameters[parameterIndex]}
                    onChange={handleChange}
                    error={Boolean(errors[index]?.intrinsic_parameters?.[parameterIndex])}
                    helperText={t(errors[index]?.intrinsic_parameters?.[parameterIndex] || "")}
                  />
                </Grid>
              ))}
              <div style={{ width: "24px" }}></div>
            </Grid>

            <div style={{ marginBottom: "0.5rem" }}></div>
          </div>
        )
      )}
      <Box display="flex">
        <Box flexGrow={1} />
        <div hidden={values.length < 8 ? false : true}>
          <Button onClick={add} className="float-right">
            <AddCircle fontSize="large" />
          </Button>
        </div>
      </Box>
    </>
  );
}

export default React.memo(React.forwardRef(CameraList));
