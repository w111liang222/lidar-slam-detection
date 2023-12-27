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

const validationSchema = yup.object({
  network: yup.array().of(
    yup.object({
      IP: (yup.string().required("invalidAddress") as any).ipv4("invalidAddress"),
      mask: (yup.string().required("invalidAddress") as any).ipv4("invalidAddress"),
      gateway: (yup.string().required("invalidAddress") as any).ipv4("invalidAddress"),
    })
  ),
  time_sync: yup.object({
    ptp: yup.array().of(
      (
        yup.object({
          interface: yup.string(),
        }) as any
      ).uniqueProperty("interface", "duplicatedInterface")
    ),
    ntp: yup.array().of(
      (
        yup.object({
          server: yup.string().required("invalidName"),
        }) as any
      ).uniqueProperty("server", "duplicatedServer")
    ),
    gps: yup.object({
      device: yup.string().required("invalidName"),
    }),
  }),
});

type IBoard = LSD.Config["board"];
type NetworkError = FormikErrors<{
  DHCP: boolean;
  IP: string;
  gateway: string;
  mask: string;
}>[];
type NTPError = FormikErrors<{
  server: string;
}>[];

export interface Props {
  initialValues: IBoard;
  t?: (x: string) => string;
}

export interface Ref {
  values: IBoard;
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

    const handlePtpChanged = (e: any, v: boolean) => {
      if (v) {
        setValues(
          produce((board) => {
            for (let i = 0; i < board.network.length; i++) {
              board.time_sync.ptp.push({
                interface: ("eth" + i) as string,
                mode: "master",
              });
            }
          })
        );
      } else {
        setValues(
          produce((board) => {
            board.time_sync.ptp.splice(0, values.time_sync.ptp.length);
          })
        );
      }
    };

    const handleNtpChanged = (e: any, v: boolean) => {
      if (v) {
        setValues(
          produce((board) => {
            board.time_sync.ntp.push({
              server: "",
            });
          })
        );
      } else {
        setValues(
          produce((board) => {
            board.time_sync.ntp.splice(0, values.time_sync.ntp.length);
          })
        );
      }
    };

    const { values, handleChange, touched, errors, setValues } = formik;
    return (
      <>
        <Typography variant="subtitle2" style={{ marginTop: "1rem" }} className="subtitle-no-mt">
          {t("network")}
        </Typography>
        {values.network.map(({ IP, mask, gateway, DHCP }, index: number) => (
          <Grid container key={index}>
            <Grid item md>
              <TextField
                label={t(("eth" + index) as string)}
                name={`network.${index}.IP`}
                value={IP}
                onChange={handleChange}
                error={errors && errors.network ? Boolean((errors.network as NetworkError)[index]?.IP) : false}
                helperText={errors && errors.network ? t((errors.network as NetworkError)[index]?.IP as string) : ""}
                disabled={DHCP}
              />
            </Grid>
            <Grid item md>
              <TextField
                label={t("netmask")}
                name={`network.${index}.mask`}
                value={mask}
                onChange={handleChange}
                error={errors && errors.network ? Boolean((errors.network as NetworkError)[index]?.mask) : false}
                helperText={errors && errors.network ? t((errors.network as NetworkError)[index]?.mask as string) : ""}
                disabled={DHCP}
              />
            </Grid>
            <Grid item md>
              <TextField
                label={t("gateway")}
                name={`network.${index}.gateway`}
                value={gateway}
                onChange={handleChange}
                error={errors && errors.network ? Boolean((errors.network as NetworkError)[index]?.gateway) : false}
                helperText={
                  errors && errors.network ? t((errors.network as NetworkError)[index]?.gateway as string) : ""
                }
                disabled={DHCP}
              />
            </Grid>
            <Grid item md>
              <FormControl>
                <InputLabel>{t("DHCP")}</InputLabel>
                <Select label={t("DHCP")} name={`network.${index}.DHCP`} value={DHCP} onChange={handleChange}>
                  <MenuItem value={true as any}>{t("True")}</MenuItem>
                  <MenuItem value={false as any}>{t("False")}</MenuItem>
                </Select>
              </FormControl>
            </Grid>
          </Grid>
        ))}
        <Typography style={{ marginTop: "1rem" }} variant="subtitle2" className="subtitle-no-mt">
          {t("timesync")}
        </Typography>
        {/* PTP */}
        <Grid container>
          <Grid item md>
            <FormControlLabel
              control={<Switch checked={values.time_sync.ptp.length == 0 ? false : true} onChange={handlePtpChanged} />}
              label={t("PTP")}
            />
          </Grid>
        </Grid>
        <Grid container>
          {values.time_sync.ptp.map(({}, index: number) => (
            <Grid item md>
              <FormControl>
                <InputLabel>{t(values.time_sync.ptp[index].interface)}</InputLabel>
                <Select
                  label={t("mode")}
                  name={`time_sync.ptp.${index}.mode`}
                  value={values.time_sync.ptp[index].mode}
                  onChange={handleChange}>
                  <MenuItem value={"master"}>{t("master")}</MenuItem>
                  <MenuItem value={"slave"}>{t("slave")}</MenuItem>
                </Select>
              </FormControl>
            </Grid>
          ))}
          <Grid item md />
          {values.time_sync.ptp.length == 1 && <Grid item md />}
        </Grid>
        {/* NTP */}
        <Grid container>
          <Grid item md>
            <FormControlLabel
              control={<Switch checked={values.time_sync.ntp.length == 0 ? false : true} onChange={handleNtpChanged} />}
              label={t("NTP")}
            />
          </Grid>
        </Grid>
        <Grid container>
          {values.time_sync.ntp.map(({ server }, index: number) => (
            <Grid item md>
              <TextField
                label={t("server")}
                name={`time_sync.ntp.${index}.server`}
                value={server}
                onChange={handleChange}
                error={
                  errors && errors.time_sync && errors.time_sync.ntp
                    ? Boolean((errors.time_sync.ntp as NTPError)[index]?.server)
                    : false
                }
                helperText={
                  errors && errors.time_sync && errors.time_sync.ntp
                    ? t((errors.time_sync.ntp as NTPError)[index]?.server as string)
                    : ""
                }
              />
            </Grid>
          ))}
          <Grid item md />
          {values.time_sync.ntp.length == 1 && <Grid item md />}
        </Grid>
        {/* GPS */}
        <Grid container>
          <Grid item md>
            <FormControlLabel
              control={<Switch checked={values.time_sync.gps.use} onChange={handleChange} name="time_sync.gps.use" />}
              label={t("GPS")}
            />
          </Grid>
        </Grid>
        <div hidden={!values.time_sync.gps.use}>
          <Grid style={{ marginTop: "0.5rem" }} container>
            <Grid item md>
              <TextField
                label={t("inputDevice")}
                name={`time_sync.gps.device`}
                value={values.time_sync.gps.device}
                onChange={handleChange}
                error={
                  errors && errors.time_sync && errors.time_sync.gps ? Boolean(errors.time_sync.gps.device) : false
                }
                helperText={
                  errors && errors.time_sync && errors.time_sync.gps ? t(errors.time_sync.gps.device as string) : ""
                }
              />
            </Grid>
            <Grid item md />
            <Grid item md />
          </Grid>
        </div>
      </>
    );
  })
);
