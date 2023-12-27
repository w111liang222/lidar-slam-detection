import { useDialog, useSnackbar } from "@hooks/misc";
import { Grid, Box, FormControlLabel, Switch, TextField, Typography, Button } from "@mui/material";
import yup from "@plugins/yup-extended";
import { restoreConfig, reboot } from "@rpc/http";
import { useFormik } from "formik";
import React, { useEffect, useImperativeHandle } from "react";

const validationSchema = yup.object({
  lidar: yup.object({
    use: yup.boolean(),
    destination: (yup.string().required("invalidAddress") as any).ipv4("invalidAddress"),
  }),
  ins: yup.object({
    use: yup.boolean(),
    destination: (yup.string().required("invalidAddress") as any).ipv4("invalidAddress"),
  }),
}) as any;

type IAdvance = {
  lidar: LSD.Config["output"]["point_cloud"];
  ins: LSD.Config["ins"]["relay"];
};

export interface Props {
  initialValues: IAdvance;
  t?: (x: string | undefined) => string;
}

export interface Ref {
  values: IAdvance;
  isValid: boolean;
  validationSchema: typeof validationSchema;
}

function Advance({ initialValues, t = (x) => x || "" }: Props, ref: React.Ref<Ref>) {
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

  const handleRestoreConfirm = async () => {
    await restoreConfig();
    showMessage(t("doReboot"));
    const doRebootResponse = await reboot(true);
    if (doRebootResponse.hostname) {
      let href = `http://${doRebootResponse.hostname}`;
      const port = window.location.host.split(":")[1];
      href += port == null ? "" : `:${port}`;
      window.location.href = href;
    }
  };

  const handleReStoreCancel = async () => {};

  const [dialog, openDialog] = useDialog({
    content: t("needRestore"),
    onConfirm: handleRestoreConfirm,
    onCancel: handleReStoreCancel,
  });
  const [snackbar, showMessage] = useSnackbar();

  return (
    <>
      {/* Lidar UDP Relay */}
      <Typography variant="subtitle2" className="subtitle-no-mt">
        {t("pointcloudForward")}
      </Typography>

      <Grid container>
        <Grid item md>
          <FormControlLabel
            control={<Switch checked={values.lidar.use} name="lidar.use" onChange={handleChange} />}
            label="UDP"
          />
        </Grid>
        <Grid item md>
          <Box hidden={!values.lidar.use}>
            <TextField
              label={t("destination")}
              value={values.lidar.destination}
              name="lidar.destination"
              onChange={handleChange}
              error={Boolean(errors?.lidar?.destination)}
              helperText={t(errors?.lidar?.destination)}
            />
          </Box>
        </Grid>
        <Grid item md />
      </Grid>

      {/* INS UDP Relay */}
      <Typography variant="subtitle2" style={{ marginTop: "1rem" }} className="subtitle-no-mt">
        {t("insRelay")}
      </Typography>
      <Grid container>
        <Grid item md>
          <FormControlLabel
            control={<Switch checked={values.ins.use} name="ins.use" onChange={handleChange} />}
            label="UDP"
          />
        </Grid>
        <Grid item md>
          <Box hidden={!values.ins.use}>
            <TextField
              label={t("destination")}
              value={values.ins.destination}
              name="ins.destination"
              onChange={handleChange}
              error={Boolean(errors?.ins?.destination)}
              helperText={t(errors?.ins?.destination)}
            />
          </Box>
        </Grid>
        <Grid item md />
      </Grid>

      {/* <Typography variant="subtitle2" className="subtitle-no-mt">
        {t("system")}
      </Typography>
      <Grid container>
        <Grid item md>
          <Button
            variant="contained"
            color="primary"
            onClick={() => {
              openDialog();
            }}>
            {t("resetSetting")}
          </Button>
        </Grid>
      </Grid>
      {dialog} */}
    </>
  );
}

export default React.memo(React.forwardRef(Advance));
