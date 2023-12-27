import React, { useEffect, useRef } from "react";
import { Box, Container, Button } from "@mui/material";
import makeStyles from "@mui/styles/makeStyles";

import JSONEditor from "jsoneditor";
import "jsoneditor/dist/jsoneditor.min.css";

import { getConfig, postConfig, reboot } from "@rpc/http";
import { HOSTNAME, PORT } from "@rpc/http-upgrade";
import { useDialog, useSnackbar } from "@hooks";
import { useTranslation } from "react-i18next";

const useStyles = makeStyles((theme) => ({
  editor: {
    height: "50rem",
  },
}));

export default function BoardConfig({ setIsLoaded }) {
  const { t } = useTranslation();
  const editorRef = useRef(null);
  const editorEl = useRef(null);
  useEffect(() => {
    const editor = new JSONEditor(editorEl.current, {
      mode: "code",
    });
    editorRef.current = editor;
    return () => {
      editor.destroy();
    };
  }, []);

  useEffect(() => {
    handleDownload();
  }, []);

  const handleDownload = async () => {
    setIsLoaded(false);
    editorRef.current.set(await getConfig(false));
    setIsLoaded(true);
    showMessage(t("configReset"), 2000);
  };

  const handleUpload = async () => {
    setIsLoaded(false);
    const configUpdateResponse = await postConfig(editorRef.current.get(), false);
    setIsLoaded(true);
    const retStatus = configUpdateResponse.status;
    showMessage(t("configUpdated"), 2000);
    if (retStatus === "Reboot") {
      openDialog();
    }
  };
  const [snackbar, showMessage] = useSnackbar();
  const handleRebootCancel = async () => {
    await reboot(false);
    await handleDownload();
  };
  const handleRebootConfirm = async () => {
    showMessage(t("doReboot"));
    const doRebootResponse = await reboot(true);
    let href = `http://${doRebootResponse.hostname}`;
    const port = window.location.host.split(":")[1];
    href += port == null ? "" : `:${port}`;
    window.location.href = href;
  };
  const [dialog, openDialog] = useDialog({
    content: t("needReboot"),
    onConfirm: handleRebootConfirm,
    onCancel: handleRebootCancel,
    t: t,
  });

  const classes = useStyles();
  return (
    <div>
      <Container maxWidth="md">
        <div ref={editorEl} className={classes.editor} />
        <Box display="flex">
          <Button
            variant="contained"
            color="primary"
            onClick={() => {
              window.open(`http://${HOSTNAME}:${PORT}`);
            }}>
            {t("OpenTerminal")}
          </Button>
          <Box flexGrow={1} />
          <Button onClick={handleDownload} color="primary" variant="contained">
            {t("reset")}
          </Button>
          <Button onClick={handleUpload} color="primary" variant="contained">
            {t("update")}
          </Button>
        </Box>
        {dialog}
        {snackbar}
      </Container>
    </div>
  );
}
