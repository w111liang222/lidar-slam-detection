import { postUpgradeFirmware, getVersion } from "@rpc/http-upgrade";
import { usePersistFn, useRequest, useLocalStorageState } from "ahooks";
import React, { useEffect, useRef, useState, useImperativeHandle } from "react";
import { Box, Container, Backdrop, Button } from "@mui/material";
import UpgradeStatus from "./Status";
import Dialog from "@components/general/Dialog";
import { useTranslation } from "react-i18next";
import makeStyles from "@mui/styles/makeStyles";
import { Status } from "@rpc/http-upgrade";
// import { isPermit } from "@rpc/http";

async function parse(file: File) {
  let meta = file.slice(0, 4 * 2 ** 10);
  const magic = await meta.slice(0, 5).text();
  console.log(magic);
  if (magic !== "LSD") {
    return { valid: false };
  }
  meta = meta.slice(5);
  const parsePart = async (meta: Blob) => {
    let len, part;
    len = new DataView(await meta.slice(0, 4).arrayBuffer()).getInt32(0);
    part = await meta.slice(4, 4 + len).text();
    return [part, meta.slice(4 + len)] as [string, Blob];
  };
  let version, releaseNote;
  [version, meta] = await parsePart(meta);
  [releaseNote, meta] = await parsePart(meta);
  console.log({ valid: true, version, releaseNote });
  return { valid: true, version, releaseNote };
}

const MetaInfo = React.memo(({ file, isValid, setIsValid }: { file?: File; isValid?: boolean; setIsValid: any }) => {
  const { t } = useTranslation();
  const [meta, setMeta] = useState<any>();
  useEffect(() => {
    file && parse(file).then(setMeta);
  }, [file]);

  const { valid, version, releaseNote } = meta || {};
  setIsValid(valid);
  return (
    <div
      style={{
        maxWidth: "1200px",
        whiteSpace: "pre-wrap",
        overflow: "auto",
        height: "20rem",
        border: "solid",
      }}>
      {file ? valid || t("InvalidFirmware") : ""}
      {valid && <div>{version}</div>}
      {valid && <div>{releaseNote}</div>}
    </div>
  );
});

const useStyles = makeStyles((theme) => ({
  input: {
    display: "none",
  },
}));

export default function Upgrade() {
  const { t } = useTranslation();
  const classes = useStyles();

  const [file, setFile] = useLocalStorageState<File>("firmwareFile");
  const [isUpgrade, setIsUpgrade] = useLocalStorageState<boolean>("isUpgrade", false);
  const [isFileValid, setIsFileValid] = useLocalStorageState<boolean>("isFileValid", false);
  const [isUpgradeDone, setIsUpgradeDone] = useLocalStorageState<boolean>("isUpgradeDone", false);

  const [stage, setStage] = useLocalStorageState<Status["stage"]>("upgradeStage", "uploading");

  const [percentage, setPercentage] = useLocalStorageState<number>("upgradePercentage", 0);
  const [isUploading, setIsUploading] = useState(false);

  const { data, error, loading } = useRequest(getVersion, {
    pollingInterval: 1000,
    loadingDelay: 1000,
  });

  const ver = data;

  const onUploadProgress = (progressEvent: any) => {
    let percentCompleted = Math.round((progressEvent.loaded * 16) / progressEvent.total);
    setPercentage(percentCompleted);
  };

  const upload = usePersistFn(async () => {
    if (!file) {
      alert("Please select a firmware!");
      return;
    }
    // const isAcitived = await isPermit();
    // if (isAcitived.licence != "registered" && isAcitived.licence != "active") {
    //   alert(t("NoPermitUpgrade"));
    //   return;
    // }
    setIsUpgrade(true);
    setIsUpgradeDone(false);
    setIsUploading(true);
    refStatus.current?.restart();
    await postUpgradeFirmware(file, onUploadProgress).catch(function (err) {
      setIsUpgradeDone(true);
      setStage("failed");
    });
    setIsUploading(false);
  });

  const upgradeDone = () => {
    setFile(undefined);
    setIsUpgrade(false);
    setIsFileValid(false);
    setIsUpgradeDone(false);
    setStage("uploading");
    setPercentage(0);
  };

  let upgradeDoneButton = (
    <div style={{ display: "flex" }}>
      <Button
        variant="contained"
        color="primary"
        style={{
          marginLeft: "auto",
          marginTop: "0.5rem",
          marginRight: "2.2rem",
        }}
        onClick={upgradeDone}>
        {t("UpgradeDone")}
      </Button>
    </div>
  );

  let upgradeButton = (
    <div style={{ display: "flex" }}>
      <Button
        variant="contained"
        color="primary"
        disabled={error ? true : false}
        style={{ marginLeft: "auto", marginTop: "1rem", marginRight: "2.2rem" }}
        onClick={() => ref.current?.open()}>
        {t("Upgrade")}
      </Button>
    </div>
  );

  let selectFirmwire = (
    <>
      <input
        className={classes.input}
        id="button-file"
        type="file"
        onChange={(ev) => {
          if (ev.target.files) setFile(ev.target.files[0]);
        }}
      />
      <label htmlFor="button-file">
        <Button variant="contained" color="primary" component="span">
          {t("Choose")}
        </Button>
      </label>

      <div style={{ marginTop: "1rem" }}>
        <h4>{t("FirmwareInfo")}</h4>
      </div>
      <MetaInfo file={file} isValid={isFileValid} setIsValid={setIsFileValid} />
    </>
  );

  const ref = useRef<any>();
  const refStatus = useRef<any>();
  return (
    <Container style={{ marginTop: "1rem" }}>
      <div>
        <h2> {t("currentVersion") + ": " + (ver ? ver : t("UnKnown"))} </h2>
      </div>
      {isUpgrade || selectFirmwire}
      <Dialog ref={ref} content={t("uploadFirmware")} onConfirm={upload} />
      {!isFileValid || isUpgrade || upgradeButton}
      {!isFileValid || !isUpgrade || (
        <UpgradeStatus
          percentage={percentage}
          setPercentage={isUploading ? undefined : setPercentage}
          isUpgradeDone={isUpgradeDone}
          setIsUpgradeDone={setIsUpgradeDone}
          stage={stage}
          setStage={setStage}
          ref={refStatus}
        />
      )}
      {!isUpgradeDone || upgradeDoneButton}
    </Container>
  );
}
