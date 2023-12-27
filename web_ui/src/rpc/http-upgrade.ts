import _axios from "axios";

const url = new URL(window.location.href);
export const PORT = url.port == "" || url.port == "1234" ? 1235 : parseInt(url.port) + 500;
export const HOSTNAME = window.location.host.split(":")[0];

export const axios = _axios.create({
  baseURL: `http://${HOSTNAME}:${PORT}/`,
  // timeout: 2000,
});

export type Status = {
  stage:
    | "idle"
    | "uploading"
    | "preparing"
    | "upgrading"
    | "verifying"
    | "postprocessing"
    | "restarting"
    | "failed"
    | "success";
  percentage: number;
  log: string;
};

export type Version = {
  ver: string;
};

export async function getUpgradeStatus(): Promise<Status> {
  return (await axios.get("v1/status")).data;
}

export async function getVersion(): Promise<Version> {
  var version = await axios.get("v1/version");
  return version.data.version;
}

export async function postUpgradeFirmware(firmware: File, onUploadProgress: any) {
  const formData = new FormData();
  formData.append("file", firmware);
  return axios.post(`v1/firmware`, formData, {
    headers: {
      "Content-Type": "multipart/form-data",
    },
    onUploadProgress: onUploadProgress,
  });
}

export async function getLogFiles() {
  return (await axios.get(`v1/log-file-list`)).data;
}

export async function getLogContent(logFile: string) {
  return (await axios.get(`v1/log-content?filename=${encodeURIComponent(logFile)}`)).data;
}

export async function PowerAction(action: string) {
  return await axios.post("v1/system-power-action", { action: action });
}
