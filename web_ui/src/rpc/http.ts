import _axios from "axios";
import * as THREE from "three";
import "protobufjs/dist/protobuf";
import protoObject from "@proto/detection.json";
import { IDetection, IFreespace, ILidarPointcloudMap, IObject } from "@proto/detection";

const pbf = window.protobuf;
console.log(pbf);

// https://github.com/protobufjs/protobuf.js/issues/1516
// let Detection: Type, LidarPointcloudMap: Type;
const root = pbf.Root.fromJSON(protoObject);
const Detection = root.lookupType("Detection");
const Freespace = root.lookupType("Freespace");
const LidarPointcloudMap = root.lookupType("LidarPointcloudMap");

const url = new URL(window.location.href);
export const PORT = url.port == "" ? 80 : parseInt(url.port);
export const HOSTNAME = window.location.host.split(":")[0];

export const axios = _axios.create({
  baseURL: `http://${HOSTNAME}:${PORT}/`,
  // timeout: 2000,
});

export async function isPermit(): Promise<{ licence: string; time: number }> {
  const response = await axios.get("v1/is-permit", { timeout: 2000 });
  return response.data;
}

export async function licenceRegister(licence: string): Promise<string> {
  const response = await axios.post("v1/licence-register", { licence: licence });
  return response.data;
}

export async function getUsers(): Promise<LSD.ClientUsers> {
  const response = await axios.get("v1/client-users", { timeout: 2000 });
  return response.data;
}

export async function addBlackList(ip: string): Promise<string> {
  const response = await axios.post("v1/add-blacklist", { ip: ip });
  return response.data;
}

export async function removeBlackList(ip: string): Promise<string> {
  const response = await axios.post("v1/remove-blacklist", { ip: ip });
  return response.data;
}

export async function getAvfuns(): Promise<LSD.webStore> {
  const response = await axios.get("v1/get-web-store");
  return response.data;
}

export async function setAvfuns(store: LSD.webStore): Promise<LSD.webStore> {
  const payload = {
    method: "set_web_store",
    params: {
      store,
    },
    jsonrpc: 2.0,
    id: 0,
  };
  return (await axios.post("api/", payload)).data.result;
}

export async function getConfig(): Promise<LSD.Config> {
  const response = await axios.get("v1/config", { timeout: 5000 });
  return response.data;
}

export async function postConfig(config: LSD.Config): Promise<{ status: string }> {
  const response = await axios.post("v1/config", config);
  return response.data;
}

export async function restoreConfig(): Promise<string> {
  const response = await axios.get("v1/restore-config", { timeout: 5000 });
  return response.data;
}

export async function getRoi(): Promise<{ roi: number[][]; inside: boolean }> {
  const rois: LSD.Config["roi"] = (await axios.get("v1/roi")).data;
  if (rois.length === 0) return { roi: [], inside: true };
  else return { roi: rois[0].contour, inside: rois[0].include };
}

export async function postRoi(roi: number[][], inside: boolean) {
  return (await axios.post("v1/roi", { contour: roi, include: inside })).data;
}

export async function restartMapping() {
  return await axios.get("v1/restart-mapping");
}

export async function rotateGroundConstraint(): Promise<string> {
  return (await axios.post("v1/rotate-ground-constraint", {})).data;
}

export async function saveMapping(name: string | undefined = undefined): Promise<string> {
  return (await axios.post("v1/save-map", { map_name: name })).data;
}

export async function setInitPose(pose: number[]) {
  return (await axios.post("v1/set-init-pose", { pose_range: pose })).data;
}

export async function getEstimatePose(pose: number[][]): Promise<number[]> {
  return (await axios.post("v1/get-estimate-pose", { pose_range: pose })).data;
}

export async function getSavingProgress(): Promise<string> {
  return (await axios.get("v1/get-save-progress")).data;
}

export async function getStatus(): Promise<LSD.Status> {
  const date = new Date().getTime();
  const response = await axios.post("v1/status", { host_date: date });
  return response.data;
}

export async function getPlayerStatus(): Promise<LSD.PlayerStatus> {
  const response = await axios.get("v1/player-status");
  return response.data;
}

export async function startPlayer() {
  return await axios.get("v1/player-start");
}

export async function pausePlayer() {
  return await axios.get("v1/player-pause");
}

export async function seekPlayer(percent: number): Promise<string> {
  return (await axios.post(`v1/player-seek`, { percent: percent })).data;
}

export async function setPlaybackRate(rate: number) {
  return await axios.post(`v1/player-rate`, { rate: rate });
}

export async function setPlayerStep(step: number): Promise<string> {
  return (await axios.post(`v1/player-step`, { step: step })).data;
}

export async function getRecordFiles(): Promise<LSD.RecordFiles> {
  const response = await axios.get("v1/record-files");
  return response.data;
}

export async function getMapFiles(): Promise<LSD.RecordFiles> {
  const response = await axios.get("v1/map-files");
  return response.data;
}

export async function playRecordFile(file: string) {
  return await axios.post("v1/play-record-file", { record_file: file });
}

export async function openMapFile(file: string) {
  return await axios.post("v1/open-map-file", { map_file: file });
}

export async function mergeMapFile(file: string) {
  return await axios.post("v1/merge-map-file", { map_file: file });
}

export async function getDetection(displayOnImage: boolean = false, sampleStep = 0): Promise<LSD.Detection> {
  const r = await axios.post(
    `v1/detection-pb`,
    { display_on_image: displayOnImage, sample_size: sampleStep },
    {
      responseType: "arraybuffer",
    }
  );
  if (!Detection) throw new Error("Detection has not been parsed!");
  let det = Detection.decode(new Uint8Array(r.data)) as any as IDetection;
  let points = new Float32Array(
    det.points && det.points.length > 0
      ? det.points.buffer.slice(det.points.byteOffset, det.points.byteOffset + det.points.byteLength)
      : []
  );

  // let radarData = (det.radar) ? det.radar : undefined;
  let radarData: { [id: string]: IObject[] } = {};
  if (det.radar) {
    for (let radar of det.radar || []) {
      if (radar.radarName != null && radar.radarObject != null) {
        radarData[radar.radarName] = radar.radarObject;
      }
    }
  } else {
    radarData = {};
  }

  let imageData: { [index: string]: Uint8Array } = {};
  for (let camera of det.image || []) {
    imageData[camera.cameraName] = new Uint8Array(
      camera.image!.buffer.slice(camera.image.byteOffset, camera.image.byteOffset + camera.image.byteLength)
    );
  }

  let freespace = det.freespace ? (Freespace.decode(det.freespace) as any as IFreespace) : undefined;
  return {
    header: det.header || undefined,
    points,
    objects: det.object || [],
    freespace: freespace,
    images: imageData,
    pose: det.pose || undefined,
    radar: radarData,
  };
}

export async function getSourceData(doDistort: boolean): Promise<LSD.Detection> {
  const r = await axios.post(`v1/source-data`, { do_distort: doDistort }, { responseType: "arraybuffer" });
  let det = Detection.decode(new Uint8Array(r.data)) as any as IDetection;
  let points = new Float32Array([]);
  let imageData: { [index: string]: Uint8Array } = {};
  for (let camera of det.image || []) {
    imageData[camera.cameraName] = new Uint8Array(
      camera.image!.buffer.slice(camera.image.byteOffset, camera.image.byteOffset + camera.image.byteLength)
    );
  }

  return {
    points,
    objects: [],
    freespace: undefined,
    images: imageData,
  };
}

export async function getProjectionForward(
  lat0: number,
  lon0: number,
  lat1: number,
  lon1: number
): Promise<[number, number]> {
  const payload = {
    method: "get_projection_forward",
    params: {
      lat0: lat0,
      lon0: lon0,
      lat1: lat1,
      lon1: lon1,
    },
    jsonrpc: 2.0,
    id: 0,
  };
  return (await axios.post("api/", payload)).data.result;
}

export async function getProjectionBackward(
  lat0: number,
  lon0: number,
  x: number,
  y: number
): Promise<[number, number]> {
  const payload = {
    method: "get_projection_backward",
    params: {
      lat0: lat0,
      lon0: lon0,
      x: x,
      y: y,
    },
    jsonrpc: 2.0,
    id: 0,
  };
  return (await axios.post("api/", payload)).data.result;
}

export async function getTransform(extrinsicParameters: number[]) {
  const payload = {
    method: "get_transform",
    params: {
      extrinsic_parameters: extrinsicParameters,
    },
    jsonrpc: 2.0,
    id: 0,
  };
  return arrayToTransform((await axios.post("api/", payload)).data.result);
}

export async function getVecFromTransform(transform: THREE.Matrix4): Promise<number[]> {
  const payload = {
    method: "get_vector_from_transform",
    params: {
      transform: transform.transpose().toArray(),
    },
    jsonrpc: 2.0,
    id: 0,
  };
  return (await axios.post("api/", payload)).data;
}

export async function getLidarPointcloud(): Promise<Float32Array[]> {
  const r = await axios.get(`v1/lidar-pointcloud-map`, {
    responseType: "arraybuffer",
  });
  if (!LidarPointcloudMap) throw new Error("LidarPointcloudMap has not been parsed!");
  let lidarPointcloudMap = LidarPointcloudMap.decode(new Uint8Array(r.data)) as any as ILidarPointcloudMap;

  let lpcs = [];
  for (let lp of lidarPointcloudMap.lp || []) {
    lpcs[Number(lp.lidarName)] = new Float32Array(
      lp.points.buffer.slice(lp.points.byteOffset, lp.points.byteOffset + lp.points.byteLength)
    );
  }
  return lpcs;
}

function _arrayBufferToBase64(buffer: ArrayBuffer) {
  var binary = "";
  var bytes = new Uint8Array(buffer);
  var len = bytes.byteLength;
  for (var i = 0; i < len; i++) {
    binary += String.fromCharCode(bytes[i]);
  }
  return window.btoa(binary);
}

function _base64ToArrayBuffer(base64: string) {
  var binary_string = window.atob(base64);
  var len = binary_string.length;
  var bytes = new Uint8Array(len);
  for (var i = 0; i < len; i++) {
    bytes[i] = binary_string.charCodeAt(i);
  }
  return bytes;
}

export function arrayToTransform(array: number[] | string) {
  if (typeof array === "string") {
    array = JSON.parse(array);
  }
  return new THREE.Matrix4().set(
    ...(array as [
      number,
      number,
      number,
      number,
      number,
      number,
      number,
      number,
      number,
      number,
      number,
      number,
      number,
      number,
      number,
      number
    ])
  );
}

export async function getMapVertex(): Promise<LSD.MapVertex> {
  const response = await axios.get("v1/map-vertex");
  return response.data;
}

export async function getMapStatus(): Promise<LSD.MapStatus> {
  const response = await axios.get("v1/map-status");
  return response.data;
}

export async function getMapEdge(): Promise<LSD.MapEdge> {
  const payload = {
    method: "get_map_edge",
    params: {},
    jsonrpc: 2.0,
    id: 0,
  };
  return (await axios.post("api/", payload)).data.result;
}

export async function getMapMeta(): Promise<LSD.MapMeta> {
  const payload = {
    method: "get_map_meta",
    params: {},
    jsonrpc: 2.0,
    id: 0,
  };
  return (await axios.post("api/", payload)).data.result;
}

export async function getColorMap() {
  const r = await axios.post("v1/get-color-map", {}, { responseType: "arraybuffer" });
  let lidarPointcloudMap = LidarPointcloudMap.decode(new Uint8Array(r.data)) as any as ILidarPointcloudMap;
  let points: Float32Array | undefined = undefined;
  let attr: Float32Array | Uint8Array | undefined = undefined;
  let type: string | undefined = undefined;
  for (let lp of lidarPointcloudMap.lp || []) {
    points = new Float32Array(
      lp.points.buffer.slice(lp.points.byteOffset, lp.points.byteOffset + lp.points.byteLength)
    );
    if (lp.type) {
      type = lp.type;
    }
    if (lp.attr) {
      attr = new Uint8Array(lp.attr.buffer.slice(lp.attr.byteOffset, lp.attr.byteOffset + lp.attr.byteLength));
    }
  }

  return {
    points,
    attr,
    type,
  };
}

export async function getVertexData(id: string, item: string = "p"): Promise<LSD.MapKeyframe> {
  const r = await axios.post(
    `v1/vertex-data`,
    { id: id, item: item },
    {
      responseType: "arraybuffer",
    }
  );
  if (!LidarPointcloudMap) throw new Error("LidarPointcloudMap has not been parsed!");
  let lidarPointcloudMap = LidarPointcloudMap.decode(new Uint8Array(r.data)) as any as ILidarPointcloudMap;

  let lpcs: Float32Array[] = [];
  for (let lp of lidarPointcloudMap.lp || []) {
    lpcs[Number(lp.lidarName)] = new Float32Array(
      lp.points.buffer.slice(lp.points.byteOffset, lp.points.byteOffset + lp.points.byteLength)
    );
  }

  let imageData: { [index: string]: Uint8Array } = {};
  for (let camera of lidarPointcloudMap.image || []) {
    imageData[camera.cameraName] = new Uint8Array(
      camera.image!.buffer.slice(camera.image.byteOffset, camera.image.byteOffset + camera.image.byteLength)
    );
  }
  return {
    points: lpcs[Number(id)],
    images: imageData,
  };
}

export async function addMapEdge(prev: string, next: string, relative: THREE.Matrix4) {
  const payload = {
    method: "map_add_edge",
    params: {
      prev: parseInt(prev),
      next: parseInt(next),
      relative: relative.transpose().toArray(),
    },
    jsonrpc: 2.0,
    id: 0,
  };
  return (await axios.post("api/", payload)).data.result;
}

export async function delteMapVertex(id: string): Promise<string> {
  return (await axios.post("v1/map-del-vertex", { id: parseInt(id) })).data;
}

export async function deleteMapPoints(index: any) {
  const payload = {
    method: "map-del-points",
    params: {
      index: index,
    },
    jsonrpc: 2.0,
    id: 0,
  };
  return (await axios.post("api/", payload)).data.result;
}

export async function delteMapEdge(id: string): Promise<string> {
  return (await axios.post("v1/map-del-edge", { id: parseInt(id) })).data;
}

export async function addMapArea(area: any): Promise<string> {
  return (await axios.post("v1/map-add-area", area)).data;
}

export async function delMapArea(id: string): Promise<string> {
  return (await axios.post("v1/map-del-area", { id: id })).data;
}

export async function setMapVertexFix(id: string, fix: boolean): Promise<string> {
  return (await axios.post("v1/map-set-vertex-fix", { id: parseInt(id), fix: fix })).data;
}

export async function setExportMapConfig(zMin: number, zMax: number, color: string) {
  return (await axios.post("v1/set-export-map-config", { z_min: zMin, z_max: zMax, color: color })).data;
}

export async function mapOptimization() {
  const response = await axios.get("v1/map-optimize");
  return response.data;
}

export async function KeyframeAlign(source: string, target: string, guess: THREE.Matrix4) {
  const payload = {
    method: "map_keyframe_align",
    params: {
      source: source,
      target: target,
      guess: guess.transpose().toArray(),
    },
    jsonrpc: 2.0,
    id: 0,
  };
  return arrayToTransform((await axios.post("api/", payload)).data.result);
}

export async function calibrateGround({
  points,
  contour,
  key,
}: {
  points: Float32Array;
  contour: [number, number][];
  key: number;
}) {
  const payload = {
    method: "calibrate_ground",
    params: {
      points: _arrayBufferToBase64(points.buffer),
      contour,
      key,
    },
    jsonrpc: 2.0,
    id: 0,
  };
  return arrayToTransform((await axios.post("api/", payload)).data.result);
}

export async function calibrateHeading({
  source,
  target,
  key,
}: {
  source: [number, number][];
  target: [number, number][];
  key: number;
}) {
  const payload = {
    method: "calibrate_heading",
    params: {
      source,
      target,
      key,
    },
    jsonrpc: 2.0,
    id: 0,
  };
  return arrayToTransform((await axios.post("api/", payload)).data.result);
}

export async function finetune({ transform, lidarIndex }: { transform: number[]; lidarIndex: number }) {
  const payload = {
    method: "finetune_lidar",
    params: {
      transform,
      lidarIndex,
    },
    jsonrpc: 2.0,
    id: 0,
  };
  return arrayToTransform((await axios.post("api/", payload)).data.result);
}

export async function calibrateLidarCamera({
  pointsLidar,
  pointsCamera,
  cameraName,
}: {
  pointsLidar: THREE.Vector3[];
  pointsCamera: THREE.Vector3[];
  cameraName: string;
}) {
  const payload = {
    method: "calibrate_lidar_camera",
    params: {
      pointsLidar: pointsLidar.map((pt) => [pt.x, pt.y, pt.z]),
      pointsCamera: pointsCamera.map((pt) => [pt.x, pt.y]),
      cameraName,
    },
    jsonrpc: 2.0,
    id: 0,
  };
  return arrayToTransform((await axios.post("api/", payload)).data.result);
}

export async function finetuneCamera({ transform, cameraName }: { transform: THREE.Matrix4; cameraName: string }) {
  const payload = {
    method: "finetune_camera",
    params: {
      transform: transform.transpose().toArray(),
      cameraName,
    },
    jsonrpc: 2.0,
    id: 0,
  };
  return arrayToTransform((await axios.post("api/", payload)).data.result);
}

export async function findCorners({
  imageData,
  cameraName,
  config,
}: {
  imageData: Uint8Array;
  cameraName: string;
  config: object;
}): Promise<LSD.ImageCorner> {
  const payload = {
    method: "find_corners",
    params: {
      imageData: _arrayBufferToBase64(imageData.buffer),
      cameraName,
      config,
    },
    jsonrpc: 2.0,
    id: 0,
  };
  let response = (await axios.post("api/", payload)).data.result;
  let images = _base64ToArrayBuffer(response.images);
  return {
    images: images,
    corners: response.corners,
    result: response.result,
  };
}

export async function getHomography(
  cameras: string[],
  name0: string,
  name1: string,
  image0: Uint8Array,
  image1: Uint8Array,
  pointsLeft: THREE.Vector3[],
  pointsRight: THREE.Vector3[],
  order: string
): Promise<LSD.ImageHomography> {
  const payload = {
    method: "get_homography",
    params: {
      cameras: cameras,
      name0: name0,
      name1: name1,
      image0: _arrayBufferToBase64(image0.buffer),
      image1: _arrayBufferToBase64(image1.buffer),
      kpoint0: pointsLeft.map((pt) => [pt.x, pt.y]),
      kpoint1: pointsRight.map((pt) => [pt.x, pt.y]),
      order: order,
    },
    jsonrpc: 2.0,
    id: 0,
  };
  let response = (await axios.post("api/", payload)).data.result;
  let images = _base64ToArrayBuffer(response.images);
  return {
    images: images,
    homography: response.homography,
    result: response.result,
  };
}

export async function getPanorama(): Promise<Uint8Array> {
  const r = await axios.get(`v1/get-panorama`, {
    responseType: "arraybuffer",
  });
  let det = Detection.decode(new Uint8Array(r.data)) as any as IDetection;
  let imageData: { [index: string]: Uint8Array } = {};
  for (let camera of det.image || []) {
    imageData[camera.cameraName] = new Uint8Array(
      camera.image!.buffer.slice(camera.image.byteOffset, camera.image.byteOffset + camera.image.byteLength)
    );
  }
  return imageData["panorama"];
}

export async function setPanoramaConfig(): Promise<void> {
  return await axios.get("v1/set-panorama-config");
}

export async function calibrateCamera({
  pointsCamera,
  cameraName,
  config,
}: {
  pointsCamera: [][][];
  cameraName: string;
  config: object;
}) {
  const payload = {
    method: "calibrate_camera",
    params: {
      pointsCamera: pointsCamera,
      cameraName,
      config,
    },
    jsonrpc: 2.0,
    id: 0,
  };
  return (await axios.post("api/", payload)).data.result;
}

export async function restartLidarInsCalibration() {
  const payload = {
    method: "restart_lidar_ins_calibration",
    jsonrpc: 2.0,
    id: 0,
  };
  return (await axios.post("api/", payload)).data;
}

export async function getPositionPoints(): Promise<LSD.PositionPoints> {
  const r = await axios.get(`v1/get-position-points`, {
    responseType: "arraybuffer",
  });
  let det = Detection.decode(new Uint8Array(r.data)) as any as IDetection;
  let points = new Float32Array(
    det.points && det.points.length > 0
      ? det.points.buffer.slice(det.points.byteOffset, det.points.byteOffset + det.points.byteLength)
      : []
  );
  let position = [0, 0, 0, 0, 0, 0];
  let result = false;
  let is_reset = false;
  if (det.pose) {
    position = [det.pose.x, det.pose.y, det.pose.z, det.pose.heading, det.pose.pitch, det.pose.roll];
    result = det.pose.status == 1;
  }

  return { points, position, result, is_reset };
}

export async function calibrateLidarIns() {
  const payload = {
    method: "calibrate_lidar_ins",
    jsonrpc: 2.0,
    id: 0,
  };
  return (await axios.post("api/", payload)).data.result;
}

export async function getLidarInsCalibration() {
  const payload = {
    method: "get_lidar_ins_calibration",
    jsonrpc: 2.0,
    id: 0,
  };
  return (await axios.post("api/", payload)).data.result;
}

export async function getLidarInsTransform() {
  const payload = {
    method: "get_lidar_ins_transform",
    jsonrpc: 2.0,
    id: 0,
  };
  return arrayToTransform((await axios.post("api/", payload)).data.result);
}

export async function setLidarInsTransform({ transform }: { transform: THREE.Matrix4 }) {
  const payload = {
    method: "set_lidar_ins_transform",
    params: {
      transform: transform.transpose().toArray(),
    },
    jsonrpc: 2.0,
    id: 0,
  };
  return (await axios.post("api/", payload)).data.result;
}

export async function restartLidarImuCalibration() {
  const payload = {
    method: "restart_lidar_imu_calibration",
    jsonrpc: 2.0,
    id: 0,
  };
  return (await axios.post("api/", payload)).data;
}

export async function getImuPositionPoints(): Promise<LSD.PositionPoints> {
  const r = await axios.get(`v1/get-imu-position-points`, {
    responseType: "arraybuffer",
  });
  let det = Detection.decode(new Uint8Array(r.data)) as any as IDetection;
  let points = new Float32Array(
    det.points && det.points.length > 0
      ? det.points.buffer.slice(det.points.byteOffset, det.points.byteOffset + det.points.byteLength)
      : []
  );
  let position = [0, 0, 0, 0, 0, 0];
  let result = false;
  let is_reset = false;
  if (det.pose) {
    position = [det.pose.x, det.pose.y, det.pose.z, det.pose.heading, det.pose.pitch, det.pose.roll];
    result = det.pose.status == 1;
    is_reset = det.pose.status == 2;
  }

  return { points, position, result, is_reset };
}

export async function calibrateLidarImu() {
  const payload = {
    method: "calibrate_lidar_imu",
    jsonrpc: 2.0,
    id: 0,
  };
  return (await axios.post("api/", payload)).data.result;
}

export async function getLidarImuLidarPoses(): Promise<LSD.MapVertex> {
  const payload = {
    method: "lidar_imu_get_lidar_poses",
    jsonrpc: 2.0,
    id: 0,
  };
  return (await axios.post("api/", payload)).data.result;
}

export async function getLidarImuImuPoses(): Promise<LSD.MapVertex> {
  const payload = {
    method: "lidar_imu_get_imu_poses",
    jsonrpc: 2.0,
    id: 0,
  };
  return (await axios.post("api/", payload)).data.result;
}

export async function setLidarImuExtrinsics() {
  const payload = {
    method: "set_lidar_imu_extrinsics",
    jsonrpc: 2.0,
    id: 0,
  };
  return (await axios.post("api/", payload)).data.result;
}

export async function startRecord() {
  const payload = {
    method: "start_record",
    jsonrpc: 2.0,
    id: 0,
  };
  return (await axios.post("api/", payload)).data;
}
export async function stopRecord() {
  const payload = {
    method: "stop_record",
    jsonrpc: 2.0,
    id: 0,
  };
  return (await axios.post("api/", payload)).data;
}
export async function reboot(flag: boolean) {
  const payload = {
    method: "reboot",
    jsonrpc: 2.0,
    id: 0,
    params: {
      is_confirmed: flag,
      hostname: HOSTNAME,
    },
  };
  return (await axios.post("api/", payload)).data.result;
}

export async function startMessageSubscribe() {
  return await axios.get("v1/start-message-subscribe");
}

export async function stopMessageSubscribe() {
  return await axios.get("v1/stop-message-subscribe");
}

export async function getMessageMeta(): Promise<LSD.MessageMeta> {
  const response = await axios.get("v1/get-message-meta");
  return response.data;
}

export async function getMessageData(name: string, type: string) {
  if (type == "PointCloud") {
    const r = await axios.post("v1/get-message-data", { name: name }, { responseType: "arraybuffer" });
    let lidarPointcloudMap = LidarPointcloudMap.decode(new Uint8Array(r.data)) as any as ILidarPointcloudMap;
    let points: Float32Array | undefined = undefined;
    let attr: Float32Array | Uint8Array | undefined = undefined;
    let type: string | undefined = undefined;
    for (let lp of lidarPointcloudMap.lp || []) {
      points = new Float32Array(
        lp.points.buffer.slice(lp.points.byteOffset, lp.points.byteOffset + lp.points.byteLength)
      );
      if (lp.type) {
        type = lp.type;
      }
      if (lp.attr) {
        if (type == "intensity") {
          attr = new Float32Array(lp.attr.buffer.slice(lp.attr.byteOffset, lp.attr.byteOffset + lp.attr.byteLength));
        } else if (type == "rgb") {
          attr = new Uint8Array(lp.attr.buffer.slice(lp.attr.byteOffset, lp.attr.byteOffset + lp.attr.byteLength));
        }
      }
    }

    return {
      points,
      attr,
      type,
    };
  } else if (type == "CompressedImage" || type == "Image") {
    const r = await axios.post("v1/get-message-data", { name: name }, { responseType: "arraybuffer" });
    let lidarPointcloudMap = LidarPointcloudMap.decode(new Uint8Array(r.data)) as any as ILidarPointcloudMap;
    let image: Uint8Array | undefined = undefined;
    for (let camera of lidarPointcloudMap.image || []) {
      image = new Uint8Array(
        camera.image!.buffer.slice(camera.image.byteOffset, camera.image.byteOffset + camera.image.byteLength)
      );
    }
    return image;
  } else {
    return (await axios.post("v1/get-message-data", { name: name })).data;
  }
}
