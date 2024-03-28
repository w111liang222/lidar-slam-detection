import * as THREE from "three";

export function transformPoints(pointsInput: Float32Array, transform: THREE.Matrix4) {
  const points = pointsInput.slice();
  const vec3 = new THREE.Vector3();
  for (let i = 0; i < points.length / 4; i++) {
    vec3.set(points[4 * i + 0], points[4 * i + 1], points[4 * i + 2]);
    vec3.applyMatrix4(transform);
    // T = T.invert();
    points[4 * i + 0] = vec3.x;
    points[4 * i + 1] = vec3.y;
    points[4 * i + 2] = vec3.z;
  }
  return points;
}

export function getTransformFromVec(transform: number[]) {
  const _transform = new Array(...transform) as number[];
  const _transMatrix = new THREE.Matrix4();
  const _pos = new THREE.Vector3(_transform[0], _transform[1], _transform[2]);
  const _quat = new THREE.Quaternion();
  const _scale = new THREE.Vector3(1.0, 1.0, 1.0);
  const _euler = new THREE.Euler(
    (_transform[3] * Math.PI) / 180,
    (_transform[4] * Math.PI) / 180,
    (_transform[5] * Math.PI) / 180,
    "XYZ"
  );
  _quat.setFromEuler(_euler);
  // quat.set(quat.y, quat.x, quat.z, quat.w);
  _transMatrix.compose(_pos, _quat, _scale);
  // T.transpose();
  return _transMatrix;
}

export function getVecFromTransform(transMatrix: THREE.Matrix4) {
  const _transposedT = transMatrix.clone();
  const _pos = new THREE.Vector3();
  const _quat = new THREE.Quaternion();
  const _scale = new THREE.Vector3();
  const _euler = new THREE.Euler(0, 0, 0, "XYZ");
  _transposedT.decompose(_pos, _quat, _scale);

  _euler.setFromQuaternion(_quat);
  const _rotation = _euler.toArray().slice(0, 3) as [number, number, number];
  _rotation[0] *= 180 / Math.PI;
  _rotation[1] *= 180 / Math.PI;
  _rotation[2] *= 180 / Math.PI;
  let _vector;
  _vector = _pos.toArray().concat(_rotation) as [number, number, number, number, number, number];
  return _vector;
}

export function getPoseClient(point: THREE.Vector3, ev: any, camera: THREE.Camera) {
  const poseClient = point.clone().project(camera);
  const element = ev.srcElement ? ev.srcElement : ev.nativeEvent.srcElement;
  poseClient.x = ((poseClient.x + 1) / 2) * element.offsetWidth;
  poseClient.y = (-(poseClient.y - 1) / 2) * element.offsetHeight;
  poseClient.z = 0;
  return poseClient;
}

export function pointToLineDistance(x: number, y: number, x1: number, y1: number, x2: number, y2: number) {
  const A = x - x1;
  const B = y - y1;
  const C = x2 - x1;
  const D = y2 - y1;

  const dot = A * C + B * D;
  const len_sq = C * C + D * D;
  let param = -1;
  if (len_sq != 0)
    //in case of 0 length line
    param = dot / len_sq;

  let xx, yy;
  if (param < 0) {
    xx = x1;
    yy = y1;
  } else if (param > 1) {
    xx = x2;
    yy = y2;
  } else {
    xx = x1 + param * C;
    yy = y1 + param * D;
  }

  const dx = x - xx;
  const dy = y - yy;
  return Math.sqrt(dx * dx + dy * dy);
}

export function insidePolygon(point: THREE.Vector3, vs: THREE.Vector3[]) {
  const x = point.x,
    y = point.y;

  let inside = false;
  for (let i = 0, j = vs.length - 1; i < vs.length; j = i++) {
    const xi = vs[i].x,
      yi = vs[i].y;
    const xj = vs[j].x,
      yj = vs[j].y;

    const intersect = yi > y != yj > y && x < ((xj - xi) * (y - yi)) / (yj - yi) + xi;
    if (intersect) inside = !inside;
  }

  return inside;
}
