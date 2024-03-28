import { OrbitControls, MapControls } from "three/examples/jsm/controls/OrbitControls.js";
import FirstPersonControl from "./first-person-control";
import { useThree } from "@react-three/fiber";
import { useState, useEffect } from "react";

export const CONTROL_TYPE = {
  NORMAL: 0,
  UP: 1,
  SELF: 2,
  IMAGE: 3,
  ORTHOGRAPHIC: 4,
  CAMERA: 5,
  FOLLOW: 6,
  DRIVER: 7,
};

export default function Controls({
  type = CONTROL_TYPE.NORMAL,
  height = 50,
  position = [0, 0, 0],
  target = [0, 0, 0],
  enable = true,
}) {
  const { camera, gl } = useThree();

  const [controls, setControls] = useState();
  let control;

  useEffect(() => {
    gl.localClippingEnabled = true;
    if (controls) controls.dispose();
    switch (type) {
      case CONTROL_TYPE.ORTHOGRAPHIC:
        camera.up.set(0, 0, 1);
        camera.position.set(-0.001, 0, 150);
        camera.lookAt(0, 0, 0);
        camera.zoom = 20;
        camera.updateProjectionMatrix();
        control = new OrbitControls(camera, gl.domElement);
        control.rotateSpeed = 0.2;
        setControls(control);
        break;
      case CONTROL_TYPE.FOLLOW:
      case CONTROL_TYPE.NORMAL:
        // camera.up.set(0, 0, 1);
        // camera.position.set(15, 15, 15);
        // camera.lookAt(0, 0, 0);
        // setControls(new OrbitControls(camera, gl.domElement));
        camera.up.set(0, 0, 1);
        camera.position.set(position[0] - 15, position[1], position[2] + 20);
        camera.lookAt(position[0], position[1], position[2]);
        control = new MapControls(camera, gl.domElement);
        control.target.set(position[0] + target[0], position[1] + target[1], position[2] + target[2]);
        setControls(control);
        control.update();
        break;
      case CONTROL_TYPE.CAMERA:
        camera.up.set(0, -1, 0);
        camera.position.set(0, 0, -10);
        camera.lookAt(0, 0, 0);
        camera.rotateX(Math.PI);
        setControls(new OrbitControls(camera, gl.domElement));
        break;
      case CONTROL_TYPE.UP:
        camera.position.set(-0.001, 0, height);
        camera.lookAt(0, 0, 0);
        camera.up.set(0, 0, 1);
        control = new MapControls(camera, gl.domElement);
        control.enableRotate = false;
        setControls(control);
        break;
      case CONTROL_TYPE.IMAGE:
        camera.position.set(0, 0, height);
        camera.lookAt(0, 0, 0);
        camera.up.set(0, 0, 1);
        control = new MapControls(camera, gl.domElement);
        control.maxPolarAngle = 0;
        control.maxAzimuthAngle = 0;
        control.minAzimuthAngle = 0;
        setControls(control);
        break;
      case CONTROL_TYPE.DRIVER:
        camera.up.set(0, 0, 1);
        camera.position.set(-1, 0, 1);
        camera.lookAt(20, 0, 0);
        setControls(new FirstPersonControl(camera, gl.domElement));
        break;
      case CONTROL_TYPE.SELF:
        camera.up.set(0, 0, 1);
        camera.position.set(-15, 0, 20);
        camera.lookAt(0, 0, 0);
        setControls(new MapControls(camera, gl.domElement));
      default:
        break;
    }
  }, [camera, gl, type]);

  useEffect(() => {
    if (type == CONTROL_TYPE.FOLLOW && controls) {
      // controls.object.position.set(position[0] - 15, position[1], position[2] + 20);
      controls.object.lookAt(position[0], position[1], position[2]);
      controls.target.set(position[0], position[1], position[2]);
    } else if (type == CONTROL_TYPE.NORMAL && controls) {
      controls.target.set(controls.target.x, controls.target.y, position[2] + target[2]);
      controls.update();
    }
  }, [position, target]);

  useEffect(() => {
    if (controls) {
      controls.enableZoom = enable;
      controls.enableRotate = enable;
      controls.enablePan = enable;
    }
  }, [enable]);

  // useFrame(() => {
  // })
  return null;
}
