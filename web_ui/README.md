# LSD Web UI

## Introduction

- Display device status
- Configure device parameters and algorithm parameters
- Preview input(pointcloud, image) and output(object bounding box, trajectory, freespace) of algorithms (detection, prediction)
- Calibrate lidar, lidar-camera, etc
- Other advanced functions (json format config editor, log viewer and firmware upgrader)

## Used technology

- Basic language [nodejs](https://nodejs.org) and package manager [yarn](https://yarnpkg.com/)
- Web framework [react](https://reactjs.org/)
- Build system [vite](https://vitejs.dev/)
- Type system [typescript](https://www.typescriptlang.org/)
- 3D render [three.js](https://threejs.org/) with its react wrapper [react-three-fiber](https://docs.pmnd.rs/react-three-fiber/getting-started/introduction)
- UI framework [MaterialUI](https://material-ui.com/)
- Navigating [react-router](https://reactrouter.com/)
- Useful react helper [ahooks](https://ahooks.js.org/hooks/async)
- HTTP client [axios](https://github.com/axios/axios)
- Protobuf with javascript in browser [protobufjs](https://github.com/protobufjs/protobuf.js/)

## Basic Enviroment

[Node 18.16.0](https://nodejs.org/dist/v18.16.0/node-v18.16.0-linux-x64.tar.xz),  [Installing Guide](https://github.com/nodejs/help/wiki/Installation)

## Get started

- Install dependency

```bash
npm install
```

- Start dev server

```bash
npm run dev
```

## Deploy

- Build

```bash
npm run build
```

Rename the **dist** to **www** and replace the **lidar-slam-detection/www**, then refrese the browser


