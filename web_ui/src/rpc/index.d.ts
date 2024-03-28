import * as proto from "@proto/detection";
import config from "./sample/config.json";
import status from "./sample/status.json";
import webStore from "./sample/webstore.json";

type Config = typeof config;
type webStore = typeof webStore;
type LidarConfig = typeof config["lidar"];
type Status =
  | {
      active_clients: string[];
      lidar: {
        [name: string]: {
          timestamp: number;
          num: [number, number];
          valid: boolean;
        };
      };
      camera: {
        [name: string]: {
          h: number;
          in_h: number;
          in_w: number;
          valid: boolean;
          w: number;
        };
      };
      radar: {
        [name: string]: {
          num: number;
          valid: boolean;
        };
      };
      disk: {
        disk_name: string;
        has_disk: boolean;
        status: boolean;
        total: number;
        used_percent: number;
        frame_success?: number;
        frame_lost?: number;
        frame_error?: number;
      };
      ins: {
        received_count: number;
        valid_count: number;
        latitude: number;
        longitude: number;
        valid: boolean;
      };
      status: string;
      time: string;
    }
  | undefined;
type PlayerStatus = {
  now_time: string;
  left_time: string;
  percent: number;
};
type RecordFiles = {
  [id: string]: [string, number];
};
type Detection = {
  header?: proto.IHeader;
  points: Float32Array;
  objects: proto.IObject[];
  freespace?: proto.IFreespace;
  images: { [key: string]: Uint8Array };
  pose?: proto.IPose;
  radar?: { [id: string]: proto.IObject[] };
};
type MapVertex = {
  [id: string]: number[];
};
type MapStatus = {
  loop_detected: boolean;
};
type MapEdge = {
  [id: string]: number[];
};
type MapMeta = {
  vertex: {
    [id: string]: {
      stamps: number;
      fix: boolean;
      edge_num: number;
    };
  };
  edge: {
    [id: string]: {
      vertex_num: number;
    };
  };
  area: {
    [id: string]: {
      type: string;
      name: string;
      polygon: number[][];
    };
  };
};
type MapKeyframe = {
  points: Float32Array;
  images?: { [key: string]: Uint8Array };
};
type IObject = proto.IObject;
type IFreespace = proto.IFreespace;
type ITrajectory = proto.ITrajectory;
type ImageCorner = {
  images: Uint8Array;
  corners: [][];
  result: boolean;
};
type ImageHomography = {
  images: Uint8Array;
  homography: [];
  result: boolean;
};
type PositionPoints = {
  points: Float32Array;
  position: number[];
  result: boolean;
  is_reset: boolean;
};

type ClientUsers = {
  client_ip: string;
  users: {
    [ip: string]: {
      time: number;
      requests: number;
      disable: boolean;
    };
  };
};

type MessageMeta = {
  channels: string[];
  types: string[];
};

type PointCloud2 = {
  points: Float32Array;
  attr: Float32Array | Uint8Array;
  type: string;
};

export as namespace LSD;
