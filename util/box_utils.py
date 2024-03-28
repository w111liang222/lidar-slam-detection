import numpy as np
import copy

def cart_to_hom(pts):
    """
    :param pts: (N, 3 or 2)
    :return pts_hom: (N, 4 or 3)
    """
    pts_hom = np.hstack((pts, np.ones((pts.shape[0], 1), dtype=np.float32)))
    return pts_hom

def lidar_to_rect(pts_lidar, V2C):
    """
    :param pts_lidar: (N, 3)
    :return pts_rect: (N, 3)
    """
    pts_lidar_hom = cart_to_hom(pts_lidar)
    pts_rect = np.dot(pts_lidar_hom, V2C.T)
    return pts_rect

def rect_to_lidar(pts_rect, V2C):
    """
    :param pts_lidar: (N, 3)
    :return pts_rect: (N, 3)
    """
    pts_rect_hom = cart_to_hom(pts_rect)  # (N, 4)
    pts_lidar = np.dot(pts_rect_hom, np.linalg.inv(V2C.T))
    return pts_lidar[:, 0:3]

def rect_to_img(pts_rect, intrinsic):
    """
    :param pts_rect: (N, 3)
    :return pts_img: (N, 2)
    """
    pts_rect_hom = cart_to_hom(pts_rect)
    pts_2d_hom = np.dot(pts_rect_hom, intrinsic.T)
    pts_img = (pts_2d_hom[:, 0:2].T / pts_rect_hom[:, 2]).T  # (N, 2)
    pts_rect_depth = pts_2d_hom[:, 2] - intrinsic.T[3, 2]  # depth in rect camera coord
    return pts_img, pts_rect_depth

def lidar_to_img(pts_lidar, V2C, intrinsic):
    """
    :param pts_lidar: (N, 3)
    :return pts_img: (N, 2)
    """
    pts_rect = lidar_to_rect(pts_lidar, V2C)
    pts_img, pts_depth = rect_to_img(pts_rect, intrinsic)
    return pts_img, pts_depth

def boxes3d_camera_to_lidar(boxes3d_camera, V2C):
    """
    Args:
        boxes3d_camera: (N, 7) [x, y, z, l, h, w, r] in rect camera coords
        V2C: vehicle to camera transformation

    Returns:
        boxes3d_lidar: [x, y, z, dx, dy, dz, heading], (x, y, z) is the box center

    """
    xyz_camera, r = boxes3d_camera[:, 0:3], boxes3d_camera[:, 6:7]
    l, h, w = boxes3d_camera[:, 3:4], boxes3d_camera[:, 4:5], boxes3d_camera[:, 5:6]

    xyz_lidar = rect_to_lidar(xyz_camera, V2C)
    xyz_lidar[:, 2] += h[:, 0] / 2
    return np.concatenate([xyz_lidar, l, w, h, -(r + np.pi / 2)], axis=-1).astype(np.float32)

def boxes3d_lidar_to_camera(boxes3d_lidar, V2C):
    """
    :param boxes3d_lidar: (N, 7) [x, y, z, dx, dy, dz, heading], (x, y, z) is the box center
    :param V2C: vehicle to camera transformation
    :return:
        boxes3d_camera: (N, 7) [x, y, z, l, h, w, r] in rect camera coords
    """
    boxes3d_lidar_copy = copy.deepcopy(boxes3d_lidar)
    xyz_lidar = boxes3d_lidar_copy[:, 0:3]
    l, w, h = boxes3d_lidar_copy[:, 3:4], boxes3d_lidar_copy[:, 4:5], boxes3d_lidar_copy[:, 5:6]
    r = boxes3d_lidar_copy[:, 6:7]

    xyz_lidar[:, 2] -= h.reshape(-1) / 2
    xyz_cam = lidar_to_rect(xyz_lidar, V2C)
    r = -r - np.pi / 2
    return np.concatenate([xyz_cam, l, h, w, r], axis=-1)

def boxes3d_to_corners3d_camera(boxes3d, bottom_center=True):
    """
    :param boxes3d: (N, 7) [x, y, z, l, h, w, ry] in camera coords, see the definition of ry in KITTI dataset
    :param bottom_center: whether y is on the bottom center of object
    :return: corners3d: (N, 8, 3)
        7 -------- 4
       /|         /|
      6 -------- 5 .
      | |        | |
      . 3 -------- 0
      |/         |/
      2 -------- 1
    """
    boxes_num = boxes3d.shape[0]
    l, h, w = boxes3d[:, 3], boxes3d[:, 4], boxes3d[:, 5]
    x_corners = np.array([l / 2., l / 2., -l / 2., -l / 2., l / 2., l / 2., -l / 2., -l / 2], dtype=np.float32).T
    z_corners = np.array([w / 2., -w / 2., -w / 2., w / 2., w / 2., -w / 2., -w / 2., w / 2.], dtype=np.float32).T
    if bottom_center:
        y_corners = np.zeros((boxes_num, 8), dtype=np.float32)
        y_corners[:, 4:8] = -h.reshape(boxes_num, 1).repeat(4, axis=1)  # (N, 8)
    else:
        y_corners = np.array([h / 2., h / 2., h / 2., h / 2., -h / 2., -h / 2., -h / 2., -h / 2.], dtype=np.float32).T

    ry = boxes3d[:, 6]
    zeros, ones = np.zeros(ry.size, dtype=np.float32), np.ones(ry.size, dtype=np.float32)
    rot_list = np.array([[np.cos(ry), zeros, -np.sin(ry)],
                         [zeros, ones, zeros],
                         [np.sin(ry), zeros, np.cos(ry)]])  # (3, 3, N)
    R_list = np.transpose(rot_list, (2, 0, 1))  # (N, 3, 3)

    temp_corners = np.concatenate((x_corners.reshape(-1, 8, 1), y_corners.reshape(-1, 8, 1),
                                   z_corners.reshape(-1, 8, 1)), axis=2)  # (N, 8, 3)
    rotated_corners = np.matmul(temp_corners, R_list)  # (N, 8, 3)
    x_corners, y_corners, z_corners = rotated_corners[:, :, 0], rotated_corners[:, :, 1], rotated_corners[:, :, 2]

    x_loc, y_loc, z_loc = boxes3d[:, 0], boxes3d[:, 1], boxes3d[:, 2]

    x = x_loc.reshape(-1, 1) + x_corners.reshape(-1, 8)
    y = y_loc.reshape(-1, 1) + y_corners.reshape(-1, 8)
    z = z_loc.reshape(-1, 1) + z_corners.reshape(-1, 8)

    corners = np.concatenate((x.reshape(-1, 8, 1), y.reshape(-1, 8, 1), z.reshape(-1, 8, 1)), axis=2)

    return corners.astype(np.float32)

def corners3d_to_img_boxes(corners3d, intrinsic):
    """
    :param corners3d: (N, 8, 3) corners in rect coordinate
    :return: boxes_corner: (None, 8) [xi, yi] in rgb coordinate
    """
    sample_num = corners3d.shape[0]
    corners3d_hom = np.concatenate((corners3d, np.ones((sample_num, 8, 1))), axis=2)  # (N, 8, 4)

    img_pts = np.matmul(corners3d_hom, intrinsic.T)  # (N, 8, 3)

    x, y = img_pts[:, :, 0] / img_pts[:, :, 2], img_pts[:, :, 1] / img_pts[:, :, 2]
    x1, y1 = np.min(x, axis=1), np.min(y, axis=1)
    x2, y2 = np.max(x, axis=1), np.max(y, axis=1)

    boxes_corner = np.concatenate((x.reshape(-1, 8, 1), y.reshape(-1, 8, 1)), axis=2)
    boxes_corner_depth = img_pts[:, :, 2] - intrinsic.T[3, 2]

    return boxes_corner, boxes_corner_depth

def boxes3d_to_corners3d_kitti_camera(boxes3d, bottom_center=True):
    """
    :param boxes3d: (N, 7) [x, y, z, l, h, w, ry] in camera coords, see the definition of ry in KITTI dataset
    :param bottom_center: whether y is on the bottom center of object
    :return: corners3d: (N, 8, 3)
        7 -------- 4
       /|         /|
      6 -------- 5 .
      | |        | |
      . 3 -------- 0
      |/         |/
      2 -------- 1
    """
    boxes_num = boxes3d.shape[0]
    l, h, w = boxes3d[:, 3], boxes3d[:, 4], boxes3d[:, 5]
    x_corners = np.array([l / 2., l / 2., -l / 2., -l / 2., l / 2., l / 2., -l / 2., -l / 2], dtype=np.float32).T
    z_corners = np.array([w / 2., -w / 2., -w / 2., w / 2., w / 2., -w / 2., -w / 2., w / 2.], dtype=np.float32).T
    if bottom_center:
        y_corners = np.zeros((boxes_num, 8), dtype=np.float32)
        y_corners[:, 4:8] = -h.reshape(boxes_num, 1).repeat(4, axis=1)  # (N, 8)
    else:
        y_corners = np.array([h / 2., h / 2., h / 2., h / 2., -h / 2., -h / 2., -h / 2., -h / 2.], dtype=np.float32).T

    ry = boxes3d[:, 6]
    zeros, ones = np.zeros(ry.size, dtype=np.float32), np.ones(ry.size, dtype=np.float32)
    rot_list = np.array([[np.cos(ry), zeros, -np.sin(ry)],
                         [zeros, ones, zeros],
                         [np.sin(ry), zeros, np.cos(ry)]])  # (3, 3, N)
    R_list = np.transpose(rot_list, (2, 0, 1))  # (N, 3, 3)

    temp_corners = np.concatenate((x_corners.reshape(-1, 8, 1), y_corners.reshape(-1, 8, 1),
                                   z_corners.reshape(-1, 8, 1)), axis=2)  # (N, 8, 3)
    rotated_corners = np.matmul(temp_corners, R_list)  # (N, 8, 3)
    x_corners, y_corners, z_corners = rotated_corners[:, :, 0], rotated_corners[:, :, 1], rotated_corners[:, :, 2]

    x_loc, y_loc, z_loc = boxes3d[:, 0], boxes3d[:, 1], boxes3d[:, 2]

    x = x_loc.reshape(-1, 1) + x_corners.reshape(-1, 8)
    y = y_loc.reshape(-1, 1) + y_corners.reshape(-1, 8)
    z = z_loc.reshape(-1, 1) + z_corners.reshape(-1, 8)

    corners = np.concatenate((x.reshape(-1, 8, 1), y.reshape(-1, 8, 1), z.reshape(-1, 8, 1)), axis=2)

    return corners.astype(np.float32)

class DetectionDrawer():
    class_color = dict()
    class_color[1] = (240, 145,  78)  # Vehicle
    class_color[2] = (177, 198, 255)  # Pedestrian
    class_color[3] = (203, 127, 150)  # Cyclist
    class_color[4] = (255, 255, 255)  # Traffic_Cone
    class_color[5] = (150, 127, 203)
    class_color[6] = (255, 198, 177)

    traffic_color = dict()
    traffic_color[0] = (0,   0,   255)  # Traffic light, red
    traffic_color[1] = (0,   255,   0)  # Traffic light, green
    traffic_color[2] = (0,   255, 255)  # Traffic light, yellow
    traffic_color[3] = (255, 255, 255)  # Traffic light, off

    @staticmethod
    def draw_boxes(image, image_param, lidar_boxes, label):
        import cv2
        from util.image_util import cvt_image
        w = image_param['w']
        h = image_param['h']
        intrinsic = image_param['intrinsic']
        V2C = image_param['V2C'][:3]

        image = cvt_image(image, w, h)
        camera_boxes = boxes3d_lidar_to_camera(lidar_boxes, V2C)
        box2d_corner, box2d_corner_depth = corners3d_to_img_boxes(boxes3d_to_corners3d_camera(camera_boxes), intrinsic)
        mask = np.all(box2d_corner_depth > 0, axis=1)
        box2d_corner = box2d_corner[mask]
        label = label[mask]
        box2d_corner = box2d_corner.astype(np.int32)

        edges = [[0, 1], [0, 3], [0, 4], [1, 2], [1, 5], [3, 2], [3, 7], [4, 5], [4, 7], [5, 6], [6 ,7], [6, 2]]
        for i in range(0, box2d_corner.shape[0]):
            color = DetectionDrawer.class_color[label[i]]
            for j, e in enumerate(edges):
                cv2.line(image, (box2d_corner[i, e[0], 0], box2d_corner[i, e[0], 1]),
                        (box2d_corner[i, e[1], 0], box2d_corner[i, e[1], 1]), color, 1,
                         lineType=cv2.LINE_AA)
        return image

    @staticmethod
    def draw_boxes_2D(images, image_param, trafficlight):
        import cv2
        from util.image_util import cvt_image

        image_name     = trafficlight['image_name']
        padding_width  = image_param[image_name]['w'] // 32 * 32
        padding_height = image_param[image_name]['h'] // 32 * 32
        images[image_name] = cvt_image(images[image_name], padding_width, padding_height)

        for i in range(trafficlight['pred_boxes'].shape[0]):
            color = DetectionDrawer.traffic_color[trafficlight['pred_colors'][i]]
            box = trafficlight['pred_boxes'][i, :]
            cv2.rectangle(images[image_name], (round(box[0]), round(box[1])), (round(box[2]), round(box[3])), color, 2)
            cv2.putText(images[image_name], trafficlight['pred_names'][i], (round(box[0]), round(box[1])-10), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

        return images