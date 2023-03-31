import numpy as np
from scipy.spatial.transform import Rotation as R
from .filter import crop_points
from .fit import fit_plane


def calc_rot_from(vec1, vec2):
    a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 /
                                                      np.linalg.norm(vec2)).reshape(3)
    c = np.cross(a, b)
    cos = np.dot(a, b)
    sin = np.sqrt(np.dot(c, c))
    n = (c / np.linalg.norm(c))
    A = np.array([
        [0, -n[2], n[1]],
        [n[2], 0, -n[0]],
        [-n[1], n[0], 0],
    ])  # skew-sym matrix
    # rodriges
    return cos * np.eye(3) + (1 - cos) * np.dot(n.reshape(3, 1), n.reshape(1, 3)) + sin * A
    # return R.from_rotvec(c).as_matrix()


def align_points_to_xyplane(points, roi_xy):
    sigma = 0.05
    num_iteration = 100
    thresh = 0.9

    points = points.reshape(-1, 4)[:, :3]
    points_crop = crop_points(points, np.array(roi_xy))
    a, b, c, d = fit_plane(np.array(points_crop), sigma=sigma,
                           num_iteration=num_iteration, thresh=thresh)

    rot = calc_rot_from(np.array([a, b, c]),
                        np.array([0, 0, 1]))  # pc to 0 plane

    # https://eigen.tuxfamily.org/dox/group__Geometry__Module.html#ga17994d2e81b723295f5bc3b1f862ed3b
    #   Eigen::Vector3d eulerAngle =
    #   tran.rotation().toRotationMatrix().eulerAngles(2, 0, 1);
    #   yaw = eulerAngle(0) / Ang2Rad;
    #   pitch = eulerAngle(1) / Ang2Rad;
    #   roll = eulerAngle(2) / Ang2Rad;
    # roll_pitch_yaw = R.from_matrix(rot).as_euler('YXZ', degrees=True)
    T = np.eye(4, 4)
    T[:3, :3] = rot
    T[:3, 3] = np.array([0, 0, d / c])

    return T


def align_pq_to_known(p, q, pp, qq):
    x1, y1 = p[:2]
    x2, y2 = q[:2]
    x1t, y1t = pp[:2]
    x2t, y2t = qq[:2]
    A = np.array([
        [x1, -y1, 1, 0],  # c -s clockwise as minus
        [y1, x1, 0, 1],  # s c
        [x2, -y2, 1, 0],
        [y2, x2, 0, 1],
    ])
    b = np.array([x1t, y1t, x2t, y2t]).reshape(-1, 1)
    x = np.dot(np.linalg.inv(A), b)

    cos = x[0].item()
    sin = x[1].item()
    dx = x[2].item()
    dy = x[3].item()
    # return [dx, dy, 0, 0, 0, math.atan2(sin, cos) / math.pi * 180]
    return np.array([
        [cos, -sin, 0],
        [sin, cos, 0],
        [0, 0, 1],
    ]), np.array([dx, dy, 0])


def align_points(p0s, p1s):
    # Ax=b, x=[c, s, dx, dy]
    A = []
    b = []
    for p0, p1 in zip(p0s, p1s):
        x0, y0 = p0[:2]
        x1, y1 = p1[:2]
        A.append([x0, -y0, 1, 0])
        b.append(x1)
        A.append([y0, x0, 0, 1])
        b.append(y1)
    A = np.array(A)
    b = np.array(b).reshape(-1, 1)
    x = np.dot(np.linalg.pinv(A), b)

    cos = x[0].item()
    sin = x[1].item()
    dx = x[2].item()
    dy = x[3].item()

    T = np.eye(4, 4)
    T[:3, :3] = np.array([[cos, -sin, 0], [sin, cos, 0], [0, 0, 1]])
    T[:3, 3] = np.array([dx, dy, 0])
    return T
