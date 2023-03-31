import numpy as np

def gen_3box_np(kps, dim, rot, calib, opinv, const):
    c=kps.shape[0]

    opinv = np.tile(opinv, (c, 1, 1))
    kps = np.transpose(kps.reshape(c, -1, 2), (0, 2, 1))
    hom = np.ones((c, 1, 9))
    kps = np.concatenate((kps, hom), axis=1)
    kps = np.matmul(opinv, kps).reshape(c, 2, 9)
    kps = np.transpose(kps, (0, 2, 1)).reshape(c, -1)
    si = np.zeros_like(kps[:, 0:1]) + calib[0:1, 0:1]
    rots = rot[..., 0:1]
    rotc = rot[..., 1:2]
    alpha = np.arctan2(rots, rotc)

    rot_y = alpha + np.arctan2(kps[:, 16:17] - calib[0:1, 2:3], si)
    rot_y[rot_y > np.pi] = rot_y[rot_y > np.pi] - 2 * np.pi
    rot_y[rot_y < - np.pi] = rot_y[rot_y < - np.pi] + 2 * np.pi

    calib = np.tile(calib, (c, 1, 1))
    kpoint = kps[:, :16]
    f = np.expand_dims(calib[:, 0, 0], axis=1)
    f = np.tile(f, (1, 16))
    cx, cy = np.expand_dims(calib[:, 0, 2], axis=1), np.expand_dims(calib[:, 1, 2], axis=1)
    cxy = np.concatenate((cx, cy), axis=1)
    cxy = np.tile(cxy, (1, 8))
    kp_norm = (kpoint - cxy) / f

    l = dim[:, 2:3]
    h = dim[:, 0:1]
    w = dim[:, 1:2]
    cosori = np.cos(rot_y)
    sinori = np.sin(rot_y)

    B = np.zeros_like(kpoint)
    C = np.zeros_like(kpoint)

    kp = np.expand_dims(kp_norm, axis=2)
    const = np.tile(const, (c, 1, 1))
    A = np.concatenate([const, kp], axis=2)

    B[:, 0:1] = l * 0.5 * cosori + w * 0.5 * sinori
    B[:, 1:2] = h * 0.5
    B[:, 2:3] = l * 0.5 * cosori - w * 0.5 * sinori
    B[:, 3:4] = h * 0.5
    B[:, 4:5] = -l * 0.5 * cosori - w * 0.5 * sinori
    B[:, 5:6] = h * 0.5
    B[:, 6:7] = -l * 0.5 * cosori + w * 0.5 * sinori
    B[:, 7:8] = h * 0.5
    B[:, 8:9] = l * 0.5 * cosori + w * 0.5 * sinori
    B[:, 9:10] = -h * 0.5
    B[:, 10:11] = l * 0.5 * cosori - w * 0.5 * sinori
    B[:, 11:12] = -h * 0.5
    B[:, 12:13] = -l * 0.5 * cosori - w * 0.5 * sinori
    B[:, 13:14] = -h * 0.5
    B[:, 14:15] = -l * 0.5 * cosori + w * 0.5 * sinori
    B[:, 15:16] = -h * 0.5

    C[:, 0:1] = -l * 0.5 * sinori + w * 0.5 * cosori
    C[:, 1:2] = -l * 0.5 * sinori + w * 0.5 * cosori
    C[:, 2:3] = -l * 0.5 * sinori - w * 0.5 * cosori
    C[:, 3:4] = -l * 0.5 * sinori - w * 0.5 * cosori
    C[:, 4:5] = l * 0.5 * sinori - w * 0.5 * cosori
    C[:, 5:6] = l * 0.5 * sinori - w * 0.5 * cosori
    C[:, 6:7] = l * 0.5 * sinori + w * 0.5 * cosori
    C[:, 7:8] = l * 0.5 * sinori + w * 0.5 * cosori
    C[:, 8:9] = -l * 0.5 * sinori + w * 0.5 * cosori
    C[:, 9:10] = -l * 0.5 * sinori + w * 0.5 * cosori
    C[:, 10:11] = -l * 0.5 * sinori - w * 0.5 * cosori
    C[:, 11:12] = -l * 0.5 * sinori - w * 0.5 * cosori
    C[:, 12:13] = l * 0.5 * sinori - w * 0.5 * cosori
    C[:, 13:14] = l * 0.5 * sinori - w * 0.5 * cosori
    C[:, 14:15] = l * 0.5 * sinori + w * 0.5 * cosori
    C[:, 15:16] = l * 0.5 * sinori + w * 0.5 * cosori

    B = B - kp_norm * C

    AT = np.transpose(A, (0, 2, 1))
    B = np.expand_dims(B, axis=2)

    pinv = np.matmul(AT, A)
    pinv = np.linalg.inv(pinv)

    pinv = np.matmul(pinv, AT)
    pinv = np.matmul(pinv, B)
    pinv = np.squeeze(pinv, axis=2)
    pred_boxes = np.concatenate([pinv, dim, rot_y], axis=1)

    return pred_boxes