import cv2
import numpy as np

def align_lidar_camera(cameraCfg, pointsLidar, pointsCamera):
    if len(pointsLidar) < 4:
        return np.eye(4, 4)

    fx, fy, cx, cy, k1, k2, p1, p2 = cameraCfg['intrinsic_parameters']
    w = cameraCfg.get("output_width", 640)
    h = cameraCfg.get("output_height", 480)
    # todo: undistort image
    intrinsicsMatrix = np.array([fx, 0, cx, 0, fy, cy, 0, 0, 1], dtype=np.float32).reshape(3, 3)

    pointsLidar = np.array(pointsLidar)
    pointsCamera = np.array([[x * w, y * h] for x, y in pointsCamera])

    success, r, t = cv2.solvePnP(
        pointsLidar[:4],
        pointsCamera[:4],
        intrinsicsMatrix,
        None,
        flags=cv2.SOLVEPNP_P3P,
    )
    if success and len(pointsLidar) > 4:
        success, r, t = cv2.solvePnP(
            pointsLidar,
            pointsCamera,
            intrinsicsMatrix,
            None,
            rvec=r,
            tvec=t,
            useExtrinsicGuess=True,
        )
    R, _ = cv2.Rodrigues(r)

    T = np.eye(4, 4)
    T[:3, :3] = R
    T[:3, 3] = t.flatten()
    return T