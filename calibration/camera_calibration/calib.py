import cv2
import numpy as np
import base64

class CameraCalib():
    def __init__(self):
        pass

    @staticmethod
    def detect_checkborad(imageData, cameraName, config):
        checkboard = (int(config['chessboardRow']), int(config['chessboardCol']))

        img = cv2.imdecode(np.frombuffer(base64.b64decode(imageData), dtype=np.uint8), cv2.IMREAD_COLOR)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        try:
            ret, corners = cv2.findChessboardCorners(gray, checkboard, None)
        except Exception as e:
            print(e)
            ret = False

        cornerPoints = np.array([])
        if ret == True:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            if [corners2]:
                cornerPoints = corners2
            else:
                cornerPoints = corners
            cv2.drawChessboardCorners(img, checkboard, cornerPoints, ret)

        imageData = base64.b64encode(cv2.imencode('.jpg', img)[1]).decode("utf-8")

        return {"images": imageData, "corners": cornerPoints.tolist(), "result": ret}

    @staticmethod
    def calibration(pointsCamera, cameraCfg, config):
        w = cameraCfg.get("output_width", 640)
        h = cameraCfg.get("output_height", 480)

        row = int(config['chessboardRow'])
        col = int(config['chessboardCol'])
        size = int(config['chessboardSize'])

        fisheye = cameraCfg.get("fisheye", False)

        objp = np.zeros((row * col, 3), np.float32)
        objp[:, :2] = np.mgrid[0:row, 0:col].T.reshape(-1, 2) * size

        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.

        for i in range(len(pointsCamera)):
            objpoints.append(objp)
            corners = np.array(pointsCamera[i], dtype=np.float32)
            imgpoints.append(corners)

        if fisheye:
            objpoints = np.array(objpoints).reshape(len(pointsCamera), row*col, 1, 3)
            flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC |cv2.fisheye.CALIB_CHECK_COND | cv2.fisheye.CALIB_FIX_SKEW
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
            ret, mtx, dist, rvecs, tvecs = cv2.fisheye.calibrate(objpoints, imgpoints, (w, h), K=None, D=None,rvecs=None, tvecs=None,flags= flags, criteria=criteria)
            dist = dist.T
        else:
            flags = cv2.CALIB_FIX_K3 | cv2.CALIB_FIX_K4 | cv2.CALIB_FIX_K5
            dist = np.array([[0, 0, 0, 0]], dtype=np.float32)
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, (w, h), None, dist, flags=flags)
        fx = mtx[0, 0]
        fy = mtx[1, 1]
        cx = mtx[0, 2]
        cy = mtx[1, 2]
        intrinsic = [fx, fy, cx, cy, *dist[0, :4]]

        mean_error = 0
        project_func = cv2.fisheye.projectPoints if fisheye else cv2.projectPoints
        for i in range(len(objpoints)):
            imgpoints2, _ = project_func(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
            error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
            mean_error += error
        print("reprojection error: {}".format(mean_error/len(objpoints)))
        return intrinsic