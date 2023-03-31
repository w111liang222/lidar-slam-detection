import cv2
import numpy as np
import base64
import copy
import math

from module.export_interface import call
from proto import detection_pb2
from util.image_util import cvt_image, get_image_size
from third_party.turbojpeg import TurboJPEG

def drawMatches(imageA, imageB, kpsA, kpsB):
    # initialize the output visualization image
    (hA, wA) = imageA.shape[:2]
    (hB, wB) = imageB.shape[:2]
    vis = np.zeros((max(hA, hB), wA + wB, 3), dtype="uint8")
    vis[0:hA, 0:wA] = imageA
    vis[0:hB, wA:] = imageB

    # loop over the matches
    for (psA, psB) in zip(kpsA, kpsB):
        # draw the match
        ptA = (int(psA[0]), int(psA[1]))
        ptB = (int(psB[0]) + wA, int(psB[1]))
        cv2.line(vis, ptA, ptB, (0, 255, 0), 1)
    # return the visualization
    cv2.imwrite('match.jpg', vis)
    return vis

class Matcher:
    def __init__(self):
        self.extractor = cv2.SIFT_create()
        self.matcher = cv2.BFMatcher()

    def match(self, i1, i2, points_left, points_right):
        if points_left.shape[0] != points_right.shape[0] or points_left.shape[0] <= 4:
            points_current, points_prev = self.get_match_keypoint(i1, i2)
            method = cv2.RANSAC
        else:
            points_current, points_prev = points_right, points_left
            method = 0

        if points_current.shape[0] > 4:
            H, _ = cv2.findHomography(points_current, points_prev, method, 4)
            # drawMatches(i1, i2, points_prev, points_current)
            return H

        return None

    def get_match_keypoint(self, i1, i2):
        image_set_1 = self.get_features(i1)
        image_set_2 = self.get_features(i2)

        matches = self.matcher.knnMatch(image_set_2["des"], image_set_1["des"], k=2)
        good = []
        for i, (m, n) in enumerate(matches):
            if m.distance < 0.7 * n.distance:
                good.append((m.trainIdx, m.queryIdx))

        points_current = image_set_2["kp"]
        points_previous = image_set_1["kp"]

        matched_points_current = np.float32(
            [points_current[i].pt for (_, i) in good]
        )
        matched_points_prev = np.float32(
            [points_previous[i].pt for (i, _) in good]
        )

        return matched_points_current, matched_points_prev

    def get_features(self, im):
        gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        kp, des = self.extractor.detectAndCompute(gray, None)
        return {'kp': kp, 'des': des}

def get_corner(image, homography):
    top_right = np.dot(homography, np.array([image.shape[1], 0, 1]))
    top_right = top_right / top_right[-1]

    bottom_left = np.dot(homography, np.array([0, image.shape[0], 1]))
    bottom_left = bottom_left / bottom_left[-1]

    bottom_right = np.dot(homography, np.array([image.shape[1], image.shape[0], 1]))
    bottom_right = bottom_right / bottom_right[-1]
    return int(max(top_right[0], bottom_right[0])), int(max(bottom_left[1], bottom_right[1]))

class PanoramaCameraCalib():
    database = dict(
        cameras=[],
    )
    jpeg = None
    def __init__(self):
        pass

    @staticmethod
    def set_homography(cameras, name, perspective_size, homography, affine_size, affine):
        if PanoramaCameraCalib.database['cameras'] != cameras:
            PanoramaCameraCalib.database = dict()
            PanoramaCameraCalib.database['cameras'] = []
            PanoramaCameraCalib.database['parameters'] = dict()

        PanoramaCameraCalib.database['cameras'] = cameras
        PanoramaCameraCalib.database['parameters'][name] = dict(
            do_wrap=(homography is not None),
            perspective_size=perspective_size,
            homography=homography,
            affine_size=affine_size,
            affine=affine,
        )

    @staticmethod
    def get_panorama_config():
        return copy.deepcopy(PanoramaCameraCalib.database)

    @staticmethod
    def get_panorama():
        if PanoramaCameraCalib.jpeg is None:
            PanoramaCameraCalib.jpeg = TurboJPEG()

        det = detection_pb2.Detection()
        result_dict = call('sink.get_proto_http', raw_data = True)

        database = copy.deepcopy(PanoramaCameraCalib.database)
        images = dict()
        if 'image' in result_dict:
            images = result_dict['image']

        is_camera_ready = True
        for name in database['cameras']:
            if name not in images:
                is_camera_ready = False
                print('camera: {} has no data for panorama'.format(name))

        cameras_num = len(database['cameras'])
        if not is_camera_ready or cameras_num < 2 or cameras_num > 3:
            return det.SerializeToString()

        pano_width, pano_height = 0, 0
        camera_cuda, stream_cuda = [], []
        for name in database['cameras']:
            image = images[name]
            w, h = get_image_size(image)
            image = cvt_image(image, w, h)

            stream = cv2.cuda_Stream()
            if not database['parameters'][name]['do_wrap']:
                image_cuda = image
            else:
                image_cuda = cv2.cuda_GpuMat()
                image_cuda.upload(image, stream=stream)

            camera_cuda.append(image_cuda)
            stream_cuda.append(stream)

        wrap_camera_cuda = []
        for idx, image_cuda in enumerate(camera_cuda):
            name = database['cameras'][idx]
            stream = stream_cuda[idx]
            if not database['parameters'][name]['do_wrap']:
                wrap_camera_cuda.append(image_cuda)
                pano_width = pano_width + image_cuda.shape[1]
                pano_height = image_cuda.shape[0]
            else:
                perspective_size = database['parameters'][name]['perspective_size']
                homography = np.float32(database['parameters'][name]['homography'])
                affine_size = database['parameters'][name]['affine_size']
                affine = np.float32(database['parameters'][name]['affine'])
                pano_width = pano_width + affine_size[0]

                wrap_gpu = cv2.cuda_GpuMat()
                wrap_gpu = cv2.cuda.warpPerspective(image_cuda, homography, perspective_size, stream=stream)
                affine_gpu = cv2.cuda_GpuMat()
                affine_gpu = cv2.cuda.warpAffine(wrap_gpu, affine, affine_size, stream=stream)
                wrap_camera_cuda.append(affine_gpu)

        wrap_camera = []
        for idx, image_cuda in enumerate(wrap_camera_cuda):
            name = database['cameras'][idx]
            stream = stream_cuda[idx]
            if not database['parameters'][name]['do_wrap']:
                wrap_camera.append(image_cuda)
            else:
                affine_size = database['parameters'][name]['affine_size']
                wrap_image = np.empty((affine_size[1], affine_size[0], 3), np.uint8)
                wrap_camera.append(wrap_image)
                image_cuda.download(stream, wrap_image)

        for stream in stream_cuda:
            stream.waitForCompletion()

        images = np.zeros((pano_height, pano_width, 3), np.uint8)
        offsetx = 0
        for image in wrap_camera:
            images[:, offsetx: offsetx + image.shape[1]] = image
            offsetx = offsetx + image.shape[1]

        img = PanoramaCameraCalib.jpeg.encode(images)
        camera_image = det.image.add()
        camera_image.camera_name = "panorama"
        camera_image.image = img
        return det.SerializeToString()

    @staticmethod
    def get_homography(cameras, name0, name1, image0, image1, kpoint0, kpoint1, order):
        image_left = cv2.imdecode(np.frombuffer(base64.b64decode(image0), dtype=np.uint8), cv2.IMREAD_COLOR)
        image_right = cv2.imdecode(np.frombuffer(base64.b64decode(image1), dtype=np.uint8), cv2.IMREAD_COLOR)
        points_left = kpoint0
        points_right = kpoint1

        l_w, l_h = get_image_size(image_left)
        r_w, r_h = get_image_size(image_right)
        points_left = np.float32([[x * l_w, y * l_h] for x, y in points_left])
        points_right = np.float32([[x * r_w, y * r_h] for x, y in points_right])

        matcher = Matcher()
        homography = matcher.match(image_left, image_right, points_left, points_right)
        if homography is None:
            result, images, homography = False, "", np.zeros((3, 3), dtype=np.float32)
        else:
            result = True
            if order == "left":
                homography = np.linalg.inv(homography)
                image_left, image_right = image_right, image_left

            anchor_top = np.dot(homography, np.array([0, 0, 1]))
            anchor_top = anchor_top / anchor_top[-1]
            anchor_tottom = np.dot(homography, np.array([0, image_right.shape[0], 1]))
            anchor_tottom = anchor_tottom / anchor_tottom[-1]
            anchor_x = max(anchor_top[0], anchor_tottom[0])
            if order == "left":
                anchor_x = abs(int(anchor_x * homography[2][2]))
                anchor_x = min(anchor_x, 1080)
                anchor_x = math.ceil(anchor_x / 8) * 8
                homography[0][-1] += anchor_x
                new_width = anchor_x + image_left.shape[1]
                perspective_width = new_width - image_left.shape[1]
                affine = np.float32([[1, 0, 0], [0, 1, 0]])
                affine_width = perspective_width
            else:
                new_right, new_bottom = get_corner(image_right, homography)
                new_width = max(image_left.shape[1], new_right)
                new_width = min(new_width, int(image_left.shape[1] + 1080))
                new_width = math.ceil(new_width / 16) * 16
                perspective_width = new_width
                affine = np.float32([[1, 0, -image_left.shape[1]], [0, 1, 0]])
                affine_width = new_width - image_left.shape[1]

            new_height = image_left.shape[0]
            images = np.zeros((new_height, new_width, 3), np.uint8)

            PanoramaCameraCalib.set_homography(cameras, name0 if order == "left" else name1,
                                                (perspective_width, new_height), homography.tolist(),
                                                (affine_width, new_height), affine.tolist())
            PanoramaCameraCalib.set_homography(cameras, name1 if order == "left" else name0,
                                                (image_left.shape[1], image_left.shape[0]), None,
                                                (image_left.shape[1], image_left.shape[0]), None)

            image_right = cv2.warpPerspective(image_right, homography, (perspective_width, new_height))
            image_right = cv2.warpAffine(image_right, affine, (affine_width, new_height))
            if order == "left":
                image_left, image_right = image_right, image_left
            images[:, :image_left.shape[1]] = image_left
            images[:, image_left.shape[1]:image_left.shape[1] + image_right.shape[1]] = image_right

        images = base64.b64encode(cv2.imencode('.jpg', images)[1]).decode("utf-8")

        return {"images": images, "homography": homography.tolist(), "result": result}