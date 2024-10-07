import os
import sys

sys.path.append(os.getcwd())

import copy
import argparse
import cv2
import torch
import torch_scatter
import numpy as np
import matplotlib as mpl
import third_party.pypcd as pypcd

BRIGHTNESS = 110.0

colors_map = mpl.colormaps['gray'].resampled(256)
colors = [np.array(colors_map(i / 256.0)) * 255 for i in range(256)]
colors = np.stack(colors)
colors = colors[:, 0]

def intensity_normalize(xs, ys, intensity):
    if intensity.shape[0] < 100:
        return intensity, None

    # histogram
    histogram, bins = np.histogram(intensity, bins=1024, range=[0.0, 1.0], density=True)
    cdf = histogram.cumsum() # cumulative distribution function
    cdf = 255.0 * cdf / cdf[-1] # normalize

    # use linear interpolation of cdf to find new pixel values
    intensity_cdf = np.interp(intensity, bins[:-1], cdf)

    # adaptive clip limit
    clip_limit = 200
    mean_intensity = 0
    while clip_limit < 2000 and mean_intensity < BRIGHTNESS:
        amplify = intensity_cdf / np.clip(intensity, 0.001, None)
        amplify = np.clip(amplify, None, clip_limit)
        intensity_norm = intensity * amplify
        # calculate the statistics
        mean_intensity = np.mean(intensity_norm)
        clip_limit = clip_limit + 100.0

    intensity = np.clip(intensity_norm, 0, 255)
    return intensity, {'cdf': cdf, 'bins': bins, 'clip_limit': clip_limit}

def bev_generate(xs, ys, intensity, image_w, image_h):
    intensity = np.round(intensity).astype(int)

    # projection to image
    image = np.zeros((image_h, image_w), np.uint8) # black image
    image[ys, xs] = colors[intensity]
    return image

def quantify_points(points, pixel_per_meter):
    x_min, x_max = np.min(points[:, 0]), np.max(points[:, 0])
    y_min, y_max = np.min(points[:, 1]), np.max(points[:, 1])
    x_range, y_range = (x_max - x_min), (y_max - y_min)
    x_pixel_size = np.ceil(x_range * pixel_per_meter).astype(int)
    y_pixel_size = np.ceil(y_range * pixel_per_meter).astype(int)
    image_w, image_h = x_pixel_size + 1, y_pixel_size + 1

    print("X: [{}, {}], Y: [{}, {}]".format(x_min, x_max, y_min, y_max))

    xs = np.round( ((points[:, 0] - x_min) / x_range) * x_pixel_size).astype(int)
    ys = np.round(-((points[:, 1] - y_max) / y_range) * y_pixel_size).astype(int)
    return xs, ys, image_w, image_h, {'x_min': x_min, 'x_max': x_max, 'y_min': y_min, 'y_max': y_max}

def filter_noise(xs, ys, intensity):
    num = intensity.shape[0]
    filter_h_num = int(num * 0.999)
    filter_l_num = int(num * 0.01)

    idx = np.argsort(intensity)[filter_l_num:filter_h_num]
    return xs[idx], ys[idx], intensity[idx]

def scatter(xs, ys, intensity, image_w, image_h):
    intensity  = torch.from_numpy(intensity)
    coordinate = torch.from_numpy(ys * image_w + xs).int()
    unq_coords, unq_inv, unq_cnt = torch.unique(coordinate, return_inverse=True, return_counts=True)
    intensity = torch_scatter.scatter_mean(intensity, unq_inv, dim=0)
    xs = unq_coords %  image_w
    ys = unq_coords // image_w
    return xs.numpy(), ys.numpy(), intensity.numpy()

def load_pointcloud(filename):
    print('loading {}'.format(filename))
    pc        = pypcd.PointCloud.from_path(filename)
    points    = np.stack((pc.pc_data['x'], pc.pc_data['y'], pc.pc_data['z']), axis=1)
    intensity = pc.pc_data['intensity']
    return points, intensity

def process_cloud(points, intensity, pixel_per_meter):
    print('start processing total {} points'.format(intensity.shape[0]))
    xs, ys, image_w, image_h, meta = quantify_points(points, pixel_per_meter)

    xs, ys, intensity = filter_noise(xs, ys, intensity)
    print('after noise filter: {} points'.format(intensity.shape[0]))

    xs, ys, intensity = scatter(xs, ys, intensity, image_w, image_h)
    print('after scatter: {} points'.format(intensity.shape[0]))
    return xs, ys, intensity, image_w, image_h, meta

def preprocess(filename, pixel_per_meter):
    points, intensity = load_pointcloud(filename)
    return process_cloud(points, intensity, pixel_per_meter)

def convert(xs, ys, intensity, image_w, image_h, pixel_per_meter):
    intensity_norm = copy.deepcopy(intensity)

    patch_size      = int(10.0 * pixel_per_meter)
    half_patch_size = int(patch_size / 2)
    quat_patch_size = int(half_patch_size / 2)
    image_w = int((image_w + half_patch_size) / half_patch_size) * half_patch_size
    image_h = int((image_h + half_patch_size) / half_patch_size) * half_patch_size
    print("Image Size: {}, {}".format(image_w, image_h))

    # histogram equalization by patch
    statistics = []
    for xi in range(0, image_w + 1, half_patch_size):
        for yi in range(0, image_h + 1, half_patch_size):
            xy_mask_l = (xs >= (xi - half_patch_size)) & (xs <= (xi + half_patch_size)) & (ys >= (yi - half_patch_size)) & (ys <= (yi + half_patch_size))
            xy_mask_s = (xs >= (xi - quat_patch_size)) & (xs <= (xi + quat_patch_size)) & (ys >= (yi - quat_patch_size)) & (ys <= (yi + quat_patch_size))

            # equalization with large patch
            xs_patch, ys_patch, intensity_patch = xs[xy_mask_l], ys[xy_mask_l], intensity[xy_mask_l]
            intensity_patch, hist = intensity_normalize(xs_patch, ys_patch, intensity_patch)

            # update the inner patch
            xy_mask = (xs_patch >= (xi - quat_patch_size)) & (xs_patch <= (xi + quat_patch_size)) & (ys_patch >= (yi - quat_patch_size)) & (ys_patch <= (yi + quat_patch_size))
            intensity_norm[xy_mask_s] = intensity_patch[xy_mask]

            x_bound = [xi - quat_patch_size, xi + quat_patch_size]
            y_bound = [yi - quat_patch_size, yi + quat_patch_size]
            statistics.append({'x_bound': x_bound, 'y_bound': y_bound, 'hist': hist})

    # projection
    image = bev_generate(xs, ys, intensity_norm, image_w, image_h)
    return image, statistics

def main():
    parser = argparse.ArgumentParser(description='project the pointcloud to BEV')
    parser.add_argument('-i', '--data_path', required=True, help='pointcloud path')
    parser.add_argument('-r', '--pixel_per_meter', default=20, type=int, help='pixel per meter')
    parser.add_argument('-o', '--output', required=True, help='output path for save')
    args = parser.parse_args()

    xs, ys, intensity, image_w, image_h, meta = preprocess(args.data_path, args.pixel_per_meter)
    image, statistics = convert(xs, ys, intensity, image_w, image_h, args.pixel_per_meter)
    cv2.imwrite("{}/bev.png".format(args.output), image)

if __name__ == "__main__":
    main()