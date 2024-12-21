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

BRIGHTNESS = (80.0 * 256.0)

colors_map = mpl.colormaps['gray'].resampled(65536)
colors = [np.array(colors_map(i / 65536.0)) * 65535 for i in range(65536)]
colors = np.stack(colors)
colors = colors[:, 0]

def pixel_to_world(xs, ys, meta):
    px =  xs / meta['pixel_per_meter'] + meta['x_min']
    py = -ys / meta['pixel_per_meter'] + meta['y_max']
    return px, py

def intensity_normalize(xs, ys, intensity):
    # histogram
    cut_off = 0 # np.percentile(intensity, 10)
    intensity = intensity - cut_off
    intensity = intensity * 65535.0
    histogram, bins = np.histogram(intensity, bins=1024, range=[0.0, 65535.0], density=True)
    cdf = histogram.cumsum() # cumulative distribution function
    cdf = 65535.0 * cdf / cdf[-1] # normalize

    # use linear interpolation of cdf to find new pixel values
    intensity_cdf = np.interp(intensity, bins[:-1], cdf)
    amplify = intensity_cdf / np.clip(intensity, 0.001, None)

    # adaptive clip limit
    clip_limit = max(1.0, BRIGHTNESS / np.mean(intensity))
    intensity_norm = intensity * np.clip(amplify, None, clip_limit)
    while clip_limit < 120.0 and np.mean(intensity_norm) < BRIGHTNESS:
        clip_limit = clip_limit + 0.1
        intensity_norm = intensity * np.clip(amplify, None, clip_limit)

    intensity = np.clip(intensity_norm, 0, 65535)
    return intensity, {'cdf': cdf, 'bins': bins, 'clip_limit': clip_limit, 'cut_off': cut_off}

def bev_generate(xs, ys, intensity, image_w, image_h):
    intensity = np.round(intensity).astype(int)

    # projection to image
    image = np.zeros((image_h, image_w), np.uint16) # black image
    image[ys, xs] = colors[intensity]
    return image

def filter_noise(xs, ys, zs, intensity):
    num = intensity.shape[0]
    filter_h_num = int(num * 0.999)
    filter_l_num = int(num * 0.01)

    idx = np.argsort(intensity)[filter_l_num:filter_h_num]
    return xs[idx], ys[idx], zs[idx], intensity[idx]

def scatter(xs, ys, zs, intensity, image_w, image_h):
    zs         = torch.from_numpy(zs)
    intensity  = torch.from_numpy(intensity)
    coordinate = torch.from_numpy(ys * image_w + xs).long()

    unq_coords, unq_inv, unq_cnt = torch.unique(coordinate, return_inverse=True, return_counts=True)

    zs        = torch_scatter.scatter_mean(zs, unq_inv, dim=0)
    intensity = torch_scatter.scatter_mean(intensity, unq_inv, dim=0)
    xs = unq_coords %  image_w
    ys = unq_coords // image_w
    return xs.numpy(), ys.numpy(), zs.numpy(), intensity.numpy()

def load_pointcloud(filename, pixel_per_meter):
    print('loading {}'.format(filename))
    pc = pypcd.PointCloud.from_path(filename)

    print('start to process total {} points'.format(pc.pc_data['intensity'].shape[0]))
    x_min, x_max = float(np.min(pc.pc_data['x'])), float(np.max(pc.pc_data['x']))
    y_min, y_max = float(np.min(pc.pc_data['y'])), float(np.max(pc.pc_data['y']))
    x_range, y_range = (x_max - x_min), (y_max - y_min)
    image_w = np.ceil(x_range * pixel_per_meter).astype(int) + 1
    image_h = np.ceil(y_range * pixel_per_meter).astype(int) + 1

    print("X: [{}, {}], Y: [{}, {}]".format(x_min, x_max, y_min, y_max))

    xs        = np.round( (pc.pc_data['x'] - x_min) * pixel_per_meter).astype(int)
    ys        = np.round(-(pc.pc_data['y'] - y_max) * pixel_per_meter).astype(int)
    zs        = np.copy(pc.pc_data['z'])
    intensity = np.copy(pc.pc_data['intensity'])
    del pc

    return xs, ys, zs, intensity, image_w, image_h, {'x_min': x_min, 'x_max': x_max, 'y_min': y_min, 'y_max': y_max, 'pixel_per_meter': pixel_per_meter}

def process_cloud(xs, ys, zs, intensity, image_w, image_h, meta):
    xs, ys, zs, intensity = filter_noise(xs, ys, zs, intensity)
    print('after noise filter: {} points'.format(intensity.shape[0]))

    xs, ys, zs, intensity = scatter(xs, ys, zs, intensity, image_w, image_h)
    print('after scatter: {} points'.format(intensity.shape[0]))
    return xs, ys, zs, intensity, image_w, image_h, meta

def preprocess(filename, pixel_per_meter):
    return process_cloud(*load_pointcloud(filename, pixel_per_meter))

def convert(xs, ys, zs, intensity, image_w, image_h, window, pixel_per_meter):
    intensity_norm = copy.deepcopy(intensity)

    patch_size      = int(window * pixel_per_meter)
    half_patch_size = int(patch_size / 2)
    quat_patch_size = int(half_patch_size / 2)
    image_w = int((image_w + half_patch_size) / half_patch_size) * half_patch_size
    image_h = int((image_h + half_patch_size) / half_patch_size) * half_patch_size
    print("image size: {}, {}".format(image_w, image_h))

    # histogram equalization by patch
    for xi in range(0, image_w + 1, half_patch_size):
        for yi in range(0, image_h + 1, half_patch_size):
            xy_mask_l = (xs >= (xi - patch_size)) & (xs <= (xi + patch_size)) & (ys >= (yi - patch_size)) & (ys <= (yi + patch_size))
            if np.count_nonzero(xy_mask_l) > 100:
                # equalization with large patch
                xs_patch, ys_patch, intensity_patch = xs[xy_mask_l], ys[xy_mask_l], intensity[xy_mask_l]
                intensity_patch, hist = intensity_normalize(xs_patch, ys_patch, intensity_patch)

                # update the inner patch
                xy_mask_s = (xs >= (xi - quat_patch_size)) & (xs <= (xi + quat_patch_size)) & (ys >= (yi - quat_patch_size)) & (ys <= (yi + quat_patch_size))
                xy_mask_p = (xs_patch >= (xi - quat_patch_size)) & (xs_patch <= (xi + quat_patch_size)) & (ys_patch >= (yi - quat_patch_size)) & (ys_patch <= (yi + quat_patch_size))
                intensity_norm[xy_mask_s] = intensity_patch[xy_mask_p]

    # projection
    image = bev_generate(xs, ys, intensity_norm, image_w, image_h)
    return image

def main():
    parser = argparse.ArgumentParser(description='project the pointcloud to BEV')
    parser.add_argument('-i', '--data_path', required=True, help='pointcloud path')
    parser.add_argument('-w', '--window', default=50.0, type=float, help='sliding window to equalization (m)')
    parser.add_argument('-r', '--pixel_per_meter', default=25, type=int, help='pixel per meter')
    parser.add_argument('-o', '--output', required=True, help='output path for save')
    args = parser.parse_args()

    xs, ys, zs, intensity, image_w, image_h, meta = preprocess(args.data_path, args.pixel_per_meter)
    image = convert(xs, ys, zs, intensity, image_w, image_h, args.window, args.pixel_per_meter)

    # BEV Image
    cv2.imwrite("{}/bev.png".format(args.output), image)

if __name__ == "__main__":
    main()