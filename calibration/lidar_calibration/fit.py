import numpy as np
import math

def get_plane_coefficient(p1, p2, p3):
    # from normal vector
    # https://blog.csdn.net/zhouschina/article/details/8784908
    a = (p2[1] - p1[1]) * (p3[2] - p1[2]) - \
        (p3[1] - p1[1]) * (p2[2] - p1[2])  # yz
    b = (p2[2] - p1[2]) * (p3[0] - p1[0]) - \
        (p3[2] - p1[2]) * (p2[0] - p1[0])  # zx
    c = (p2[0] - p1[0]) * (p3[1] - p1[1]) - \
        (p3[0] - p1[0]) * (p2[1] - p1[1])  # xy
    norm = math.sqrt(a**2 + b**2 + c**2)
    d = -(a * p1[0] + b * p1[1] + c * p1[2])
    if c > 0:
        return a / norm, b / norm, c / norm, d / norm
    else:
        return -a / norm, -b / norm, -c / norm, -d / norm


def is_colinear(p1, p2, p3):
    a = (p2[1] - p1[1]) * (p3[2] - p1[2]) - \
        (p3[1] - p1[1]) * (p2[2] - p1[2])  # yz
    b = (p2[2] - p1[2]) * (p3[0] - p1[0]) - \
        (p3[2] - p1[2]) * (p2[0] - p1[0])  # zx
    c = (p2[0] - p1[0]) * (p3[1] - p1[1]) - \
        (p3[0] - p1[0]) * (p2[1] - p1[1])  # xy
    return (abs(a) + abs(b) + abs(c)) < 1e-5


def fit_plane(points: np.ndarray, sigma=1e-2, num_iteration=1000, thresh=0.9):
    # https://lixin97.com/2019/04/10/RANSAC/
    num_inliners_max = 0
    best_coef = np.zeros(4)
    for _ in range(num_iteration):
        # print(_)
        while True:
            # [i, j, k] = random.sample(range(len(points)), 3)
            [i, j, k] = np.random.choice(
                np.arange(len(points)), 3, replace=False)
            if not is_colinear(points[i], points[j], points[k]):
                break
        coef = [a, b, c, d] = get_plane_coefficient(
            points[i], points[j], points[k])

        # count_inliners(points, [a,b,c,d])
        num_inliners = 0
        for p in points:
            # distance = np.abs(np.dot(coef, np.append(p, 1)))
            x, y, z = p
            distance = abs(a * x + b * y + c * z + d)
            if distance < sigma:
                num_inliners += 1

        if num_inliners > thresh * len(points):
            for i, val in enumerate(coef):
                best_coef[i]=val
            break

        if num_inliners > num_inliners_max:
            num_inliners_max=num_inliners
            for i, val in enumerate(coef):
                best_coef[i]=val

    # print(num_inliners_max, len(points))

    return best_coef
