import numpy as np

def test_point_in_polygon(point: list, polygon: list):
    # http://geomalgorithms.com/a03-_inclusion.html
    # crossing number algs
    x = point[0]
    y = point[1]
    cn = 0
    for i in range(len(polygon)):
        p = polygon[i]
        q = polygon[i + 1] if i < len(polygon) - 1 else polygon[0]
        up, down = (p, q) if p[1] > q[1] else (q, p)
        if down[1] < y < up[1]:
            x_intersect = down[0] + (up[0] - down[0]) * \
                (y - down[1]) / (up[1] - down[1])
            if x_intersect > x:
                cn += 1
    return cn & 1


def crop_points(points: np.ndarray, polygon: np.ndarray):
    points_left = []
    for i in range(len(points)):
        p = points[i]
        if test_point_in_polygon(p, polygon):
            points_left.append(p)
    # print(points_left)
    return points_left
