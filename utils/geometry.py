import numpy as np
import cv2


def constrain_to_polygon(point, polygon):
    if len(polygon) < 3:
        return point

    point = np.array(point)
    polygon_np = np.array(polygon, dtype=np.int32)

    inside = cv2.pointPolygonTest(polygon_np, (int(point[0]), int(point[1])), False) >= 0

    if inside:
        return (int(point[0]), int(point[1]))

    closest_point = None
    min_dist = float('inf')

    for i in range(len(polygon)):
        start = np.array(polygon[i])
        end = np.array(polygon[(i + 1) % len(polygon)])

        edge_vec = end - start
        point_vec = point - start
        edge_len_sq = np.dot(edge_vec, edge_vec)

        if edge_len_sq == 0:
            continue

        t = np.dot(point_vec, edge_vec) / edge_len_sq
        t = max(0, min(1, t))
        projection = start + t * edge_vec

        dist = np.linalg.norm(point - projection)
        if dist < min_dist:
            min_dist = dist
            closest_point = projection

    # Modified return statement to handle numpy arrays properly
    if closest_point is not None:
        return (int(closest_point[0]), int(closest_point[1]))
    else:
        return point