import numpy as np

def skew(vec):
    x, y, z = vec
    return np.array([
        [0, -z, y],
        [z, 0, -x],
        [-y, x, 0]])

def select(data, selectors):
    selectors = np.asarray(selectors).reshape(-1)
    return [d for d, s in zip(data, selectors) if bool(s)]


def grid_index(point, image_shape, grid_row, grid_col, grid_height, grid_width):
    point = np.asarray(point, dtype=float).reshape(-1)
    if point.size < 2 or not np.all(np.isfinite(point[:2])):
        return None

    x, y = point[:2]
    height, width = image_shape[:2]
    if x < 0 or x >= width or y < 0 or y >= height:
        return None

    row = min(grid_row - 1, max(0, int(y / grid_height)))
    col = min(grid_col - 1, max(0, int(x / grid_width)))
    return row * grid_col + col


def clip_patch_bounds(point, image_shape, radius=3):
    point = np.asarray(point, dtype=float).reshape(-1)
    if point.size < 2 or not np.all(np.isfinite(point[:2])):
        return None

    x, y = int(round(point[0])), int(round(point[1]))
    height, width = image_shape[:2]
    if x < 0 or x >= width or y < 0 or y >= height:
        return None

    return max(0, y - radius), min(height, y + radius + 1), max(0, x - radius), min(width, x + radius + 1)
