import numpy as np
from nav_msgs.msg import OccupancyGrid

def world_to_grid(pos, origin, resolution):
    gx = int((pos[0] - origin[0]) / resolution)
    gy = int((pos[1] - origin[1]) / resolution)
    return gx, gy

def bresenham_line(x0, y0, x1, y1):
    points = []
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    x, y = x0, y0
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        points.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
    return points

def is_visible(p1, p2, occupancy_grid, resolution, origin, width, height, wall_threshold=50):
    x0, y0 = world_to_grid(p1, origin, resolution)
    x1, y1 = world_to_grid(p2, origin, resolution)
    for x, y in bresenham_line(x0, y0, x1, y1):
        if 0 <= x < width and 0 <= y < height:
            idx = y * width + x
            if occupancy_grid[idx] >= wall_threshold:
                return False
    return True

def dbscan_map_aware(poses, object_labels, eps=0.5, min_samples=1, map: OccupancyGrid = None):
    poses = np.array(poses)
    N = poses.shape[0]
    labels = np.full(N, -1, dtype=int)
    visited = np.zeros(N, dtype=bool)
    cluster_id = 0

    if map:
        resolution = map.info.resolution
        origin = (map.info.origin.position.x, map.info.origin.position.y)
        width = map.info.width
        height = map.info.height
        grid_data = map.data

    def region_query(i):
        neighbors = []
        for j in range(N):
            if i == j:
                continue
            dist = np.linalg.norm(poses[i] - poses[j])
            if dist <= eps:
                if map:
                    if is_visible(poses[i], poses[j], grid_data, resolution, origin, width, height):
                        neighbors.append(j)
                else:
                    neighbors.append(j)
        return neighbors

    for i in range(N):
        if visited[i]:
            continue
        visited[i] = True
        neighbors = region_query(i)
        if len(neighbors) < min_samples:
            labels[i] = -1
        else:
            labels[i] = cluster_id
            seeds = list(neighbors)
            while seeds:
                current = seeds.pop()
                if not visited[current]:
                    visited[current] = True
                    current_neighbors = region_query(current)
                    if len(current_neighbors) >= min_samples:
                        seeds += [n for n in current_neighbors if n not in seeds]
                if labels[current] == -1:
                    labels[current] = cluster_id
            cluster_id += 1

    clusters = []
    cluster_labels = []
    for cluster_label in range(cluster_id):
        cluster = poses[labels == cluster_label].tolist()
        cluster_objects = [object_labels[i] for i in range(N) if labels[i] == cluster_label]
        clusters.append(cluster)
        cluster_labels.append(cluster_objects)

    return clusters, cluster_labels
