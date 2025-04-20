import numpy as np

def dbscan(poses, object_labels, eps=0.5, min_samples=1):
    poses = np.array(poses)
    N = poses.shape[0]
    labels = np.full(N, -1, dtype=int)  # -1 = noise
    visited = np.zeros(N, dtype=bool)
    cluster_id = 0

    def region_query(i):
        dists = np.linalg.norm(poses - poses[i], axis=1)
        return np.where(dists <= eps)[0]

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
            seeds.remove(i)
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

    # Group points and object labels into clusters
    clusters = []
    cluster_labels = []
    for cluster_label in range(cluster_id):
        cluster = poses[labels == cluster_label].tolist()
        cluster_objects = [object_labels[i] for i in range(N) if labels[i] == cluster_label]
        clusters.append(cluster)
        cluster_labels.append(cluster_objects)

    return clusters, cluster_labels

if __name__ == "__main__":

    # Example usage
    poses = np.array([
        [1.0, 2.0],
        [1.2, 2.1],
        [8.0, 8.0],
        [8.1, 8.2],
        [0.9, 2.0],
        [50.0, 50.0],  # Outlier
    ])

    object_labels = ["bottle", "cola", "table", "ball", "wrench", "outlier"]

    clusters, cluster_labels = dbscan(poses, object_labels, eps=0.5, min_samples=2)

    # Print results
    for i, (cluster, labels) in enumerate(zip(clusters, cluster_labels)):
        print(f"Cluster {i}: {cluster}")
        print(f"Object Labels: {labels}")