import numpy as np

def dbscan(poses, eps=0.5, min_samples=1):
    poses= np.array(poses)
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

    # Group points into clusters
    clusters = []
    for cluster_label in range(cluster_id):
        cluster = poses[labels == cluster_label].tolist()
        clusters.append(cluster)

    return clusters

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

    clusters = dbscan(poses, eps=0.5, min_samples=2)

    # Print results
    for i, cluster in enumerate(clusters):
        print(f"Cluster {i}: {cluster}")