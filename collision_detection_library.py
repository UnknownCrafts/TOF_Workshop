import open3d as o3d
import numpy as np
import os

# returns a filtered pcd after voxel downsampling and 
def preprocessing(pcd):
    # Voxel downsampling
    voxel_size = 2  # Set the voxel size (e.g., 5 mm)
    pcd = pcd.voxel_down_sample(voxel_size=voxel_size)

    # Set maximum distance threshold
    max_distance = 200.0  # replace with your desired threshold

    # Convert point cloud to numpy array for filtering
    points = np.asarray(pcd.points)

    # Filter points based on distance from the origin (0, 0, 0)
    distances = np.linalg.norm(points, axis=1)
    filtered_points = points[distances <= max_distance]

    # Create a new point cloud with the filtered points
    filtered_pcd = o3d.geometry.PointCloud()
    filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)

    # Radius outlier removal
    # Set parameters: radius to consider neighbors and minimum number of neighbors within that radius
    radius = 4   # Replace with your desired radius
    min_neighbors = 20  # Minimum neighbors within radius to consider a point as inlier

    filtered_pcd, ind = filtered_pcd.remove_radius_outlier(nb_points=min_neighbors, radius=radius)

    return filtered_pcd

# Returns the following
# 1. Array of PCD containing each cluster
# 2. PCD containing noise points (non clusters)
# 3. Number of clusters
def clustering(pcd, cluster_folder="clusters", logging=False):

    if(logging == True):
        if not os.path.exists(cluster_folder):
            os.makedirs(cluster_folder)
    # Convert point cloud to numpy array for processing
    points = np.asarray(pcd.points)

    print(points.shape)

    # DBSCAN parameters
    epsilon = 3  # Maximum distance between points in the same cluster
    min_points = 15  # Minimum number of points to form a cluster

    # Perform DBSCAN clustering using Open3D
    labels = np.array(pcd.cluster_dbscan(eps=epsilon, min_points=min_points, print_progress=True))

    # Get unique labels, -1 is the noise label
    unique_labels = set(labels)

    
    # Array containing all cluster data points
    cluster_arr = []
    cluster_counter = 0
    # Noise points
    noise_points = None

    # Prepare to save clusters
    for label in unique_labels:
        if label == -1:  # This is the noise label
            noise_points = points[labels == -1]

            if(logging == True):
                # Save the noise as an .xyz
                noise_file_path = f"{cluster_folder}/noise.xyz"
                np.savetxt(noise_file_path, noise_points, header="Noise Points", comments='', fmt='%.6f')
                print(f"Saved noise points to {noise_file_path}")
        else:
            # Save each cluster to its own file
            cluster_points = points[labels == label]
            cluster_arr.append(cluster_points)
            cluster_counter += 1

            if(logging == True):
                # Save the clusters as an .xyz
                cluster_file_path = f"{cluster_folder}/cluster_{label}.xyz"
                np.savetxt(cluster_file_path, cluster_points, header=f"Cluster {label}", comments='', fmt='%.6f')
                print(f"Saved cluster {label} to {cluster_file_path}")

    print("Clustering Complete")
    
    # Convert noise to PCD from numpy array
    noise_pcd =  o3d.geometry.PointCloud()
    noise_pcd.points = o3d.utility.Vector3dVector(noise_points)

    # Convert clusters to PCD from numpy array
    cluster_pcd_arr = []
    for cluster in cluster_arr:

        # Convert the NumPy array to an Open3D point cloud
        temp_pcd = o3d.geometry.PointCloud()
        temp_pcd.points = o3d.utility.Vector3dVector(cluster)

        cluster_pcd_arr.append(temp_pcd)

    return cluster_pcd_arr, noise_pcd, cluster_counter

def distance_calc(pcd1, pcd2):
    # Convert the points in the second point cloud to a KDTree for efficient nearest neighbor search
    pcd2_tree = o3d.geometry.KDTreeFlann(pcd2)

    # Initialize a list to hold the minimum distances for each point in pcd1
    distances = []

    # Find the closest point in pcd2 for each point in pcd1
    for point in pcd1.points:
        # Perform nearest neighbor search
        [_, idx, dist] = pcd2_tree.search_knn_vector_3d(point, 1)
        # Take the square root of the distance to get Euclidean distance
        closest_distance = np.sqrt(dist[0])
        distances.append(closest_distance)

    # Find the minimum distance among all computed distances
    min_distance = min(distances)
    print(f"The closest distance between the two point clouds is: {min_distance}")
    return min_distance
