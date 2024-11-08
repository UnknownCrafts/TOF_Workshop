import open3d as o3d
import numpy as np

# Custom algorithm libraries
import collision_detection_library 

# Load or create your point cloud
# Replace 'your_point_cloud.pcd' with the path to your file or generate point cloud data
raw_pcd = o3d.io.read_point_cloud("sample-data/touching.pcd")

filtered_pcd = collision_detection_library.preprocessing(raw_pcd)

# Uncomment line below to visualize the resulting filtered point cloud
# o3d.visualization.draw_geometries([filtered_pcd])

# Perform DBscan clustering
cluster_pcd_arr, noise_pcd, cluster_counter = collision_detection_library.clustering(filtered_pcd)

# Optional: visualize the point cloud
# o3d.visualization.draw_geometries([cluster_pcd_arr[0]])
if(cluster_counter > 1):
    closest_distance = collision_detection_library.distance_calc(cluster_pcd_arr[0], cluster_pcd_arr[1])
else:
    print("Only one cluster")
