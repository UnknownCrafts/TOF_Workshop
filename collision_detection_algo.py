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

# Count number of clusters to determine 
if(cluster_counter > 1):
    # Case 3: Two objects present, determine the distance
    closest_distance = collision_detection_library.distance_calc(cluster_pcd_arr[0], cluster_pcd_arr[1])
    print(f'RESULT: Two objects present, {closest_distance} mm apart!')
else:
    # Case 1 and 2: Object may not be present or Object is touching the EE
    print("Only one cluster")
    bounding_box_size = collision_detection_library.boundbox_size_calc(cluster_pcd_arr[0])
    if(bounding_box_size > collision_detection_library.cfg_bounding_box_size_threshold):
        print("RESULT: Object is touching!")
    else:
        print("RESULT: No object is present!")
    
