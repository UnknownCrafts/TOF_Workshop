import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

# Custom algorithm libraries
import collision_detection_library 

# Load or create your point cloud
# Replace 'your_point_cloud.pcd' with the path to your file or generate point cloud data
raw_pcd = o3d.io.read_point_cloud("sample-data/hand-touching.pcd")

filtered_pcd = collision_detection_library.preprocessing(raw_pcd)

# Uncomment line below to visualize the resulting filtered point cloud
# o3d.visualization.draw_geometries([filtered_pcd])

# Perform DBscan clustering
labels = np.array(filtered_pcd.cluster_dbscan(eps=collision_detection_library.cfg_dbscan_epsilon, min_points=collision_detection_library.cfg_dbscan_min_neighbouring_pts, print_progress=True))

max_label = labels.max()
print(f"point cloud has {max_label + 1} clusters")
colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
colors[labels < 0] = 0
filtered_pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
o3d.visualization.draw_geometries([filtered_pcd])
