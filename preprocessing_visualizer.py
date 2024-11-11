import open3d as o3d
import numpy as np
import collision_detection_library

# Load or create your point cloud
# Replace 'your_point_cloud.pcd' with the path to your file or generate point cloud data
pcd = o3d.io.read_point_cloud("sample-data/touching.pcd")

# Raw data visualization
print("DISPLAYING RAW DATA")
o3d.visualization.draw_geometries([pcd])

voxel_size = collision_detection_library.cfg_voxel_size  # Set the voxel size (e.g., 5 mm)
print("DISPLAYING VOXEL DOWNSAMPLE")
pcd = pcd.voxel_down_sample(voxel_size=voxel_size)

# Down sample visualization
o3d.visualization.draw_geometries([pcd])

# Set maximum distance threshold
max_distance = collision_detection_library.cfg_roi_max_distance # replace with your desired threshold

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
radius = collision_detection_library.cfg_outlier_removal_radius
min_neighbors = collision_detection_library.cfg_outlier_removal_min_neighbors

filtered_pcd, ind = filtered_pcd.remove_radius_outlier(nb_points=min_neighbors, radius=radius)

print("DISPLAYING NOISE REDUCED POINT CLOUD")
o3d.visualization.draw_geometries([filtered_pcd])
