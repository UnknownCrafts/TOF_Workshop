import open3d as o3d
import numpy as np

input = input("Enter a number from 1 to 7: ")

if input == "2":
    pcd = o3d.io.read_point_cloud("sample-data/close.pcd")
elif input == "3":
    pcd = o3d.io.read_point_cloud("sample-data/hand-close.pcd")
elif input == "4":
    pcd = o3d.io.read_point_cloud("sample-data/hand-touching.pcd")
elif input == "5":
    pcd = o3d.io.read_point_cloud("sample-data/no-obj.pcd")
elif input == "6":
    pcd = o3d.io.read_point_cloud("sample-data/superclose.pcd")
elif input == "7":
    pcd = o3d.io.read_point_cloud("sample-data/touching.pcd")
else:
    pcd = o3d.io.read_point_cloud("sample-data/3mm-distance.pcd")

o3d.visualization.draw_geometries([pcd])
