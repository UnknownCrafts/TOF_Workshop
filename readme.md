# IEEE uOttawa Point Cloud Processing with ToF Sensor Workshop

## Solution (DO NOT VIEW UNTIL THE END OF THE WORKSHOP)

<details>
  <summary>Possible Solution</summary>

  ```
    cfg_voxel_size = 2  # voxel downsample
    cfg_roi_max_distance = 200.0 # mm, remove points further than this
    cfg_outlier_removal_radius = 4 # radius outlier removal
    cfg_outlier_removal_min_neighbors = 20 # radius outlier removal
    cfg_dbscan_epsilon = 3 # dbscan 
    cfg_dbscan_min_neighbouring_pts = 10

    cfg_bounding_box_size_threshold = 25000
  ```
</details>