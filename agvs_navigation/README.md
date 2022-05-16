# Tuning Guide For Using TEB Without a Map.

1. Comment ```Plugins``` parts in both ```global_costmap_params.yaml``` and ```local_costmap_params.yaml```.
2. Set ```global_frame``` to ```odom``` in both ```global_costmap_params.yaml``` and ```local_costmap_params.yaml```.
3. Set ```static_map``` to ```false``` and ```rolling_window``` to ```true``` in ```global_costmap_params.yaml```.