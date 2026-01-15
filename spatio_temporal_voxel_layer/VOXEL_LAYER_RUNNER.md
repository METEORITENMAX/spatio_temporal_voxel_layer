# Voxel Layer Runner

## Overview

The `voxel_layer_runner` is a standalone ROS 2 node that enables using the `SpatioTemporalVoxelLayer` plugin independently of Nav2. This node provides a way to run the voxel layer for obstacle detection and environment mapping without requiring a full Nav2 navigation stack.

## Features

- **Standalone Operation**: Runs the SpatioTemporalVoxelLayer plugin without Nav2
- **Point Cloud Processing**: Subscribes to point cloud topics (configured via plugin parameters)
- **Costmap Publishing**: Publishes the processed voxel grid as an OccupancyGrid message
- **Configurable Parameters**: Supports extensive configuration via ROS 2 parameters
- **TF Integration**: Automatically tracks robot pose via TF2

## Building

To build the package with the voxel_layer_runner node:

```bash
cd <your_ros2_workspace>
colcon build --packages-select spatio_temporal_voxel_layer
```

## Running

### Basic Usage

Run the node with default parameters:

```bash
ros2 run spatio_temporal_voxel_layer voxel_layer_runner
```

### With Parameter File

Run the node with a custom parameter file:

```bash
ros2 run spatio_temporal_voxel_layer voxel_layer_runner --ros-args --params-file /path/to/voxel_layer_runner_params.yaml
```

An example parameter file is provided in `example/voxel_layer_runner_params.yaml`.

## Parameters

### Node Parameters

- `global_frame` (string, default: "map"): The global reference frame for the costmap
- `robot_base_frame` (string, default: "base_link"): The robot's base frame
- `update_frequency` (double, default: 5.0): Update rate in Hz
- `width` (double, default: 10.0): Costmap width in meters
- `height` (double, default: 10.0): Costmap height in meters
- `resolution` (double, default: 0.05): Costmap resolution in meters per cell
- `origin_x` (double, default: -5.0): X-coordinate of costmap origin
- `origin_y` (double, default: -5.0): Y-coordinate of costmap origin
- `update_bounds_padding` (double, default: 5.0): Padding in meters around robot position for update region

### Voxel Layer Parameters

The node supports all parameters of the SpatioTemporalVoxelLayer plugin under the `voxel_layer` namespace. Key parameters include:

- `voxel_size` (double, default: 0.05): Size of each voxel in meters
- `voxel_decay` (double, default: 15.0): Voxel decay time in seconds (linear) or e^n (exponential)
- `decay_model` (int, default: 0): 0=linear, 1=exponential, -1=persistent
- `publish_voxel_map` (bool, default: true): Whether to publish the voxel grid for visualization
- `observation_sources` (string): Space-separated list of observation source names

For each observation source (e.g., `rgbd1_mark`, `rgbd1_clear`), configure:

- `topic` (string): The point cloud topic to subscribe to
- `data_type` (string): Either "PointCloud2" or "LaserScan"
- `marking` (bool): Whether this source marks obstacles
- `clearing` (bool): Whether this source clears obstacles
- `obstacle_range` (double): Maximum range for marking in meters
- `min_obstacle_height` (double): Minimum obstacle height in meters
- `max_obstacle_height` (double): Maximum obstacle height in meters

See `example/voxel_layer_runner_params.yaml` for a complete example configuration.

## Topics

### Subscribed Topics

- Configured point cloud topics (e.g., `/camera/depth/points`) - sensor_msgs/PointCloud2

### Published Topics

- `voxel_costmap` - nav_msgs/OccupancyGrid: The processed costmap
- `voxel_grid` - sensor_msgs/PointCloud2: The voxel grid visualization (if `publish_voxel_map` is true)

## Required TF Transforms

The node requires the following transforms to be available:

- `global_frame` → `robot_base_frame`: For robot localization

If TF transforms are not available, the node will use the origin position (0, 0, 0) and issue warnings.

## Example Setup

### 1. Prepare your environment

Ensure you have:
- ROS 2 installed
- Point cloud data published on a topic (e.g., from a depth camera)
- TF transforms published (e.g., from robot_state_publisher)

### 2. Create a parameter file

Copy and modify the example parameter file:

```bash
cp install/spatio_temporal_voxel_layer/share/spatio_temporal_voxel_layer/example/voxel_layer_runner_params.yaml my_params.yaml
```

Edit `my_params.yaml` to match your sensor topics and robot configuration.

### 3. Launch the node

```bash
ros2 run spatio_temporal_voxel_layer voxel_layer_runner --ros-args --params-file my_params.yaml
```

### 4. Visualize in RViz

Add the following displays in RViz:
- OccupancyGrid: Subscribe to `/voxel_costmap`
- PointCloud2: Subscribe to `/voxel_grid` (if published)

## Troubleshooting

### Plugin fails to load

Ensure that the `spatio_temporal_voxel_layer_core` library is properly installed:

```bash
ros2 pkg list | grep spatio_temporal_voxel_layer
```

### No costmap updates

Check that:
1. Point cloud data is being published on the configured topics
2. TF transforms are available
3. The `enabled` parameter is set to `true`

### High CPU usage

Consider:
1. Reducing `update_frequency`
2. Reducing `obstacle_range` for observation sources
3. Enabling the "voxel" filter to downsample point clouds
4. Reducing the costmap size (`width` and `height`)

## See Also

- [SpatioTemporalVoxelLayer Documentation](../README.md)
- [Nav2 Costmap 2D](https://navigation.ros.org/)
