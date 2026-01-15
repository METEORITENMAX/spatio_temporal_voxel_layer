# Launch Files

This directory contains example launch files for the Spatio-Temporal Voxel Layer plugin.

## Available Launch Files

### 1. `standalone_example.launch.py`

A minimal standalone example that demonstrates the plugin running with a simple costmap node.

**Usage:**
```bash
ros2 launch spatio_temporal_voxel_layer standalone_example.launch.py
```

**Parameters:**
- `use_sim_time` (default: false): Whether to use simulation time

**Requirements:**
- A depth camera or 3D lidar publishing on `/camera/depth/points`
- TF frames: `map` and `base_link` properly configured

**Use this for:**
- Quick testing of the plugin
- Learning how the plugin works
- Debugging sensor configuration

### 2. `example_nav2.launch.py`

An example showing how to launch a costmap with external configuration files.

**Usage:**
```bash
ros2 launch spatio_temporal_voxel_layer example_nav2.launch.py
```

Or with a custom config:
```bash
ros2 launch spatio_temporal_voxel_layer example_nav2.launch.py \
  params_file:=/path/to/your/config.yaml
```

**Parameters:**
- `params_file` (default: standard_indoor_environment_config.yaml): Path to ROS 2 parameters file
- `use_sim_time` (default: false): Whether to use simulation time

**Use this for:**
- Testing with different configuration files from the example directory
- Understanding how to integrate with external configs

## Integration with Your Robot

**Important:** These launch files are examples for testing and learning. In production:

1. **Add the plugin to your Nav2 params file:**
   - Modify your existing `nav2_params.yaml` 
   - Add the plugin to your global and/or local costmap
   - See the [example directory](../example/) for configuration samples

2. **Launch with your existing Nav2 bringup:**
   ```bash
   ros2 launch your_robot_bringup navigation.launch.py
   ```
   or
   ```bash
   ros2 launch nav2_bringup navigation_launch.py params_file:=/path/to/your/nav2_params.yaml
   ```

3. The plugin will automatically load as part of the costmap layers.

## Troubleshooting

If the launch files don't work:

1. **Check sensor topic:** Make sure your sensor is publishing to the correct topic
   ```bash
   ros2 topic list | grep depth
   ros2 topic echo /camera/depth/points --once
   ```

2. **Check TF frames:**
   ```bash
   ros2 run tf2_ros tf2_echo map base_link
   ```

3. **Ubuntu 20.04 users:** Set the LD_PRELOAD environment variable:
   ```bash
   export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libjemalloc.so.2
   ```

4. **Verify installation:**
   ```bash
   ros2 pkg list | grep spatio_temporal_voxel_layer
   ```

## See Also

- [Main README](../README.md) - Full documentation
- [Example configs](../example/) - Sample configuration files
- [Nav2 Documentation](https://navigation.ros.org/) - ROS 2 Navigation
