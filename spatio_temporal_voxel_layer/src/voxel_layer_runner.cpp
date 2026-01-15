/*********************************************************************
 *
 * Software License Agreement
 *
 *  Copyright (c) 2018, Simbe Robotics, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Simbe Robotics, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Steve Macenski (steven.macenski@simberobotics.com)
 * Purpose: Standalone ROS 2 node to run SpatioTemporalVoxelLayer
 *          independently of Nav2
 *********************************************************************/

#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "spatio_temporal_voxel_layer/spatio_temporal_voxel_layer.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/utils.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class VoxelLayerRunner : public rclcpp::Node
{
public:
  VoxelLayerRunner()
  : Node("voxel_layer_runner")
  {
    RCLCPP_INFO(this->get_logger(), "Initializing VoxelLayerRunner node...");

    // Declare parameters
    this->declare_parameter("global_frame", "map");
    this->declare_parameter("robot_base_frame", "base_link");
    this->declare_parameter("update_frequency", 5.0);
    this->declare_parameter("width", 10.0);
    this->declare_parameter("height", 10.0);
    this->declare_parameter("resolution", 0.05);
    this->declare_parameter("origin_x", -5.0);
    this->declare_parameter("origin_y", -5.0);
    this->declare_parameter("update_bounds_padding", 5.0);

    // Get parameters
    std::string global_frame = this->get_parameter("global_frame").as_string();
    std::string robot_base_frame = this->get_parameter("robot_base_frame").as_string();
    double update_frequency = this->get_parameter("update_frequency").as_double();
    double width = this->get_parameter("width").as_double();
    double height = this->get_parameter("height").as_double();
    double resolution = this->get_parameter("resolution").as_double();
    double origin_x = this->get_parameter("origin_x").as_double();
    double origin_y = this->get_parameter("origin_y").as_double();

    RCLCPP_INFO(this->get_logger(), "Creating costmap with size: %.2fx%.2f at resolution %.3f",
                width, height, resolution);

    // Initialize TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create a layered costmap
    layered_costmap_ = std::make_shared<nav2_costmap_2d::LayeredCostmap>(
      global_frame, false, false);
    
    // Initialize the underlying costmap with proper dimensions
    layered_costmap_->resizeMap(
      static_cast<unsigned int>(width / resolution),
      static_cast<unsigned int>(height / resolution),
      resolution, origin_x, origin_y);

    // Initialize plugin loader
    plugin_loader_ = std::make_unique<pluginlib::ClassLoader<nav2_costmap_2d::Layer>>(
      "nav2_costmap_2d", "nav2_costmap_2d::Layer");

    try {
      // Load the spatio_temporal_voxel_layer plugin
      RCLCPP_INFO(this->get_logger(), "Loading SpatioTemporalVoxelLayer plugin...");
      voxel_layer_ = plugin_loader_->createSharedInstance(
        "spatio_temporal_voxel_layer::SpatioTemporalVoxelLayer");

      // The initialize method signature for nav2_costmap_2d::Layer
      // void initialize(LayeredCostmap * parent, std::string name, 
      //                 tf2_ros::Buffer * tf, rclcpp_lifecycle::LifecycleNode::SharedPtr node,
      //                 rclcpp::CallbackGroup::SharedPtr callback_group)
      
      // Create a lifecycle node for the plugin
      lifecycle_node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("voxel_layer_lifecycle");
      
      // Activate the lifecycle node so the plugin can function properly
      lifecycle_node_->configure();
      lifecycle_node_->activate();
      
      // Create callback group
      auto callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      
      // Initialize the layer
      voxel_layer_->initialize(
        layered_costmap_.get(),
        "voxel_layer",
        tf_buffer_.get(),
        lifecycle_node_,
        callback_group);

      RCLCPP_INFO(this->get_logger(), "SpatioTemporalVoxelLayer initialized successfully!");

      // Add the layer to the layered costmap
      layered_costmap_->addPlugin(voxel_layer_);

    } catch (const pluginlib::PluginlibException& ex) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load plugin: %s", ex.what());
      throw;
    } catch (const std::exception& ex) {
      RCLCPP_ERROR(this->get_logger(), "Exception during initialization: %s", ex.what());
      throw;
    }

    // Create a publisher for the costmap
    costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "voxel_costmap", rclcpp::QoS(1).transient_local());

    // Create a timer to update the costmap
    auto update_period = std::chrono::duration<double>(1.0 / update_frequency);
    update_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(update_period),
      std::bind(&VoxelLayerRunner::updateCostmap, this));

    RCLCPP_INFO(this->get_logger(), "VoxelLayerRunner node initialized successfully!");
  }

  ~VoxelLayerRunner()
  {
    voxel_layer_.reset();
    plugin_loader_.reset();
  }

private:
  void updateCostmap()
  {
    // Get robot pose
    geometry_msgs::msg::PoseStamped robot_pose;
    std::string robot_base_frame = this->get_parameter("robot_base_frame").as_string();
    std::string global_frame = this->get_parameter("global_frame").as_string();

    double robot_x = 0.0;
    double robot_y = 0.0;
    double robot_yaw = 0.0;

    try {
      // Try to get robot pose from TF
      geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
        global_frame, robot_base_frame, tf2::TimePointZero);
      
      robot_x = transform.transform.translation.x;
      robot_y = transform.transform.translation.y;
      
      // Extract yaw from quaternion
      robot_yaw = tf2::getYaw(transform.transform.rotation);

    } catch (const tf2::TransformException& ex) {
      // If we can't get the transform, use origin position
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Could not get robot pose, using origin: %s", ex.what());
      robot_x = 0.0;
      robot_y = 0.0;
      robot_yaw = 0.0;
    }

    // Update bounds
    double update_bounds_padding = this->get_parameter("update_bounds_padding").as_double();
    double min_x = robot_x - update_bounds_padding;
    double min_y = robot_y - update_bounds_padding;
    double max_x = robot_x + update_bounds_padding;
    double max_y = robot_y + update_bounds_padding;

    // Update the voxel layer
    voxel_layer_->updateBounds(
      robot_x, robot_y, robot_yaw,
      &min_x, &min_y, &max_x, &max_y);

    // Get the costmap and update it
    auto costmap = layered_costmap_->getCostmap();
    unsigned int min_i, min_j, max_i, max_j;
    costmap->worldToMapEnforceBounds(min_x, min_y, min_i, min_j);
    costmap->worldToMapEnforceBounds(max_x, max_y, max_i, max_j);

    voxel_layer_->updateCosts(*costmap, min_i, min_j, max_i, max_j);

    // Publish the costmap as OccupancyGrid
    publishCostmap();
  }

  void publishCostmap()
  {
    auto costmap = layered_costmap_->getCostmap();
    auto msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
    
    msg->header.stamp = this->now();
    msg->header.frame_id = this->get_parameter("global_frame").as_string();
    
    msg->info.resolution = costmap->getResolution();
    msg->info.width = costmap->getSizeInCellsX();
    msg->info.height = costmap->getSizeInCellsY();
    msg->info.origin.position.x = costmap->getOriginX();
    msg->info.origin.position.y = costmap->getOriginY();
    msg->info.origin.position.z = 0.0;
    msg->info.origin.orientation.w = 1.0;

    msg->data.resize(msg->info.width * msg->info.height);
    
    unsigned char* data = costmap->getCharMap();
    for (unsigned int i = 0; i < msg->data.size(); ++i) {
      msg->data[i] = static_cast<int8_t>(data[i]);
    }

    costmap_pub_->publish(std::move(msg));
  }

  std::shared_ptr<nav2_costmap_2d::Layer> voxel_layer_;
  std::unique_ptr<pluginlib::ClassLoader<nav2_costmap_2d::Layer>> plugin_loader_;
  std::shared_ptr<nav2_costmap_2d::LayeredCostmap> layered_costmap_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> lifecycle_node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  rclcpp::TimerBase::SharedPtr update_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VoxelLayerRunner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
