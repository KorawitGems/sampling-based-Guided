/*
Copyright (C) 2022 Hongkai Ye (kyle_yeh@163.com)
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/
#include "occ_grid/occ_map.h"
#include "occ_grid/raycast.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>
#include <random>

namespace env
{
  inline void OccMap::setOccupancy(const Eigen::Vector3d &pos)
  {
    Eigen::Vector3i id;
    posToIndex(pos, id);
    // cout << "id: " << id.transpose() << ", idx: " << idxToAddress(id) << ", is in map? " << isInMap(id) << endl;
    if (!isInMap(id))
      return;

    occupancy_buffer_[idxToAddress(id)] = true;
    //inflated_occupancy_buffer_(id(0),id(1),id(2)) = true;
  }

  void OccMap::globalOccVisCallback(const ros::TimerEvent &e)
  {
    //modified
    if (!(param_publish_occupancy_cloud_))
    {
      return;
    }
    //apply occupancy grid to output cloud
    glb_cloud_ptr_->points.clear();
    for (int x = 0; x < grid_size_[0]; ++x)
    {
      for (int y = 0; y < grid_size_[1]; ++y)
      {
        for (int z = 0; z < grid_size_[2]; ++z)
        {
          if (occupancy_buffer_[idxToAddress(x, y, z)] == true)
          //if (inflated_occupancy_buffer_(x,y,z) == true)
          {
            Eigen::Vector3d pos;
            indexToPos(x, y, z, pos);
            glb_cloud_ptr_->points.emplace_back(pos[0], pos[1], pos[2]);
          }
        }
      }
    }
    glb_cloud_ptr_->is_dense = true;
    glb_cloud_ptr_->header = global_cloud_ptr_->header;
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*glb_cloud_ptr_, cloud_msg);
    cloud_msg.header.stamp = ros::Time::now();
    glb_occ_pub_.publish(cloud_msg);
  }

  void OccMap::globalCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
  {

    //modified
    pcl::fromROSMsg(*msg, *global_cloud_ptr_);
    // ROS_ERROR_STREAM(", global_cloud_ptr_->points.size(): " << global_cloud_ptr_->points.size());

    if (global_cloud_ptr_->points.empty())
    {
      return;
    }

    // Get the current position
    // Eigen::Vector3d current_position_vector = Eigen::Vector3d(current_pose_.pose.position.x, current_pose_.pose.position.y, current_pose_.pose.position.z);
    // // Reset occupancy buffer within the specified range around the current position
    // Eigen::Vector3d reset_min_range = current_position_vector - Eigen::Vector3d(range_x_ + range_offset_, range_y_ + range_offset_, range_z_ + range_offset_);
    // Eigen::Vector3d reset_max_range = current_position_vector + Eigen::Vector3d(range_x_ + range_offset_, range_y_ + range_offset_, range_z_ + range_offset_);
    // Eigen::Vector3i index_min_range;
    // posToIndex(reset_min_range, index_min_range);
    // Eigen::Vector3i index_max_range;
    // posToIndex(reset_max_range, index_max_range);
    // // clear occupancy grid in range with false before apply obstacle point cloud to grid
    // for (int x = floor(index_max_range(0)/2); x <= index_max_range(0); x += 1) // use only +x/2 to +x
    // {
    //   for (int y = index_min_range(1); y <= index_max_range(0); y += 1) 
    //   {
    //     for (int z = index_min_range(2); z <= index_max_range(0); z += 1) 
    //     {
    //       occupancy_buffer_[idxToAddress(x, y, z)] = false;
    //       //inflated_occupancy_buffer_(clear_index(0),clear_index(1),clear_index(2)) = false;
    //     }
    //   }
    // }

    //apply obstacle point cloud to occupancy grid buffer
    for (const auto& point : global_cloud_ptr_->points)
    {
      double distance = sqrt(pow(current_pose_.pose.position.x - point.x, 2) +
                              pow(current_pose_.pose.position.y - point.y, 2) +
                              pow(current_pose_.pose.position.z - point.z, 2));
      if (distance > param_inflated_boundary_)
      {
          continue;
      }
      // Inflate the point along the unit vector toward current pose
      for (int k = 0; k <= inflated_radius_index_; k += inflated_radius_step_index_)
      {
          for (int j = 0; j <= inflated_radius_index_; j += inflated_radius_step_index_)
          {
              for (int i = 0; i <= inflated_radius_index_; i += inflated_radius_step_index_)
              {
                  pcl::PointXYZ inflated_point;
                  inflated_point.x = point.x + (current_pose_.pose.position.x - point.x) / distance * i * resolution_;
                  inflated_point.y = point.y + (current_pose_.pose.position.y - point.y) / distance * j * resolution_;
                  inflated_point.z = point.z + (current_pose_.pose.position.z - point.z) / distance * k * resolution_;
                  Eigen::Vector3d p3d(inflated_point.x, inflated_point.y, inflated_point.z);
                  this->setOccupancy(p3d);
              }
          }
      }
    }
    is_global_map_valid_ = true;

  }

  void OccMap::currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
    {
      current_pose_ = *pose_msg;
      // origin_(0) = current_pose_.pose.position.x;
      // origin_(1) = current_pose_.pose.position.y;
      // origin_(2) = current_pose_.pose.position.z;
    }

  void OccMap::init(const ros::NodeHandle &pnh)
  {
    pnh_ = pnh;
    /* ---------- param ---------- */
    // modified
    pnh_.param("collision_check/collision_radius", param_collision_radius_, 0.8);
    pnh_.param("collision_check/radius_step", param_collision_radius_step_, 0.4);
    pnh_.param("occupancy_grid/dynamics/range_x", range_x_, 5.0);
    pnh_.param("occupancy_grid/dynamics/range_y", range_y_, 5.0);
    pnh_.param("occupancy_grid/dynamics/range_z", range_z_, 5.0);
    pnh_.param("occupancy_grid/dynamics/range_offset", range_offset_, -0.05);
    pnh_.param("occupancy_grid/cloud/publish", param_publish_occupancy_cloud_, true);
    pnh_.param("inflation/radius", param_inflated_radius_, 0.6);
    pnh_.param("inflation/radius_step", param_inflated_radius_step_, 0.2);
    pnh_.param("inflation/boundary", param_inflated_boundary_, 5.0);

    pnh_.param("occ_map/origin_x", origin_(0), -20.0);
    pnh_.param("occ_map/origin_y", origin_(1), -20.0);
    pnh_.param("occ_map/origin_z", origin_(2), 0.0);
    pnh_.param("occ_map/map_size_x", map_size_(0), 40.0);
    pnh_.param("occ_map/map_size_y", map_size_(1), 40.0);
    pnh_.param("occ_map/map_size_z", map_size_(2), 5.0);
    pnh_.param("occ_map/resolution", resolution_, 0.2);
    resolution_inv_ = 1 / resolution_;
    is_global_map_valid_ = false;
    //modified
    collision_radius_index_ = floor(param_collision_radius_ / resolution_ + 0.5);
    collision_radius_step_index_ = floor(param_collision_radius_step_ / resolution_ + 0.5);

    inflated_radius_index_ = floor(param_inflated_radius_ / resolution_ + 0.5);
    inflated_radius_step_index_ = floor(param_inflated_radius_step_ / resolution_ + 0.5);

    for (int i = 0; i < 3; ++i)
    {
      grid_size_(i) = ceil(map_size_(i) * resolution_inv_);
    }

    min_range_ = origin_;
    max_range_ = origin_ + map_size_ ;

    // initialize size of buffer
    grid_size_y_multiply_z_ = grid_size_(1) * grid_size_(2);
    int buffer_size = grid_size_(0) * grid_size_y_multiply_z_;
    occupancy_buffer_.resize(buffer_size);
    fill(occupancy_buffer_.begin(), occupancy_buffer_.end(), false);
    //inflated_occupancy_buffer_.resize(grid_size_(0), grid_size_(1), grid_size_(2));
    //inflated_occupancy_buffer_.setConstant(false); // set all values to false

    //set x-y boundary occ
    for (double cx = min_range_[0] + resolution_ / 2; cx <= max_range_[0] - resolution_ / 2; cx += resolution_)
      for (double cz = min_range_[2] + resolution_ / 2; cz <= max_range_[2] - resolution_ / 2; cz += resolution_)
      {
        this->setOccupancy(Eigen::Vector3d(cx, min_range_[1] + resolution_ / 2, cz));
        this->setOccupancy(Eigen::Vector3d(cx, max_range_[1] - resolution_ / 2, cz));
      }
    for (double cy = min_range_[1] + resolution_ / 2; cy <= max_range_[1] - resolution_ / 2; cy += resolution_)
      for (double cz = min_range_[2] + resolution_ / 2; cz <= max_range_[2] - resolution_ / 2; cz += resolution_)
      {
        this->setOccupancy(Eigen::Vector3d(min_range_[0] + resolution_ / 2, cy, cz));
        this->setOccupancy(Eigen::Vector3d(max_range_[0] - resolution_ / 2, cy, cz));
      }
    //set z-low boundary occ
    for (double cx = min_range_[0] + resolution_ / 2; cx <= max_range_[0] - resolution_ / 2; cx += resolution_)
      for (double cy = min_range_[1] + resolution_ / 2; cy <= max_range_[1] - resolution_ / 2; cy += resolution_)
      {
        this->setOccupancy(Eigen::Vector3d(cx, cy, min_range_[2] + resolution_ / 2));
      }
    //modified
    //boundary_occupancy_buffer_ = occupancy_buffer_;
    //inflated_boundary_occupancy_buffer_ = inflated_occupancy_buffer_;

    global_occ_vis_timer_ = pnh_.createTimer(ros::Duration(1.0), &OccMap::globalOccVisCallback, this);
    global_cloud_sub_ = pnh_.subscribe<sensor_msgs::PointCloud2>("/global_cloud", 100, &OccMap::globalCloudCallback, this);
    glb_occ_pub_ = pnh_.advertise<sensor_msgs::PointCloud2>("/occ_map/glb_map", 10);
    //modified
    current_pose_sub_ = pnh_.subscribe("/current_pose", 10, &OccMap::currentPoseCallback, this);
    pnh_.subscribe("/current_pose", 10, &OccMap::currentPoseCallback, this);

    glb_cloud_ptr_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    global_cloud_ptr_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cout << "map initialized: " << endl;
  }

} // namespace env
