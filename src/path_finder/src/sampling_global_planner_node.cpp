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
#include "self_msgs_and_srvs/GlbObsRcv.h"
#include "occ_grid/occ_map.h"
#include "path_finder/rrt_sharp.h"
#include "path_finder/rrt_star.h"
#include "path_finder/rrt.h"
#include "path_finder/brrt.h"
#include "path_finder/brrt_star.h"
#include "visualization/visualization.hpp"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <chrono>
#include <cmath>

class GlobalPlanner
{
private:
    ros::NodeHandle pnh_;
    ros::Subscriber goal_sub_;
    ros::Timer execution_timer_;
    ros::Rate rate_;

    // modified
    ros::Timer goal_distance_timer_;
    ros::Time last_time_set_param_;
    ros::Duration time_up_plan_;
    double param_collision_radius_, param_inflated_plan_;
    Eigen::Vector3d start_, goal_, current_vector_;

    env::OccMap::Ptr env_ptr_;
    std::shared_ptr<visualization::Visualization> vis_ptr_;
    shared_ptr<path_plan::BRRT> brrt_ptr_;
    //shared_ptr<path_plan::BRRTStar> brrt_star_ptr_;

    bool run_brrt_star_;
    bool param_time_up_;

public:
    GlobalPlanner(const ros::NodeHandle &pnh) : pnh_(pnh), rate_(10.0)
    {
        env_ptr_ = std::make_shared<env::OccMap>();
        env_ptr_->init(pnh_);

        vis_ptr_ = std::make_shared<visualization::Visualization>(pnh_);
        vis_ptr_->registe<visualization_msgs::Marker>("start");
        vis_ptr_->registe<visualization_msgs::Marker>("goal");

        brrt_ptr_ = std::make_shared<path_plan::BRRT>(pnh_, env_ptr_);
        brrt_ptr_->setVisualizer(vis_ptr_);
        vis_ptr_->registe<nav_msgs::Path>("brrt_final_path");
        vis_ptr_->registe<sensor_msgs::PointCloud2>("brrt_final_wpts");

        // brrt_star_ptr_ = std::make_shared<path_plan::BRRTStar>(pnh_, env_ptr_);
        // brrt_star_ptr_->setVisualizer(vis_ptr_);
        // vis_ptr_->registe<nav_msgs::Path>("brrt_star_final_path");
        // vis_ptr_->registe<sensor_msgs::PointCloud2>("brrt_star_final_wpts");

        goal_sub_ = pnh_.subscribe("/command/goal", 50, &GlobalPlanner::goalCallback, this);
        execution_timer_ = pnh_.createTimer(ros::Duration(1), &GlobalPlanner::executionCallback, this);
        
        last_time_set_param_ = ros::Time::now();
        time_up_plan_ = ros::Duration(3.0);
        start_.setZero();

        pnh_.param("collision_check/collision_radius", param_collision_radius_, 2.0);
        pnh_.param("plan/time_up", param_time_up_, false);
        ROS_WARN("[sampling global planner] Param: collision_check/collision_radius: %f", param_collision_radius_);
        ROS_WARN("[global occ map] Param: collision_check/collision_radius: %f", env_ptr_->getCollisionRadius());
        
    }
    ~GlobalPlanner(){};

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &goal_msg)
    {
        start_[0] = env_ptr_->getCurrentPose().pose.position.x;
        start_[1] = env_ptr_->getCurrentPose().pose.position.y;
        start_[2] = env_ptr_->getCurrentPose().pose.position.z;
        goal_[0] = goal_msg->pose.position.x;
        goal_[1] = goal_msg->pose.position.y;
        goal_[2] = goal_msg->pose.position.z;
        ROS_INFO_STREAM("\n-----------------------------\ngoal rcved at " << goal_.transpose());
        vis_ptr_->visualize_a_ball(start_, 0.3, "start", visualization::Color::pink);
        vis_ptr_->visualize_a_ball(goal_, 0.3, "goal", visualization::Color::steelblue);

        bool brrt_res = false;
        ros::Time last_time_plan = ros::Time::now();
        while (!brrt_res)
        {
            if ( (ros::Time::now() - last_time_plan) > time_up_plan_ )
            {
                ROS_WARN("Time up planning");
                pnh_.setParam("plan/time_up", true);
                break;
            }
            auto start_time = std::chrono::steady_clock::now();
            brrt_res = brrt_ptr_->plan(start_, goal_);
            auto end_time = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            ROS_WARN("global planner [BRRT] Execution time: %ld milliseconds", duration.count());
            
            if (brrt_res)
            {
                vector<Eigen::Vector3d> final_path = brrt_ptr_->getPath();
                vis_ptr_->visualize_path(final_path, "brrt_final_path");
                vis_ptr_->visualize_pointcloud(final_path, "brrt_final_wpts");
                vector<std::pair<double, double>> slns = brrt_ptr_->getSolutions();
                ROS_INFO_STREAM("[BRRT] final path len: " << slns.back().first);
            }
            rate_.sleep();
        }

        // bool brrt_star_res = false;
        // ros::Time last_time_plan = ros::Time::now();
        // while (!brrt_star_res )
        // {
        //     if ( (ros::Time::now() - last_time_plan) > time_up_plan_ )
        //     {
        //         ROS_WARN("Time up planning");
        //         pnh_.setParam("plan/time_up", true);
        //         break;
        //     }

        //     auto start_time = std::chrono::steady_clock::now();
        //     brrt_star_res = brrt_star_ptr_->plan(start_, goal_);
        //     auto end_time = std::chrono::steady_clock::now();
        //     auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        //     ROS_WARN("global planner [BRRT*] Execution time: %ld milliseconds", duration.count());
            
        //     if (brrt_star_res)
        //     {
        //         vector<Eigen::Vector3d> final_path = brrt_star_ptr_->getPath();
        //         vis_ptr_->visualize_path(final_path, "brrt_star_final_path");
        //         vis_ptr_->visualize_pointcloud(final_path, "brrt_star_final_wpts");
        //         vector<std::pair<double, double>> slns = brrt_star_ptr_->getSolutions();
        //         ROS_INFO_STREAM("[BRRT*] final path len: " << slns.back().first);
        //     }
        //     rate_.sleep();
        // }
        
    }

    void executionCallback(const ros::TimerEvent &event)
    {
        if (!env_ptr_->mapValid()) // check receive map in occ_map
        {
            ROS_INFO("no map rcved yet.");
        }
        else
        {
            execution_timer_.stop();
        }
    };
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_planner");
    ros::NodeHandle pnh("~");
    ros::Duration(1).sleep();
    GlobalPlanner global_planner(pnh);

    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}