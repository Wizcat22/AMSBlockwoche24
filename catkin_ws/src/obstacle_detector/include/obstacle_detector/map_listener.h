
#pragma once

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

namespace obstacle_detector
{
    class MapListener{
        public:
        MapListener(ros::NodeHandle& nh, ros::NodeHandle& nh_local);
        // ~MapListener();

        void frontiersCallback();
        void explorationFinished(std_srvs::Empty::Response &res);

        private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_local_;

        ros::Subscriber frontier_sub_;
        ros::Publisher nav_goal_pub_;
    };
}