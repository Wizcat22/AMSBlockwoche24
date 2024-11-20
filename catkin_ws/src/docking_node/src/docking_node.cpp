#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <chrono>
#include <thread>
#include <ros/console.h>

// current distant and angle to barrel
static float current_dist = 1000.f;
static float current_angle;

static bool got_first_scan = false;

static float approx_start_dist_barrel = 1000.f;
int current_init_iter;

#define NUM_INIT_LOOPS 15
#define ROT_SPEED 0.15f
#define TRANSL_SPEED 0.1f
#define DIST_TO_BARREL_M 0.19
#define CONTROL_RATE 100
#define DEG_TO_RAD(x) (x/180.0 * M_PI)
#define TOLERANCE_TO_BARREL 0.6
#define ANGLE_TOLERANCE 6

void move_to_barrel(ros::Publisher& pub){
    geometry_msgs::Twist msg;
    ros::Rate ctrl_rate(CONTROL_RATE);
    msg.linear.x = -TRANSL_SPEED;
    pub.publish(msg);
    ctrl_rate.sleep();
    ros::spinOnce();
}

void correct_attitude(ros::Publisher& pub){
    geometry_msgs::Twist msg;
    ros::Rate ctrl_rate(CONTROL_RATE);
        msg.linear.x = 0.f;
        msg.linear.y = 0.f;
        if(current_angle < 0.0){
            msg.angular.z = ROT_SPEED;
        } else {
            msg.angular.z = -ROT_SPEED;
        }
        pub.publish(msg);
        ctrl_rate.sleep();
        ros::spinOnce();
}

void raw_laser_callback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    if(current_init_iter < NUM_INIT_LOOPS){
        float min_of_all_scans = scan_in->ranges[0];
        for (size_t i = 0; i < scan_in->ranges.size(); ++i) {
            float range = scan_in->ranges[i];
            if (range < min_of_all_scans) {
                min_of_all_scans = range;
            }
        }
        if(min_of_all_scans < approx_start_dist_barrel){
            approx_start_dist_barrel = min_of_all_scans;
        }
        current_init_iter++;
        return;
    }

    ROS_INFO("Got new raw_scan");
    float min_of_all_scans = scan_in->ranges[0];
    for (size_t i = 0; i < scan_in->ranges.size(); ++i) {
        float range = scan_in->ranges[i];
        if (range < min_of_all_scans) {
            min_of_all_scans = range;
            current_angle = scan_in->angle_min + i * scan_in->angle_increment;
        }
    }

    if(min_of_all_scans < approx_start_dist_barrel + TOLERANCE_TO_BARREL){
        current_dist = min_of_all_scans;
        got_first_scan = true;
    }
    ROS_INFO("Current distance: %f", current_dist);
    ROS_INFO("Current angle: %f", current_angle);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "docking_node");
    ROS_INFO("Staring Docking_Node");
    
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/raw_scan", 1000, raw_laser_callback);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    ros::Rate loop_rate(20);
    while(!got_first_scan){
        loop_rate.sleep();
        ros::spinOnce();
    }
    ROS_INFO("Starting to rotate..");
    ros::Rate ctrl_rate(CONTROL_RATE);
    while(std::fabs(current_dist) > DIST_TO_BARREL_M || std::fabs(std::fabs(current_angle) - M_PI) > DEG_TO_RAD(ANGLE_TOLERANCE)){
        while(std::fabs(std::fabs(current_angle) - M_PI) > DEG_TO_RAD(ANGLE_TOLERANCE) && ros::ok()){
            // if(current_angle > M_PI){
            //     msg.linear.x = 0.f;
            //     msg.linear.y = 0.f;
            //     msg.angular.z = ROT_SPEED;
            // } else {
            //     msg.angular.z = -ROT_SPEED;
            // }
            // pub.publish(msg);
            // ctrl_rate.sleep();
            // ros::spinOnce();
            correct_attitude(pub);
        }
        ROS_INFO("Startint to move towards barrel");
        // std::fabs(current_dist) > DIST_TO_BARREL_M && std::fabs(std::fabs(current_angle) - M_PI) > DEG_TO_RAD(ANGLE_TOLERANCE) &&ros::ok()
        while(std::fabs(current_dist) > DIST_TO_BARREL_M && ros::ok()){
            move_to_barrel(pub);
            if(std::fabs(std::fabs(current_angle) - M_PI) > DEG_TO_RAD(ANGLE_TOLERANCE)){
                break;
            }
            // msg.linear.x = -TRANSL_SPEED;
            // pub.publish(msg);
            // ctrl_rate.sleep();
            // ros::spinOnce();
        }
    }
    geometry_msgs::Twist msg;
    msg.linear.x = 0.f;
    msg.linear.y = 0.f;
    msg.angular.z = 0.f;
    pub.publish(msg);
    ROS_INFO("Done!");

    return 0;
}
