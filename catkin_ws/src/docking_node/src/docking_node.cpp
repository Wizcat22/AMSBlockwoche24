#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <chrono>
#include <thread>
#include <ros/console.h>
#include "std_msgs/Bool.h"


static float current_dist = 1000.f;                 // aktuelle Distanz zum Barrel
static float current_angle;                         // akueller Winkel zum Barrel

static bool got_first_scan = false;                 // Flag, dass ein erster Scan erkannt wurde
static bool start_docking = false;                  // Start-Flag
static float approx_start_dist_barrel = 1000.f;     // wird benötigt, um Scans die nicht das Barrel enthalten zu ignorieren
int current_init_iter;                              // aktuelle Iteration beim Schätzen der Entfernung zum Barrel


#define NUM_INIT_LOOPS 15                           // Anzahl der Scan Nachrichten die abgewartet werden, um die approx Distanz zum Barrel zu ermitteln
#define ROT_SPEED 0.3f                              // Rotationsgeschwindigkeit
#define TRANSL_SPEED 0.12f                          // translatorische Geschwindigkeit
#define DIST_TO_BARREL_M 0.17                       // Zieldistanz zur Tonne
#define CONTROL_RATE 100                            // sleep time twischen Controller-Loops
#define DEG_TO_RAD(x) (x/180.0 * M_PI)              // grad zu radiant
#define TOLERANCE_TO_BARREL 0.06                    // Toleranz-Wert zum geschätzen Abstand vom Barrel, damit ein Scan noch als relevant eingestuft wird, nicht die Anfahr-Toleranz
#define ANGLE_TOLERANCE 2                           // Toleranz-Winkel beim Ausrichten

void move_to_barrel(ros::Publisher& pub){
    geometry_msgs::Twist msg;
    ros::Rate ctrl_rate(CONTROL_RATE);
    msg.angular.z = 0.f;
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

void starter_callback(const std_msgs::Bool::ConstPtr& start_flag){
    ROS_INFO("Received Starting-Flag.");
    start_docking = start_flag->data;
}

void raw_laser_callback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{   
    // zunächst werden NUM_INIT_LOOPS Scans abgewartet, um den Abstand zum Barrel zu schätzen 
    // und irrelevante Scans ohne das Barrel anhand dessen ausschließen zu können
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

    ROS_DEBUG("Got new raw_scan");

    // der minimale Abstand wird aus allen scans gesucht
    float min_of_all_scans = scan_in->ranges[0];
    for (size_t i = 0; i < scan_in->ranges.size(); ++i) {
        float range = scan_in->ranges[i];
        if (range < min_of_all_scans) {
            min_of_all_scans = range;
            current_angle = scan_in->angle_min + i * scan_in->angle_increment;
        }
    }

    // der gefundene Werte muss abzüglich Toleranz kleiner sein als der geschätze Abstand
    // zum Barrel. Ansonsten war das Barrel nicht im Blickfeld.
    if(min_of_all_scans < approx_start_dist_barrel + TOLERANCE_TO_BARREL){
        current_dist = min_of_all_scans;
        got_first_scan = true;
    }

    ROS_DEBUG("Current distance: %f", current_dist);
    ROS_DEBUG("Current angle: %f", current_angle);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "docking_node");
    ros::NodeHandle nh;

    ros::Subscriber sub_scan = nh.subscribe("/raw_scan", 1000, raw_laser_callback);
    ros::Subscriber sub_action = nh.subscribe("/docking_node/start", 5, starter_callback);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    
    ROS_INFO("Wating for Starter-Flag");
    ros::Rate loop_wait_flag(20);
    while(!start_docking){
        loop_wait_flag.sleep();
        ros::spinOnce();
    }
    ROS_INFO("Staring Docking_Node");

    ros::Rate loop_rate(20);
    while(!got_first_scan){
        loop_rate.sleep();
        ros::spinOnce();
    }

    ros::Rate ctrl_rate(CONTROL_RATE);
    while((std::fabs(current_dist) > DIST_TO_BARREL_M || std::fabs(std::fabs(current_angle) - M_PI) > DEG_TO_RAD(ANGLE_TOLERANCE)) && ros::ok()){
        // zum Barrel Drehen
        while(std::fabs(std::fabs(current_angle) - M_PI) > DEG_TO_RAD(ANGLE_TOLERANCE) && ros::ok()){
            correct_attitude(pub);
        }

        // auf Barrel zufahren
        while(std::fabs(current_dist) > DIST_TO_BARREL_M && ros::ok()){
            move_to_barrel(pub);
            if(std::fabs(std::fabs(current_angle) - M_PI) > DEG_TO_RAD(ANGLE_TOLERANCE)){
                break;
            }
        }
    }

    //Stehenbleiben, sobald ausgerichtet
    geometry_msgs::Twist msg;
    msg.linear.x = 0.f;
    msg.linear.y = 0.f;
    msg.angular.z = 0.f;
    
    ros::Rate stop_rate(100);
    
    // einmaliges Senden hat nicht immer gereicht
    for(int i = 0; i < 5; i++){
        pub.publish(msg);
        stop_rate.sleep();
        ros::spinOnce();
    }
    
    ROS_INFO("Done!");

    // \todo Publish Gripper Message
    // \todo Await Force Feedback
    // \todo Send Home Goal To Move_Base Action Server

    return 0;
}
