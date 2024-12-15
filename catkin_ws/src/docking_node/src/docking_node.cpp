#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <chrono>
#include <thread>
#include <ros/console.h>
#include "std_msgs/Bool.h"
#include <std_msgs/Float32.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>

static float current_dist = 1000.f;                 // current distant and angle to barrel
static float current_angle;

static bool got_first_scan = false;                 // Verhindert das Starten des Andockens, wenn noch keine Scans eingetroffen sind
static bool start_docking = false;                  // Flag, welches das Andocken einleitet
static float approx_start_dist_barrel = 1000.f;     // wird benötigt, um Scans die nicht das Barrel enthalten, zu ignorieren
int current_init_iter;                              // Zähler für die initiale Schätzung des Abstandes zum Ziel

#define NUM_INIT_LOOPS 15                           // Anzahl der Scan Nachrichten die abgewartet werden, um die approx. Distanz zum Barrel zu ermitteln
#define ROT_SPEED 0.2f                              // Rotationsgeschwindigkeit beim Drehen zum Ziel
#define TRANSL_SPEED 0.09f                          // translatorische Geschwindigkeit zum Ziel
#define DIST_TO_BARREL_M 0.17                       // Zieldistanz zum Barrel
#define CONTROL_RATE 100                            // sleep time twischen Controller-Loops
#define DEG_TO_RAD(x) (x/180.0 * M_PI)              // Umwandlung von Grad zu Radiant
#define TOLERANCE_TO_BARREL 0.6                     // Toleranzwert, um Scans die das Zielobjekt nicht enthalten, zu filtern
#define ANGLE_TOLERANCE 2                           // Winkeltoleranz beim Ausrichten zum Zielobjekt
#define RESENDS 3                                   // gibt an, wie oft eine Nachricht geschickt werden soll, einmaliges Senden hat oftmals nicht ausgereicht

#define GRIP_OPEN 350                               // Greifer in offener Position
#define GRIP_CLOSE 270                              // Greifer in geschlossener Position, niedriger als 240 kann Brownout zur Folge haben -> besser Forcefeedback (ToDo)

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/**
 * @brief Setzt die translatorische Geschwindigkeit mittels Twist-Messages für die Dauer von CONTROL_RATE.
 * @param pub Publisher für ROS-Nachrichten.
 */
void move_to_barrel(ros::Publisher& pub){
    geometry_msgs::Twist msg;
    ros::Rate ctrl_rate(CONTROL_RATE);
    msg.linear.x = -TRANSL_SPEED;
    pub.publish(msg);
    ctrl_rate.sleep();
    ros::spinOnce();
}

/**
 * @brief Setzt die Drehgeschwindigkeit mittels Twist-Messages für die Dauer von CONTROL_RATE.
 * @param pub Publisher für ROS-Nachrichten.
 */
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

/**
 * @brief Starter-Callback, dass die Andock-Prozedur begonnen werden kann.
 * @param start_flag 'true' leitet das Andocken ein.
 */
void starter_callback(const std_msgs::Bool::ConstPtr& start_flag){
    ROS_INFO("Received Starting-Flag.");
    start_docking = start_flag->data;
}

/**
 * @brief Callback, welches die geschätze Pose des Zielobjekts vom Detection-Node entgegennimmt
 * und anschließend einen Zielpunkt in 30cm Entfernung des Barrels berechnet. Die Entfernung
 * von 30cm liegen in Richtung des Startpunktes des EduRobs.
 * 
 * @param barrel_pose Geschätze Zielpose des Barrel vom Detection-Node.
 * 
 * \todo unvollständige Implementierung. Besser wäre den Pfad vom 'make_plan'-Service anzufragen
 * und 30cm vorher zu Stoppen.
 */
void barrel_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& barrel_pose){
    static tf::TransformListener tf2_listener;
    tf::StampedTransform transform;
    tf2_listener.lookupTransform("map", "base_link", ros::Time(0), transform);
    tf::Vector3 robot_coord = transform.getOrigin();
    tf::Vector3 barrel_vec = tf::Vector3(barrel_pose->pose.position.x, barrel_pose->pose.position.y, barrel_pose->pose.position.z);
    tf::Vector3 dir_barrel_to_robot = robot_coord - barrel_vec;
    dir_barrel_to_robot.normalize();
    barrel_vec = barrel_vec + 0.3 * dir_barrel_to_robot;


    MoveBaseClient ac("move_base", true);
    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = barrel_vec.x();
    goal.target_pose.pose.position.y = barrel_vec.y();
    goal.target_pose.pose.orientation.w = 1.0;
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);
    ac.waitForResult();

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    start_docking = true;
}

/**
 * \todo Beim Greifen Forcefeedback beachten, um zu hohen Strom im Gripper zu verhindern.
 */
void feedback_callback(const std_msgs::Float32::ConstPtr& feedback){}


/**
 * @brief Callback für ungefilterte LaserScans. Berechnet zunächst eine geschätzte Entfernung zum
 * Zielobjekt und bewertet anschließend jede weitere Nachricht anhand dessen, ob Sie das Zielobjekt
 * beinhaltet oder nicht. Danach wird für relevante Nachrichten das Minimum aus den gemessenen Abständen
 * ermittelt und dieses in 'current_dist' gespeichert. 
 * 
 * @param scan_in ungefilterter LaserScan vom RP_Lidar Node.
 */
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
    ros::NodeHandle nh;

    ros::Subscriber sub_scan = nh.subscribe("/raw_scan", 1000, raw_laser_callback);
    ros::Subscriber sub_action = nh.subscribe("/docking_node/start", 5, starter_callback);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    ros::Publisher pub_gripper = nh.advertise<std_msgs::Float32>("grabber/angle", 10);
    ros::Subscriber sub_force_feedback = nh.subscribe("grabber/feedback", 5, feedback_callback);
    ros::Subscriber sub_near_barrel = nh.subscribe("/docking_node/goal_position", 1000, barrel_pose_callback);

    ROS_INFO("Wating for Starter-Flag");
    ros::Rate loop_wait_flag(20);
    while(!start_docking){
        loop_wait_flag.sleep();
        ros::spinOnce();
    }
    ROS_INFO("Staring Docking_Node");
    ros::Rate grab_rate(200);

    std_msgs::Float32 open_msg;
    open_msg.data = GRIP_OPEN;

    for(int i = 0; i < 2; i++){
        pub_gripper.publish(open_msg);
        grab_rate.sleep();
        ros::spinOnce();
    }

    ros::Rate loop_rate(20);
    while(!got_first_scan){
        loop_rate.sleep();
        ros::spinOnce();
    }
    ROS_INFO("Starting to rotate..");
    ros::Rate ctrl_rate(CONTROL_RATE);
    while((std::fabs(current_dist) > DIST_TO_BARREL_M || std::fabs(std::fabs(current_angle) - M_PI) > DEG_TO_RAD(ANGLE_TOLERANCE)) && ros::ok()){
        while(std::fabs(std::fabs(current_angle) - M_PI) > DEG_TO_RAD(ANGLE_TOLERANCE) && ros::ok()){
            correct_attitude(pub);
        }
        ROS_INFO("Startint to move towards barrel");
        while(std::fabs(current_dist) > DIST_TO_BARREL_M && ros::ok()){
            move_to_barrel(pub);
            if(std::fabs(std::fabs(current_angle) - M_PI) > DEG_TO_RAD(ANGLE_TOLERANCE)){
                break;
            }
        }
    }
    geometry_msgs::Twist msg;
    msg.linear.x = 0.f;
    msg.linear.y = 0.f;
    msg.angular.z = 0.f;
    
    ros::Rate stop_rate(100);
    for(int i = 0; i < 5; i++){
        pub.publish(msg);
        stop_rate.sleep();
        ros::spinOnce();
    }
    
    ROS_INFO("Docking complete!");

    ros::Publisher pub_complete = nh.advertise<std_msgs::Bool>("/docking_node/docking_complete", 5);
    std_msgs::Bool value;
    value.data = true;
    
    // close Gripper
    std_msgs::Float32 grapper_msg;
    grapper_msg.data = GRIP_CLOSE;

    for(int i = 0; i < RESENDS; i++){
        pub_gripper.publish(grapper_msg);
        grab_rate.sleep();
        ros::spinOnce();
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(4000));

    // Zurück zum Ursprung der Map fahren
    MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = 0.0;
    goal.target_pose.pose.position.y = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);
    ac.waitForResult();
    
    // Barrel loslassen
    std_msgs::Float32 drop_msg;
    drop_msg.data = GRIP_OPEN;

    for(int i = 0; i < RESENDS; i++){
        pub_gripper.publish(drop_msg);
        grab_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
