#include "obstacle_detector/map_listener.h"

using namespace obstacle_detector;
using namespace std;

MapListener::MapListener(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : nh_(nh), nh_local_(nh_local){
    frontier_sub_ = nh.subscribe("frontiers", 10, &MapListener::frontiersCallback, this);
}

void MapListener::frontiersCallback(const visualization_msgs::MarkerArray::ConstPtr frontiers){
    

    //if Message Array länger Leer -> Funktionaufruf
    if(false == true) {
        this.explorationFinished(&res);
    }
}

void MapListener::explorationFinished(std_srvs::Empty::Response &res){
    //Publish Position des Obstacles
}