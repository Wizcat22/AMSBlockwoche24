/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Poznan University of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Poznan University of Technology nor the names
 *       of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Mateusz Przybyla
 */

#pragma once

#include <obstacle_detector/Obstacles.h>
#include "obstacle_detector/utilities/kalman.h"
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

namespace obstacle_detector
{

class TrackedObstacle {
public:
  ros::Time first_detected_time_;
  ros::Time last_seen_time_;
  bool is_persistent_;
  ros::Time seen_time = ros::Time().fromSec(0.0);
  double min_tracking_duration_ = 5.0d; // 2.0d
  bool is_visible;
  geometry_msgs::PointStamped local_coordinates;
  geometry_msgs::PointStamped map_coordinates;


  TrackedObstacle(const CircleObstacle& obstacle) : obstacle_(obstacle), kf_x_(0, 1, 2), kf_y_(0, 1, 2), kf_r_(0, 1, 2) {
    fade_counter_ = s_fade_counter_size_;
    first_detected_time_ = ros::Time::now();
    last_seen_time_ = first_detected_time_;
    is_persistent_ = false;
    is_visible = true;
    local_coordinates.header.frame_id = "base_link";
    local_coordinates.header.stamp = ros::Time::now();
    local_coordinates.point.x = 0.0;
    local_coordinates.point.y = 0.0;
    local_coordinates.point.z = 0.0;

    map_coordinates.header.frame_id = "map";
    map_coordinates.header.stamp = ros::Time::now();
    map_coordinates.point.x = 0.0;
    map_coordinates.point.y = 0.0;
    map_coordinates.point.z = 0.0;
    
    initKF();
  }

  void predictState() {
    kf_x_.predictState();
    kf_y_.predictState();
    kf_r_.predictState();

    obstacle_.center.x = kf_x_.q_pred(0);
    obstacle_.center.y = kf_y_.q_pred(0);

    obstacle_.velocity.x = kf_x_.q_pred(1);
    obstacle_.velocity.y = kf_y_.q_pred(1);

    obstacle_.radius = kf_r_.q_pred(0);

    fade_counter_--;
  }

  void correctState(const CircleObstacle& new_obstacle) {
    kf_x_.y(0) = new_obstacle.center.x;
    kf_y_.y(0) = new_obstacle.center.y;
    kf_r_.y(0) = new_obstacle.radius;

    kf_x_.correctState();
    kf_y_.correctState();
    kf_r_.correctState();

    obstacle_.center.x = kf_x_.q_est(0);
    obstacle_.center.y = kf_y_.q_est(0);

    obstacle_.velocity.x = kf_x_.q_est(1);
    obstacle_.velocity.y = kf_y_.q_est(1);

    obstacle_.radius = kf_r_.q_est(0);

    fade_counter_ = s_fade_counter_size_;
  }

  void updateState() {
    last_seen_time_ = ros::Time::now();

    kf_x_.predictState();
    kf_y_.predictState();
    kf_r_.predictState();

    kf_x_.correctState();
    kf_y_.correctState();
    kf_r_.correctState();

    obstacle_.center.x = kf_x_.q_est(0);
    obstacle_.center.y = kf_y_.q_est(0);

    // local Coordinates und Timestamp setzen 
    local_coordinates.header.stamp = ros::Time::now();
    local_coordinates.point.x = obstacle_.center.x;
    local_coordinates.point.y = obstacle_.center.y;
    map_coordinates.header.stamp = ros::Time::now();
    map_coordinates.point.x = obstacle_.center.x;
    map_coordinates.point.y = obstacle_.center.y;
    transformLocalToMap(local_coordinates, map_coordinates);
    // ROS_WARN("Objekt in Map transformed: x = %f, y = %f, z = %f", map_coordinates.point.x,map_coordinates.point.y, map_coordinates.point.z);

    obstacle_.velocity.x = kf_x_.q_est(1);
    obstacle_.velocity.y = kf_y_.q_est(1);

    obstacle_.radius = kf_r_.q_est(0);

    fade_counter_--;

    if (!this->is_visible) {
      is_visible = true;
      first_detected_time_ = ros::Time::now();
    }

    double tracking_time_ = (this->last_seen_time_ - this->first_detected_time_).toSec();
    if (tracking_time_ >= min_tracking_duration_) {
      is_persistent_ = true;
    }
  }

  void transformLocalToMap(const geometry_msgs::PointStamped& local_coords, geometry_msgs::PointStamped& map_coords) {
    static tf::TransformListener tf_listener;

    try {
      tf_listener.waitForTransform("map", "base_link", local_coordinates.header.stamp, ros::Duration(3.0));
      tf_listener.transformPoint("map", local_coords, map_coords);
      // ROS_WARN("Objekt in Map transformed: x = %f, y = %f, z = %f", map_coords.point.x,map_coords.point.y, map_coords.point.z);
    } catch (tf::TransformException& ex) {
      ROS_ERROR("Transform konnte nicht durchgefuehrt werden: %s", ex.what());
    }
  }

  static void setSamplingTime(double tp) {
    s_sampling_time_ = tp;
  }

  static void setCounterSize(int size) {
    s_fade_counter_size_ = size;
  }

  static void setCovariances(double process_var, double process_rate_var, double measurement_var) {
    s_process_variance_ = process_var;
    s_process_rate_variance_ = process_rate_var;
    s_measurement_variance_ = measurement_var;
  }

  bool hasFaded() const { return ((fade_counter_ <= 0) ? true : false); }
  const CircleObstacle& getObstacle() const { return obstacle_; }
  const KalmanFilter& getKFx() const { return kf_x_; }
  const KalmanFilter& getKFy() const { return kf_y_; }
  const KalmanFilter& getKFr() const { return kf_r_; }

private:
  void initKF() {
    kf_x_.A(0, 1) = s_sampling_time_;
    kf_y_.A(0, 1) = s_sampling_time_;
    kf_r_.A(0, 1) = s_sampling_time_;

    kf_x_.C(0, 0) = 1.0;
    kf_y_.C(0, 0) = 1.0;
    kf_r_.C(0, 0) = 1.0;

    kf_x_.R(0, 0) = s_measurement_variance_;
    kf_y_.R(0, 0) = s_measurement_variance_;
    kf_r_.R(0, 0) = s_measurement_variance_;

    kf_x_.Q(0, 0) = s_process_variance_;
    kf_r_.Q(0, 0) = s_process_variance_;
    kf_y_.Q(0, 0) = s_process_variance_;

    kf_x_.Q(1, 1) = s_process_rate_variance_;
    kf_y_.Q(1, 1) = s_process_rate_variance_;
    kf_r_.Q(1, 1) = s_process_rate_variance_;

    kf_x_.q_pred(0) = obstacle_.center.x;
    kf_r_.q_pred(0) = obstacle_.radius;
    kf_y_.q_pred(0) = obstacle_.center.y;

    kf_x_.q_pred(1) = obstacle_.velocity.x;
    kf_y_.q_pred(1) = obstacle_.velocity.y;

    kf_x_.q_est(0) = obstacle_.center.x;
    kf_r_.q_est(0) = obstacle_.radius;
    kf_y_.q_est(0) = obstacle_.center.y;

    kf_x_.q_est(1) = obstacle_.velocity.x;
    kf_y_.q_est(1) = obstacle_.velocity.y;
  }

  CircleObstacle obstacle_;

  KalmanFilter kf_x_;
  KalmanFilter kf_y_;
  KalmanFilter kf_r_;

  int fade_counter_;

  // Common variables
  static int s_fade_counter_size_;
  static double s_sampling_time_;
  static double s_process_variance_;
  static double s_process_rate_variance_;
  static double s_measurement_variance_;
};

}
