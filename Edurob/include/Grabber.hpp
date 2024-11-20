#ifndef EDUROB_GRABBER_H
#define EDUROB_GRABBER_H

#include <std_msgs/Float32.h>
#include <ros.h>

#define DXL_SERIAL   Serial1
constexpr uint8_t DXL_DIR_PIN = 3; //  DIR pin
constexpr uint8_t DXL_RX_PIN = 15; //  RX PIN
constexpr uint8_t DXL_TX_PIN = 16; //  TX PIN
constexpr uint8_t DXL_ID = 11;
constexpr float DXL_PROTOCOL_VERSION = 2.0;
constexpr uint16_t DXL_BAUD_RATE = 57600;


extern ros::Subscriber<std_msgs::Float32> grabber_angle_sub;

void initDynamixel();

void grabberAngle_callback(const std_msgs::Float32 &msg);

#endif // EDUROB_GRABBER_H