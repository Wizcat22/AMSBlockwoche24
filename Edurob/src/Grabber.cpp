#include "Grabber.hpp"

#include <Dynamixel2Arduino.h>
#include "ESP32SerialPortHandler.cpp"

ros::Subscriber<std_msgs::Float32> grabber_angle_sub("motor_cmd", &grabberAngle_callback);

// New way of creating dxl
Dynamixel2Arduino dxl;

// Our custom handler with RX and TX pins specified.
ESP32SerialPortHandler esp_dxl_port(DXL_SERIAL, DXL_RX_PIN, DXL_TX_PIN, DXL_DIR_PIN);

void initDynamixel()
{
    // Set custom port handler
    dxl.setPort(esp_dxl_port);

    dxl.begin(DXL_BAUD_RATE);

    dxl.setOperatingMode(DXL_ID, OP_POSITION);
    dxl.setGoalVelocity(DXL_ID, 20, UNIT_PERCENT);
    dxl.torqueOn(DXL_ID);

    dxl.ledOn(DXL_ID);

    dxl.setGoalPosition(DXL_ID, 350, UNIT_DEGREE);
}

void grabberAngle_callback(const std_msgs::Float32 &msg)
{
    dxl.setGoalPosition(DXL_ID, msg.data, UNIT_DEGREE);
}
