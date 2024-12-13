#include "Grabber.hpp"

#include <Dynamixel2Arduino.h>
#include "ESP32SerialPortHandler.cpp"

// Expects angle for Dynamixel from docking node
ros::Subscriber<std_msgs::Float32> grabber_angle_sub("grabber/angle", &grabberAngle_callback);

Dynamixel2Arduino dxl;

/* Our custom handler with RX and TX pins specified
because default arduino only has one serial hardware */
ESP32SerialPortHandler esp_dxl_port(DXL_SERIAL, DXL_RX_PIN, DXL_TX_PIN, DXL_DIR_PIN);

// Publishes the successful closing of the grapper
std_msgs::Float32 feedback_msg;
ros::Publisher feedback_pub("grabber/feedback", &feedback_msg);

void forceFeedbackTask(void *pvParameters)
{
    while (true)
	{
		vTaskDelay(pdMS_TO_TICKS(500));
        feedback_msg.data = dxl.getPresentCurrent(DXL_ID, UNIT_MILLI_AMPERE);
        feedback_pub.publish(&feedback_msg);
        
        int32_t errorFlags = dxl.readControlTableItem(ControlTableItem::HARDWARE_ERROR_STATUS, DXL_ID);
        DXLLibErrorCode_t communicationResult = dxl.getLastLibErrCode();
        
        
        if(errorFlags != 0x00 && communicationResult == DXLLibErrorCode::DXL_LIB_OK)
        {
            dxl.reboot(DXL_ID);
            Serial.printf("Dynamixel Error: 0x%08x %n", errorFlags);
        }
    }
}

void initDynamixel()
{
    // Set custom port handler
    dxl.setPort(esp_dxl_port);

    dxl.begin(DXL_BAUD_RATE);

    dxl.ledOn(DXL_ID);

    // Torque off (some registers can only be set in torque-off mode)
    dxl.torqueOff(DXL_ID);
    dxl.writeControlTableItem(ControlTableItem::CURRENT_LIMIT, DXL_ID, 1200);

    dxl.setOperatingMode(DXL_ID, OP_CURRENT_BASED_POSITION);
    dxl.setGoalPosition(DXL_ID, 350.0, UNIT_DEGREE);
    dxl.setGoalCurrent(DXL_ID, 600, UNIT_MILLI_AMPERE);

    // Torque on
    dxl.torqueOn(DXL_ID);

    xTaskCreate(
		forceFeedbackTask,	  /* Task function. */
		"forceFeedbackTask", /* String with name of task. */
		10000,		  /* Stack size in bytes. */
		NULL,		  /* Parameter passed as input of the task */
		4,			  /* Priority of the task. */
		NULL);		  /* Task handle. */
}

/**
 * 
 */
void grabberAngle_callback(const std_msgs::Float32 &msg)
{
    dxl.setGoalPosition(DXL_ID, msg.data, UNIT_DEGREE);
}
