#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "uart_ws_bridge/ws_bridge.h"
#include "uart_ws_bridge/serial_slip.h"
#include "uart_ws_bridge/rosTopic.h"
#include "ros/ros.h"
#include <string>
#include <sstream>


// Funkcja do wypisywanias zawartości wiadomości IMU


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "uwu_bridge");
    ros::NodeHandle nh("~");  // prywatna przestrzeń nazw dla parametrów z <param>

    std::string master_ip, own_ip, uart_port;
    int master_port;

    std::string twist_topic, odom_topic, imu_topic, magnet_topic, line_detector_topic;

    nh.param<std::string>("master_ip", master_ip, "127.0.0.1");
    nh.param<int>("master_port", master_port, 9090);
    nh.param<std::string>("own_ip", own_ip, "127.0.0.1");
    nh.param<std::string>("uart_port", uart_port, "/dev/ttyUSB0");

    nh.param<std::string>("twist_topic", twist_topic, "/cmd_vel");
    nh.param<std::string>("odom_topic", odom_topic, "/low_level_odom");
    nh.param<std::string>("imu_topic", imu_topic, "/imu");
    nh.param<std::string>("magnet_topic", magnet_topic, "/magnet");
    nh.param<std::string>("line_detector_topic", line_detector_topic, "/line_detector");

    std::ostringstream host_stream, origin_stream;
    host_stream << master_ip << ":" << master_port;
    origin_stream << own_ip << ":" << master_port;

    std::string host = host_stream.str();
    std::string origin = origin_stream.str();

    SerialSlip *slip = serial_slip_open("COM9");
    uint8_t recvBuffer[8192];
    size_t recvLen = 0;

    wsb_topic_t *cmd_vel = wsb_add_topic("/cmd_vel", NULL, WSB_TOPIC_SUBSCRIBE);
    // Initialize the connection – parameters can be modified as needed
    if (wsb_init(master_ip.c_str(), master_port, host.c_str(), origin.c_str()) != 0)
    {
        return -1;
    }

    while (1)
    {
        if (cmd_vel->recv_ready == true)
        {
            char *received = cJSON_PrintUnformatted(cmd_vel->json_received);
            if (received)
            {
                CmdVelMsg_t msg = {0};

                // Parsuj pola linear
                cJSON *linear = cJSON_GetObjectItem(cmd_vel->json_received, "msg");
                if (linear) linear = cJSON_GetObjectItem(linear, "linear");
                if (linear) {
                    cJSON *x = cJSON_GetObjectItem(linear, "x");
                    cJSON *y = cJSON_GetObjectItem(linear, "y");
                    cJSON *z = cJSON_GetObjectItem(linear, "z");
                    if (x && y && z) {
                        msg.linear.x = x->valuedouble;
                        msg.linear.y = y->valuedouble;
                        msg.linear.z = z->valuedouble;
                    }
                }

                // Parsuj pola angular
                cJSON *angular = cJSON_GetObjectItem(cmd_vel->json_received, "msg");
                if (angular) angular = cJSON_GetObjectItem(angular, "angular");
                if (angular) {
                    cJSON *x = cJSON_GetObjectItem(angular, "x");
                    cJSON *y = cJSON_GetObjectItem(angular, "y");
                    cJSON *z = cJSON_GetObjectItem(angular, "z");
                    if (x && y && z) {
                        msg.angular.x = x->valuedouble;
                        msg.angular.y = y->valuedouble;
                        msg.angular.z = z->valuedouble;
                    }
                }

                msg.msgID = CMD_VEL_MSG;
                serial_slip_write(slip, (uint8_t*)&msg, sizeof(msg));
                free(received);
            }
            cmd_vel->recv_ready = false;
        }

//
//        if (SS_OK == serial_slip_get_message(slip, recvBuffer, 8192, &recvLen, 1))
//        {
//            switch ((msgID_t)recvBuffer[0])
//            {
//            case IMU_MSG:
//                ImuMsg_t *imuMsg = (ImuMsg_t *)recvBuffer;
//                printImuMsg(imuMsg);
//                break;
//
//            case TRACKER_SENSOR_MSG:
//                TrackerSensorMsg_t *trackerMsg = (TrackerSensorMsg_t *)recvBuffer;
//                printTrackerSensorMsg(trackerMsg);
//                break;
//
//            default:
//                break;
//            }
//        }
        sleep(1);
    }
}