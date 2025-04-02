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
#include "geometry_msgs/Twist.h"
#include "std_msgs/UInt16MultiArray.h"
#include "sensor_msgs/Imu.h"

SerialSlip* slip = nullptr;
// Funkcja do wypisywanias zawartości wiadomości IMU
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    CmdVelMsg_t packet = {0};
    packet.msgID = CMD_VEL_MSG;

    // Przepisz wartości z ROS na strukturę
    packet.linear.x  = msg->linear.x;
    packet.linear.y  = msg->linear.y;
    packet.linear.z  = msg->linear.z;

    packet.angular.x = msg->angular.x;
    packet.angular.y = msg->angular.y;
    packet.angular.z = msg->angular.z;

    // Wyślij przez serial_slip
    int result = serial_slip_write(slip, reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
    if (result < 0) {
        ROS_ERROR("serial_slip_write failed");
    } else {
        ROS_DEBUG("Sent cmd_vel packet (%lu bytes)", sizeof(packet));
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "uwu_bridge");
    ros::NodeHandle nh("~");  // prywatna przestrzeń nazw dla parametrów z <param>

    std::string master_ip, own_ip, uart_port;
    int master_port;

    std::string twist_topic, odom_topic, imu_topic, magnet_topic, line_detector_topic, leds_topic;

    nh.param<std::string>("master_ip", master_ip, "127.0.0.1");
    nh.param<int>("master_port", master_port, 9090);
    nh.param<std::string>("own_ip", own_ip, "127.0.0.1");
    nh.param<std::string>("uart_port", uart_port, "/dev/ttyUSB0");

    nh.param<std::string>("twist_topic", twist_topic, "/cmd_vel");
    nh.param<std::string>("odom_topic", odom_topic, "/low_level_odom");
    nh.param<std::string>("imu_topic", imu_topic, "/imu");
    nh.param<std::string>("magnet_topic", magnet_topic, "/magnet");
    nh.param<std::string>("line_detector_topic", line_detector_topic, "/line_detector");
    nh.param<std::string>("leds_topic", leds_topic, "/leds");

    std::ostringstream host_stream, origin_stream;
    host_stream << master_ip << ":" << master_port;
    origin_stream << own_ip << ":" << master_port;

    std::string host = host_stream.str();
    std::string origin = origin_stream.str();

    slip = serial_slip_open(uart_port.c_str());
    uint8_t recvBuffer[8192];
    size_t recvLen = 0;

    wsb_topic_t *odom = wsb_add_topic(odom_topic.c_str(), "nav_msgs/Odometry", WSB_TOPIC_PUBLISH);
    wsb_topic_t *imu = wsb_add_topic(imu_topic.c_str(), "sensor_msgs/Imu", WSB_TOPIC_PUBLISH);
    wsb_topic_t *magnet = wsb_add_topic(magnet_topic.c_str(), "std_msgs/Float64MultiArray", WSB_TOPIC_PUBLISH);
    wsb_topic_t *line = wsb_add_topic(line_detector_topic.c_str(), "std_msgs/UInt16MultiArray", WSB_TOPIC_PUBLISH);


    wsb_topic_t *cmd_vel = wsb_add_topic(twist_topic.c_str(), "geometry_msgs/Twist", WSB_TOPIC_SUBSCRIBE);

    ros::Subscriber sub = nh.subscribe(twist_topic, 10, cmdVelCallback);
    ros::Publisher imu_publisher = nh.advertise<sensor_msgs::Imu>(imu_topic, 10);
    ros::Publisher line_detector_publisher = nh.advertise<std_msgs::UInt16MultiArray>(line_detector_topic, 10);


    wsb_topic_t *leds = wsb_add_topic(leds_topic.c_str(), "std_msgs/ColorRGBA", WSB_TOPIC_SUBSCRIBE);


    if (wsb_init(master_ip.c_str(), master_port, host.c_str(), origin.c_str()) != 0)
    {
        return -1;
    }


    ros::Rate loop_rate(50);  // 50 Hz pętla odbioru danych
    while (ros::ok()) {
        ROS_ERROR("loop");
        if (SS_OK == serial_slip_get_message(slip, recvBuffer, sizeof(recvBuffer), &recvLen, 0)) {
            ROS_ERROR("if");;
            switch ((msgID_t)recvBuffer[0]) {
                case IMU_MSG: {
                    ImuMsg_t *imuMsg = (ImuMsg_t *)recvBuffer;
                    ROS_ERROR("imu");
                    sensor_msgs::Imu ros_imu;

                    // Uzupełnij nagłówek
                    ros_imu.header.stamp = ros::Time::now();
                    ros_imu.header.frame_id = "imu_link";


                    // Orientacja
                    ros_imu.orientation.x = imuMsg->orientation.x;
                    ros_imu.orientation.y = imuMsg->orientation.y;
                    ros_imu.orientation.z = imuMsg->orientation.z;
                    ros_imu.orientation.w = imuMsg->orientation.w;


                    // Przykładowa covariance
                    for (int i = 0; i < 9; i++) {
                        ros_imu.orientation_covariance[i] = 0.0;
                        ros_imu.angular_velocity_covariance[i] = 0.0;
                        ros_imu.linear_acceleration_covariance[i] = 0.0;
                    }

                    // Prędkość kątowa
                    ros_imu.angular_velocity.x = imuMsg->angular_velocity.x;
                    ros_imu.angular_velocity.y = imuMsg->angular_velocity.y;
                    ros_imu.angular_velocity.z = imuMsg->angular_velocity.z;

                    // Przyspieszenie liniowe
                    ros_imu.linear_acceleration.x = imuMsg->linear_acceleration.x;
                    ros_imu.linear_acceleration.y = imuMsg->linear_acceleration.y;
                    ros_imu.linear_acceleration.z = imuMsg->linear_acceleration.z;
                    ROS_ERROR("imu publisher");
                    imu_publisher.publish(ros_imu);
                    break;
                }
                case TRACKER_SENSOR_MSG: {
                    TrackerSensorMsg_t *trackerMsg = (TrackerSensorMsg_t *)recvBuffer;
                    ROS_ERROR("tracker");
                    std_msgs::UInt16MultiArray ros_msg;

                    // Wypełnij dane
                    for (int i = 0; i < 5; ++i) {
                        ros_msg.data.push_back(trackerMsg->data[i]);
                    }

                    // Opcjonalnie możesz ustawić layout (jeśli ważny)
                    ros_msg.layout.dim.resize(1);
                    ros_msg.layout.dim[0].label = "lines";
                    ros_msg.layout.dim[0].size = 5;
                    ros_msg.layout.dim[0].stride = 5;
                    ros_msg.layout.data_offset = 0;

                    line_detector_publisher.publish(ros_msg);
                    break;
                }
                default:
                    break;
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    //    while (1)
//    {
//        if (cmd_vel->recv_ready == true)
//        {
//            char *received = cJSON_PrintUnformatted(cmd_vel->json_received);
//            if (received)
//            {
//                CmdVelMsg_t msg = {0};
//

//                // Parsuj pola linear
//                cJSON *linear = cJSON_GetObjectItem(cmd_vel->json_received, "msg");
//                if (linear) linear = cJSON_GetObjectItem(linear, "linear");
//                if (linear) {
//                    cJSON *x = cJSON_GetObjectItem(linear, "x");
//                    cJSON *y = cJSON_GetObjectItem(linear, "y");
//                    cJSON *z = cJSON_GetObjectItem(linear, "z");
//                    if (x && y && z) {
//                        msg.linear.x = x->valuedouble;
//                        msg.linear.y = y->valuedouble;
//                        msg.linear.z = z->valuedouble;
//                    }
//                }
//
//                // Parsuj pola angular
//                cJSON *angular = cJSON_GetObjectItem(cmd_vel->json_received, "msg");
//                if (angular) angular = cJSON_GetObjectItem(angular, "angular");
//                if (angular) {
//                    cJSON *x = cJSON_GetObjectItem(angular, "x");
//                    cJSON *y = cJSON_GetObjectItem(angular, "y");
//                    cJSON *z = cJSON_GetObjectItem(angular, "z");
//                    if (x && y && z) {
//                        msg.angular.x = x->valuedouble;
//                        msg.angular.y = y->valuedouble;
//                        msg.angular.z = z->valuedouble;
//                    }
//                }
//
//                msg.msgID = CMD_VEL_MSG;
//                serial_slip_write(slip, (uint8_t*)&msg, sizeof(msg));
//                free(received);
//            }
//            cmd_vel->recv_ready = false;
//        }

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
//        sleep(1);
//    }
}