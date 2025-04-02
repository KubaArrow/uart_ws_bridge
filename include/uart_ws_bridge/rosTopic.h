/*
 * rosTopic.h
 *
 *  Created on: Mar 24, 2025
 *      Author: bkfcb
 */

#ifndef INC_ROSTOPIC_H_
#define INC_ROSTOPIC_H_

#include <stdint.h>

typedef enum 
{
    IMU_MSG = 0x30,
    TRACKER_SENSOR_MSG,
    CMD_VEL_MSG,
}msgID_t;
#pragma pack(1)

typedef struct
{
    uint32_t seq;
    uint32_t stamp_sec;
    uint32_t stamp_nsec;
} Header_t;

typedef struct
{
    double x;
    double y;
    double z;
    double w;
} Quaternion_t;

typedef struct
{
    double x;
    double y;
    double z;
} Vector3_t;

typedef struct
{
    uint8_t msgID;
    Header_t header;
    Quaternion_t orientation;
    Vector3_t angular_velocity;
    Vector3_t linear_acceleration;
} ImuMsg_t;

typedef struct
{
    uint8_t msgID;
    uint16_t data[5];
} TrackerSensorMsg_t;

typedef struct
{
    uint8_t msgID;
    Vector3_t linear;
    Vector3_t angular;
}CmdVelMsg_t;

#pragma pack()

#endif /* INC_ROSTOPIC_H_ */
