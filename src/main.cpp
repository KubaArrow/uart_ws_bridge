#include "uart_ws_bridge/ws_bridge.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <cjson/cJSON.h>

int main(void)
{

    // Register topics using the public API
    wsb_topic_t *example_topic = wsb_add_topic("/example_topic", "std_msgs/String", WSB_TOPIC_PUBLISH);
    wsb_topic_t *numbers_topic = wsb_add_topic("/numbers_topic", "std_msgs/UInt16MultiArray", WSB_TOPIC_PUBLISH);
    wsb_topic_t *chatter = wsb_add_topic("/chatter", NULL, WSB_TOPIC_SUBSCRIBE);
    wsb_topic_t *cmd_vel = wsb_add_topic("/cmd_vel", NULL, WSB_TOPIC_SUBSCRIBE);

    // Initialize the connection – parameters can be modified as needed
    if (wsb_init("192.168.18.221", 9090, "192.168.18.221:9090", "192.168.18.59:9090") != 0)
    {
        return -1;
    }
    while (true)
    {
        // Handle received messages
        if (chatter->recv_ready == true)
        {
            char *received = cJSON_PrintUnformatted(chatter->json_received);
            if (received)
            {

                WSB_LOG_TRACE("%s\n", received);
                free(received);
            }
            chatter->recv_ready = false;
        }
        if (cmd_vel->recv_ready == true)
        {
            char *received = cJSON_PrintUnformatted(cmd_vel->json_received);
            if (received)
            {

                WSB_LOG_TRACE("%s\n", received);
                free(received);
            }
            cmd_vel->recv_ready = false;
        }
        // Publish std_msgs/String message every 1 second
        if ((time(NULL) - example_topic->last_publish_time) > 1)
        {
            char time_str[32];
            snprintf(time_str, sizeof(time_str), "%ld", time(NULL));
            cJSON *msg = cJSON_CreateObject();
            cJSON_AddStringToObject(msg, "data", time_str);
            cJSON *publish_msg = cJSON_CreateObject();
            cJSON_AddStringToObject(publish_msg, "op", "publish");
            cJSON_AddStringToObject(publish_msg, "topic", example_topic->name);
            cJSON_AddItemToObject(publish_msg, "msg", msg);
            wsb_publish(example_topic, publish_msg);
        }
        // Publish std_msgs/UInt16MultiArray message (an array of 5 numbers) every 2 seconds
        if ((time(NULL) - numbers_topic->last_publish_time) > 2)
        {
            cJSON *msg = cJSON_CreateObject();
            cJSON *data_array = cJSON_CreateArray();
            for (int i = 0; i < 5; i++)
            {

                cJSON_AddItemToArray(data_array, cJSON_CreateNumber(100 + i * 10));
            }
            cJSON_AddItemToObject(msg, "data", data_array);
            cJSON *publish_msg = cJSON_CreateObject();
            cJSON_AddStringToObject(publish_msg, "op", "publish");
            cJSON_AddStringToObject(publish_msg, "topic", numbers_topic->name);
            cJSON_AddItemToObject(publish_msg, "msg", msg);
            wsb_publish(numbers_topic, publish_msg);
        }
        sleep(1);
    }
    wsb_deinit();
    return 0;
}
