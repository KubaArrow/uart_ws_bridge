#ifndef WS_BRIDGE_H
#define WS_BRIDGE_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <time.h>
#include <libwebsockets.h>
#include <cjson/cJSON.h>
#include "log.h"

#ifndef WS_BRIDGE_LOG_LEVEL
#define WS_BRIDGE_LOG_LEVEL LOG_ERROR
#endif

#define WSB_LOG_ERROR(fmt, ...)   LOG_MODULE(WS_BRIDGE_LOG_LEVEL, LOG_ERROR, fmt, ##__VA_ARGS__)
#define WSB_LOG_WARN(fmt, ...)    LOG_MODULE(WS_BRIDGE_LOG_LEVEL, LOG_WARN,  fmt, ##__VA_ARGS__)
#define WSB_LOG_INFO(fmt, ...)    LOG_MODULE(WS_BRIDGE_LOG_LEVEL, LOG_INFO,  fmt, ##__VA_ARGS__)
#define WSB_LOG_DEBUG(fmt, ...)   LOG_MODULE(WS_BRIDGE_LOG_LEVEL, LOG_DEBUG, fmt, ##__VA_ARGS__)
#define WSB_LOG_TRACE(fmt, ...)   LOG_MODULE(WS_BRIDGE_LOG_LEVEL, LOG_TRACE, fmt, ##__VA_ARGS__)


// Maximum number of topics
#define WSB_MAX_TOPICS 10


    // Enumeration for topic direction
    typedef enum
    {
        WSB_TOPIC_PUBLISH,
        WSB_TOPIC_SUBSCRIBE
    } wsb_topic_direction_t;

    // Structure for a topic with additional JSON and flags
    typedef struct
    {
        const char *name;                // topic name
        const char *msg_type;            // message type (for published topics)
        wsb_topic_direction_t direction; // direction: publish or subscribe
        int advertised;                  // whether advertise has been performed
        int subscribed;                  // whether subscribe has been performed
        time_t last_publish_time;        // time of the last publication
        cJSON *json_to_send;             // JSON object to send (prepared by the user)
        cJSON *json_received;            // most recently received JSON object for the topic
        int send_ready;                  // flag: JSON to send is ready
        int recv_ready;                  // flag: new JSON received
    } wsb_topic_t;

    // Public API functions
    wsb_topic_t *wsb_add_topic(const char *name, const char *msg_type, wsb_topic_direction_t direction);
    void wsb_publish(wsb_topic_t *topic, cJSON *msg);
    int wsb_init(const char *address, int port, const char *host, const char *origin);
    void wsb_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // WS_BRIDGE_H
