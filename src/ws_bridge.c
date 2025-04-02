#include "uart_ws_bridge/ws_bridge.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <pthread.h>
#include <libwebsockets.h>
#include <cjson/cJSON.h>

// Global variables internal to the library
static wsb_topic_t wsb_topics[WSB_MAX_TOPICS];
static int wsb_topic_count = 0;
static pthread_t wsb_hThread;  // zamieniono z HANDLE
static struct lws_context_creation_info wsb_info;
static struct lws_client_connect_info wsb_ccinfo;
static struct lws_context *wsb_context = NULL;
static struct lws *wsb_global_wsi = NULL;

// Internal function: WebSockets callback
static int wsb_callback_websockets(struct lws *wsi, enum lws_callback_reasons reason, void *user, void *in, size_t len)
{
    (void)(user);
    int i = 0;
    unsigned char buf[LWS_PRE + 512];
    size_t msg_len = 0;
    char *json_str = NULL;
    cJSON *root = NULL;
    switch (reason)
    {
        case LWS_CALLBACK_CLIENT_ESTABLISHED:
        {
            WSB_LOG_DEBUG("Connected to rosbridge!\n");
            lws_callback_on_writable(wsi);
            break;
        }
        case LWS_CALLBACK_CLIENT_WRITEABLE:
        {
            for (i = 0; i < wsb_topic_count; i++)
            {
                if ((wsb_topics[i].send_ready) && (wsb_topics[i].json_to_send != NULL))
                {
                    json_str = cJSON_PrintUnformatted(wsb_topics[i].json_to_send);
                    cJSON_Delete(wsb_topics[i].json_to_send);
                    wsb_topics[i].json_to_send = NULL;
                    wsb_topics[i].send_ready = 0;
                    WSB_LOG_DEBUG("Sent user JSON for topic: %s\n", wsb_topics[i].name);
                }
                else if ((wsb_topics[i].direction == WSB_TOPIC_PUBLISH) && (!wsb_topics[i].advertised))
                {
                    root = cJSON_CreateObject();
                    cJSON_AddStringToObject(root, "op", "advertise");
                    cJSON_AddStringToObject(root, "topic", wsb_topics[i].name);
                    if (wsb_topics[i].msg_type != NULL)
                    {
                        cJSON_AddStringToObject(root, "type", wsb_topics[i].msg_type);
                    }
                    json_str = cJSON_PrintUnformatted(root);
                    wsb_topics[i].advertised = 1;
                    WSB_LOG_DEBUG("Advertised topic: %s\n", wsb_topics[i].name);
                }
                else if ((wsb_topics[i].direction == WSB_TOPIC_SUBSCRIBE) && (!wsb_topics[i].subscribed))
                {
                    root = cJSON_CreateObject();
                    cJSON_AddStringToObject(root, "op", "subscribe");
                    cJSON_AddStringToObject(root, "topic", wsb_topics[i].name);
                    json_str = cJSON_PrintUnformatted(root);
                    wsb_topics[i].subscribed = 1;
                    WSB_LOG_DEBUG("Subscribed to topic: %s\n", wsb_topics[i].name);
                }
                else
                {
                    if (root)
                    {
                        cJSON_Delete(root);
                    }
                    continue;
                }
                if (json_str != NULL)
                {
                    msg_len = strlen(json_str);
                    memcpy(&buf[LWS_PRE], json_str, msg_len);
                    lws_write(wsi, &buf[LWS_PRE], msg_len, LWS_WRITE_TEXT);
                    free(json_str);
                }
                if (root)
                {
                    cJSON_Delete(root);
                }
                lws_callback_on_writable(wsi);
                break; // send one message per cycle
            }
            break;
        }
        case LWS_CALLBACK_CLIENT_RECEIVE:
        {
            cJSON *root_local = cJSON_Parse((const char *)in);
            if (root_local == NULL)
            {
                WSB_LOG_ERROR("Error parsing received JSON.\n");
                break;
            }
            cJSON *topic_item = cJSON_GetObjectItem(root_local, "topic");
            if ((topic_item) && (cJSON_IsString(topic_item)))
            {
                const char *received_topic = topic_item->valuestring;
                for (i = 0; i < wsb_topic_count; i++)
                {
                    if (strcmp(wsb_topics[i].name, received_topic) == 0)
                    {
                        if (wsb_topics[i].json_received)
                        {
                            cJSON_Delete(wsb_topics[i].json_received);
                        }
                        wsb_topics[i].json_received = cJSON_Duplicate(root_local, 1);
                        wsb_topics[i].recv_ready = 1;
                        WSB_LOG_DEBUG("Received new JSON for topic: %s\n", wsb_topics[i].name);
                        char *recv_str = cJSON_PrintUnformatted(wsb_topics[i].json_received);
                        if (recv_str)
                        {
                            WSB_LOG_TRACE("Content: %s\n", recv_str);
                            free(recv_str);
                        }
                    }
                }
            }
            else
            {
                WSB_LOG_ERROR("Received JSON without topic field: %.*s\n", (int)len, (const char *)in);
            }
            cJSON_Delete(root_local);
            break;
        }
        case LWS_CALLBACK_CLIENT_CONNECTION_ERROR:
        {
            WSB_LOG_ERROR("Connection error!\n");
            break;
        }
        case LWS_CALLBACK_CLOSED:
        {
            WSB_LOG_DEBUG("Connection closed.\n");
            break;
        }
        default:
            break;
    }
    return 0;
}

// Internal thread function for libwebsockets service (z użyciem pthread)
static void* wsb_ServiceThread(void *lpParam)
{
    struct lws_context *local_context = (struct lws_context *)lpParam;
    while (lws_service(local_context, 50) >= 0)
    {
        // empty loop body
    }
    return NULL;
}

// API function to add a topic to the global list
wsb_topic_t *wsb_add_topic(const char *name, const char *msg_type, wsb_topic_direction_t direction)
{
    wsb_topic_t *newTopic = NULL;
    if (wsb_topic_count < WSB_MAX_TOPICS)
    {
        wsb_topics[wsb_topic_count].name = name;
        wsb_topics[wsb_topic_count].msg_type = msg_type;
        wsb_topics[wsb_topic_count].direction = direction;
        wsb_topics[wsb_topic_count].advertised = 0;
        wsb_topics[wsb_topic_count].subscribed = 0;
        wsb_topics[wsb_topic_count].last_publish_time = 0;
        wsb_topics[wsb_topic_count].json_to_send = NULL;
        wsb_topics[wsb_topic_count].json_received = NULL;
        wsb_topics[wsb_topic_count].send_ready = 0;
        wsb_topics[wsb_topic_count].recv_ready = 0;
        newTopic = &wsb_topics[wsb_topic_count];
        wsb_topic_count++;
    }
    else
    {
        WSB_LOG_ERROR("Reached maximum number of topics.\n");
    }
    return newTopic;
}

// API function to publish a message
void wsb_publish(wsb_topic_t *topic, cJSON *msg)
{
    if ((topic == NULL) || (topic->direction != WSB_TOPIC_PUBLISH))
    {
        return;
    }
    if (topic->json_to_send)
    {
        cJSON_Delete(topic->json_to_send);
    }
    char *temp_str = cJSON_PrintUnformatted(msg);
    if (temp_str)
    {
        WSB_LOG_TRACE("%s\n", temp_str);
        free(temp_str);
    }
    topic->json_to_send = msg;
    topic->send_ready = 1;
    topic->last_publish_time = time(NULL);
    lws_callback_on_writable(wsb_global_wsi);
}

// API function to initialize the connection with rosbridge and start the service loop
int wsb_init(const char *address, int port, const char *host, const char *origin)
{
    memset(&wsb_info, 0, sizeof(wsb_info));
    wsb_info.port = CONTEXT_PORT_NO_LISTEN;
    wsb_info.protocols = (struct lws_protocols[]){
        {"example-protocol", wsb_callback_websockets, 0, 0, 0, NULL, 0},
        {NULL, NULL, 0, 0, 0, NULL, 0}};
    wsb_info.gid = -1;
    wsb_info.uid = -1;
    wsb_context = lws_create_context(&wsb_info);
    if (wsb_context == NULL)
    {
        WSB_LOG_ERROR("lws_create_context failed\n");
        return -1;
    }
    memset(&wsb_ccinfo, 0, sizeof(wsb_ccinfo));
    wsb_ccinfo.context = wsb_context;
    wsb_ccinfo.address = address;
    wsb_ccinfo.port = port;
    wsb_ccinfo.path = "/";
    wsb_ccinfo.host = host;
    wsb_ccinfo.origin = origin;
    wsb_ccinfo.protocol = "example-protocol";
    wsb_ccinfo.ssl_connection = 0;
    wsb_global_wsi = lws_client_connect_via_info(&wsb_ccinfo);
    if (wsb_global_wsi == NULL)
    {
        WSB_LOG_ERROR("Failed to establish connection\n");
        lws_context_destroy(wsb_context);
        return -1;
    }
    // Uruchomienie wątku z usługą libwebsockets przy użyciu pthread
    if (pthread_create(&wsb_hThread, NULL, wsb_ServiceThread, wsb_context) != 0)
    {
        WSB_LOG_ERROR("Failed to create thread\n");
        lws_context_destroy(wsb_context);
        return -1;
    }
    return 0;
}

// API function to deinitialize the library and clean up resources
void wsb_deinit(void)
{
    pthread_join(wsb_hThread, NULL);
    for (int i = 0; i < wsb_topic_count; i++)
    {
        if (wsb_topics[i].json_received)
        {
            cJSON_Delete(wsb_topics[i].json_received);
        }
    }
    lws_context_destroy(wsb_context);
}
