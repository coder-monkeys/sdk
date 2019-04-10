#ifndef _MQTT_PROC_H_
#define _MQTT_PROC_H_


#define MQTT_BUFF_SIZE         256
#define MQTT_OK                0
#define MQTT_ERR               -1
#define MQTT_READ_TIMEOUT      (-1000)
#define MQTT_DEBUG_EN          1
#if MQTT_DEBUG_EN
#define mqtt_debug             printf
#else
#define mqtt_debug
#endif
#define MQTT_TASK_SIZE         1024
#define MQTT_USE_SSL           (1 && TLS_CONFIG_HTTP_CLIENT_SECURE)
#define MQTT_SERVER_NAME       "49.4.93.24"
#define MQTT_SERVER_PORT       8883
#define MQTT_CLIENT_ID         "938ea1c3-7702-4470-bf59-fbb49a7f5cf5"
#define MQTT_USER_NAME         "YH_WA710"
#define MQTT_USER_KEY          "b74f08312f240d316b9f"
#define MQTT_RECV_TASK_PRIO    63
#define MQTT_PING_INTERVAL     30


typedef enum _mqtt_state{
    WAIT_WIFI_OK,
    CONNECT_SERVER,
    RECV_FROM_SERVER,
    SEND_HEARTBEAT,
} mqtt_state;

#endif

