#ifndef _MQTT_PROC_H_
#define _MQTT_PROC_H_


#define MQTT_BUFF_SIZE  50

#define MQTT_UNBIND 1
#define MQTT_OK     2
#define MQTT_ERR    -1
#define MQTT_READ_TIMEOUT   (-1000)

typedef struct _mqtt_param_{
    char clientid[33];
    char username[33];
    char password[33];
    char ip[16];
    unsigned short port;
    int keepalive;
}_mqtt_param;

int mqtt_open(void);

void mqtt_send_heart(void);

int mqtt_rcv(void);

#endif

