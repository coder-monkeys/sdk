#include "wm_include.h"
#include "libemqtt.h"
#include "HTTPClientWrapper.h"

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
#define MQTT_DEVICE_NAME       "YH_WA710"
#define MQTT_DEVICE_ID         "938ea1c3-7702-4470-bf59-fbb49a7f5cf5"
#define MQTT_USER_KEY          "b74f08312f240d316b9f"
#define MQTT_RECV_TASK_PRIO    63
#define MQTT_PING_INTERVAL     30

typedef enum _mqtt_state{
    WAIT_WIFI_OK,
    CONNECT_SERVER,
    RECV_FROM_SERVER,
    SEND_HEARTBEAT,
} mqtt_state;

static mqtt_broker_handle_t mqtt_broker;
static tls_ssl_t *ssl;
static char packet_buffer[MQTT_BUFF_SIZE] = { 0 };
static OS_STK DemoMqttRecvStk[MQTT_TASK_SIZE];

static int mqtt_send(int socket_info, const void *buf, unsigned int count)
{
    int fd = socket_info;
	
#if MQTT_USE_SSL
    return HTTPWrapperSSLSend(ssl, fd, buf, count, 0);
#else
	return send(fd, buf, count, 0);
#endif
}

static void mqtt_close(void)
{
#if MQTT_USE_SSL
	closesocket(mqtt_broker.socketid);
    HTTPWrapperSSLClose(ssl, mqtt_broker.socketid);
#else
	closesocket(mqtt_broker.socketid);
#endif
}

/* Socket safe API,do not need to close socket or ssl session if this fucntion return MQTT_ERR; */
static int init_socket(mqtt_broker_handle_t *broker, const char *hostname, unsigned short port, int keepalive)
{
    int socket_id = -1, ret = -1;
    struct sockaddr_in socket_addr;
    
    socket_id = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if(socket_id < 0) {
        return MQTT_ERR;
    }
    memset(&socket_addr, 0, sizeof(struct sockaddr_in));
    socket_addr.sin_family = AF_INET;
    socket_addr.sin_port = htons(port);
    socket_addr.sin_addr.s_addr = ipaddr_addr(hostname);
#if MQTT_USE_SSL
    ret = HTTPWrapperSSLConnect(&ssl, socket_id, (struct sockaddr *)&socket_addr, sizeof(socket_addr), NULL);
#else
	ret = connect(socket_id, (struct sockaddr*)&socket_addr, sizeof(socket_addr))
#endif
    if(ret < 0)
    {
        mqtt_debug("mqtt connect failed = %d\n", ret);
        mqtt_close();
        return MQTT_ERR;
    }
    mqtt_set_alive(broker, keepalive);
    broker->socketid = socket_id;
    broker->mqttsend = mqtt_send;

    return MQTT_OK;
}

/* Socket safe API,do not need to close socket or ssl session if this fucntion return MQTT_ERR; */
static int read_packet(int timeout)
{
    int ret = 0;
    fd_set readfds;
    struct  timeval tmv = {.tv_sec = timeout, .tv_usec = 0};
    int total_bytes = 0, byte_rcvd = 0, packet_len = 0;

    if(timeout <= 0) {
        return MQTT_READ_TIMEOUT;
    }

    FD_ZERO(&readfds);
    FD_SET(mqtt_broker.socketid, &readfds);

    ret = select(mqtt_broker.socketid + 1, &readfds, NULL, NULL, &tmv);
    if(ret < 0) {
		mqtt_close();
        return MQTT_ERR;
    }
    else if(ret == 0) {
        return MQTT_READ_TIMEOUT;
    }

    memset(packet_buffer, 0, sizeof(packet_buffer));
#if MQTT_USE_SSL
    byte_rcvd = HTTPWrapperSSLRecv(ssl, mqtt_broker.socketid, packet_buffer, MQTT_BUFF_SIZE, 0);
#else
	byte_rcvd = recv(mqtt_broker.socketid, packet_buffer, MQTT_BUFF_SIZE, 0)
#endif
    if(byte_rcvd <= 0) {
		mqtt_close();
        return MQTT_ERR;
    }

    total_bytes += byte_rcvd;
    if(total_bytes < 2) {
        return MQTT_ERR;
    }

    unsigned short rem_len = mqtt_parse_rem_len(packet_buffer);
    unsigned char rem_len_byte = mqtt_num_rem_len_bytes(packet_buffer);

    packet_len = rem_len + rem_len_byte + 1;
    while(total_bytes < packet_len)
    {
#if MQTT_USE_SSL
    	byte_rcvd = HTTPWrapperSSLRecv(ssl, mqtt_broker.socketid, (packet_buffer + total_bytes), MQTT_BUFF_SIZE, 0);
#else
		byte_rcvd = recv(mqtt_broker.socketid, (packet_buffer + total_bytes), MQTT_BUFF_SIZE, 0)
#endif
        if(byte_rcvd <= 0) {
			mqtt_close();
            return MQTT_ERR;
        }
        total_bytes += byte_rcvd;
    }

    return packet_len;
}

/* Socket safe API,do not need to close socket or ssl session if this fucntion return MQTT_ERR; */
static int mqtt_send_heartbeat(void)
{
	int ret = -1;
    printf("mqtt send ping\n");
    ret = mqtt_ping(&mqtt_broker);
	if( ret == -1 ) {
		mqtt_close();
	}
	return ret;
}

/* Socket safe API,do not need to close socket or ssl session if this fucntion return MQTT_ERR; */
static int subscribe_topic(char *topic)
{
    unsigned short msg_id = 0, msg_id_rcv = 0;
    int packet_len = 0;
	int ret = -1;

    if(topic == NULL) {
        return MQTT_ERR;
    }
    
    ret = mqtt_subscribe(&mqtt_broker, topic, &msg_id);
	if( ret == -1 ) {
		mqtt_close();
		return MQTT_ERR;
	}
    packet_len = read_packet(5);
    if(packet_len < 0)
    {
        mqtt_debug("error (%d) on read packet!\n", packet_len);
        return MQTT_ERR;
    }

    if(MQTTParseMessageType(packet_buffer) != MQTT_MSG_SUBACK)
    {
        mqtt_debug("SUBACK expected!\n");
        mqtt_close();
        return MQTT_ERR;
    }

    msg_id_rcv = mqtt_parse_msg_id(packet_buffer);
	if(msg_id != msg_id_rcv)
	{
		mqtt_debug("%d message id was expected, but %d message id was found!\n", msg_id, msg_id_rcv);
        mqtt_close();
        return MQTT_ERR;
	}

    return MQTT_OK;
}

/* Socket safe API,do not need to close socket or ssl session if this fucntion return MQTT_ERR; */
static int mqtt_open(void)
{
    int err = 0, packet_len = 0;
    char sub_topic[64] = { 0 };
    unsigned short msg_id = 0, msg_id_rcv = 0;


    memset(packet_buffer, 0, MQTT_BUFF_SIZE);
    
    mqtt_init(&mqtt_broker, MQTT_DEVICE_NAME);
    mqtt_init_auth(&mqtt_broker, MQTT_DEVICE_ID, MQTT_USER_KEY);

    err = init_socket(&mqtt_broker, MQTT_SERVER_NAME, MQTT_SERVER_PORT, MQTT_PING_INTERVAL);
    if(err != 0)
    {
        mqtt_debug("mqtt init socket faild\n");
        return MQTT_ERR;
    }

    err = mqtt_connect(&mqtt_broker);
	if(err != 0)
    {
        mqtt_debug("mqtt_connect faild\n");
		mqtt_close();
        return MQTT_ERR;
    }

    packet_len = read_packet(10);
    if(packet_len < 0)
    {
        mqtt_debug("error (%d) on read packet!\n", packet_len);
        return MQTT_ERR;
    }
	mqtt_debug("recv:%d\n", packet_len);
	for(int i=0; i<packet_len; i++) {
		mqtt_debug("%x ", packet_buffer[i]);
	}
	mqtt_debug("\n");
    if(MQTTParseMessageType(packet_buffer) != MQTT_MSG_CONNACK || packet_buffer[3] != 0x00)
    {
        mqtt_debug("CONNACK expected or failed!\n");
        mqtt_close();
        return MQTT_ERR;
    }
    
    sprintf(sub_topic, "/device/{%s}/downward", MQTT_DEVICE_ID);
    if(subscribe_topic(sub_topic) != 0) {
        return MQTT_ERR;
    }

    memset(sub_topic, 0, 64);
    sprintf(sub_topic, "/device/{%s}/upward", MQTT_DEVICE_ID);
    if(subscribe_topic(sub_topic) != 0) {
        return MQTT_ERR;
    }

    mqtt_debug("mqtt connect success\n");
    
    return MQTT_OK;
}

/* Socket safe API,do not need to close socket or ssl session if this fucntion return MQTT_ERR; */
static int mqtt_recv(int selectTimeOut)
{
    int packet_len = 0, ret = 0;

    packet_len = read_packet(selectTimeOut);
    if(packet_len == MQTT_READ_TIMEOUT)
    {
        return MQTT_READ_TIMEOUT;
    }
    else if(packet_len > 0)
    {
        ret = MQTTParseMessageType(packet_buffer);
        if(ret == MQTT_MSG_PUBLISH) {
			mqtt_debug("MQTT_MSG_PUBLISH,0x%x\n", packet_buffer[0]);
        }
        else if(ret == MQTT_MSG_PINGRESP) {
            mqtt_debug("mqtt recv pong\n");
        }
        else {
            mqtt_debug("Packet Header: 0x%x\n", packet_buffer[0]);
        }
		return MQTT_OK;
    }
    else if(packet_len == -1)
    {
        return MQTT_ERR;
    }
}

static int isWifiNetworkOk(void)
{
	struct tls_ethif* etherIf= tls_netif_get_ethif();

	return (WM_WIFI_JOINED == tls_wifi_get_state() && etherIf!=NULL && *((u32*)&etherIf->ip_addr)!=0);
}

static void test(void)
{
    int packet_length;
    
    packet_length = read_packet(10);
    mqtt_debug("packet_length = %d\n", packet_length);
    mqtt_debug("Packet Header: 0x%x...\n", packet_buffer[0]);
	if(MQTTParseMessageType(packet_buffer) == MQTT_MSG_PUBLISH)
	{
		uint8_t topic[20], *msg;
		uint16_t len;
		len = mqtt_parse_pub_topic(packet_buffer, topic);
		topic[len] = '\0'; // for printf
		len = mqtt_parse_publish_msg(packet_buffer, &msg);
		msg[len] = '\0'; // for printf
		mqtt_debug("%s %s\n", topic, msg);
	}
}

static void mqttHandleTask( void* lparam )
{
	int ret = MQTT_ERR;
	mqtt_state now_state = WAIT_WIFI_OK;

	while (1) 
    {
        mqtt_debug("now_state: %d\n", now_state);
        switch(now_state)
        {
            case WAIT_WIFI_OK:
            {
				ret = isWifiNetworkOk();
                if( ret ) {
					now_state = CONNECT_SERVER;
					tls_os_time_delay(HZ*5);
                }
                else {
                    mqtt_debug("wifi offline\r\n");
					now_state = WAIT_WIFI_OK;
                    tls_os_time_delay(HZ*5);
                }
            }
            break;
            
            case CONNECT_SERVER:
            {
            	ret = mqtt_open();
				if(ret == MQTT_OK ) {
					now_state = RECV_FROM_SERVER;
				}
				else {
					now_state = WAIT_WIFI_OK;
				}
            }
            break;
            
            case RECV_FROM_SERVER:
            {
            	ret = mqtt_recv(MQTT_PING_INTERVAL);
            	if( ret == MQTT_READ_TIMEOUT ) {
					now_state = SEND_HEARTBEAT;
            	}
				else if( ret == MQTT_ERR ) {
					now_state = WAIT_WIFI_OK;
				}
				else if( ret == MQTT_OK ) {
					now_state = RECV_FROM_SERVER;
				}
            }
            break;
            
            case SEND_HEARTBEAT:
            {
				ret = mqtt_send_heartbeat();
				if( ret == -1 ) {
					now_state = WAIT_WIFI_OK;
				}
				else {
                	now_state = RECV_FROM_SERVER;
				}
            }
            break;
            
            default:
            break;          
        }    
	}
}

static void setAutoConnectMode(void)
{
    u8 auto_reconnect = 0xff;

    tls_wifi_auto_connect_flag(WIFI_AUTO_CNT_FLAG_GET, &auto_reconnect);
    if(auto_reconnect != WIFI_AUTO_CNT_ON)
    {
    	auto_reconnect = WIFI_AUTO_CNT_ON;
    	tls_wifi_auto_connect_flag(WIFI_AUTO_CNT_FLAG_SET, &auto_reconnect);
    }
}

static int demoMqttTaskCreate( void )
{
	setAutoConnectMode();
    tls_os_task_create(NULL, "mqttHandleTask",
            mqttHandleTask,
                    (void *)NULL,
                    (void *)DemoMqttRecvStk, 
                    MQTT_TASK_SIZE * sizeof(u32),
                    MQTT_RECV_TASK_PRIO,
                    0);
    return 0;
}

int mqttDemoTest(void)
{
	demoMqttTaskCreate();
    return 0;
}


