#include "wm_include.h"
#include "mqtt_proc.h"
#include "libemqtt.h"
#include "HTTPClientWrapper.h"
//#include "net_status_proc.h"

static mqtt_broker_handle_t mqtt_broker;
_mqtt_param mqtt_param;
static tls_ssl_t *ssl;
char device_id[32] = {0};
char *packet_buffer = NULL;

tls_os_timer_t *hearttimer = NULL;

static int send_packet(int socket_info, const void *buf, unsigned int count)
{
    int fd = socket_info;

    return HTTPWrapperSSLSend(ssl, fd, buf, count, 0);
}

int init_socket(mqtt_broker_handle_t *broker, const char *hostname, unsigned short port, int keepalive)
{
    int socket_id = -1, ret = -1;
    struct sockaddr_in socket_addr;
    
    socket_id = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if(socket_id < 0)
        return -1;
    
    memset(&socket_addr, 0, sizeof(struct sockaddr_in));
    socket_addr.sin_family = AF_INET;
    socket_addr.sin_port = htons(port);
    socket_addr.sin_addr.s_addr = ipaddr_addr(hostname);

    ret = HTTPWrapperSSLConnect(&ssl, socket_id, (struct sockaddr *)&socket_addr, sizeof(socket_addr), NULL);
    if(ret < 0)
    {
        printf("mqtt connect failed = %d\n", ret);
        HTTPWrapperSSLClose(ssl, socket_id);
        return -1;
    }
    mqtt_set_alive(broker, keepalive);
    broker->socketid = socket_id;
    broker->mqttsend = send_packet;

    return 0;
}

int read_packet(int timeout)
{
    int ret = 0;
    fd_set readfds;
    struct  timeval tmv;
    int total_bytes = 0, byte_rcvd = 0, packet_len = 0;

    if(timeout <= 0)
        return MQTT_READ_TIMEOUT;

    FD_ZERO(&readfds);
    FD_SET(mqtt_broker.socketid, &readfds);

    tmv.tv_sec = timeout;
    tmv.tv_usec = 0;

    ret = select(mqtt_broker.socketid + 1, &readfds, NULL, NULL, &tmv);
    if(ret < 0)
        return -1;
    else if(ret == 0)
        return MQTT_READ_TIMEOUT;

    memset(packet_buffer, 0, sizeof(packet_buffer));
    byte_rcvd = HTTPWrapperSSLRecv(ssl, mqtt_broker.socketid, packet_buffer, MQTT_BUFF_SIZE, 0);
    if(byte_rcvd <= 0)
        return -1;

    total_bytes += byte_rcvd;
    if(total_bytes < 2)
        return -1;

    unsigned short rem_len = mqtt_parse_rem_len(packet_buffer);
    unsigned char rem_len_byte = mqtt_num_rem_len_bytes(packet_buffer);

    packet_len = rem_len + rem_len_byte + 1;
    while(total_bytes < packet_len)
    {
        byte_rcvd = HTTPWrapperSSLRecv(ssl, mqtt_broker.socketid, (packet_buffer + total_bytes), MQTT_BUFF_SIZE, 0);
        if(byte_rcvd < 0)
            return -1;
        total_bytes += byte_rcvd;
    }

    return packet_len;
}

int subscribe_topic(char *topic)
{
    unsigned short msg_id = 0, msg_id_rcv = 0;
    int packet_len = 0;

    if(topic == NULL)
        return -1;
    
    mqtt_subscribe(&mqtt_broker, topic, &msg_id);
    packet_len = read_packet(1);
    if(packet_len < 0)
    {
        printf("error (%d) on read packet!\n", packet_len);
        return -1;
    }

    if(MQTTParseMessageType(packet_buffer) != MQTT_MSG_SUBACK)
    {
        printf("SUBACK expected!\n");
        HTTPWrapperSSLClose(ssl, mqtt_broker.socketid);
        return -1;
    }

    msg_id_rcv = mqtt_parse_msg_id(packet_buffer);
	if(msg_id != msg_id_rcv)
	{
		printf("%d message id was expected, but %d message id was found!\n", msg_id, msg_id_rcv);
        HTTPWrapperSSLClose(ssl, mqtt_broker.socketid);
        return -1;
	}

    return 0;
}

void test(void)
{
    int packet_length, ret=0;
    
    packet_length = read_packet(10);
    printf("packet_length = %d\n", packet_length);

    printf("Packet Header: 0x%x...\n", packet_buffer[0]);
	if(MQTTParseMessageType(packet_buffer) == MQTT_MSG_PUBLISH)
	{
		uint8_t topic[20], *msg;
		uint16_t len;
		len = mqtt_parse_pub_topic(packet_buffer, topic);
		topic[len] = '\0'; // for printf
		len = mqtt_parse_publish_msg(packet_buffer, &msg);
		msg[len] = '\0'; // for printf
		printf("%s %s\n", topic, msg);
	}

}

static void mqtt_timer_cb(void *ptmr, void *parg)
{
    //mqtt_send_heart_msg();
}

void mqtt_send_heart(void)
{
    printf("mqtt send ping\n");
    mqtt_ping(&mqtt_broker);
}

static void mqtt_close(void)
{
    tls_os_timer_stop(hearttimer);
    HTTPWrapperSSLClose(ssl, mqtt_broker.socketid);
}

int mqtt_open(void)
{
    int ret = -1, err = 0, packet_len = 0;
    _mqtt_param *p = &mqtt_param;
    char *sub_topic = NULL;
    unsigned short msg_id = 0, msg_id_rcv = 0;

    if(packet_buffer == NULL)
    {
        packet_buffer = tls_mem_alloc(MQTT_BUFF_SIZE);
        if(packet_buffer == NULL)
        {
            printf("mqtt rcv buf malloc error\n");
            return -1;
        }
    }
    memset(packet_buffer, 0, MQTT_BUFF_SIZE);

    if(hearttimer == NULL)
    {
        printf("mqtt_param.keepalive = %d\n", mqtt_param.keepalive);
        tls_os_timer_create(&hearttimer, 
            mqtt_timer_cb, 
            NULL, 
            mqtt_param.keepalive * HZ, 
            TRUE, 
            NULL);
    }
    
    mqtt_init(&mqtt_broker, p->clientid);
    mqtt_init_auth(&mqtt_broker, p->username, p->password);

    err = init_socket(&mqtt_broker, p->ip, p->port, p->keepalive);
    if(err != 0)
    {
        printf("mqtt init socket faild\n");
        goto out;
    }

    mqtt_connect(&mqtt_broker);

    packet_len = read_packet(10);
    if(packet_len < 0)
    {
        printf("error (%d) on read packet!\n", packet_len);
        goto out;
    }
    if(MQTTParseMessageType(packet_buffer) != MQTT_MSG_CONNACK || packet_buffer[3] != 0x00)
    {
        printf("CONNACK expected or failed!\n");
        HTTPWrapperSSLClose(ssl, mqtt_broker.socketid);
        goto out;
    }
    
    sub_topic = tls_mem_alloc(64);
    if(sub_topic == NULL)
    {
        printf("sub_topic malloc err\n");
        goto out;
    }
    memset(sub_topic, 0, 64);
    sprintf(sub_topic, "/device/{%s}/downward", device_id);
    if(subscribe_topic(sub_topic) != 0)
        goto out;

    memset(sub_topic, 0, 64);
    sprintf(sub_topic, "/device/{%s}/upward", device_id);
    if(subscribe_topic(sub_topic) != 0)
        goto out;

    printf("mqtt connect success\n");
    tls_os_timer_start(hearttimer);

//    printf("Sending ping...\n");
//	mqtt_ping(&mqtt_broker);
//    test();

    ret = 0;
out:
    if(sub_topic)
        tls_mem_free(sub_topic);
    
    return ret;
}


int mqtt_rcv(void)
{
    int packet_len = 0, ret = 0;

    packet_len = read_packet(2);
    if(packet_len == MQTT_READ_TIMEOUT)
    {
        return MQTT_READ_TIMEOUT;
    }
    else if(packet_len > 0)
    {
        ret = MQTTParseMessageType(packet_buffer);
        if(ret == MQTT_MSG_PUBLISH)
        {
            printf("unbind\n");
            //device_unbind();
            mqtt_close();
            return MQTT_UNBIND;
        }
        else if(ret == MQTT_MSG_PINGRESP)
        {
            printf("mqtt rcv pong\n");
            return MQTT_OK;
        }
        else
        {
            printf("Packet Header: 0x%x...\n", packet_buffer[0]);
            return MQTT_OK;
        }
    }
    else if(packet_len == -1)
    {
        mqtt_close();
        return MQTT_ERR;
    }
}



