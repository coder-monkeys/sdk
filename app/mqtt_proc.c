#include "wm_include.h"
#include "libemqtt.h"
#include "HTTPClientWrapper.h"
#include <time.h>
#include "wm_rtc.h"
#include "wm_ntp.h"
#include "cJSON.h"

#define MQTT_BUFF_SIZE         1024
#define MQTT_PUBACK_SIZE       1024
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
#define DEVICE_ID              "21d081dd-2762-4ad2-9715-3120be5cc89b"
#define PASSWORD               "89f69ba0bccfd0fd77da"
#define MQTT_CLIENT_ID         (DEVICE_ID##"_0_1_%s")
#define MQTT_USER_NAME         DEVICE_ID
#define SUB_TOPIC              "/huawei/v1/devices/"##DEVICE_ID##"/command/json"
#define PUB_TOPIC              "/huawei/v1/devices/"##DEVICE_ID##"/data/json"
#define MQTT_RECV_TASK_PRIO    63
#define MQTT_PING_INTERVAL     45
#define BEBUG_BYTES            0
#define DEBUG_PUBLISH          1

typedef struct _mqtt_para{
	char clientid[64+8];
	char username[64];
	char userkey[64+8];
	char timestamp[16];
} mqtt_para;

typedef enum _mqtt_state{
    WAIT_WIFI_OK = 0,
    CONNECT_SERVER,
    RECV_FROM_SERVER,
    SEND_HEARTBEAT,
} mqtt_state;

static mqtt_broker_handle_t mqtt_broker;
static tls_ssl_t *ssl;
static char packet_buffer[MQTT_BUFF_SIZE] = { 0 };
static OS_STK DemoMqttRecvStk[MQTT_TASK_SIZE];
static mqtt_para loginPara = { 0 };

static void getConnectKey(u8 *hash)
{
	u32 keyLen = 0;
	u32 psdLen = strlen(PASSWORD);
	unsigned char hash_hex[32+1] = { 0 };
	struct tm *tblock;
	
	u32 t = tls_ntp_client();
    mqtt_debug("Time: %s\r\n", ctime(&t));
    tblock=localtime(&t);	//把日历时间转换成本地时间，已经加上与世界时间8小时的偏差,以1900为基准
	sprintf(loginPara.timestamp, "%04d%02d%02d%02d", tblock->tm_year+1900, tblock->tm_mon+1, tblock->tm_mday, tblock->tm_hour);
	keyLen = strlen(loginPara.timestamp);
    tls_set_rtc(tblock);
	psHmacSha2(loginPara.timestamp, keyLen, PASSWORD, psdLen, hash_hex, loginPara.timestamp, &keyLen, SHA256_HASH_SIZE);
	for(int i=0, j=0; i<SHA256_HASH_SIZE; i+=1, j+=2) {
	    sprintf((hash+j), "%02x", *(hash_hex+i));
	}
	mqtt_debug("hash: %s\r\n", hash);
}

static int mqtt_send(int socket_info, unsigned char *buf, unsigned int count)
{
    int fd = socket_info;

#if BEBUG_BYTES
	mqtt_debug("mqtt_send %d:\r\n", count);
	for(int i=0; i<count; i++) {
		mqtt_debug("%x ", *(buf+i));
	}
	mqtt_debug("\r\n");
#endif
#if MQTT_USE_SSL
    return HTTPWrapperSSLSend(ssl, fd, buf, count, 0);
#else
	return send(fd, buf, count, 0);
#endif
}

static void mqtt_close(void)
{
#if MQTT_USE_SSL
	mqtt_disconnect(&mqtt_broker);
	closesocket(mqtt_broker.socketid);
    HTTPWrapperSSLClose(ssl, mqtt_broker.socketid);
#else
	mqtt_disconnect(&mqtt_broker);
	closesocket(mqtt_broker.socketid);
#endif
	mqtt_broker.socketid = -1;
}

static int connect_nonblock(int sockfd, const struct sockaddr_in *saptr, socklen_t salen, int nsec)
{
    int flags, n, error, code;
    socklen_t len;
    fd_set wset;
    struct timeval tval;

    flags = fcntl(sockfd, F_GETFL, 0);
    fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);

    error = 0;
    if ((n = connect(sockfd, saptr, salen)) == 0) {
        goto done;
    } else if (n < 0 && errno != EINPROGRESS){
        return (-1);
    }
    /* Do whatever we want while the connect is taking place */
    FD_ZERO(&wset);
    FD_SET(sockfd, &wset);
    tval.tv_sec = nsec;
    tval.tv_usec = 0;

    if ((n = select(sockfd+1, NULL, &wset, 
                    NULL, nsec ? &tval : NULL)) == 0) {
        close(sockfd);  /* timeout */
        errno = ETIMEDOUT;
        return (-1);
    }

    if (FD_ISSET(sockfd, &wset)) {
        len = sizeof(error);
        code = getsockopt(sockfd, SOL_SOCKET, SO_ERROR, &error, &len);
        /* 
        Èç¹û·¢Éú´íÎó£¬SolarisÊµÏÖµÄgetsockopt·µ»Ø-1,°Ñpending errorÉèÖÃ¸øerrno. 
        BerkeleyÊµÏÖµÄgetsockopt·µ»Ø0, pending error·µ»Ø¸øerror.ÎÒÃÇÐèÒª´¦ÀíÕâÁ½ÖÖÇé¿ö 
        */
        if (code < 0 || error) {
            close(sockfd);
            if (error) 
                errno = error;
            return (-1);
        }
    } else {
        mqtt_debug("select error: sockfd not set\r\n");
		return (-1);
    }

done:
	/* restore file status flags */
    fcntl(sockfd, F_SETFL, flags);
    return (0);
}

/* Socket safe API,do not need to close socket or ssl session if this fucntion return MQTT_ERR; */
static int init_socket(mqtt_broker_cut_t *broker, const char *hostname, unsigned short port)
{
    int socket_id = -1, ret = -1;
    struct sockaddr_in socket_addr;
    
    socket_id = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	broker->socketid = socket_id;
    if(socket_id < 0) {
		mqtt_debug("socket() error\n");
        return MQTT_ERR;
    }
    memset(&socket_addr, 0, sizeof(struct sockaddr_in));
    socket_addr.sin_family = AF_INET;
    socket_addr.sin_port = htons(port);
    socket_addr.sin_addr.s_addr = ipaddr_addr(hostname);
	mqtt_debug("mqtt connect ing\n");
#if MQTT_USE_SSL
    ret = HTTPWrapperSSLConnect(&ssl, socket_id, (struct sockaddr *)&socket_addr, sizeof(socket_addr), NULL);
#else
	/* Â·ÓÉÉÏµç£¬ÎÞÍâÍø£¬´Ëº¯Êý´ó¸Å18Ãë×óÓÒ»á·µ»ØÊ§°Ü£»Â·ÓÉ¶Ïµç£¬´Ëº¯Êý´ó¸Å²»»á·µ»Ø£¬ÓÐ²âÊÔÁË15·ÖÖÓ¶¼Ã»ÓÐ·µ»Ø */
	//ret = connect(socket_id, (struct sockaddr*)&socket_addr, sizeof(socket_addr));
	ret = connect_nonblock(socket_id, (struct sockaddr*)&socket_addr, sizeof(socket_addr), 5);
#endif
	mqtt_debug("mqtt connect ret= %d\n", ret);
    if(ret < 0)
    {
        mqtt_close(&(broker->socketid));
        return MQTT_ERR;
    }
		
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
		mqtt_debug("1 byte_rcvd:%d\r\n", byte_rcvd);
		mqtt_close();
        return MQTT_ERR;
    }

    total_bytes += byte_rcvd;
    if(total_bytes < 2) {
		mqtt_debug("2 byte_rcvd:%d\r\n", byte_rcvd);
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
			mqtt_debug("3 byte_rcvd:%d\r\n", byte_rcvd);
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
    packet_len = read_packet(10);
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
    unsigned short msg_id = 0, msg_id_rcv = 0;

    memset(packet_buffer, 0, MQTT_BUFF_SIZE);
	getConnectKey(loginPara.userkey);
	sprintf(loginPara.clientid, MQTT_CLIENT_ID, loginPara.timestamp);
    mqtt_init(&mqtt_broker, loginPara.clientid);
	mqtt_debug("clientid: %s\n", loginPara.clientid);
    mqtt_init_auth(&mqtt_broker, MQTT_USER_NAME, loginPara.userkey);
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
#if BEBUG_BYTES
	mqtt_debug("recv:%d\n", packet_len);
	for(int i=0; i<packet_len; i++) {
		mqtt_debug("%x ", packet_buffer[i]);
	}
	mqtt_debug("\n");
#endif
    if(MQTTParseMessageType(packet_buffer) != MQTT_MSG_CONNACK || packet_buffer[3] != 0x00)
    {
        mqtt_debug("CONNACK expected or failed!\n");
        mqtt_close();
        return MQTT_ERR;
    }
    
    if(subscribe_topic(SUB_TOPIC) != 0) {
        return MQTT_ERR;
    }
    mqtt_debug("mqtt connect & subscribe topic success\n");
    
    return MQTT_OK;
}

static int parseReceivedData(uint8_t *data)
{
	cJSON *json = NULL;
	cJSON *msgType, *cmd;
	int mid = -1;
	
	json = cJSON_Parse(data);
	if(json)
	{
		cmd = cJSON_GetObjectItem(json, "cmd");
		msgType = cJSON_GetObjectItem(json, "msgType");
		mid = cJSON_GetObjectItem(json, "mid")->valueint;
		mqtt_debug("cmd:%s\r\n", cmd->valuestring);
		mqtt_debug("msgType:%s\r\n", msgType->valuestring);
		cJSON_Delete(json); 
	}
	else
	{
		mqtt_debug("parse json error!\r\n");
	}
	return mid;
}

static int packPublishAck(u32 mid, char *jsonBuffer)
{
	cJSON *jsRet = NULL;
	cJSON *jsBody = NULL;
	int ackLen = 0;
	
	jsRet = cJSON_CreateObject();
	if(jsRet)
	{
		cJSON_AddStringToObject(jsRet, "msgType", "deviceRsp");
		cJSON_AddNumberToObject(jsRet, "mid", mid);
		cJSON_AddNumberToObject(jsRet, "errcode", 0);
		cJSON_AddNumberToObject(jsRet, "hasMore", 0);
		jsBody = cJSON_CreateObject();
		if( jsBody )
		{
			cJSON_AddNumberToObject(jsBody, "result", 0);
			cJSON_AddItemToObject(jsRet, "body", jsBody);
			char *databuf = cJSON_PrintUnformatted(jsRet);
			if(databuf) {
				mqtt_debug("json:%s\r\n", databuf);
				if( jsonBuffer ) {
					ackLen = strlen(databuf);
					memcpy( jsonBuffer, databuf, ackLen );
				}
				tls_mem_free(databuf);
			}
		}
		cJSON_Delete(jsRet); 
	}
	return ackLen;
}


static int packPublishReq(char *jsonBuffer)
{
	cJSON *jsRet = NULL;
	cJSON *jsArray = NULL;
	int ackLen = 0;
	
	jsRet = cJSON_CreateObject();
	if(jsRet)
	{
		cJSON_AddStringToObject(jsRet, "msgType", "deviceReq");
		cJSON_AddNumberToObject(jsRet, "hasMore", 0);
		jsArray = cJSON_CreateArray();
		cJSON_AddItemToObject(jsRet, "data", jsArray);
		{
			cJSON *arrayObj_1 = cJSON_CreateObject();
			cJSON_AddItemToArray(jsArray, arrayObj_1);
			cJSON_AddStringToObject(arrayObj_1, "serviceId", "CtrlDevice");

			cJSON *arrayObj_1_0 = cJSON_CreateObject();
			cJSON_AddItemToObject(arrayObj_1, "serviceData", arrayObj_1_0);
			cJSON_AddStringToObject(arrayObj_1_0, "cellId", "555");
		}
		char *databuf = cJSON_PrintUnformatted(jsRet);
		if(databuf) {
			mqtt_debug("json: %s\r\n", databuf);
			if( jsonBuffer ) {
				ackLen = strlen(databuf);
				memcpy( jsonBuffer, databuf, ackLen );
			}
			tls_mem_free(databuf);
		}
		cJSON_Delete(jsRet); 
	}
	return ackLen;
}

static int publish_topic(void)
{
	char *ackBuffer = NULL;
	int ackLen = 0;

	ackBuffer = tls_mem_alloc(MQTT_PUBACK_SIZE);
	if( ackBuffer )
	{
		memset(ackBuffer, 0, MQTT_PUBACK_SIZE);
		ackLen = packPublishReq(ackBuffer);
		mqtt_publish(&mqtt_broker, PUB_TOPIC, ackBuffer, ackLen, 0);
		tls_mem_free(ackBuffer);
	}
	return ackLen;
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
			uint8_t topic[64+16] = { 0 }, *msg;
			uint16_t len;
			int32_t midValue = 0, ackLen = 0;
			char *ackBuffer = NULL;
			
			len = mqtt_parse_pub_topic(packet_buffer, topic);
			topic[len] = '\0';
			len = mqtt_parse_publish_msg(packet_buffer, &msg);
			msg[len] = '\0';
			mqtt_debug("#topic: %s\r\n#msg: %s\r\n", topic, msg);
			midValue = parseReceivedData(msg);
			if( midValue != -1 ) {
				ackBuffer = tls_mem_alloc(MQTT_PUBACK_SIZE);
				if( ackBuffer )
				{
					memset(ackBuffer, 0, MQTT_PUBACK_SIZE);
					ackLen = packPublishAck(midValue, ackBuffer);
					mqtt_publish(&mqtt_broker, PUB_TOPIC, ackBuffer, ackLen, 0);
					tls_mem_free(ackBuffer);
				}
			}
			else {
				return MQTT_ERR;
			}
        }
        else if(ret == MQTT_MSG_PINGRESP) {
            mqtt_debug("recv pong\n");
        }
        else {
            mqtt_debug("Packet Header: 0x%x\n", packet_buffer[0]);
        }
		return MQTT_OK;
    }
    else if(packet_len == -1)
    {
    	mqtt_debug("packet_len:%d\n", packet_len);
        return MQTT_ERR;
    }
}

static int isWifiNetworkOk(void)
{
	struct tls_ethif* etherIf= tls_netif_get_ethif();

	return (WM_WIFI_JOINED == tls_wifi_get_state() && etherIf!=NULL && *((u32*)&etherIf->ip_addr)!=0);
}

static void mqttHandleTask( void* lparam )
{
	int ret = MQTT_ERR;
	mqtt_state now_state = WAIT_WIFI_OK;

	while (1) 
    {
        //mqtt_debug("now_state: %d\n", now_state);
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
#if DEBUG_PUBLISH
				publish_topic();
#endif
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


