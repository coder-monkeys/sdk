
/*
*************************************************************
 what : tcp client state machine
 who  : zhangwl
 when : 20190803
 where: shenzhen
*************************************************************
*/
#include "wm_include.h"
#include "HTTPClientWrapper.h"
#include "wm_cmdp.h"

#define MQTT_BUFF_SIZE         1024
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
#define MQTT_USE_SSL           (0 && TLS_CONFIG_HTTP_CLIENT_SECURE)
#define MQTT_SERVER_NAME       "47.107.240.244"
#define MQTT_SERVER_PORT       8588
#define HEATBAT_XML            "<?xml version=\"1.0\" encoding=\"UTF-8\"?><REQ><devId>\"%s\"</devId></REQ"
#define DEVICE_ID              "WL0000000022"
#define MQTT_RECV_TASK_PRIO    31
#define MQTT_PING_INTERVAL     60
#define BEBUG_BYTES            1
#define NET_STATE_LED          WM_IO_PB_18
#define DEMO_CTRL_LED          WM_IO_PB_17
#define DEFAUTL_SSID           "6666"
#define DEFAUTL_PWD            "12345678"
#define DEVICE_ID_ADDR         0xF0000

#define LOGIN_INFO             0x40
#define HEART_BEAT_INFO        0x4d
#define CONTROL_INFO           0x4c

#define WIFI_RECON_INTERVAL    (HZ/4)
#define WIFI_RESET_TIMEOUT     (120*HZ)

typedef enum _mqtt_state{
    WAIT_WIFI_OK = 0,
    CONNECT_SERVER,
    RECV_FROM_SERVER,
    SEND_HEARTBEAT,
} mqtt_state;

typedef struct {
	int socketid;
	char packet_buffer[MQTT_BUFF_SIZE];
	int (*mqttsend)(int socket_info, const void* buf, unsigned int count);
} mqtt_broker_cut_t;

static OS_STK DemoMqttRecvStk[MQTT_TASK_SIZE];
static mqtt_broker_cut_t mqtt_broker;
static tls_ssl_t *ssl;

static int mqtt_send(int socket, unsigned char *buf, unsigned int count)
{
    int fd = socket;

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

static void mqtt_close(int socket)
{
#if MQTT_USE_SSL
	closesocket(socket);
    HTTPWrapperSSLClose(ssl, socket);
#else
	closesocket(socket);
#endif
}

/* Socket safe API,do not need to close socket or ssl session if this fucntion return MQTT_ERR; */
static int init_socket(mqtt_broker_cut_t *broker, const char *hostname, unsigned short port)
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
	ret = connect(socket_id, (struct sockaddr*)&socket_addr, sizeof(socket_addr));
#endif
    if(ret < 0)
    {
        mqtt_debug("mqtt connect failed = %d\n", ret);
        mqtt_close(broker->socketid);
        return MQTT_ERR;
    }
	broker->socketid = socket_id;
																														
    return MQTT_OK;
}

/* Socket safe API,do not need to close socket or ssl session if this fucntion return MQTT_ERR; */
static int read_packet(int timeout_s, mqtt_broker_cut_t *broker)
{
    int ret = 0;
    fd_set readfds;
    struct  timeval tmv = {.tv_sec = timeout_s, .tv_usec = 0};
    int total_bytes = 0, byte_rcvd = 0;

    if(timeout_s <= 0) {
        return MQTT_READ_TIMEOUT;
    }

    FD_ZERO(&readfds);
    FD_SET(broker->socketid, &readfds);

    ret = select(broker->socketid + 1, &readfds, NULL, NULL, &tmv);
    if(ret < 0) {
		mqtt_close(broker->socketid);
        return MQTT_ERR;
    }
    else if(ret == 0) {
        return MQTT_READ_TIMEOUT;
    }

    memset(broker->packet_buffer, 0, sizeof(broker->packet_buffer));
#if MQTT_USE_SSL
    byte_rcvd = HTTPWrapperSSLRecv(ssl, broker->socketid, broker->packet_buffer, MQTT_BUFF_SIZE, 0);
#else
	byte_rcvd = recv(broker->socketid, broker->packet_buffer, MQTT_BUFF_SIZE, 0);
#endif
    if(byte_rcvd <= 0) {
		mqtt_debug("# byte_rcvd:%d\r\n", byte_rcvd);
		mqtt_close(broker->socketid);
        return MQTT_ERR;
    }
    total_bytes += byte_rcvd;

    return total_bytes;
}

/* Socket safe API,do not need to close socket or ssl session if this fucntion return MQTT_ERR; */
static int mqtt_send_heartbeat(mqtt_broker_cut_t *broker, u8 login_or_hb)
{
	int nBufSize = 0, ret = -1;
	u8 sendBuf[128] = { 0 };

	sprintf(sendBuf+5, HEATBAT_XML, DEVICE_ID);
	nBufSize = strlen( sendBuf + 5);
	u8 head[5] = {0x40,0x40,login_or_hb,nBufSize>>8,nBufSize};
	memcpy( sendBuf, head, sizeof(head));
	nBufSize += sizeof(head);
	mqtt_debug("send %x:%d\r\n", login_or_hb,nBufSize);
	ret = mqtt_send( broker->socketid, sendBuf, nBufSize);
	if( ret < 0 ) {
		mqtt_close(broker->socketid);
	}
	return ret;
}

/* Socket safe API,do not need to close socket or ssl session if this fucntion return MQTT_ERR; */
static int mqtt_open(mqtt_broker_cut_t *broker)
{
    int err = 0, packet_len = 0;
    unsigned short msg_id = 0, msg_id_rcv = 0;

    memset(broker->packet_buffer, 0, MQTT_BUFF_SIZE);
    err = init_socket(broker, MQTT_SERVER_NAME, MQTT_SERVER_PORT);
    if(err != 0)
    {
        mqtt_debug("mqtt init socket faild\n");
        return MQTT_ERR;
    }
    mqtt_debug("mqtt connect success\n");
    
    return MQTT_OK;
}

static int findPointedString(char *inputData, int inputLen, int offset, char *outputStr)
{
	int colonNum = 0;
	int Index = 0;
	
	for( int i=0; i<inputLen; i++ )
	{
		if( inputData[i] = ':' ) {
			colonNum ++;
		}
		if( colonNum==offset )
		{
			outputStr[Index] = inputData[i+1];
			Index ++;
		}
		if( colonNum>offset ) {
			break;
		}
	}
	return 0;
}

static int local_connect_net(char *ssid, char *pwd)
{
    struct tls_param_ip *ip_param = NULL;
    u8 wireless_protocol = 0;

    tls_wifi_disconnect();
    tls_param_get(TLS_PARAM_ID_WPROTOCOL, (void *) &wireless_protocol, TRUE);
    if (TLS_PARAM_IEEE80211_INFRA != wireless_protocol)
    {
        tls_wifi_softap_destroy();
        wireless_protocol = TLS_PARAM_IEEE80211_INFRA;
        tls_param_set(TLS_PARAM_ID_WPROTOCOL, (void *) &wireless_protocol, FALSE);
    }
    tls_wifi_set_oneshot_flag(0);
    ip_param = tls_mem_alloc(sizeof(struct tls_param_ip));
    if (ip_param)
    {
        tls_param_get(TLS_PARAM_ID_IP, ip_param, FALSE);
        ip_param->dhcp_enable = TRUE;
        tls_param_set(TLS_PARAM_ID_IP, ip_param, FALSE);
        tls_mem_free(ip_param);
    }
    tls_wifi_connect((u8 *)ssid, strlen(ssid), (u8 *)pwd, strlen(pwd));

    return WM_SUCCESS;
}

static int parseReceivedData(uint8_t *data, uint16_t len)
{
	int mid = 0;
#if BEBUG_BYTES
	mqtt_debug("recv %d:\r\n", len);
	for(int i=0; i<len; i++) {
		mqtt_debug("%x ", *(data+i));
	}
	mqtt_debug("\r\n");
#endif
	mqtt_debug("cmd:%X\r\n", data[0]);
	switch(data[0])
	{
		case 0xa1:
			{
				char ssid[32] = { 0 };
				char key[64] = { 0 };
				
				findPointedString(data, len, 2, ssid);
				findPointedString(data, len, 3, key);
				local_connect_net(ssid, key);
				mqtt_debug("conn:%s,pwd:%s\r\n", ssid, key);
			}
			break;
		case 0xb1:
			{
				tls_gpio_write(DEMO_CTRL_LED, 1);
			}
			break;
		case 0xb0:
			{
				tls_gpio_write(DEMO_CTRL_LED, 0);
			}
			break;
		case 0xc1:
			{
				//t_http_fwup();
			}
			break;
		case 0xd1:
			{
				int level = 0;
				
				level = tls_gpio_read(DEMO_CTRL_LED);
				mqtt_debug("level:%d\r\n", level);
			}
			break;
		default:
			break;
	}

	return mid;
}

/* Socket safe API,do not need to close socket or ssl session if this fucntion return MQTT_ERR; */
static int mqtt_recv(int selectTimeOut, mqtt_broker_cut_t *broker)
{
    int packet_len = 0, ret = 0;

    packet_len = read_packet(selectTimeOut, broker);
    if(packet_len == MQTT_READ_TIMEOUT)
    {
        return MQTT_READ_TIMEOUT;
    }
    else if(packet_len > 0)
    {
		parseReceivedData(broker->packet_buffer, packet_len);
    }
    else if(packet_len == -1)
    {
    	mqtt_debug("packet_len:%d\n", packet_len);
        return MQTT_ERR;
    }
}

/* return 1 if ip is gotten */
static int isNetOk(void)
{
	struct tls_ethif* etherIf= tls_netif_get_ethif();

	return etherIf->status;
}

static int WifiConfigReset(char *ssid, char *key)
{
	struct tls_cmd_ssid_t ssid_f[1] = { 0 };
	struct tls_cmd_key_t  key_f[1] = { 0 };

    ssid_f->ssid_len = strlen(ssid);
    MEMCPY(ssid_f->ssid, ssid, ssid_f->ssid_len);
    tls_cmd_set_ssid(ssid_f, 1);

    memcpy(key_f->key, key, strlen(key));
    key_f->key_len = strlen(key);
    tls_cmd_set_key(key_f, 1);
	mqtt_debug("ssid:%d %s, key:%d %s\n", ssid_f->ssid_len, ssid_f->ssid, key_f->key_len,key_f->key);
	return 0;
}

static void mqttHandleTask( void* lparam )
{
	int ret = MQTT_ERR;
	int connWifiCount = 0;
	mqtt_state now_state = WAIT_WIFI_OK;

	while (1) 
    {
        //mqtt_debug("now_state: %d\n", now_state);
        switch(now_state)
        {
            case WAIT_WIFI_OK:
            {
				ret = isNetOk();
                if( ret ) {
					mqtt_debug("\n");
					now_state = CONNECT_SERVER;
					connWifiCount = 0;
					tls_os_time_delay(HZ);
                }
                else {
					static u8 level = 1;
					
                    mqtt_debug(".");
					now_state = WAIT_WIFI_OK;
                    tls_os_time_delay(WIFI_RECON_INTERVAL);
					connWifiCount ++;
					if( connWifiCount> WIFI_RESET_TIMEOUT/WIFI_RECON_INTERVAL)
					{
						connWifiCount = 0;
						WifiConfigReset(DEFAUTL_SSID, DEFAUTL_PWD);
					}
					(level ==1)?(level = 0):(level = 1);
					tls_gpio_write(NET_STATE_LED, level);
                }
            }
            break;
            
            case CONNECT_SERVER:
            {
            	ret = mqtt_open( &mqtt_broker );
				if(ret == MQTT_OK ) {
					tls_os_time_delay(HZ);
					ret = mqtt_send_heartbeat(&mqtt_broker, LOGIN_INFO);
					if( ret <= 0 ) {
						now_state = WAIT_WIFI_OK;
					}
					else {
	                	now_state = RECV_FROM_SERVER;
					}
				}
				else {
					now_state = WAIT_WIFI_OK;
				}
            }
            break;
            
            case RECV_FROM_SERVER:
            {
            	ret = mqtt_recv(MQTT_PING_INTERVAL, &mqtt_broker);
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
				ret = mqtt_send_heartbeat(&mqtt_broker, HEART_BEAT_INFO);
				if( ret <= 0 ) {
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

static void autoConnect(void)
{
    u8 auto_reconnect = 0xff;

    tls_wifi_auto_connect_flag(WIFI_AUTO_CNT_FLAG_GET, &auto_reconnect);
    if(auto_reconnect != WIFI_AUTO_CNT_ON)
    {
    	auto_reconnect = WIFI_AUTO_CNT_ON;
    	tls_wifi_auto_connect_flag(WIFI_AUTO_CNT_FLAG_SET, &auto_reconnect);
    }
}

static int wifiConfigInitial(void)
{
	struct tls_cmd_ssid_t ssid[1] = { 0 };
	struct tls_cmd_key_t  key[1] = { 0 };
	
	tls_cmd_get_ssid(ssid);
	tls_cmd_get_key(key);
	mqtt_debug("ssid:%d %s, key:%d %s\n", ssid->ssid_len, ssid->ssid, key->key_len,key->key);

    memset(&ssid, 0, sizeof(struct tls_cmd_ssid_t));
    if(ssid->ssid_len==0 && key->key_len==0 ) 
	{
		WifiConfigReset(DEFAUTL_SSID, DEFAUTL_PWD);
    }
	autoConnect();

	return 0;
}

static int gpioAndIdInitial(void)
{
	u8 dev_id[16] = { 0 };
	
	tls_gpio_cfg(NET_STATE_LED, WM_GPIO_DIR_OUTPUT, WM_GPIO_ATTR_PULLHIGH);
	tls_gpio_cfg(DEMO_CTRL_LED, WM_GPIO_DIR_OUTPUT, WM_GPIO_ATTR_PULLHIGH);

	
	tls_fls_read(DEVICE_ID_ADDR, dev_id, sizeof(dev_id));
	mqtt_debug("id: %s\n", dev_id);
	return 0;
}

static int demoMqttTaskCreate( void )
{
    tls_os_task_create(NULL, "mqttHandleTask",
            mqttHandleTask,
                    (void *)NULL,
                    (void *)DemoMqttRecvStk, 
                    MQTT_TASK_SIZE * sizeof(u32),
                    MQTT_RECV_TASK_PRIO,
                    0);
    return 0;
}

int tcpClientExample(void)
{
	wifiConfigInitial();
	gpioAndIdInitial();
	demoMqttTaskCreate();
    return 0;
}

void UserMain(void)
{	
	printf("application start.\n");
	tcpClientExample();
}

