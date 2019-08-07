
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
#include "wm_internal_flash.h"

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
#define HEARTBEAT_XML          "<?xml version=\"1.0\" encoding=\"UTF-8\"?><REQ><devId>%s</devId></REQ"
#define MQTT_RECV_TASK_PRIO    31
#define MQTT_PING_INTERVAL     60
#define BEBUG_BYTES            0
#define NET_STATE_LED          WM_IO_PA_01
#define DEFAUTL_SSID           "6666"
#define DEFAUTL_PWD            "12345678"
#define DEVICE_ID_ADDR         0xF0000  //32 bytes max
#define SSID_INFO_ADDR         0xF0020  //100 bytes max
#define WIFI_RECON_INTERVAL    (HZ/4)
#define WIFI_RESET_TIMEOUT     (60*HZ)
#define LED_POWER_ON           1
#define LED_POWER_OFF          0

typedef enum _mqtt_state{
    WAIT_WIFI_OK = 0,
    CONNECT_SERVER,
    RECV_FROM_SERVER,
    SEND_HEARTBEAT,
} mqtt_state;

typedef enum _cmd_type{
    HEART_BEAT = 0x4d,
    DEV_LOGIN = 0x40,
    DEV_CTRL = 0x4c,
} cmd_type;

typedef struct {
	int socketid;
	char packet_buffer[MQTT_BUFF_SIZE];
	char dev_id[16];
	int (*mqttsend)(int socket_info, const void* buf, unsigned int count);
} mqtt_broker_cut_t;

typedef struct {
	int flag; //0,never receive a1 command; 1, receive a1 command before;
	char ssid[32];
	char pwd[64];
} currnt_ssid_info_t;

static OS_STK DemoMqttRecvStk[MQTT_TASK_SIZE];
static mqtt_broker_cut_t mqtt_broker;
static currnt_ssid_info_t ssid_info;
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
	if( socket >= 0 )
	{
#if MQTT_USE_SSL
		closesocket(socket);
	    HTTPWrapperSSLClose(ssl, socket);
#else
		closesocket(socket);
#endif
	}
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
static int mqtt_send_info(mqtt_broker_cut_t *broker, cmd_type type, char *data)
{
	int nBufSize = 0, ret = -1;
	u8 sendBuf[256] = { 0 };

	sprintf(sendBuf+5, HEARTBEAT_XML, data);
	nBufSize = strlen( sendBuf + 5);
	u8 head[5] = {0x40,0x40,type,nBufSize>>8,nBufSize};
	memcpy( sendBuf, head, sizeof(head));
	nBufSize += sizeof(head);
	mqtt_debug("send %x:%d\r\n", type, nBufSize);
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
	if( broker->socketid >= 0 ) 
	{
		mqtt_close(broker->socketid);
	}
    err = init_socket(broker, MQTT_SERVER_NAME, MQTT_SERVER_PORT);
    if(err != 0)
    {
        mqtt_debug("mqtt init socket faild\n");
        return MQTT_ERR;
    }
    mqtt_debug("mqtt connect success\n");
    
    return MQTT_OK;
}

static int findColonNumber(char *inputData, int inputLen)
{
	int colonNum = 0;
	
	for( int i=0; i<inputLen; i++ )
	{
		if( inputData[i] == ':' ) {
			colonNum ++;
		}
	}
	return colonNum;
}

static int findPointedString(char *inputData, int inputLen, int offset, char *outputStr)
{
	int colonNum = 0;
	int Index = 0;
	
	for( int i=0; i<inputLen; i++ )
	{
		if( inputData[i] == ':' ) {
			colonNum ++;
		}
		if( colonNum==offset )
		{
			outputStr[Index] = inputData[i+1];
			Index ++;
		}
		if( colonNum>offset ) {
			outputStr[Index-1] = 0x0;
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

static int findGpioIndex(char *gpio)
{
	int gpioNum = 0;
	
	if( gpio[0]=='A' ) 
	{
		gpioNum = WM_IO_PA_00 + (gpio[1]-0x30)*10 + (gpio[2]-0x30);
	}
	else if( gpio[0]=='B' )
	{
		gpioNum = WM_IO_PB_00 + (gpio[1]-0x30)*10 + (gpio[2]-0x30);
	}
	
	return gpioNum;
}

static int updateSsidInfo(currnt_ssid_info_t *info)
{
	currnt_ssid_info_t now_info[1] = { 0 };
	
	tls_fls_read(SSID_INFO_ADDR, now_info, sizeof(currnt_ssid_info_t));
	if( memcmp( now_info->ssid, info->ssid, sizeof(info->ssid))==0 \
		&& memcmp( now_info->pwd, info->pwd, sizeof(info->pwd))==0 \ 
		&& now_info->flag==info->flag )
	{
		mqtt_debug("no need to update! %d %s %s\r\n", now_info->flag, now_info->ssid, now_info->pwd);
	}
	else
	{
		tls_fls_write(SSID_INFO_ADDR, info, sizeof(currnt_ssid_info_t));
	}
	
	return 0;
}

/* return 1 if need to relogin */
static int parseReceivedData(uint8_t *data, uint16_t len)
{
	int mid = 0;

	mqtt_debug("recv %d: %s\r\n", len, data);
#if BEBUG_BYTES
	mqtt_debug("recv %d\r\n", len);
	for(int i=0; i<len; i++) {
		mqtt_debug("%x ", *(data+i));
	}
	mqtt_debug("\r\n");
#endif
	mqtt_debug("cmd:%c%c\r\n", data[0], data[1]);
	switch(data[0])
	{
		case 'A':
			{
				if( data[1]=='1' )
				{
					memset(ssid_info.ssid, 0, sizeof(ssid_info.ssid));
					memset(ssid_info.pwd, 0, sizeof(ssid_info.pwd));
					ssid_info.flag = 1;
					findPointedString(data, len, 2, ssid_info.ssid);
					findPointedString(data, len, 3, ssid_info.pwd);
					local_connect_net(ssid_info.ssid, ssid_info.pwd);
					updateSsidInfo(&ssid_info);
					mqtt_debug("conn:%s,pwd:%s\r\n", ssid_info.ssid, ssid_info.pwd);
					mid = 1;
				}
			}
			break;
		case 'B':
			{
				char colonNum = 0;
				char ack[64] = { 0 };
				
				colonNum = findColonNumber(data, len);
				for( int i=2; i<=colonNum; i++ )
				{
					char gpio[4] = { 0 };
					char gpioNum = 0;

					findPointedString(data, len, i, gpio);
					gpioNum = findGpioIndex(gpio);
					mqtt_debug("gpio:%d\r\n", gpioNum);
					tls_gpio_cfg(gpioNum, WM_GPIO_DIR_OUTPUT, WM_GPIO_ATTR_PULLHIGH);
					if( data[1]=='1' )
					{
						tls_gpio_write(gpioNum, LED_POWER_ON);
					}
					else if( data[1]=='0' )
					{
						tls_gpio_write(gpioNum, LED_POWER_OFF);
					}
				}
				sprintf(ack, "%sOK", data);
				mqtt_send_info(&mqtt_broker, DEV_CTRL, ack);
			}
			break;

		case 'C':
			{
				if( data[1]=='1' )
				{
					t_http_fwup("http://47.107.240.244/wm_w600_gz.img");
				}
			}
			break;
		case 'D':
			{
				if( data[1]=='1' )
				{
					int level = 0;
					char colonNum = 0;
					char ack[64] = { 0 };
					char gpioLevel[32] = { 0 };
					
					colonNum = findColonNumber(data, len);
					for( int i=2; i<=colonNum; i++ )
					{
						char gpio[4] = { 0 };
						char gpioNum = 0;
					
						findPointedString(data, len, i, gpio);
						gpioNum = findGpioIndex(gpio);
						level = tls_gpio_read(gpioNum);
						gpioLevel[i-2] = (0x30+level);
						mqtt_debug("gpio:%d, level:%d\r\n", gpioNum, level);
					}
					sprintf(ack, "%s:%s", data, gpioLevel);
					mqtt_send_info(&mqtt_broker, DEV_CTRL, ack);
				}
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
		ret = parseReceivedData(broker->packet_buffer, packet_len);
		if( ret==1 )
		{
			return MQTT_ERR;
		}
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

/* updateFlash: 0, just update ram; 1, update into flash */
static int WifiConfigReset(char *ssid, char *key, char updateFlash)
{
	struct tls_cmd_ssid_t ssid_f[1] = { 0 };
	struct tls_cmd_key_t  key_f[1] = { 0 };

    ssid_f->ssid_len = strlen(ssid);
    MEMCPY(ssid_f->ssid, ssid, ssid_f->ssid_len);
    tls_cmd_set_ssid(ssid_f, updateFlash);

    memcpy(key_f->key, key, strlen(key));
    key_f->key_len = strlen(key);
    tls_cmd_set_key(key_f, updateFlash);
	return 0;
}

static int wifiConfigInitial(int exchange)
{
	struct tls_cmd_ssid_t ssid[1] = { 0 };
	struct tls_cmd_key_t  key[1] = { 0 };
	
	tls_cmd_get_ssid(ssid);
	tls_cmd_get_key(key);
	mqtt_debug("ssid:%d %s, key:%d %s\n", ssid->ssid_len, ssid->ssid, key->key_len,key->key);

    if(ssid->ssid_len==0 && key->key_len==0 )
	{
		WifiConfigReset(DEFAUTL_SSID, DEFAUTL_PWD, 0);
    }

	if( exchange )
	{
		if(memcmp( ssid->ssid, DEFAUTL_SSID, strlen(DEFAUTL_SSID))==0 && memcmp( key->key, DEFAUTL_PWD, strlen(DEFAUTL_PWD))==0 )
		{
			currnt_ssid_info_t now_info[1] = { 0 };
	
			tls_fls_read(SSID_INFO_ADDR, (u8 *)now_info, sizeof(currnt_ssid_info_t));
			if(now_info->flag ==1) 
			{
				WifiConfigReset(now_info->ssid, now_info->pwd, 0);
				mqtt_debug("ssid exchange, %s\n", now_info->ssid);
			}
	    }
		else
		{
			WifiConfigReset(DEFAUTL_SSID, DEFAUTL_PWD, 0);
		}
	}
	autoConnect();

	return 0;
}

static int wifiConfigSave(void)
{
	struct tls_cmd_ssid_t ssid[1] = { 0 };
	struct tls_cmd_key_t  key[1] = { 0 };
	
	tls_cmd_get_ssid(ssid);
	tls_cmd_get_key(key);
	mqtt_debug("save ssid:%d %s, key:%d %s\n", ssid->ssid_len, ssid->ssid, key->key_len,key->key);

    if(ssid->ssid_len!=0 && key->key_len!=0 )
	{
		WifiConfigReset(ssid->ssid, key->key, 1);
    }
	return 0;
}

static void mqttHandleTask( void* lparam )
{
	int ret = MQTT_ERR;
	int connWifiCount = 0;
	mqtt_state now_state = WAIT_WIFI_OK;
	mqtt_broker.socketid = -1;

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
					wifiConfigSave();
					tls_gpio_write(NET_STATE_LED, LED_POWER_ON);
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
						wifiConfigInitial(1);
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
					ret = mqtt_send_info(&mqtt_broker, DEV_LOGIN, mqtt_broker.dev_id);
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
				ret = mqtt_send_info(&mqtt_broker, HEART_BEAT, mqtt_broker.dev_id);
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

static int gpioAndIdInitial(mqtt_broker_cut_t *broker)
{
	char temp[16] = { 0 };

	memset(temp, 0xff, sizeof(temp));
	memset(broker->dev_id, 0, sizeof(broker->dev_id));
	tls_gpio_cfg(NET_STATE_LED, WM_GPIO_DIR_OUTPUT, WM_GPIO_ATTR_PULLHIGH);

	//AT+&FLSW=80F0000,30304C57,30303030,32323030,0 ------>写入id:WL0000000022
	tls_fls_read(DEVICE_ID_ADDR, broker->dev_id, sizeof(broker->dev_id));
	if( memcmp( broker->dev_id, temp, sizeof(temp) )==0 )
	{
		mqtt_debug("device id is empty, please write first!\n");
	}
	mqtt_debug("id: %s\n", broker->dev_id);
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
	wifiConfigInitial(0);
	gpioAndIdInitial(&mqtt_broker);
	demoMqttTaskCreate();
    return 0;
}

void UserMain(void)
{	
	printf("application start.\n");
	tcpClientExample();
}

