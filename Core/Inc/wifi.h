/*
 * wifi.h
 *
 *  Created on: Oct 18, 2020
 *      Author: Dmitry
 */

#ifndef INC_WIFI_H_
#define INC_WIFI_H_

#define AT_RESP_SIZE							6
#define AT_RESP                                 "\r\nOK\r\n"
#define AT_RESTORE_DEFAULT_SETTING              "AT&F\r"
#define AT_SAVE_CURRENT_SETTING                 "AT&W\r"
#define AT_SET_POWER_STATE                      "AT+CFUN=%d\r"
#define AT_RESET                				"AT+CFUN=0\r"

#define AT_SET_CONFIGURATION_VALUE              "AT+S.SCFG=%s,%d\r"
#define AT_SET_SSID                             "AT+S.SSIDTXT=%s\r"
#define AT_SET_SEC_KEY                          "AT+S.SCFG=wifi_wpa_psk_text,%s\r"

#define AT_PING                                 "AT+S.PING=%s\r"
#define AT_WiFi_SCAN                            "AT+S.SCAN\r"
#define WIFI_PRIV_MODE                          "wifi_priv_mode"
#define WIFI_MODE                               "wifi_mode"

#define AT_SOCKET_OPEN                          "AT+S.SOCKON=%s,%d,%s\r"
#define AT_SOCKET_WRITE                         "AT+S.SOCKW=%d,%d\r"
#define AT_SOCKET_READ                          "AT+S.SOCKR=%d,%d\r"
#define AT_QUERY_PENDING_DATA                   "AT+S.SOCKQ=%d\r"
#define AT_SOCKET_CLOSE                         "AT+S.SOCKC=%d\r"

#define SERVER_OK_RESP							"Server_OK"
#define SERVER_AT_RESP							"Server_AT"
#define SERVER_SV_RESP							"Server_SV"
#define SERVER_DT_RESP							"Server_DT"
#define SERVER_REQUEST							"STM_RQ"
#define SERVER_AUTH 							"STM_AUTH"
#define SERVER_DATA 							"STM_DATA"
#define STM_DATETIME 							"STM_DT"

typedef enum
{
  Active_State,
  PowerSave_State,
  Sleep_State=3,
  StandBy_State=4
} WiFi_Power_State_t;

typedef enum
{
  WiFi_IDLE_MODE =0,
  WiFi_STA_MODE,
  WiFi_IBSS_MODE,
  WiFi_MiniAP_MODE
} WiFi_Mode_TypeDef;

typedef enum
{
  None          = 0,
  WEP           = 1,
  WPA_Personal  = 2,
} WiFi_Priv_Mode;

typedef enum
{
  WiFi_MODULE_SUCCESS           = 0,
  WiFi_TIME_OUT_ERROR           = 1,
  WiFi_SSID_ERROR,
  WiFi_SecKey_ERROR,
  WiFi_CONFIG_ERROR,
  WiFi_HAL_UART_ERROR
} WiFi_Status_t;

typedef enum
{
  Socket_SUCCESS           = 0,
  Socket_TIME_OUT_ERROR    = 1,
  Socket_ERROR,
  Socket_WiFi_ERROR,
  Socket_Invalid_Server_Resp
} Socket_Status_t;

typedef enum
{
  WEB_SOCKET = 0x1,
  NET_SOCKET = 0x2
} WiFi_Socket_t;


typedef struct
{
	int Port;
	WiFi_Priv_Mode PrivMode;
	char SSID[31];
	char SecKey[31];
	char IP[16];
	char Protocol[2];
} WiFi_GeneralInfo;

// PPs

// Socket_Status_t Socket_CheckConnection(int SocketID);
int Socket_Connect(const char * hostName, int port, const char * protocol);
Socket_Status_t Socket_ReceiveData(int SocketID, char *data, int dataLength);
Socket_Status_t Socket_TransmitData(int SocketID, const char *data);
int Socket_GetPendingDataLength(int SocketID);
int Socket_CheckConnection(int SocketID);
Socket_Status_t Socket_ReadData(int SocketID, char *data);
void Socket_Close(int* socketID);

void WiFi_SetCommand(char * command);
WiFi_Status_t WiFi_SetPowerState(WiFi_Power_State_t state);
WiFi_Status_t WiFi_ReceiveResponse(char *responseEnd, int timeout);
WiFi_Status_t WiFi_TransmitCommand();
WiFi_Status_t WiFi_Connect(const char * ssid, const char * sec_key, WiFi_Priv_Mode priv_mode);
WiFi_Status_t WiFi_PingServer(const char* hostName);

#endif /* INC_WIFI_H_ */
