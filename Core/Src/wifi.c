#include "wifi.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

// Considering that module reconnecting to wifi by itself given time
// I will only check for socket errors

char Resp_Buff[1000];
char Cmd_Buff[1024];
extern UART_HandleTypeDef huart1;
extern MC_GeneralInfo mc_info;

void ReceiveData()
{
	HAL_UART_Receive_DMA(&huart1, (uint8_t *)Resp_Buff, sizeof Resp_Buff);
}

void WiFi_ResetBuffer()
{
	memset(Cmd_Buff, 0x00, sizeof Cmd_Buff);
}

void WiFi_SetCommand(char * command)
{
	WiFi_ResetBuffer();
	sprintf(Cmd_Buff, command);
}

WiFi_Status_t WiFi_ReceiveResponse(char *responseEnd, int timeout)
{
	HAL_UART_Receive_DMA(&huart1, (uint8_t *)Resp_Buff, sizeof Resp_Buff);

	WiFi_Status_t status = WiFi_MODULE_SUCCESS;
	int tact = 0;

	while(strstr(Resp_Buff, responseEnd) == NULL)
	{
		HAL_Delay(2000);
		tact++;

		if (tact > timeout)
		{
			status = WiFi_TIME_OUT_ERROR;
			break;
		}
	}

	HAL_UART_DMAStop(&huart1);

	return status;
}

WiFi_Status_t WiFi_TransmitCommand()
{
	memset(Resp_Buff, 0x00, sizeof Resp_Buff);

	if (HAL_UART_Transmit(&huart1, (uint8_t *) Cmd_Buff, strlen (Cmd_Buff), 1000) == HAL_OK)
	{
		return WiFi_MODULE_SUCCESS;
	}
	else
	{
		return WiFi_HAL_UART_ERROR;
	}
}

WiFi_Status_t WiFi_SetPowerState(WiFi_Power_State_t state)
{
  WiFi_ResetBuffer();
  sprintf(Cmd_Buff, AT_SET_POWER_STATE, (int) state);

  return WiFi_TransmitCommand();
}

WiFi_Status_t WiFi_Connect(const char * ssid, const char * sec_key, WiFi_Priv_Mode priv_mode)
{
	WiFi_Status_t status = WiFi_MODULE_SUCCESS;

	if(sec_key)
	{
		WiFi_ResetBuffer();
		sprintf(Cmd_Buff, AT_SET_SEC_KEY, sec_key);
		status = WiFi_TransmitCommand();

		if(status != WiFi_MODULE_SUCCESS)
			return WiFi_SecKey_ERROR;
	}

	if(ssid)
	{
		WiFi_ResetBuffer();
		sprintf(Cmd_Buff, AT_SET_SSID, ssid);
		status = WiFi_TransmitCommand();
	}
	else
	{
		return WiFi_SSID_ERROR;
	}

	if(status != WiFi_MODULE_SUCCESS)
		return WiFi_SSID_ERROR;


	WiFi_ResetBuffer();
	sprintf(Cmd_Buff, AT_SET_CONFIGURATION_VALUE, WIFI_PRIV_MODE, priv_mode);
	status = WiFi_TransmitCommand();

	if(status != WiFi_MODULE_SUCCESS)
		return WiFi_CONFIG_ERROR;


	WiFi_ResetBuffer();
	sprintf(Cmd_Buff, AT_SET_CONFIGURATION_VALUE, WIFI_MODE, (int) WiFi_STA_MODE);
	status = WiFi_TransmitCommand();

	if(status != WiFi_MODULE_SUCCESS)
		return WiFi_CONFIG_ERROR;

	WiFi_ResetBuffer();
	sprintf(Cmd_Buff, AT_SAVE_CURRENT_SETTING);
	status = WiFi_TransmitCommand();

	WiFi_SetPowerState(PowerSave_State);

	return WiFi_ReceiveResponse(":WiFi Up:", 15);
}

WiFi_Status_t WiFi_PingServer(const char* hostName)
{
	WiFi_ResetBuffer();
	sprintf(Cmd_Buff, AT_PING, hostName);

	if(WiFi_TransmitCommand() != WiFi_MODULE_SUCCESS)
		return Socket_WiFi_ERROR;

	return WiFi_ReceiveResponse(AT_RESP, 5);
}

// -------------------------------------------------------------------------
// ----------------------------- SOCKET ------------------------------------
// -------------------------------------------------------------------------

Socket_Status_t Socket_Authenticate(int SocketID)
{
	int dataLength = 120;
	char transmitData[dataLength];
	char receivedData[10];
	snprintf(transmitData, dataLength, "%s|%s;%s;%s", SERVER_AUTH,
			mc_info.OwnerID, mc_info.MicrocontrollerID, mc_info.MicrocontrollerPassword);

	Socket_Status_t status = Socket_TransmitData(SocketID, transmitData);

	if(status == Socket_SUCCESS)
	{
		Socket_ReadData(SocketID, receivedData);

		if(strstr(receivedData, SERVER_OK_RESP) != NULL)
		{
			return Socket_SUCCESS;
		}
	}

	return Socket_ERROR;
}


int Socket_Connect(const char * hostName, int port, const char * protocol)
{
	WiFi_ResetBuffer();
	sprintf(Cmd_Buff, AT_SOCKET_OPEN, hostName, port, protocol);

	if(WiFi_TransmitCommand() != WiFi_MODULE_SUCCESS)
		return -1;

	if(WiFi_ReceiveResponse(AT_RESP, 5) == WiFi_MODULE_SUCCESS)
	{
		char *id = strstr(Resp_Buff, "ID: ");
		int socketID = (*(id + 4) - '0') * 10 + (*(id + 5) - '0');
		if (Socket_Authenticate(socketID) == Socket_SUCCESS)
		{
			return socketID;
		}
	}

	return -1;
}

void Socket_Close(int* socketID)
{
	char flushData[50];
	Socket_ReadData(*socketID, flushData);

	WiFi_ResetBuffer();
	sprintf(Cmd_Buff, AT_SOCKET_CLOSE, *socketID);

	if(WiFi_TransmitCommand() != WiFi_MODULE_SUCCESS)
		return;

	*socketID = -1;
}

// -1 - Error
int Socket_GetPendingDataLength(int SocketID)
{
	WiFi_ResetBuffer();
	sprintf(Cmd_Buff, AT_QUERY_PENDING_DATA, SocketID);

	if(WiFi_TransmitCommand() != WiFi_MODULE_SUCCESS)
		return -1;

	if(WiFi_ReceiveResponse(AT_RESP, 5) != WiFi_MODULE_SUCCESS)
		return -1;

	char *id = strstr(Resp_Buff, "DATALEN: ");
	char number[10];
	int counter = 0;
	memset(number, 0x00, sizeof number);
	id += 9;
	while (*id >= '0' && *id <= '9')
	{
		number[counter++] = *id;
		id++;
	}

	int datalen;
	sscanf(number, "%d", &datalen);

	return datalen;
}

Socket_Status_t Socket_ReceiveData(int SocketID, char *data, int dataLength)
{
	if (dataLength < 0)
		return Socket_ERROR;

	if (dataLength == 0)
		return Socket_SUCCESS;

	WiFi_ResetBuffer();
	sprintf(Cmd_Buff, AT_SOCKET_READ, SocketID, dataLength);

	if(WiFi_TransmitCommand() != WiFi_MODULE_SUCCESS)
		return Socket_ERROR;

	if(WiFi_ReceiveResponse(AT_RESP, 5) != WiFi_MODULE_SUCCESS)
		return Socket_ERROR;

	char *at_OK = strstr(Resp_Buff, AT_RESP);
	*at_OK = '\0';

	strcpy(data, Resp_Buff);

	return Socket_SUCCESS;
}

Socket_Status_t Socket_TransmitData(int SocketID, const char *data)
{
	WiFi_ResetBuffer();
	sprintf(Cmd_Buff, AT_SOCKET_WRITE, SocketID, strlen(data));

	if (WiFi_TransmitCommand() == WiFi_MODULE_SUCCESS
			&& HAL_UART_Transmit(&huart1, (uint8_t *) data, strlen (data), 1000) == HAL_OK)
	{
		return Socket_SUCCESS;
	}

	return Socket_ERROR;
}

Socket_Status_t Socket_ReadData(int SocketID, char *data)
{
	HAL_Delay(400);

	int receivedDataLen = Socket_GetPendingDataLength(SocketID);

	if (receivedDataLen > 0)
	{
		return Socket_ReceiveData(SocketID, data, receivedDataLen);
	}

	return Socket_ERROR;
}

// 1 - OK
// 0 - Error
int Socket_CheckConnection(int SocketID)
{
	char receivedData[10];
	Socket_Status_t status = Socket_TransmitData(SocketID, "STM_AT");

	if (status == Socket_SUCCESS)
	{
		if (Socket_ReadData(SocketID, receivedData) == Socket_SUCCESS)
		{
			if (strstr(receivedData, SERVER_AT_RESP) != NULL)
			{
				return 1;
			}
		}
	}

	return 0;
}
