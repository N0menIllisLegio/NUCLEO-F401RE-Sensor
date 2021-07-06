## Description

Program for microcontroller **STM32 NUCLEOF401RE** with WiFi expansion board **X-NUCLEO-IDW04A1**, microSD adapter (SPI) and Air Quality sensor **MQ-135**.
Its purpose - **read data from sensor and save it on microSD and/or transmit it to [server](https://github.com/N0menIllisLegio/Gas-Sensor-Server) by WiFi**.

## Configuration

Configuration file should be placed in the root directory of SDCard with name: **config.txt**.

File contents:

1. Date & Time configuration
  This date and time will be set as current for microcontroller.

```
Date=7;
Month=6;
Year=21;
WeekDay=3;
Hours=0;
Minutes=14;
Seconds=47;
```

2. WiFi configuration
  Private modes:
    - `0` - None
    - `1` - WEP
    - `2` - WPA Personal

```
SSID=;
SecurityKey=;
PrivateMode=2;
```

3. Intervals configuration
  Configure with what interval data will be send to server (`transmitIntervalSeconds`) or written to microSD (`writeSDIntervalSeconds`) and longpolling interval (`requestIntervalSeconds`).


```
writeSDIntervalSeconds=60;
transmitIntervalSeconds=1800;
requestIntervalSeconds=900;
```

4. Server connection configuration
  IP address and port of server  which will be receiving data
  Protocols for data transmitting:
    - `t` - TCP
    - `u` - UDP (**Not supported**)

```
IP=;
Port=;
Protocol=t;
```

5. Credentials configuration

```
OwnerID=;
MicrocontrollerID=;
MicrocontrollerPassword=;
SensorID=;
```
