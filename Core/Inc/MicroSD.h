#include "main.h"
#include "integer.h"

#define CONFIG_FILE "config.txt"

extern RTC_HandleTypeDef hrtc;
extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;

void ReadConfigs(char *buffer, UINT *bytesRead);
void WriteFile(const char *line);
void CreatePath(char *path);
