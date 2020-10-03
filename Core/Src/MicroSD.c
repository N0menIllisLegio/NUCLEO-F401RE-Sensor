#include "MicroSD.h"

#include "fatfs.h"
#include "fatfs_sd.h"

#include <string.h>
#include <stdio.h>

void GetFileName(char *fileName)
{
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	snprintf(fileName, 10, "%d.csv", sDate.Date);
}

void CreatePath(char *path)
{
	char fileName[10];
	GetFileName(fileName);

	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	snprintf(path, 30, "%d", sDate.Year);

	f_mkdir(path);

	snprintf(path, 30, "%d/%d", sDate.Year, sDate.Month);

	f_mkdir(path);

	snprintf(path, 30, "%d/%d/%s", sDate.Year, sDate.Month, fileName);
}

void WriteFile(const char *line)
{
	FATFS fs;
	FIL file;
	UINT bWriten;
	char path[30];

	// To reset if SD was ejected.
	MX_FATFS_DeInit();
	MX_FATFS_Init();

	if(f_mount(&fs, "/", 1) == FR_OK)
	{
		CreatePath(path);

		if(f_open(&file, path, FA_OPEN_APPEND | FA_WRITE) == FR_OK)
		{
			f_write(&file, line, strlen(line), &bWriten);
			f_close(&file);
		}

		f_mount(NULL, "/", 1);
	}
}

void ReadConfigs(char *buffer, UINT *bytesRead)
{
	FATFS fs;
	FRESULT fResult;
	FIL file;

	MX_FATFS_DeInit();
	MX_FATFS_Init();

	fResult = f_mount(&fs, "/", 1);

	if(fResult == FR_OK)
	{
		fResult = f_open(&file, CONFIG_FILE, FA_READ);

		if(fResult == FR_OK)
		{
			f_read(&file, buffer, f_size(&file), bytesRead);
			f_close(&file);
		}

		f_mount(NULL, "/", 1);
	}
}
