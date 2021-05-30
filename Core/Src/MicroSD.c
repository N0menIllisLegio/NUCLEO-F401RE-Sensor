#include "MicroSD.h"

#include "fatfs.h"
#include "fatfs_sd.h"

#include <string.h>
#include <stdio.h>

void GetFileName(char *fileName, size_t fileNameLength)
{
	snprintf(fileName, fileNameLength, "%d.csv", sDate.Date);
}

void CreatePath(char *path, size_t pathLength)
{
	size_t fileNameLength = 10;
	char fileName[fileNameLength];
	GetFileName(fileName, fileNameLength);

	snprintf(path, pathLength, "%d", sDate.Year);

	f_mkdir(path);

	snprintf(path, pathLength, "%d/%d", sDate.Year, sDate.Month);

	f_mkdir(path);

	snprintf(path, pathLength, "%d/%d/%s", sDate.Year, sDate.Month, fileName);
}

void WriteFile(const char *line)
{
	FATFS fs;
	FIL file;
	UINT bWriten;
	size_t pathLength = 30;
	char path[pathLength];

	// To reset if SD was ejected.
	MX_FATFS_DeInit();
	MX_FATFS_Init();

	if(f_mount(&fs, "/", 1) == FR_OK)
	{
		CreatePath(path, pathLength);

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
