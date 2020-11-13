/*
 * file_handling.c
 *
 *  Created on: 26-Jul-2019
 *      Author: arunr
 */

#include "file_handling.h"


#include "UartRingbuffer.h"

FATFS fs;  // file system
FIL fil; // File
FILINFO fno;
FRESULT fresult;  // result
UINT br, bw;  // File read/write count

/**** capacity related *****/
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;


char buffer[BUFFER_SIZE];  // to store strings..
char path[PATH_SIZE];  // buffer to store path

int i=0;

int bufsize (char *buf)
{
	int i=0;
	while (*buf++ != '\0') i++;
	return i;
}

void clear_buffer (void)
{
	for (int i=0; i<BUFFER_SIZE; i++) buffer[i] = '\0';
}

void clear_path (void)
{
	for (int i=0; i<PATH_SIZE; i++) path[i] = '\0';
}

void send_uart (char *string)
{
//	uint8_t len = strlen ((const char *) string);
//	HAL_UART_Transmit(&huart1, (uint8_t *) string, len, 2000);
	Uart_sendstring(string);
	clear_buffer();
}

int cmdlength (char *str)
{
	int i=0;
	while (*str++ != ' ') i++;
	return i;
}
void get_path (void)
{
	int start = cmdlength(buffer)+1;
	int end = bufsize(buffer)-2;

	int j=0;
	for (int i=start; i<end; i++)
	{
		if (buffer[i] != ' ') path[j++] = buffer[i];
		else break;
	}
}

void mount_sd (void)
{
	fresult = f_mount(&fs, "/", 1);
	if (fresult != FR_OK) send_uart ("error in mounting SD CARD...\n");
	else send_uart("SD CARD mounted successfully...\n");
}

void unmount_sd (void)
{
	fresult = f_mount(NULL, "/", 1);
	if (fresult == FR_OK) send_uart ("SD CARD UNMOUNTED successfully...\n");
	else send_uart("error!!! in UNMOUNTING SD CARD\n");
}

/* Start node to be scanned (***also used as work area***) */
FRESULT scan_files (char* pat)
{
    DIR dir;
    UINT i;

    char path[20];
    sprintf (path, "%s",pat);

    fresult = f_opendir(&dir, path);                       /* Open the directory */
    if (fresult == FR_OK)
    {
        for (;;)
        {
            fresult = f_readdir(&dir, &fno);                   /* Read a directory item */
            if (fresult != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
            if (fno.fattrib & AM_DIR)     /* It is a directory */
            {
            	if (!(strcmp ("SYSTEM~1", fno.fname))) continue;
            	sprintf (buffer, "Dir: %s\r\n", fno.fname);
            	send_uart(buffer);
                i = strlen(path);
                sprintf(&path[i], "/%s", fno.fname);
                fresult = scan_files(path);                     /* Enter the directory */
                if (fresult != FR_OK) break;
                path[i] = 0;
            }
            else
            {                                       /* It is a file. */
               sprintf(buffer,"File: %s/%s\n", path, fno.fname);
               send_uart(buffer);
            }
        }
        f_closedir(&dir);
    }
    return fresult;
}




void write_file (char *name)
{

	/**** check whether the file exists or not ****/
	fresult = f_stat (name, &fno);
	if (fresult != FR_OK)
	{
		sprintf (buffer, "*%s* does not exists\n", name);
		send_uart (buffer);
	}

	else
	{
	    /* Create a file with read write access and open it */
	    fresult = f_open(&fil, name, FA_OPEN_EXISTING | FA_WRITE);
	    if (fresult != FR_OK)
	    {
	    	sprintf (buffer, "error no %d in opening file *%s*\n", fresult, name);
	    	send_uart(buffer);
	    }

	    else
	    {
	    	sprintf (buffer, "file *%s* is opened. Now enter the string you want to write\n", name);
	    	send_uart (buffer);
	    }

	    while (!(wait_until("\r\n", buffer)));

	    /* Writing text */

	    fresult = f_write(&fil, buffer, bufsize(buffer), &bw);

	    if (fresult != FR_OK)
	    {
	    	clear_buffer();
	    	sprintf (buffer, "error no %d in writing file *%s*\n", fresult, name);
	    	send_uart(buffer);
	    }

	    else
	    {
	    	clear_buffer();
	    	sprintf (buffer, "*%s* written successfully\n", name);
	    	send_uart(buffer);
	    }

	    /* Close file */
	    fresult = f_close(&fil);
	    if (fresult != FR_OK)
	    {
	    	sprintf (buffer, "error no %d in closing file *%s*\n", fresult, name);
	    	send_uart(buffer);
	    }
	}
}

void read_file (char *name)
{
	/**** check whether the file exists or not ****/
	fresult = f_stat (name, &fno);
	if (fresult != FR_OK)
	{
		sprintf (buffer, "*%s* does not exists\n", name);
		send_uart (buffer);
	}

	else
	{
		/* Open file to read */
		fresult = f_open(&fil, name, FA_READ);

		if (fresult != FR_OK)
		{
			sprintf (buffer, "error no %d in opening file *%s*\n", fresult, name);
		    	send_uart(buffer);
		}

		/* Read data from the file
		* see the function details for the arguments */
		sprintf (buffer, "reading data from the file *%s*\n", name);
		send_uart (buffer);

		fresult = f_read (&fil, buffer, f_size(&fil), &br);
		if (fresult != FR_OK)
		{
		  	clear_buffer();
		 	sprintf (buffer, "error no %d in reading file *%s*\n", fresult, name);
		  	send_uart(buffer);
		}

		else send_uart(buffer);


		/* Close file */
		fresult = f_close(&fil);
		if (fresult != FR_OK)
		{
		   	sprintf (buffer, "error no %d in closing file *%s*\n", fresult, name);
		   	send_uart(buffer);
		}
	}
}

void create_file (char *name)
{
	fresult = f_stat (name, &fno);
	if (fresult == FR_OK)
	{
		sprintf (buffer, "*%s* already exists!!!!\n",name);
		send_uart(buffer);
	}
	else {
    fresult = f_open(&fil, name, FA_CREATE_ALWAYS|FA_READ|FA_WRITE);
    if (fresult != FR_OK)
    {
    	sprintf (buffer, "error no %d in creating file *%s*\n", fresult, name);
    	send_uart(buffer);
    }
    else
    {
    	sprintf (buffer, "*%s* created successfully\n",name);
    	send_uart(buffer);
    }

    fresult = f_close(&fil);
    if (fresult != FR_OK)
    {
    	sprintf (buffer, "error no %d in closing file *%s*\n", fresult, name);
    	send_uart(buffer);
    }
	}
}

void remove_file (char *name)
{
	/**** check whether the file exists or not ****/
	fresult = f_stat (name, &fno);
	if (fresult != FR_OK)
	{
		sprintf (buffer, "*%s* does not exists\n", name);
		send_uart (buffer);
	}

	else{
	fresult = f_unlink (name);
	if (fresult == FR_OK)
	{
		sprintf (buffer, "*%s* has been removed successfully\n", name);
		send_uart (buffer);
	}

	else
	{
		sprintf (buffer, "error in removing *%s*\n", name);
		send_uart (buffer);
	}
	}

}

void create_dir (char *name)
{
    fresult = f_mkdir(name);
    if (fresult == FR_OK)
    {
    	sprintf (buffer, "*%s* has been created successfully\n", name);
    	send_uart (buffer);
    }
    else
    {
    	sprintf (buffer, "error no %d in creating directory\n", fresult);
    	send_uart(buffer);
    }
}

void check_sd (void)
{
    /* Check free space */
    f_getfree("", &fre_clust, &pfs);

    total = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
    sprintf (buffer, "SD CARD Total Size: \t%lu\n",total);
    send_uart(buffer);
    free_space = (uint32_t)(fre_clust * pfs->csize * 0.5);
    sprintf (buffer, "SD CARD Free Space: \t%lu\n",free_space);
    send_uart(buffer);
}

void check_file (char *name)
{
	  fresult = f_stat(name, &fno);
	  switch (fresult)
	  {
	    case FR_OK:

	        sprintf(buffer,"Below are the details of the *%s* \nSize: %lu\n",name, fno.fsize);
	        send_uart (buffer);
	        sprintf(buffer,"Timestamp: %u/%02u/%02u, %02u:%02u\n",
	               (fno.fdate >> 9) + 1980, fno.fdate >> 5 & 15, fno.fdate & 31,
	               fno.ftime >> 11, fno.ftime >> 5 & 63);
	        send_uart (buffer);
	        sprintf(buffer,"Attributes: %c%c%c%c%c\n",
	               (fno.fattrib & AM_DIR) ? 'D' : '-',
	               (fno.fattrib & AM_RDO) ? 'R' : '-',
	               (fno.fattrib & AM_HID) ? 'H' : '-',
	               (fno.fattrib & AM_SYS) ? 'S' : '-',
	               (fno.fattrib & AM_ARC) ? 'A' : '-');
	        send_uart (buffer);
	        break;

	    case FR_NO_FILE:
	        sprintf(buffer,"*%s* does not exist.\n", name);
	        send_uart (buffer);
	        break;

	    default:
	        sprintf(buffer,"An error occurred. (%d)\n", fresult);
	        send_uart (buffer);
	    }
}

void update_file (char *name)
{
	/**** check whether the file exists or not ****/
	fresult = f_stat (name, &fno);
	if (fresult != FR_OK)
	{
		sprintf (buffer, "*%s* does not exists\n", name);
		send_uart (buffer);
	}

	else
	{
		 /* Create a file with read write access and open it */
			    fresult = f_open(&fil, name, FA_OPEN_APPEND | FA_WRITE);
			    if (fresult != FR_OK)
			    {
			    	sprintf (buffer, "error no %d in opening file *%s*\n", fresult, name);
			    	send_uart(buffer);
			    }

			    else
			    {
			    	sprintf (buffer, "file *%s* is opened. Now enter the string you want to update\n", name);
			    	send_uart (buffer);
			    }

			    while (!(wait_until("\r\n", buffer)));

			    /* Writing text */

			    fresult = f_write(&fil, buffer, bufsize(buffer), &bw);

			    if (fresult != FR_OK)
			    {
			    	clear_buffer();
			    	sprintf (buffer, "error no %d in writing file *%s*\n", fresult, name);
			    	send_uart(buffer);
			    }

			    else
			    {
			    	clear_buffer();
			    	sprintf (buffer, "*%s* written successfully\n", name);
			    	send_uart(buffer);
			    }

			    /* Close file */
			    fresult = f_close(&fil);
			    if (fresult != FR_OK)
			    {
			    	sprintf (buffer, "error no %d in closing file *%s*\n", fresult, name);
			    	send_uart(buffer);
			    }
		}
}
