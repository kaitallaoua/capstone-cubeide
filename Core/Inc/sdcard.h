/*
 * sdcard.h
 *
 *  Created on: Feb 8, 2024
 *      Author: karim
 */

// Need like this as initalizing in freertos leadings to hardfault
// increasing stack did not work, may need to tune so that it uses heap

//https://community.st.com/t5/stm32-mcus-products/stm32f103-sdio-fatfs-fault-issue/m-p/411951/highlight/true#M118933
	  FIL fil; 		//File handle
	  FRESULT fres; //Result after operations
	  DWORD free_clusters, free_sectors, total_sectors;

	  FATFS* getFreeFs;

	  //some variables for FatFs
	  FATFS FatFs; 	//Fatfs handle
	  //FIL fil; 		//File handle
	  BYTE readBuf[30];
	  UINT bytesWrote;
	  TCHAR* rres;
