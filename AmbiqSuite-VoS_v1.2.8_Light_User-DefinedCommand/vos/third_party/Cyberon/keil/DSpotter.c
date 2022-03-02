#include "am_vos_sys_config.h"
#include "am_vos_board_setup.h"

#include "am_app_utils.h"
#include "am_app_utils_task.h"

#include "am_vos_task.h"
#include "am_vos_init.h"

#if configUSE_Cyberon_Spotter

#include "DSpotterSDKApi.h"
#include "DSpotterSDTrainApi.h"
#include "DataFlashApi.h"
#include "am_vos_DSpotter.h"

void *g_hDSpotter = NULL;
BYTE *g_lpbyMemPool = NULL;
INT g_nMemUsage;
BYTE *g_lpbyDataBuf = NULL;
INT g_nDataBufSize;
BYTE *g_lpbyModelBuf = NULL;
INT g_nModelBufSize;
BYTE *g_lppbyModel[MODEL_NUM] = {0};
SHORT g_lpsMapID[MAP_ID_FILE_SIZE] = {0};
extern uint32_t u32CMDDataBegin;
extern uint32_t u32LicenseDataBegin;
uint32_t g_ui32PrgmAddr = (AM_HAL_FLASH_INSTANCE_SIZE + (FLASH_PAGE_INDEX * AM_HAL_FLASH_PAGE_SIZE));
volatile BOOL g_bTrainSDModel = TRUE;
volatile BOOL g_bProcessButtonEvent = FALSE;
SHORT g_sTargetID = 0;

BOOL ProgramDataToFlash(INT nModelSize)
{
	INT nErr;
	BOOL bErrorOccurred = FALSE;
	
	do
	{
		if((sizeof(uint32_t) + *(uint32_t *)&u32CMDDataBegin) + (MAP_ID_FILE_SIZE << 1) + nModelSize > FLASH_PAGE_NUM * AM_HAL_FLASH_PAGE_SIZE)
		{
				AM_APP_LOG_WARNING("Flash buffer isn't big enough!!\n");
				bErrorOccurred = TRUE;
				break;			
		}
		
		for(int i = 0; i < FLASH_PAGE_NUM; i++)
		{
				if((nErr = am_hal_flash_page_erase(AM_HAL_FLASH_PROGRAM_KEY, 1, FLASH_PAGE_INDEX + i)) != 0)
					break;
		}

		if(nErr)
		{
				AM_APP_LOG_WARNING("am_hal_flash_page_erase Fail!!(%d)\n", nErr);
				bErrorOccurred = TRUE;
				break;
		}
		
		if(nModelSize <= 0)
			break;
		
		nErr = am_hal_flash_program_main(AM_HAL_FLASH_PROGRAM_KEY, (uint32_t *)&u32CMDDataBegin, (uint32_t *)g_ui32PrgmAddr, (sizeof(uint32_t) + *(uint32_t *)&u32CMDDataBegin) >> 2);
		if(nErr)
		{
				AM_APP_LOG_WARNING("am_hal_flash_program_main Fail!!(%d)\n", nErr);
				bErrorOccurred = TRUE;
				break;
		}

		nErr = am_hal_flash_program_main(AM_HAL_FLASH_PROGRAM_KEY, (uint32_t *)g_lpsMapID, (uint32_t *)g_ui32PrgmAddr + ((sizeof(uint32_t) + *(uint32_t *)&u32CMDDataBegin) >> 2), MAP_ID_FILE_SIZE >> 1);
		if(nErr)
		{
				AM_APP_LOG_WARNING("am_hal_flash_program_main Fail!!(%d)\n", nErr);
				bErrorOccurred = TRUE;
				break;
		}

		nErr = am_hal_flash_program_main(AM_HAL_FLASH_PROGRAM_KEY, (uint32_t *)g_lpbyModelBuf, (uint32_t *)g_ui32PrgmAddr + ((sizeof(uint32_t) + *(uint32_t *)&u32CMDDataBegin) >> 2) + (MAP_ID_FILE_SIZE >> 1), nModelSize >> 2);
		if(nErr)
		{
				AM_APP_LOG_WARNING("am_hal_flash_program_main Fail!!(%d)\n", nErr);
				bErrorOccurred = TRUE;
				break;
		}

		if(memcmp((uint32_t *)&u32CMDDataBegin, (uint32_t *)g_ui32PrgmAddr, sizeof(uint32_t) + *(uint32_t *)&u32CMDDataBegin) != 0)
		{
				AM_APP_LOG_WARNING("memcmp Fail!!\n");
				bErrorOccurred = TRUE;
				break;
		}
		
		if(memcmp(g_lpsMapID, (uint32_t *)g_ui32PrgmAddr + ((sizeof(uint32_t) + *(uint32_t *)&u32CMDDataBegin) >> 2), MAP_ID_FILE_SIZE << 1) != 0)
		{
				AM_APP_LOG_WARNING("memcmp Fail!!\n");
				bErrorOccurred = TRUE;
				break;
		}

		if(memcmp(g_lpbyModelBuf, (uint32_t *)g_ui32PrgmAddr + ((sizeof(uint32_t) + *(uint32_t *)&u32CMDDataBegin) >> 2) + (MAP_ID_FILE_SIZE >> 1), nModelSize) != 0)
		{
				AM_APP_LOG_WARNING("memcmp Fail!!\n");
				bErrorOccurred = TRUE;
				break;
		}
	} while(0);
	
	if(bErrorOccurred)
	{
		for(int i = 0; i < FLASH_PAGE_NUM; i++)
		{
				nErr = am_hal_flash_page_erase(AM_HAL_FLASH_PROGRAM_KEY, 1, FLASH_PAGE_INDEX + i);
		}
	}
	
	return !bErrorOccurred;
}

BOOL ProcessButtonEvent()
{
	SHORT *psPtr;
	INT nNumCommand, i;
	INT nErr;
	INT nUsedSize;

	psPtr = g_lpsMapID;
	psPtr += 160;
	nNumCommand = *((INT *)psPtr);
	psPtr += 2;
	for(i = 0; i < nNumCommand; i++)
	{
		if(psPtr[i] == g_sTargetID)
			break;
	}
	
	if(i < nNumCommand)
	{
		if(nNumCommand == MAX_UTTERANCE * 6 && i == 0)
		{
			for(int j = i, k = i + MAX_UTTERANCE * 3; k < nNumCommand; j++, k++)
			{
				psPtr[j] = psPtr[k];
			}
		}
		else if(nNumCommand == MAX_UTTERANCE * 9 && i == 0)
		{
			for(int j = i, k = i + MAX_UTTERANCE * 3; k < nNumCommand; j++, k++)
			{
				psPtr[j] = psPtr[k];
			}
		}
		else if(nNumCommand == MAX_UTTERANCE * 9 && i == MAX_UTTERANCE * 3)
		{
			for(int j = i, k = i + MAX_UTTERANCE * 3; k < nNumCommand; j++, k++)
			{
				psPtr[j] = psPtr[k];
			}
		}
		
		psPtr = g_lpsMapID + 160;
		*((INT *)psPtr) = nNumCommand - MAX_UTTERANCE * 3;
		
		g_hDSpotter = DSpotterSD_Init(g_lppbyModel[0], g_lppbyModel[MODEL_NUM - 2], g_lpbyMemPool, g_nMemUsage, &nErr);
		if(g_hDSpotter == NULL){
			AM_APP_LOG_WARNING("DSpotterSD_Init Fail!!(%d)\n", nErr);
			return FALSE;
		}
		
		for(int j = 0; j < MAX_UTTERANCE; j++)
		{
			nErr = DSpotterSD_DeleteWord(g_hDSpotter, (char *)g_lpbyModelBuf, i, &nUsedSize);
			if(nErr != DSPOTTER_SUCCESS)
			{
				AM_APP_LOG_WARNING("DSpotterSD_DeleteWord Fail!!(%d)\n", nErr);
				return FALSE;
			}
			AM_APP_LOG_INFO("Delete command index [%d], nUsedSize = %d\r\n", i, nUsedSize);
		}
		
		if(!ProgramDataToFlash(nUsedSize))
		{
			AM_APP_LOG_WARNING("ProgramDataToFlash Fail!!\n");
			return FALSE;
		}
		
		AM_APP_LOG_INFO("Remove command successfully!!\r\n");
	}
	else
	{
		g_hDSpotter = DSpotterSD_Init(g_lppbyModel[0], g_lppbyModel[MODEL_NUM - 2], g_lpbyMemPool, g_nMemUsage, &nErr);
		if(g_hDSpotter == NULL){
			AM_APP_LOG_WARNING("DSpotterSD_Init Fail!!(%d)\n", nErr);
			return FALSE;
		}
	}
	
#if ENABLE_AGC	
	if((nErr = DSpotterSDAGC_Enable(g_hDSpotter)) != DSPOTTER_SUCCESS)
	{
		AM_APP_LOG_WARNING("DSpotterSDAGC_Enable Fail!!(%d)\n", nErr);
		return FALSE;
	}
	
	if((nErr = DSpotterSDAGC_SetMaxGain(g_hDSpotter, AGC_MAX_GAIN)) != DSPOTTER_SUCCESS)
	{
		AM_APP_LOG_WARNING("DSpotterSDAGC_SetMaxGain Fail!!(%d)\n", nErr);
		return FALSE;
	}
#endif
	
	if((nErr = DSpotterSD_SetBackgroundEnergyThreshd(g_hDSpotter, ENERGY_THRESHOLD)) != DSPOTTER_SUCCESS)
	{
		AM_APP_LOG_WARNING("DSpotterSD_SetBackgroundEnergyThreshd Fail!!(%d)\n", nErr);
		return FALSE;
	}
	
	return TRUE;
}

INT UnpackBin(BYTE lpbyBin[], BYTE *lppbyModel[], INT nMaxNumModel)
{
	UINT *lpnBin = (UINT *)lpbyBin;
	INT nNumBin = lpnBin[0];
	UINT *lpnBinSize = lpnBin + 1;
	INT i;

	lppbyModel[0] = (BYTE *)(lpnBinSize + nNumBin);
	for (i = 1; i < nNumBin; i++){
		if (i >= nMaxNumModel)
			break;
		lppbyModel[i] = lppbyModel[i-1] + lpnBinSize[i-1];
	}

	return i;
}

void *DSpotterInit(void)
{
	HANDLE hDSpotter = NULL;
	INT nMemUsage, nErr;
	SHORT *psPtr;
	
	/** Initialize VR engine */
	// Unpack command data
	if(UnpackBin((BYTE *)&u32CMDDataBegin + (sizeof(uint32_t) + *(uint32_t *)&u32CMDDataBegin), g_lppbyModel, MODEL_NUM) < MODEL_NUM){
		am_app_utils_stdio_printf(2, "Invalid bin\r\n");
		return NULL;
	}
	
	// Check the memory usage
	nMemUsage = DSpotterSD_GetMemoryUsage(g_lppbyModel[0], g_lppbyModel[MODEL_NUM - 2]);
	if(nMemUsage > xPortGetFreeHeapSize()){
		am_app_utils_stdio_printf(2, "Need more memory, memory usage = %d available memory = %d\r\n", nMemUsage, xPortGetFreeHeapSize());
		return NULL;
	}
	g_nMemUsage = nMemUsage;
	g_lpbyMemPool = pvPortMalloc(nMemUsage);
	
	hDSpotter = DSpotterSD_Init(g_lppbyModel[0], g_lppbyModel[MODEL_NUM - 2], g_lpbyMemPool, g_nMemUsage, &nErr);
	if(hDSpotter == NULL){
		am_app_utils_stdio_printf(2, "g_hDSpotter == NULL\r\n");
		return NULL;
	}
	
	// Allocate memory for SD model training
	nMemUsage = DSpotterSD_AddUttrStart(hDSpotter, NULL, 0);
	if(nMemUsage > xPortGetFreeHeapSize()){
		am_app_utils_stdio_printf(2, "Need more memory, memory usage = %d available memory = %d\r\n", nMemUsage, xPortGetFreeHeapSize());
		return NULL;
	}
	g_nDataBufSize = nMemUsage;
	g_lpbyDataBuf = pvPortMalloc(nMemUsage);
	
	nMemUsage = k_nFlashPageSize;
	if(nMemUsage > xPortGetFreeHeapSize()){
		am_app_utils_stdio_printf(2, "Need more memory, memory usage = %d available memory = %d\r\n", nMemUsage, xPortGetFreeHeapSize());
		return NULL;
	}
	g_nModelBufSize = nMemUsage;
	g_lpbyModelBuf = pvPortMalloc(nMemUsage);
	
	// Try loading model from flash
	memcpy(g_lpsMapID, (uint32_t *)g_ui32PrgmAddr + ((sizeof(uint32_t) + *(uint32_t *)&u32CMDDataBegin) / sizeof(uint32_t)), MAP_ID_FILE_SIZE << 1);
	memcpy(g_lpbyModelBuf, (uint32_t *)g_ui32PrgmAddr + ((sizeof(uint32_t) + *(uint32_t *)&u32CMDDataBegin) / sizeof(uint32_t)) + (MAP_ID_FILE_SIZE >> 1), k_nFlashPageSize);
	if(memcmp(g_lpbyModelBuf, "CYHD", 4) == 0)
	{
		if(memcmp((uint32_t *)&u32CMDDataBegin, (uint32_t *)g_ui32PrgmAddr, sizeof(uint32_t) + *(uint32_t *)&u32CMDDataBegin) == 0)
		{
			memcpy(&g_lpbyModelBuf[28], &g_lppbyModel[0][28], 4);
		}
	}

	hDSpotter = DSpotter_Init_Multi(g_lppbyModel[0], (BYTE **)&g_lpbyModelBuf, 1, k_nMaxTime, g_lpbyMemPool, g_nMemUsage, NULL, 0, &nErr, (BYTE *)&u32LicenseDataBegin);
	if(hDSpotter)
	{
#if ENABLE_AGC
		if((nErr = DSpotterAGC_Enable(hDSpotter)) != DSPOTTER_SUCCESS)
		{
			am_app_utils_stdio_printf(2, "DSpotterAGC_Enable Fail!!(%d)\r\n", nErr);
			return NULL;
		}
		
		if((nErr = DSpotterAGC_SetMaxGain(hDSpotter, AGC_MAX_GAIN)) != DSPOTTER_SUCCESS)
		{
			am_app_utils_stdio_printf(2, "DSpotterAGC_SetMaxGain Fail!!(%d)\n", nErr);
			return NULL;
		}
#endif

		nErr = DSpotter_SetResultMapID_Sep(hDSpotter, (BYTE *)g_lpsMapID);
		if(nErr != DSPOTTER_SUCCESS){
				am_app_utils_stdio_printf(2, "DSpotter_SetResultMapID_Sep Fail!!(%d)\r\n", nErr);
				return NULL;
		}	
		
		am_app_utils_stdio_printf(2, "Load command model from flash, and then run command recognition flow!!\r\n");
		g_bTrainSDModel = FALSE;
	}
	else
	{
		hDSpotter = DSpotterSD_Init(g_lppbyModel[0], g_lppbyModel[MODEL_NUM - 2], g_lpbyMemPool, g_nMemUsage, &nErr);
		if(hDSpotter == NULL){
			am_app_utils_stdio_printf(2, "g_hDSpotter == NULL\r\n");
			return NULL;
		}
		
#if ENABLE_AGC
		if((nErr = DSpotterSDAGC_Enable(hDSpotter)) != DSPOTTER_SUCCESS)
		{
			am_app_utils_stdio_printf(2, "DSpotterSDAGC_Enable Fail!!(%d)\r\n", nErr);
			return NULL;
		}
		
		if((nErr = DSpotterSDAGC_SetMaxGain(hDSpotter, AGC_MAX_GAIN)) != DSPOTTER_SUCCESS)
		{
			am_app_utils_stdio_printf(2, "DSpotterSDAGC_SetMaxGain Fail!!(%d)\r\n", nErr);
			return NULL;
		}
#endif
		
		if((nErr = DSpotterSD_SetBackgroundEnergyThreshd(hDSpotter, ENERGY_THRESHOLD)) != DSPOTTER_SUCCESS)
		{
			am_app_utils_stdio_printf(2, "DSpotterSD_SetBackgroundEnergyThreshd Fail!!(%d)\n", nErr);
			return NULL;
		}
		
		// Initialize map id
		memset(g_lpsMapID, 0xFF, MAP_ID_FILE_SIZE << 1);
		psPtr = g_lpsMapID;
		psPtr += 160;
		*((INT *)psPtr) = 0;
		
		// Initialize model buffer
		memset(g_lpbyModelBuf, 0xFF, k_nFlashPageSize);
		
		am_app_utils_stdio_printf(2, "No command model in flash, so run command training flow!!\r\n");
	}

	return (void *)hDSpotter;
}

uint8_t am_vos_engine_init(void)
{
    g_hDSpotter = DSpotterInit();
    if(g_hDSpotter == NULL)
		{
        AM_APP_LOG_WARNING("Cyberon Spotter Init Fail!!\n");
			  return 1;
		}			

		return 0;
}

void am_vos_engine_process(int16_t *pi16InputBuffer, int16_t i16InputLength)
{
		static BOOL bStartTraining = FALSE;
		static BOOL bErrorOccurred = FALSE;
		static INT nUtterance = 0;
		static INT nSkipFrame = 0;
		INT nUsedSize;
		INT nErr;
		SHORT *psPtr;
		INT nNumCommand;
	
//		AM_APP_LOG_INFO("[AM-VoS] i16InputLength = %d\n", i16InputLength);
	
		if(bErrorOccurred)
		{
				am_app_utils_task_send(AM_APP_TASK_AUD_PROCESSING, AM_APP_TASK_LED,
																				AM_APP_MESSAGE_SHORT, KEY_WORD_GOT_MESSAGE, NULL);
				return;
		}
		
		if(g_bProcessButtonEvent)
		{
			am_vos_mic_disable();
			bErrorOccurred = !ProcessButtonEvent();
			am_vos_mic_enable();
			if(bErrorOccurred)
			{
					AM_APP_LOG_WARNING("ProcessButtonEvent Fail!!\n");
					return;
			}
			
			AM_APP_LOG_INFO("Run command training flow!!\r\n");
			g_bTrainSDModel = TRUE;
			g_bProcessButtonEvent = FALSE;
			
			return;
		}
	
		if(g_bTrainSDModel)
		{
				if(!bStartTraining)
				{
						nErr = DSpotterSD_AddUttrStart(g_hDSpotter, (short *)g_lpbyDataBuf, g_nDataBufSize);
						if(nErr != DSPOTTER_SUCCESS)
						{
								AM_APP_LOG_WARNING("DSpotterSD_AddUttrStart Fail!!(%d)\n", nErr);
								bErrorOccurred = TRUE;
								return;
						}
						
						bStartTraining = TRUE;
						nSkipFrame = SKIP_FRAME;
				}
				
				if(nSkipFrame > 0)
				{
						nSkipFrame--;
						return;
				}
				
				nErr = DSpotterSD_AddSample(g_hDSpotter, (short *)pi16InputBuffer, i16InputLength);
				if(nErr != DSPOTTER_SUCCESS && nErr != DSPOTTER_ERR_NeedMoreSample)
				{
						AM_APP_LOG_WARNING("DSpotterSD_AddSample Fail!!(%d)\n", nErr);
						bErrorOccurred = TRUE;
						return;
				}
				else if(nErr == DSPOTTER_ERR_NeedMoreSample)
				{
						return;
				}
				else if(nErr == DSPOTTER_SUCCESS)
				{
						nErr = DSpotterSD_AddUttrEnd(g_hDSpotter);
						if(nErr != DSPOTTER_SUCCESS)
						{
								AM_APP_LOG_WARNING("DSpotterSD_AddUttrEnd Fail!!(%d)\n", nErr);
								bErrorOccurred = TRUE;
								return;
						}		
				
						am_vos_mic_disable();
						nErr = DSpotterSD_TrainWord(g_hDSpotter, (char *)g_lpbyModelBuf, g_nModelBufSize, &nUsedSize);
						am_vos_mic_enable();
						if(nErr != DSPOTTER_SUCCESS)
						{
								AM_APP_LOG_WARNING("DSpotterSD_TrainWord Fail!!(%d)\n", nErr);
								bErrorOccurred = TRUE;
								return;
						}
						
						AM_APP_LOG_INFO("nUsedSize: %d\r\n", nUsedSize);
						AM_APP_LOG_INFO("Add utterance %d successfully!!\r\n", nUtterance + 1);
						
						for(int i = 0; i <= nUtterance; i++)
						{
								am_app_utils_task_send(AM_APP_TASK_AUD_PROCESSING, AM_APP_TASK_LED,
																				AM_APP_MESSAGE_SHORT, KEY_WORD_GOT_MESSAGE, NULL);
						}
						
						bStartTraining = FALSE;
						if(++nUtterance >= MAX_UTTERANCE)
						{
								psPtr = g_lpsMapID;
								psPtr += 160;
								nNumCommand = *((INT *)psPtr);
								psPtr += 2;
								for(int i = 0; i < MAX_UTTERANCE; i++)
								{
										psPtr[nNumCommand + i * 3] = g_sTargetID;
										psPtr[nNumCommand + i * 3 + 1] = 0xFF;
										psPtr[nNumCommand + i * 3 + 2] = 0xFF;
								}
								psPtr = g_lpsMapID + 160;
								*((INT *)psPtr) = nNumCommand + MAX_UTTERANCE * 3;

								am_vos_mic_disable();
								bErrorOccurred = !ProgramDataToFlash(nUsedSize);
								am_vos_mic_enable();
								if(bErrorOccurred)
								{
										AM_APP_LOG_WARNING("ProgramDataToFlash Fail!!\n");
										return;
								}
								
								g_hDSpotter = DSpotter_Init_Multi(g_lppbyModel[0], (BYTE **)&g_lpbyModelBuf, 1, k_nMaxTime, g_lpbyMemPool, g_nMemUsage, NULL, 0, &nErr, (BYTE *)&u32LicenseDataBegin);
								if(g_hDSpotter == NULL)
								{
										AM_APP_LOG_WARNING("DSpotter_Init_Multi Fail!!(%d)\n", nErr);
										bErrorOccurred = TRUE;
										return;
								}
								
#if ENABLE_AGC
								if((nErr = DSpotterAGC_Enable(g_hDSpotter)) != DSPOTTER_SUCCESS)
								{
									AM_APP_LOG_WARNING("DSpotterAGC_Enable Fail(%d)!!\n", nErr);
									bErrorOccurred = TRUE;
									return;
								}
								
								if((nErr = DSpotterAGC_SetMaxGain(g_hDSpotter, AGC_MAX_GAIN)) != DSPOTTER_SUCCESS)
								{
									AM_APP_LOG_WARNING("DSpotterAGC_SetMaxGain Fail!!(%d)\n", nErr);
									bErrorOccurred = TRUE;
									return;
								}
#endif
								
								nErr = DSpotter_SetResultMapID_Sep(g_hDSpotter, (BYTE *)g_lpsMapID);
								if(nErr != DSPOTTER_SUCCESS)
								{
										AM_APP_LOG_WARNING("DSpotter_SetResultMapID_Sep Fail!!(%d)\n", nErr);
										bErrorOccurred = TRUE;
										return;
								}
								
								AM_APP_LOG_INFO("Train command model successfully!!\r\n");
								AM_APP_LOG_INFO("Run command recognition flow!!\r\n");
								nUtterance = 0;
								g_bTrainSDModel = FALSE;
						}
						
						return;
				}
		}
	
		int32_t result = DSpotter_AddSample(g_hDSpotter, (short *)pi16InputBuffer, i16InputLength);
		if(result == 0)
		{
				int32_t id = DSpotter_GetResultMapID(g_hDSpotter);
				AM_APP_LOG_INFO("Command %d was detected!!\r\n", id);
				
				DSpotter_Reset(g_hDSpotter);
				
				am_vos_reset_detected_flag();
				g_sVosSys.ui8KwdDetectedFlag = 1;

				am_app_utils_task_send(AM_APP_TASK_AUD_PROCESSING, AM_APP_TASK_LED,
																				AM_APP_MESSAGE_SHORT, KEY_WORD_GOT_MESSAGE, NULL);
		}
}

INT DataFlash_Write(BYTE *dest, BYTE *src, INT nSize)
{
	memcpy(dest, src, nSize);

	return 0;
}

INT DataFlash_Erase(BYTE *dest, INT nSize)
{
	INT i;

	for(i = 0; i < nSize; i += k_nFlashPageSize)
		memset(dest + i, 0xFF, k_nFlashPageSize);

	return 0;
}

void Button_Handler(short sButtonIndex)
{
	AM_APP_LOG_INFO("Button %d was clicked!!\r\n", sButtonIndex);
	
	if(g_bTrainSDModel || g_bProcessButtonEvent)
		return;
	
	g_sTargetID = sButtonIndex;
	g_bProcessButtonEvent = TRUE;
}

#endif // configUSE_Cyberon_Spotter
