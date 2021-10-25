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
SHORT g_lpsMapID[160 + 2 + 3 * MAX_UTTERANCE] = {0};
extern uint32_t u32CMDDataBegin;
extern uint32_t u32LicenseDataBegin;

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
	if(UnpackBin((BYTE *)&u32CMDDataBegin, g_lppbyModel, MODEL_NUM) < MODEL_NUM) {
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

	hDSpotter = DSpotterSD_Init(g_lppbyModel[0], g_lppbyModel[MODEL_NUM - 2], g_lpbyMemPool, nMemUsage, &nErr);
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
	
	// Initialize map id
	psPtr = g_lpsMapID;
	psPtr += 160;
	*((INT *)psPtr) = MAX_UTTERANCE * 3;
	psPtr += 2;
	for(int i = 0; i < MAX_UTTERANCE * 3; i++)
	{
		if(i % 3 == 0)
			psPtr[i] = 0x0;
		else
			psPtr[i] = 0xFFFF;
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
		static BOOL bTrainSDModel = TRUE;
		static BOOL bStartTraining = FALSE;
		static BOOL bErrorOccurred = FALSE;
		static INT nUtterance = 0;
		INT nUsedSize;
		INT nErr;
	
//		AM_APP_LOG_INFO("[AM-VoS] i16InputLength = %d\n", i16InputLength);
	
		if(bErrorOccurred)
		{
				am_app_utils_task_send(AM_APP_TASK_AUD_PROCESSING, AM_APP_TASK_LED,
																				AM_APP_MESSAGE_SHORT, KEY_WORD_GOT_MESSAGE, NULL);
				return;
		}
	
		if(bTrainSDModel)
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
						
//						AM_APP_LOG_INFO("nUsedSize: %d\r\n", nUsedSize);
						AM_APP_LOG_INFO("Add utterance %d successfully!!\r\n", nUtterance + 1);
						
						for(int i = 0; i <= nUtterance; i++)
						{
								am_app_utils_task_send(AM_APP_TASK_AUD_PROCESSING, AM_APP_TASK_LED,
																				AM_APP_MESSAGE_SHORT, KEY_WORD_GOT_MESSAGE, NULL);
						}
						
						bStartTraining = FALSE;
						if(++nUtterance >= MAX_UTTERANCE)
						{
								bTrainSDModel = FALSE;
								
								g_hDSpotter = DSpotter_Init_Multi(g_lppbyModel[0], (BYTE **)&g_lpbyModelBuf, 1, k_nMaxTime, g_lpbyMemPool, g_nMemUsage, NULL, 0, &nErr, (BYTE *)&u32LicenseDataBegin);
								if(g_hDSpotter == NULL)
								{
										AM_APP_LOG_WARNING("DSpotter_Init_Multi Fail!!(%d)\n", nErr);
										bErrorOccurred = TRUE;
										return;
								}
								
								nErr = DSpotter_SetResultMapID_Sep(g_hDSpotter, (BYTE *)g_lpsMapID);
								if(nErr != DSPOTTER_SUCCESS)
								{
										AM_APP_LOG_WARNING("DSpotter_SetResultMapID_Sep Fail!!(%d)\n", nErr);
										bErrorOccurred = TRUE;
										return;
								}
								
								AM_APP_LOG_INFO("Train SD model successfully!!\r\n");
						}
						
						return;
				}
		}
	
		int32_t result = DSpotter_AddSample(g_hDSpotter, (short *)pi16InputBuffer, i16InputLength);
		if(result == 0)
		{
				int32_t id = DSpotter_GetResultMapID(g_hDSpotter);
				if(id == 0)
					AM_APP_LOG_INFO("Command was detected!!\r\n");
				
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

#endif // configUSE_Cyberon_Spotter
