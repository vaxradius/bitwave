#include "am_vos_sys_config.h"
#include "am_vos_board_setup.h"

#include "am_app_utils.h"
#include "am_app_utils_task.h"

#include "am_vos_task.h"
#include "am_vos_init.h"

#if configUSE_Cyberon_Spotter

#include "DSpotterSDKApi.h"
#include "am_vos_DSpotter.h"

void *g_hDSpotter = NULL;
BYTE *g_lpbyMemPool = NULL;
INT g_nMemUsage;
BYTE *g_lppbyModel[MODEL_NUM] = {0};
BYTE *g_lppbyMapID[MODEL_NUM - 2] = {0};
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
	/** Initialize VR engine */
	
	// Unpack command data
	if(UnpackBin((BYTE *)&u32CMDDataBegin, g_lppbyModel, MODEL_NUM) < MODEL_NUM) {
		am_app_utils_stdio_printf(2, "Invalid bin\r\n");
		return NULL;
	}
	
	// Unpack map id
	if(UnpackBin(g_lppbyModel[5], g_lppbyMapID, MODEL_NUM - 2) < MODEL_NUM - 2) {
		am_app_utils_stdio_printf(2, "Invalid bin\r\n");
		return NULL;
	}

	// Check the memory usage
	nMemUsage = DSpotter_GetMemoryUsage_Multi(g_lppbyModel[0], (BYTE **)&g_lppbyModel[2], 1, k_nMaxTime);

	if(nMemUsage > xPortGetFreeHeapSize()){

		am_app_utils_stdio_printf(2, "Need more memory, memory usage = %d available memory = %d\r\n", nMemUsage, xPortGetFreeHeapSize());
		return NULL;
	}

	g_nMemUsage = nMemUsage;
	g_lpbyMemPool = pvPortMalloc(nMemUsage);

	hDSpotter = DSpotter_Init_Multi(g_lppbyModel[0], (BYTE **)&g_lppbyModel[1], 1, k_nMaxTime, g_lpbyMemPool, nMemUsage, NULL, 0, &nErr, (BYTE *)&u32LicenseDataBegin);
	if(hDSpotter == NULL){
		am_app_utils_stdio_printf(2, "g_hDSpotter == NULL\r\n");
		return NULL;
	}
#if ENABLE_AGC
	if((nErr = DSpotterAGC_Enable(hDSpotter)) != DSPOTTER_SUCCESS)
	{
		am_app_utils_stdio_printf(2, "Fail to enable AGC\r\n");
		return NULL;
	}
#endif
	/** Start recognize */	
	am_app_utils_stdio_printf(2, "[Wakeword detection]\r\n");

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
		INT nErr;
		static INT nTimeout = 0;
		static INT nStatus = 0;
		static INT nQuota = 200;
		static BOOL bQuotaRanOut = FALSE;
	
		if(nQuota < 0)
		{
			if(!bQuotaRanOut)
			{
				bQuotaRanOut = TRUE;
				am_app_utils_stdio_printf(2, "Quota ran out\r\n");
			}
			
			return;
		}
	
//		AM_APP_LOG_INFO("[AM-VoS] i16InputLength = %d\n", i16InputLength);
		int32_t result = DSpotter_AddSample(g_hDSpotter, (short *)pi16InputBuffer, i16InputLength);
		if(result == 0)
		{
				int32_t id = DSpotter_GetResultMapID(g_hDSpotter);
				if(nStatus == 0)
				{
						AM_APP_LOG_INFO("Hello UCLEAR\r\n");
						AM_APP_LOG_INFO("[Command detection]\r\n");
						nQuota--;
						nStatus = 1;
						nTimeout = 0;
					
						g_hDSpotter = DSpotter_Init_Multi(g_lppbyModel[0], (BYTE **)&g_lppbyModel[2], 1, k_nMaxTime, g_lpbyMemPool, g_nMemUsage, NULL, 0, &nErr, (BYTE *)&u32LicenseDataBegin);
						if(g_hDSpotter == NULL){
							am_app_utils_stdio_printf(2, "g_hDSpotter == NULL\r\n");
							return;
						}
						
						if((nErr = DSpotter_SetResultMapID_Multi(g_hDSpotter, (BYTE **)&g_lppbyMapID[1], 1)) != DSPOTTER_SUCCESS){
							am_app_utils_stdio_printf(2, "Fail to set map id\r\n");
							return;
						}
#if ENABLE_AGC						
						if((nErr = DSpotterAGC_Enable(g_hDSpotter)) != DSPOTTER_SUCCESS)
						{
							am_app_utils_stdio_printf(2, "Fail to enable AGC\r\n");
							return;
						}
#endif
				}
				else if(nStatus == 1)
				{
						if(id == 1) { AM_APP_LOG_INFO("Check battery\r\n"); }
						else if(id == 2) { AM_APP_LOG_INFO("Shutdown\r\n"); }
						else if(id == 3) { AM_APP_LOG_INFO("What can I say\r\n"); }
						else if(id == 4) { AM_APP_LOG_INFO("Phone command\r\n"); }
						else if(id == 5) { AM_APP_LOG_INFO("Play music\r\n"); }
						else if(id == 6) { AM_APP_LOG_INFO("Stop music\r\n"); }
						else if(id == 7) { AM_APP_LOG_INFO("Volume up\r\n"); }
						else if(id == 8) { AM_APP_LOG_INFO("Volume down\r\n"); }
						else if(id == 9) { AM_APP_LOG_INFO("Next track\r\n"); }
						else if(id == 10) { AM_APP_LOG_INFO("Previous track\r\n"); }
						else if(id == 11) { AM_APP_LOG_INFO("Connect intercom\r\n"); }
						else if(id == 12) { AM_APP_LOG_INFO("Stop intercom\r\n"); }
						else if(id == 13) { AM_APP_LOG_INFO("Mute microphone\r\n"); }
						else if(id == 14) { AM_APP_LOG_INFO("Unmute microphone\r\n"); }
						else if(id == 15) { AM_APP_LOG_INFO("End call\r\n"); }
						else if(id == 16) { AM_APP_LOG_INFO("Redial\r\n"); }
						else if(id == 17) { AM_APP_LOG_INFO("Enable conference\r\n"); }
						else if(id == 18) { AM_APP_LOG_INFO("Disable conference\r\n"); }
						else if(id == 19) { AM_APP_LOG_INFO("Start recording\r\n"); }
						else if(id == 20) { AM_APP_LOG_INFO("Stop recording\r\n"); }
						AM_APP_LOG_INFO("[Wakeword detection]\r\n");
						nQuota--;
						nStatus = 0;
					
						g_hDSpotter = DSpotter_Init_Multi(g_lppbyModel[0], (BYTE **)&g_lppbyModel[1], 1, k_nMaxTime, g_lpbyMemPool, g_nMemUsage, NULL, 0, &nErr, (BYTE *)&u32LicenseDataBegin);
						if(g_hDSpotter == NULL){
							am_app_utils_stdio_printf(2, "g_hDSpotter == NULL\r\n");
							return;
						}
#if ENABLE_AGC						
						if((nErr = DSpotterAGC_Enable(g_hDSpotter)) != DSPOTTER_SUCCESS)
						{
							am_app_utils_stdio_printf(2, "Fail to enable AGC\r\n");
							return;
						}
#endif
				}
				
				am_vos_reset_detected_flag();
				g_sVosSys.ui8KwdDetectedFlag = 1;

				am_app_utils_task_send(AM_APP_TASK_AUD_PROCESSING, AM_APP_TASK_LED,
																				AM_APP_MESSAGE_SHORT, KEY_WORD_GOT_MESSAGE, NULL);
		}
		
		if(nStatus == 1)
		{
				if((nTimeout += i16InputLength) > (16000 << 3))
				{
						AM_APP_LOG_INFO("[Wakeword detection]\r\n");
						nStatus = 0;
					
						g_hDSpotter = DSpotter_Init_Multi(g_lppbyModel[0], (BYTE **)&g_lppbyModel[1], 1, k_nMaxTime, g_lpbyMemPool, g_nMemUsage, NULL, 0, &nErr, (BYTE *)&u32LicenseDataBegin);
						if(g_hDSpotter == NULL){
							am_app_utils_stdio_printf(2, "g_hDSpotter == NULL\r\n");
							return;
						}
#if ENABLE_AGC						
						if((nErr = DSpotterAGC_Enable(g_hDSpotter)) != DSPOTTER_SUCCESS)
						{
							am_app_utils_stdio_printf(2, "Fail to enable AGC\r\n");
							return;
						}
#endif
				}
		}
}

#endif // configUSE_Cyberon_Spotter
