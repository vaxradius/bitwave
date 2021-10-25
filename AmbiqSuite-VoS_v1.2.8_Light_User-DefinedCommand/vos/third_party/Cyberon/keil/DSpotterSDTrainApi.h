
#ifndef __DSPOTTERSDTRAIN_API_H
#define __DSPOTTERSDTRAIN_API_H

#include "DSpotterSDKApi.h"

#ifdef __cplusplus
extern "C"{
#endif

// Purpose: Create a trainer
// lpbyCYBase(IN): a base model for trainer
// lpbyGroup(IN): a SI group model for trainer
// lpbyTrimap(IN): a trimap model for trainer
// lpbyMemPool(OUT): memory buffer for trainer
// nMemSize(IN): memory buffer size
// pnErr(OUT): CSPOTTER_SUCCESS indicates success, else error code. It can be NULL.
// Return: a trainer handle or NULL
DSPDLL_API HANDLE DSpotterSD_Init(BYTE *lpbyCYBase, BYTE *lpbyTrimap, BYTE *lpbyMemPool, INT nMemSize, INT *pnErr);

// Purpose: Destroy a trainer (free resources)
// hDSpotter(IN): a handle of the trainer
// Return: Success or error code
DSPDLL_API INT DSpotterSD_Release(HANDLE hDSpotter);

DSPDLL_API INT DSpotterSD_GetMemoryUsage(BYTE *lpbyCYBase, BYTE *lpbyTrimap);

// Purpose: Reset trainer
// hDSpotter(IN): a handle of the trainer
// Return: Success or error code
DSPDLL_API INT DSpotterSD_Reset(HANDLE hDSpotter);

// Purpose: Prepare to add a new utterance for training
// hDSpotter(IN): a handle of the trainer
// lpszDataBuf(IN/OUT): the pointer of data buffer, it must be a DATA FALSH pointer, the buffer is used to store the voice data
// nBufSize(IN): the size of data buffer
// Return: Success or error code
DSPDLL_API INT DSpotterSD_AddUttrStart(HANDLE hDSpotter, SHORT *lpsDataBuf, INT nBufSize);

// Purpose: Transfer voice samples to the trainer for training
// hDSpotter(IN): a handle of the trainer
// lpsSample(IN): the pointer of voice data buffer
// nNumSample(IN): the number of voice data (a unit is a short, we prefer to add 160 samples per call)
// Return: "CSPOTTER_ERR_NeedMoreSample" indicates call this function again, otherwise Success or error code
DSPDLL_API INT DSpotterSD_AddSample(HANDLE hDSpotter, SHORT *lpsSample, INT nNumSample);

// Purpose: Finish the adding process
// hDSpotter(IN): a handle of the trainer
// Return: Success or error code
DSPDLL_API INT DSpotterSD_AddUttrEnd(HANDLE hDSpotter);

// Purpose: Get the utterance boundary
// hDSpotter(IN): a handle of the trainer
// pnStart(OUT): starting point (samples)
// pnEnd(OUT): ending point (samples)
// Return: Success or error code
DSPDLL_API INT DSpotterSD_GetUttrEPD(HANDLE hDSpotter, INT *pnStart, INT *pnEnd);


// Purpose: Train a voice tag
// hDSpotter(IN): a handle of the trainer
// lpszModelAddr(IN/OUT): the pointer of model buffer, it must be a DATA FALSH pointer
// nBufSize(IN): the size of model buffer
// pnUsedSize(OUT): the model size
// Return: Success or error code
DSPDLL_API INT DSpotterSD_TrainWord(HANDLE hDSpotter, char *lpszModelAddr, INT nBufSize, INT *pnUsedSize);

DSPDLL_API INT DSpotterSD_DeleteWord(HANDLE hDSpotter, char *lpszModelAddr, INT nIdx, INT *pnUsedSize);


// The default is 1200. It is equal to the RMS amplitude and the wave amplitude is more than 4 times of that amount.
DSPDLL_API INT DSpotterSD_SetBackgroundEnergyThreshd(HANDLE hCSpotter, INT nThreshold);

// The default is 0. The range is [-100, 100].
DSPDLL_API INT DSpotterSD_SetEpdLevel(HANDLE hCSpotter, INT nEpdLevel);

// Purpose: Enable AGC(Auto gain control). AGC is defaultly closed. Must call this after DSpotter_Init_Multi() to activate.
// hDSpotter(IN): a handle of the recognizer
// Return: Success or error code
DSPDLL_API INT DSpotterSDAGC_Enable(HANDLE hDSpotter);

// Purpose: Disable AGC(Auto gain control).
// hDSpotter(IN): a handle of the recognizer
// Return: Success or error code
DSPDLL_API INT DSpotterSDAGC_Disable(HANDLE hDSpotter);

// Purpose: Set the upper bound of AGC Gain (and also the current AGC Gain)
// hDSpotter(IN): a handle of the recognizer
// fMaxGain(IN): The upper bound of AGC gain; Default is set to 32
// Return: Success or error code
// Note: 1 <= fMaxGain <= 32
DSPDLL_API INT DSpotterSDAGC_SetMaxGain(HANDLE hDSpotter, FLOAT fMaxGain);

// Purpose: Set threshholds to trigger AGC Gain increasing.
// hDSpotter(IN): a handle of the recognizer
// sLowerThrd(IN): AGC Gain will increasing if (current peak of frame)*(AGC Gain) < sLowerThrd; Default is set to 5000
// Return: Success or error code
// Note: 0 <= sLowerThrd <= 10000
DSPDLL_API INT DSpotterSDAGC_SetIncGainThrd(HANDLE hDSpotter, SHORT sLowerThrd);

// Purpose: Callback to get data after AGC.
// lpsOutputSample: Samples after AGC.
// nSampleNum: Number of samples
// lpParam: Parameters.
#if defined(_WIN32)
typedef INT (__stdcall *DSpotterAGC_GetAGCData_Callback)(SHORT *lpsOutputSample, INT nSampleNum, VOID *lpParam);
#else
typedef INT (*DSpotterSDAGC_GetAGCData_Callback)(SHORT *lpsOutputSample, INT nSampleNum, VOID *lpParam);
#endif

// Purpose: Set Callback to get data after AGC.
// hDSpotter(IN): a handle of the recognizer
// lpfnCallback(IN): callback
// lpParam(IN): Parameters
DSPDLL_API INT DSpotterSDAGC_SetCallback(HANDLE hDSpotter, DSpotterAGC_GetAGCData_Callback lpfnCallback, VOID *lpParam);

#ifdef __cplusplus
}
#endif

#endif // __DSPOTTERSDTRAIN_API_H
