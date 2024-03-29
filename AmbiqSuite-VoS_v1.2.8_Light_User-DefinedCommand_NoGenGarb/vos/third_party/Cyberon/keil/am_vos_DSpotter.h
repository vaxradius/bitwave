//*****************************************************************************
//
//! @file am_app_KWD_DSpotter.h
//!
//! @brief header file of Cyberon Spotter
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2017, Ambiq Micro
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision v1.2.11 of the AmbiqSuite Development Package.
//
//*****************************************************************************

#ifndef AM_APP_KWD_DSpotter_H
#define AM_APP_KWD_DSpotter_H

#define MODEL_NUM					(7)

#define k_nMaxTime				(500)

#define MAX_UTTERANCE			(3)

#define k_nFlashPageSize	(4096)

#define MAP_ID_FILE_SIZE	(160 + 2 + (3 * MAX_UTTERANCE + 1)) // Must be 4-byte alignment for programming to flash

#define ENABLE_AGC 				(0)

#define AGC_MAX_GAIN			(4)

#define ENERGY_THRESHOLD	(1200)

#define SKIP_FRAME				(1)

#define FLASH_PAGE_INDEX	(1)

#define FLASH_PAGE_NUM		(1)

#define CONFIDENCE_REWARD	(20)

extern void *g_hDSpotter;

//*****************************************************************************
// Function declaration
//*****************************************************************************
extern void *DSpotterInit(void);

extern int DSpotter_AddSample(void *hDSpotter, short *lpsSample, int nNumSample);
extern int DSpotter_GetResult(void *hDSpotter);

extern void Button_Handler(short sButtonIndex);

#endif
