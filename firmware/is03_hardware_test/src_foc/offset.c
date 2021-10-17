//#############################################################################
//
// FILE:   offset.c
//
// TITLE:  C28x InstaSPIN offset library (floating point)
//
//#############################################################################
// $TI Release: MotorControl SDK v3.00.01.00 $
// $Release Date: Tue May 26 19:13:58 CDT 2020 $
// $Copyright:
// Copyright (C) 2017-2018 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
//
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

#include "offset.h"

//*****************************************************************************
//
// OFFSET_getBeta
//
//*****************************************************************************
float32_t
OFFSET_getBeta(OFFSET_Handle handle)
{
    OFFSET_Obj *obj = (OFFSET_Obj *)handle;
    float32_t b0;
    float32_t beta;

    b0 = FILTER_FO_get_b0(obj->filterHandle);

    beta = (float32_t)2.0 * b0 / ((float32_t)1.0 - b0);

    return(beta);
} // end of OFFSET_getBeta() function

//*****************************************************************************
//
// OFFSET_init
//
//*****************************************************************************
OFFSET_Handle
OFFSET_init(void *pMemory, const size_t numBytes)
{
    OFFSET_Handle handle;
    OFFSET_Obj *obj;

    //
    // Check to ensure memory allotted can accommodate object memory needs
    //
    if(numBytes < sizeof(OFFSET_Obj))
    {
        return((OFFSET_Handle)NULL);
    }

    //
    // Assign the handle
    //
    handle = (OFFSET_Handle)pMemory;
    obj = (OFFSET_Obj *)handle;

    obj->filterHandle = FILTER_FO_init(&(obj->filter),sizeof(obj->filter));

    return(handle);
} // end of OFFSET_init() function

//*****************************************************************************
//
// OFFSET_setBeta
//
//*****************************************************************************
void
OFFSET_setBeta(OFFSET_Handle handle, const float32_t beta_rad)
{
    OFFSET_Obj *obj = (OFFSET_Obj *)handle;
    float32_t a1 = (beta_rad - (float32_t)2.0) / (beta_rad + (float32_t)2.0);
    float32_t b0 = beta_rad / (beta_rad + (float32_t)2.0);
    float32_t b1 = b0;

    FILTER_FO_setDenCoeffs(obj->filterHandle,a1);
    FILTER_FO_setNumCoeffs(obj->filterHandle,b0,b1);

    return;
} // end of OFFSET_setBeta() function

//*****************************************************************************
//
// OFFSET_setInitCond
//
//*****************************************************************************
void
OFFSET_setInitCond(OFFSET_Handle handle, const float32_t initCond)
{
    OFFSET_Obj *obj = (OFFSET_Obj *)handle;

    FILTER_FO_setInitialConditions(obj->filterHandle,initCond,initCond);
    obj->value = initCond;

    return;
} // end of OFFSET_setInitCond() function

//*****************************************************************************
//
// OFFSET_setOffset
//
//*****************************************************************************
void
OFFSET_setOffset(OFFSET_Handle handle, float32_t offsetValue)
{
    OFFSET_Obj *obj = (OFFSET_Obj *)handle;

    obj->value = offsetValue;

    return;
} // end of OFFSET_setOffset() function

// end of file
