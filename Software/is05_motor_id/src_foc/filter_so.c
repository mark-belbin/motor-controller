//#############################################################################
//
// FILE:   filter_so.c
//
// TITLE:  C28x InstaSPIN filter library, second-order
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

#include "filter_so.h"

//*****************************************************************************
//
// FILTER_SO_getDenCoeffs
//
//*****************************************************************************
void
FILTER_SO_getDenCoeffs(FILTER_SO_Handle handle, float32_t *pa1, float32_t *pa2)
{
    FILTER_SO_Obj *obj = (FILTER_SO_Obj *)handle;

    *pa1 = obj->a1;
    *pa2 = obj->a2;

    return;
} // end of FILTER_SO_getDenCoeffs() function

//*****************************************************************************
//
// FILTER_SO_getInitialConditions
//
//*****************************************************************************
void
FILTER_SO_getInitialConditions(FILTER_SO_Handle handle, float32_t *px1,
                               float32_t *px2, float32_t *py1, float32_t *py2)
{
    FILTER_SO_Obj *obj = (FILTER_SO_Obj *)handle;

    *px1 = obj->x1;
    *px2 = obj->x2;

    *py1 = obj->y1;
    *py2 = obj->y2;

    return;
} // end of FILTER_SO_getInitialConditions() function

//*****************************************************************************
//
// FILTER_SO_getNumCoeffs
//
//*****************************************************************************
void
FILTER_SO_getNumCoeffs(FILTER_SO_Handle handle, float32_t *pb0, float32_t *pb1,
                       float32_t *pb2)
{
    FILTER_SO_Obj *obj = (FILTER_SO_Obj *)handle;

    *pb0 = obj->b0;
    *pb1 = obj->b1;
    *pb2 = obj->b2;

    return;
} // end of FILTER_SO_getNumCoeffs() function

//*****************************************************************************
//
// FILTER_SO_init
//
//*****************************************************************************
FILTER_SO_Handle
FILTER_SO_init(void *pMemory, const size_t numBytes)
{
    FILTER_SO_Handle handle;

    if((int16_t)numBytes < (int16_t)sizeof(FILTER_SO_Obj))
    {
        /*LDRA_INSPECTED 95 S MR12 11.3 "Below typecasting to NULL has
        no issues"*/
        return((FILTER_SO_Handle)NULL);
    }
    //
    // Assign the handle
    //
    /*LDRA_INSPECTED 94 S MR12 11.3 "Below typecasting to void * has
    no issues"*/
    /*LDRA_INSPECTED 95 S MR12 11.3 "Below typecasting to void * no issues"*/
    handle = (FILTER_SO_Handle)pMemory;

    /*LDRA_INSPECTED 71 S MR12 11.3 "Always, address of a static object is
    passed, so ok for wider scope"*/
    return(handle);
} // end of FILTER_SO_init() function

//*****************************************************************************
//
// FILTER_SO_setDenCoeffs
//
//*****************************************************************************
void
FILTER_SO_setDenCoeffs(FILTER_SO_Handle handle, const float32_t a1,
                       const float32_t a2)
{
    FILTER_SO_Obj *obj = (FILTER_SO_Obj *)handle;

    obj->a1 = a1;
    obj->a2 = a2;

    return;
} // end of FILTER_SO_setDenCoeffs() function

//*****************************************************************************
//
// FILTER_SO_setInitialConditions
//
//*****************************************************************************
void
FILTER_SO_setInitialConditions(FILTER_SO_Handle handle, const float32_t x1,
                               const float32_t x2, const float32_t y1,
                               const float32_t y2)
{
    FILTER_SO_Obj *obj = (FILTER_SO_Obj *)handle;

    obj->x1 = x1;
    obj->x2 = x2;

    obj->y1 = y1;
    obj->y2 = y2;

    return;
} // end of FILTER_SO_setInitialConditions() function

//*****************************************************************************
//
// FILTER_SO_setNumCoeffs
//
//*****************************************************************************
void
FILTER_SO_setNumCoeffs(FILTER_SO_Handle handle, const float32_t b0,
                       const float32_t b1, const float32_t b2)
{
    FILTER_SO_Obj *obj = (FILTER_SO_Obj *)handle;

    obj->b0 = b0;
    obj->b1 = b1;
    obj->b2 = b2;

    return;
} // end of FILTER_SO_setNumCoeffs() function

// end of file
