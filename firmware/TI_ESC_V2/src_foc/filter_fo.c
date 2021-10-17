//#############################################################################
//
// FILE:   filter_fo.c
//
// TITLE:  C28x InstaSPIN filter library, first-order
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

#ifdef __TMS320C28XX_CLA__
#pragma CODE_SECTION(FILTER_FO_getDenCoeffs,"Cla1Prog2");
#pragma CODE_SECTION(FILTER_FO_getInitialConditions,"Cla1Prog2");
#pragma CODE_SECTION(FILTER_FO_getNumCoeffs,"Cla1Prog2");
#pragma CODE_SECTION(FILTER_FO_init,"Cla1Prog2");
#pragma CODE_SECTION(FILTER_FO_setDenCoeffs,"Cla1Prog2");
#pragma CODE_SECTION(FILTER_FO_setInitialConditions,"Cla1Prog2");
#pragma CODE_SECTION(FILTER_FO_setNumCoeffs,"Cla1Prog2");
#endif // __TMS320C28XX_CLA__

#include "filter_fo.h"

//*****************************************************************************
//
// FILTER_FO_getDenCoeffs
//
//*****************************************************************************
void
FILTER_FO_getDenCoeffs(FILTER_FO_Handle handle, float32_t *pa1)
{
    FILTER_FO_Obj *obj = (FILTER_FO_Obj *)handle;

    *pa1 = obj->a1;

    return;
} // end of FILTER_FO_getDenCoeffs() function

//*****************************************************************************
//
// FILTER_FO_getInitialConditions
//
//*****************************************************************************
void
FILTER_FO_getInitialConditions(FILTER_FO_Handle handle, float32_t *px1,
                               float32_t *py1)
{
    FILTER_FO_Obj *obj = (FILTER_FO_Obj *)handle;

    *px1 = obj->x1;

    *py1 = obj->y1;

    return;
} // end of FILTER_FO_getInitialConditions() function

//*****************************************************************************
//
// FILTER_FO_getNumCoeffs
//
//*****************************************************************************
void
FILTER_FO_getNumCoeffs(FILTER_FO_Handle handle, float32_t *pb0, float32_t *pb1)
{
    FILTER_FO_Obj *obj = (FILTER_FO_Obj *)handle;

    *pb0 = obj->b0;
    *pb1 = obj->b1;

    return;
} // end of FILTER_FO_getNumCoeffs() function

//*****************************************************************************
//
// FILTER_FO_init
//
//*****************************************************************************
FILTER_FO_Handle FILTER_FO_init(void *pMemory,
                                const size_t numBytes)
{
    FILTER_FO_Handle handle;

    if((int16_t)numBytes < (int16_t)sizeof(FILTER_FO_Obj))
    {
        /*LDRA_INSPECTED 95 S MR12 11.3 "Below typecasting to NULL has
        no issues"*/
        return((FILTER_FO_Handle)NULL);
    }

    //
    // Assign the handle
    //
    /*LDRA_INSPECTED 94 S MR12 11.3 "Below typecasting to void * has
    no issues"*/
    /*LDRA_INSPECTED 95 S MR12 11.3 "Below typecasting to void * no issues"*/
    handle = (FILTER_FO_Handle)pMemory;

    /*LDRA_INSPECTED 71 S MR12 11.3 "Always, address of a static object is
    passed, so ok for wider scope"*/
    return(handle);
} // end of FILTER_FO_init() function

//*****************************************************************************
//
// FILTER_FO_setDenCoeffs
//
//*****************************************************************************
void
FILTER_FO_setDenCoeffs(FILTER_FO_Handle handle, const float32_t a1)
{
    FILTER_FO_Obj *obj = (FILTER_FO_Obj *)handle;

    obj->a1 = a1;

    return;
} // end of FILTER_FO_setDenCoeffs() function

//*****************************************************************************
//
// FILTER_FO_setInitialConditions
//
//*****************************************************************************
void
FILTER_FO_setInitialConditions(FILTER_FO_Handle handle, const float32_t x1,
                               const float32_t y1)
{
    FILTER_FO_Obj *obj = (FILTER_FO_Obj *)handle;

    obj->x1 = x1;

    obj->y1 = y1;

    return;
} // end of FILTER_FO_setInitialConditions() function

//*****************************************************************************
//
// FILTER_FO_setNumCoeffs
//
//*****************************************************************************
void
FILTER_FO_setNumCoeffs(FILTER_FO_Handle handle, const float32_t b0,
                       const float32_t b1)
{
    FILTER_FO_Obj *obj = (FILTER_FO_Obj *)handle;

    obj->b0 = b0;
    obj->b1 = b1;

    return;
} // end of FILTER_FO_setNumCoeffs() function

// end of file
