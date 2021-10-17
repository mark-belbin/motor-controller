//#############################################################################
//
// FILE:   traj.c
//
// TITLE:  C28x InstaSPIN trajectory library (floating point)
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

#include "traj.h"

#ifdef __TMS320C28XX_CLA__
#pragma CODE_SECTION(TRAJ_init,"Cla1Prog2");
#endif

//*****************************************************************************
//
// TRAJ_init
//
//*****************************************************************************
TRAJ_Handle
TRAJ_init(void *pMemory, const size_t numBytes)
{
    TRAJ_Handle handle;

    if((int16_t)numBytes < (int16_t)sizeof(TRAJ_Obj))
    {
        /*LDRA_INSPECTED 95 S MR12 11.3 "Below typecasting to NULL has no
        issues"*/
        return((TRAJ_Handle)NULL);
    }

    //
    // Assign the handle
    //
    /*LDRA_INSPECTED 94 S MR12 11.3 "Below typecasting to void * has no
    issues"*/
    /*LDRA_INSPECTED 95 S MR12 11.3 "Below typecasting to void * no issues"*/
    handle = (TRAJ_Handle)pMemory;

    /*LDRA_INSPECTED 71 S MR12 11.3 "Always, address of a static object is
    passed, so ok for wider scope"*/
    return(handle);
} // end of TRAJ_init() function

// end of file
