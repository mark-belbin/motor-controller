//#############################################################################
// $TI Release: MotorControl SDK v3.00.01.00 $
// $Release Date: Tue May 26 19:13:58 CDT 2020 $
// $Copyright:
// Copyright (C) 2017-2019 Texas Instruments Incorporated - http://www.ti.com/
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

//! \file   libraries/utilities/datalog/source/datalog.c
//! \brief  Portable C fixed point code.  These functions define the
//!         data logging (DATALOG) module routines
//!

// **************************************************************************
// the includes
#include "datalog.h"

// **************************************************************************
// the defines

// **************************************************************************
// the globals

#ifdef _DATALOG_EN_
DATALOG_Obj datalog;
DATALOG_Handle datalogHandle;                   //!< the handle for the Datalog object

float32_t datalogBuff1[DATA_LOG_BUFF_SIZE + 2];
float32_t datalogBuff2[DATA_LOG_BUFF_SIZE + 2];
float32_t datalogBuff3[DATA_LOG_BUFF_SIZE + 2];
float32_t datalogBuff4[DATA_LOG_BUFF_SIZE + 2];

#pragma DATA_SECTION(datalogBuff1, "datalog_data");
#pragma DATA_SECTION(datalogBuff2, "datalog_data");
#pragma DATA_SECTION(datalogBuff3, "datalog_data");
#pragma DATA_SECTION(datalogBuff4, "datalog_data");
#endif

// **************************************************************************
// the functions

DATALOG_Handle DATALOG_init(void *pMemory, const size_t numBytes)
{
    DATALOG_Handle handle;

    if(numBytes < sizeof(DATALOG_Obj))
    {
        return((DATALOG_Handle)NULL);
    }

    // assign the handle
    handle = (DATALOG_Handle)pMemory;

    DATALOG_Obj *obj = (DATALOG_Obj *)handle;

    obj->size = DATA_LOG_BUFF_SIZE;
    obj->cntr = 0;

    return(handle);
} // end of DATALOG_init() function

// end of file
