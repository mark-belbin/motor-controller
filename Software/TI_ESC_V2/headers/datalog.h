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

#ifndef DATALOG_H
#define DATALOG_H

//! \file   libraries/utilities/datalog/include/datalog.h
//! \brief  Contains the public interface to the
//!         data logging (DATALOG) module routines
//!

// **************************************************************************
// the includes

#include "libraries/math/include/math.h"

//!
//!
//! \defgroup DATALOG DATALOG
//!
//! @{

#ifdef __cplusplus
extern "C" {
#endif

// **************************************************************************
// the defines
#define DATA_LOG_BUFF_NUM       4

#define DATA_LOG_BUFF_SIZE      400     //20*20

#define DLOG_BURST              20      // write 820 to the register for
                                        // a burst size of 20

#define DLOG_TRANSFER           42      // [(MEM_BUFFER_SIZE/(BURST)]

//! \brief Defines the default initialization for the DLOG object
//!
#define DATALOG_DEFAULTS {                          \
                            {NULL,                 \
                             NULL,                 \
                             NULL,                 \
                             NULL},               \
                            true,                  \
                            false,                 \
                            0,                     \
                            DATA_LOG_BUFF_SIZE     \
                         }

// **************************************************************************
// the typedefs

//! \brief Defines the data logging (DATALOG) object
//!
typedef struct _DATALOG_OBJ_
{
    float32_t  *iptr[DATA_LOG_BUFF_NUM];    //!< Input: First input pointer

    bool  flag_enableLogData;             //!< Parameter: Maximum data buffer
    bool  flag_enableLogOneShot;          //!< Parameter: Maximum data buffer
    uint16_t  cntr;                       //!< Variable:  Data log counter
    uint16_t  size;                       //!< Parameter: Maximum data buffer
} DATALOG_Obj;

//! \brief Defines the DATALOG handle
//!
typedef struct _DATALOG_Obj_   *DATALOG_Handle;

// **************************************************************************
// the globals
extern float32_t datalogBuff1[DATA_LOG_BUFF_SIZE+2];
extern float32_t datalogBuff2[DATA_LOG_BUFF_SIZE+2];
extern float32_t datalogBuff3[DATA_LOG_BUFF_SIZE+2];
extern float32_t datalogBuff4[DATA_LOG_BUFF_SIZE+2];

//! \brief Defines the DATALOG object
//!
extern DATALOG_Obj datalog;
extern DATALOG_Handle datalogHandle;

// **************************************************************************
// the globals

// **************************************************************************
// the function prototypes

//! \brief     Initializes the data logger
//! \param[in] ptr  The pointer to memory
extern DATALOG_Handle DATALOG_init(void *pMemory, const size_t numBytes);

//! \brief     Updates the data logger
//! \param[in] ptr  The pointer to memory
static inline void DATALOG_update(DATALOG_Handle handle)
{
    DATALOG_Obj *obj = (DATALOG_Obj *)handle;

    if(obj->flag_enableLogData == true)
    {
        if(obj->cntr >= obj->size)
        {
            obj->cntr = 0;

            if(obj->flag_enableLogOneShot == true)
            {
                obj->flag_enableLogData = false;
                obj->flag_enableLogOneShot = false;
            }
        }

        datalogBuff1[obj->cntr] = (*obj->iptr[0]);
        datalogBuff2[obj->cntr] = (*obj->iptr[1]);
        datalogBuff3[obj->cntr] = (*obj->iptr[2]);
        datalogBuff4[obj->cntr] = (*obj->iptr[3]);

        obj->cntr++;
    }
    else if(obj->flag_enableLogOneShot == true)
    {
        obj->flag_enableLogData = true;
        obj->cntr = 0;
    }
}

//! \brief     Updates the data logger
//! \param[in] ptr  The pointer to memory
static inline void DATALOG_updateWithDMA(DATALOG_Handle handle)
{
    DATALOG_Obj *obj = (DATALOG_Obj *)handle;

    uint16_t number = obj->size - 1;

    datalogBuff1[number] = (*obj->iptr[0]);
    datalogBuff2[number] = (*obj->iptr[1]);
    datalogBuff3[number] = (*obj->iptr[2]);
    datalogBuff4[number] = (*obj->iptr[3]);
}

#ifdef __cplusplus
}
#endif // extern "C"

//! @} // ingroup
#endif // end of DATALOG_H definition

