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

#ifndef GRAPH_H
#define GRAPH_H

//! \file   libraries/utilities/diagnostic/include/graph.h
//! \brief  Contains the public interface to the
//!         graphing (GRAPH) module routines
//!

// **************************************************************************
// the includes
#include "libraries/math/include/math.h"

// modules

// drivers

// platforms

// **************************************************************************
// the defines
#define GRAPH_BUFFER_NR     2       // Number of data arrays

#define GRAPH_BUFFER_SIZE   512     // Size of data arrays

#if (16 < GRAPH_BUFFER_SIZE && GRAPH_BUFFER_SIZE <= 32)
#define GRAPH_BUFFER_MASK (32-1)
#elif(32 < GRAPH_BUFFER_SIZE && GRAPH_BUFFER_SIZE <= 64)
#define GRAPH_BUFFER_MASK (64-1)
#elif(64 < GRAPH_BUFFER_SIZE && GRAPH_BUFFER_SIZE <= 128)
#define GRAPH_BUFFER_MASK (128-1)
#elif(128 < GRAPH_BUFFER_SIZE && GRAPH_BUFFER_SIZE <= 256)
#define GRAPH_BUFFER_MASK (256-1)
#elif(256 < GRAPH_BUFFER_SIZE && GRAPH_BUFFER_SIZE <= 512)
#define GRAPH_BUFFER_MASK (512-1)
#elif(512 < GRAPH_BUFFER_SIZE && GRAPH_BUFFER_SIZE <= 1024)
#define GRAPH_BUFFER_MASK (1024-1)
#elif(1024 < GRAPH_BUFFER_SIZE && GRAPH_BUFFER_SIZE <= 2048)
#define GRAPH_BUFFER_MASK (2048-1)
#elif(2048 < GRAPH_BUFFER_SIZE && GRAPH_BUFFER_SIZE <= 4096)
#define GRAPH_BUFFER_MASK (4096-1)
#elif(4096 < GRAPH_BUFFER_SIZE && GRAPH_BUFFER_SIZE <= 8192)
#define GRAPH_BUFFER_MASK (8192-1)
#else
#error GRAPH_BUFFER_SIZE is outside GRAPH_BUFFER_MASK definition in graph.h
#endif

//! \brief Initialization values of global variables
//!
#define GRAPH_VARS_INIT {                   \
                         0,                 \
                         0,                 \
                         10,                \
                         STEP_RP_CURRENT,   \
                         0,                 \
                        }

#define GRAPH_STEP_VARS_INIT {              \
                              0,            \
                                            \
                              NULL,         \
                              NULL,         \
                              NULL,         \
                                            \
                              NULL,         \
                              NULL,         \
                              NULL,         \
                                            \
                              20,           \
                              20,           \
                                            \
                              0.0,          \
                              -1.0,         \
                             }

// **************************************************************************
// the typedefs

//! \brief Enumeration for the number of buffers
//!
typedef enum
{
    GRAPH_BUFFER_NR0=0,        //!< Buffer define 0
    GRAPH_BUFFER_NR1,          //!< Buffer define 1
    GRAPH_BUFFER_NR2,          //!< Buffer define 2
    GRAPH_BUFFER_NR3           //!< Buffer define 3
} GRAPH_BufferNR_e;

typedef enum
{
    GRAPH_STEP_RP_CURRENT=1,   //!< current step response
    GRAPH_STEP_RP_SPEED=2,     //!< speed step response
    GRAPH_STEP_RP_VIEW=3       //!< No Used
} GRAPH_StepResponseMode_e;

typedef struct _GRAPH_Buffer_t_
{
    float32_t   data[GRAPH_BUFFER_SIZE];
    uint16_t    read;           // points to field with oldest content
    uint16_t    write;          // points to empty field
}GRAPH_Buffer_t;

typedef struct _GRAPH_Vars_t_
{
  uint16_t bufferCounter;       // used as an index into the buffer
  uint16_t bufferTickCounter;   // used to count the interrupts
  uint16_t bufferTick;          // defines how many interrupts per graph write

  GRAPH_StepResponseMode_e bufferMode; // used to define different values to
                                       // record for each mode during run time

  GRAPH_Buffer_t bufferData[GRAPH_BUFFER_NR];
}GRAPH_Vars_t;

typedef struct _GRAPH_StepVars_t_
{
  uint16_t stepResponse;        // value to start the step response generation

  volatile float32_t *pSpeed_in;  // Pointer to Real value of Speed
  float32_t *pId_in;              // Pointer to Real value of Id
  float32_t *pIq_in;              // Pointer to Real value of Iq

  volatile float32_t *pSpeed_ref; // Pointer to Setting value of spdRef
  float32_t *pId_ref;             // Pointer to Setting value of IdRef
  float32_t *pIq_ref;             // Pointer to Setting value of IqRef

  float32_t spdRef_Default;       // Default starting value of spdRef
  float32_t spdRef_StepSize;      // Step size of spdRef

  float32_t IdRef_Default;        // Default starting value of IdRef
  float32_t IdRef_StepSize;       // Step size of IdRef
}GRAPH_StepVars_t;

#ifdef _STEP_RESPONSE_EN_
extern GRAPH_Vars_t gGraphVars;
extern GRAPH_StepVars_t gStepVars;
#endif

// **************************************************************************
// the function prototypes

//! \brief      Read from the buffer
//! \param[in]  GRAPH_Buffer_t          The pointer to the buffer data
//! \param[out]    pWord            Read data from the buffer
bool GRAPH_BufferOut(GRAPH_Buffer_t *pBuffer, float32_t *pWord);

//! \brief      Write into the buffer
//! \param[in]  GRAPH_Buffer_t          The pointer to the buffer data
//! \param[in]    word            Write data into the buffer
bool GRAPH_BufferIn(GRAPH_Buffer_t *pBuffer, float32_t data);

//! \brief      Init function and reset function
//! \param[in]  GRAPH_Vars_t          The pointer to the gGraphVars data
void GRAPH_BufferInit(GRAPH_Vars_t *pGraphVars);

//! \brief      Data gathering function for any iq value
//! \param[in]  pGraphVars  Pointer to the graph variables
//! \param[in]  pSpeed_in The pointer to the speed reference data
//! \param[in]  pSpeed_in The pointer to the speed reference data
//! \param[in]  pSpeed_in The pointer to the speed reference data
//! \param[in]  pSpeed_in The pointer to the speed reference data
//! \param[in]  pSpeed_in The pointer to the speed reference data
//! \param[in]  pSpeed_in The pointer to the speed reference data
void GRAPH_DataPointerInit(GRAPH_StepVars_t *pStepVars,
        volatile float32_t *pSpeed_in, float32_t *pId_in, float32_t *pIq_in,
        volatile float32_t *pSpeed_ref, float32_t *pId_ref, float32_t *pIq_ref);

//! \brief      Data gathering function for any iq value
//! \param[in]  GRAPH_Vars_t          The pointer to the gGraphVars data
//! \param[in]  GRAPH_BufferNR_e      Definition of buffer number
//! \param[in]  gData                 Recorded iq data
//! \param[in]  TriggerValue          Trigger value to start the data recording
void GRAPH_Data_Gather (GRAPH_Vars_t *pGraphVars, GRAPH_BufferNR_e bufferNum,
                        float32_t gData);

//! \brief      Sets the values to collect in a data array
//! \param[in]  handle      The controller (CTRL) handle
//! \param[in]  pGraphVars  Pointer to the graph variables
//! \param[in]  pStepVars   Pointer to the step variables
void GRAPH_DATA(GRAPH_Vars_t *pGraphVars, GRAPH_StepVars_t *pStepVars);

//! \brief      Sets the values to collect in a data array
//! \param[in]  handle      The controller (CTRL) handle
//! \param[in]  pGraphVars  Pointer to the graph variables
//! \param[in]  pStepVars   Pointer to the step variables
void GRAPH_generateStepResponse(GRAPH_Vars_t *pGraphVars,
                                GRAPH_StepVars_t *pStepVars);

//! @} //defgroup
#endif // end of GRAPH_H definition

