//#############################################################################
//
// FILE:   vsf.h
//
// TITLE:  varibale switching pwm frequency
//
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

#ifndef VSF_H
#define VSF_H

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! \defgroup VSF VSF
//! @{
//
//*****************************************************************************

// the includes
#include "libraries/math/include/math.h"
#include "user.h"


//! \brief Defines the changing PWM frequency delta
//!
#define VSF_NUM_DELTA              1


//! \brief Defines the changing PWM frequency wait time
//!
#define VSF_NUM_WAIT_TIME          10


//! \brief Defines the maximum PWM frequency
//!
#define VSF_NUM_MAX_FREQ_HZ        25000


//! \brief Defines the minimum PWM frequency
//!
#define VSF_NUM_MIN_FREQ_HZ        5000



// the typedefs
typedef enum
{
    VSF_STATE_IDLE = 0,            //
    VSF_STATE_CTRL_SET = 1,        // in BK
    VSF_STATE_PERIOD_SET = 2,      // in ISR
    VSF_STATE_EST_SET = 3,         // in ISR
    VSF_STATE_ALL_DONE = 4         // N/A
} VSF_State_e;

//! \brief Defines the VSF handle
//!
typedef struct _VSF_Obj_
{
    VSF_State_e state;            //!< Defines the VSF state
    uint16_t  pwmPeriod;          //!< Defines the target PWM period value
    uint16_t  pwmPeriodNow;       //!< Defines the current PWM period value
    uint16_t  pwmFreqSet_Hz;      //!< Defines the setting PWM frequency, Hz
    uint16_t  pwmFreqTarget_Hz;   //!< Defines the target PWM frequency, Hz
    uint16_t  pwmFreqNow_Hz;      //!< Defines the current PWM frequency, Hz
    uint16_t  pwmFreqDelta_Hz;    //!< Defines the changing frequency, Hz
    uint16_t  pwmFreqMax_Hz;      //!< Defines the maximum pwm frequency, Hz
    uint16_t  pwmFreqMin_Hz;      //!< Defines the minimum pwm frequency, Hz

    uint16_t  pwmWaitTime;        //!< Define the wait time for change PWM
    uint16_t  pwmCounter;         //!< Define the counter for wait time

    uint32_t  cpuFreq_Hz;         //!< Defines the cpu frequency, Hz
}VSF_Obj;

//! \brief Defines the online variable switching frequency (VSF) handle
//!
typedef struct _VSF_Obj_ *VSF_Handle;

// the globals


// the functions

//! \brief     Initializes the variable switching frequency module
//! \param[in] pMemory   A pointer to the variable pwm frequency object memory
//! \param[in] numBytes  The number of bytes allocated for
//!                      the variable pwm frequency object, bytes
//! \return    The variable switching frequency object handle
extern VSF_Handle VSF_init(void *pMemory,const size_t numBytes);


//! \brief     init the variable switching frequency parameters
//! \param[in] vsfHandle    The variable switching frequency (VSF) object handle
//! \param[in] pUserParams  The pointer to the user param structure
extern void VSF_initParams(VSF_Handle vsfHandle, USER_Params *pUserParams);


//! \brief calculates the variable switching frequency parameters
//! \param[in] vsfHandle  The variable switching frequency object handle
extern void VSF_computeFreqParams(VSF_Handle vsfHandle);


//! \brief     Get the current switching frequency
//! \param[in] The variable switching frequency (VSF) object handle
//! \return    The current pwm frequency
inline uint16_t VSF_getFreq(VSF_Handle vsfHandle)
{
    VSF_Obj *vsfObj = (VSF_Obj *)vsfHandle;

    return(vsfObj->pwmFreqNow_Hz);
}


//! \brief     Set the variable switching frequency state
//! \param[in] The variable switching frequency (VSF) object handle
//! \return    The current pwm frequency
inline VSF_State_e VSF_getState(VSF_Handle vsfHandle)
{
    VSF_Obj *vsfObj = (VSF_Obj *)vsfHandle;

    return(vsfObj->state);
}


//! \brief     Get the calculated PWM period
//! \param[in] The variable switching frequency (VSF) object handle
//! \return    The current pwm period register value
inline void VSF_getPeriod(VSF_Handle vsfHandle, uint16_t *pPeriodValue)
{
    VSF_Obj *vsfObj = (VSF_Obj *)vsfHandle;

    *pPeriodValue = vsfObj->pwmPeriodNow;

    return;
}


//! \brief     Set the variable switching frequency target value
//! \param[in] The variable switching frequency (VSF) object handle
//! \param[in] The pwm switching frequency
inline void VSF_setFreq(VSF_Handle vsfHandle, const uint16_t vsf_Hz)
{
    VSF_Obj *vsfObj = (VSF_Obj *)vsfHandle;

    vsfObj->pwmFreqSet_Hz = vsf_Hz;

    return;
}

//! \brief     Set the variable switching frequency delta changing value
//! \param[in] The variable switching frequency (VSF) object handle
//! \param[in] The vvariable switching frequency changing delta value
inline void VSF_setFreqDelta(VSF_Handle vsfHandle,
                             const uint16_t pwmFreqDelta_Hz)
{
    VSF_Obj *vsfObj = (VSF_Obj *)vsfHandle;

    vsfObj->pwmFreqDelta_Hz = pwmFreqDelta_Hz;

    return;
}

//! \brief     Set the variable switching frequency maximum value
//! \param[in] The variable switching frequency (VSF) object handle
//! \param[in] The vvariable switching frequency maximum value
inline void VSF_setFreqMax(VSF_Handle vsfHandle, const uint16_t pwmFreqMax_Hz)
{
    VSF_Obj *vsfObj = (VSF_Obj *)vsfHandle;

    vsfObj->pwmFreqMax_Hz = pwmFreqMax_Hz;

    return;
}


//! \brief     Set the variable switching frequency minimum value
//! \param[in] The variable switching frequency (VSF) object handle
//! \param[in] The variable switching frequency minimum value
inline void VSF_setFreqMin(VSF_Handle vsfHandle, const uint16_t pwmFreqMin_Hz)
{
    VSF_Obj *vsfObj = (VSF_Obj *)vsfHandle;

    vsfObj->pwmFreqMin_Hz = pwmFreqMin_Hz;

    return;
}


//! \brief     Set the variable switching frequency PWM period
//! \param[in] The variable switching frequency (VSF) object handle
inline void VSF_setPeriod(VSF_Handle vsfHandle)
{
    VSF_Obj *vsfObj = (VSF_Obj *)vsfHandle;

    vsfObj->pwmCounter++;

    // setup the Counter-Compare Control Register (CMPCTL)
    if(vsfObj->state == VSF_STATE_PERIOD_SET)
    {
        if(vsfObj->pwmPeriod != vsfObj->pwmPeriodNow)
        {
            if(vsfObj->pwmPeriod > vsfObj->pwmPeriodNow)
            {
                vsfObj->pwmPeriodNow++;
            }
            else if(vsfObj->pwmPeriod < vsfObj->pwmPeriodNow)
            {
                vsfObj->pwmPeriodNow--;
            }
       }
        else if(vsfObj->pwmCounter > vsfObj->pwmWaitTime)
        {
            vsfObj->pwmCounter = 0;
            vsfObj->state = VSF_STATE_IDLE;
        }
    }

    return;
}


//! \brief     Set the variable switching frequency state
//! \param[in] The variable switching frequency (VSF) object handle
//! \param[in] The variable switching frequency state
inline void VSF_setState(VSF_Handle vsfHandle, const VSF_State_e state)
{
    VSF_Obj *vsfObj = (VSF_Obj *)vsfHandle;

    vsfObj->state = state;

    return;
}

//! \brief     Set the variable switching frequency wait time
//! \param[in] The variable switching frequency (VSF) object handle
//! \param[in] The variable switching frequency wait time
inline void VSF_setWaitTime(VSF_Handle vsfHandle, const uint16_t pwmWaitTime)
{
    VSF_Obj *vsfObj = (VSF_Obj *)vsfHandle;

    vsfObj->pwmWaitTime = pwmWaitTime;

    return;
}

#ifdef __cplusplus
}
#endif // extern "C"

//! @} // ingroup
#endif // end of VSF_H definition

