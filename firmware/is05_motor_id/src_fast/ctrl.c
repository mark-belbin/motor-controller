//#############################################################################
//
// FILE:   ctrl.c
//
// TITLE:  C28x InstaSPIN control (CTRL) library (floating point)
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

#if !defined(__TMS320C28XX_CLA__)
#include <math.h>
#endif

#include "libraries/math/include/math.h"

#include "userParams.h"
#include "ctrl.h"

#ifdef FLASH
#pragma CODE_SECTION(CTRL_computePhasor,"ramfuncs");
#pragma CODE_SECTION(CTRL_getCount_isr,"ramfuncs");
#pragma CODE_SECTION(CTRL_getNumIsrTicksPerCtrlTick,"ramfuncs");
#pragma CODE_SECTION(CTRL_incrCounter_isr,"ramfuncs");
#pragma CODE_SECTION(CTRL_isEnabled,"ramfuncs");
#pragma CODE_SECTION(CTRL_resetCounter_isr,"ramfuncs");
#pragma CODE_SECTION(CTRL_setup,"ramfuncs");
#pragma CODE_SECTION(CTRL_run,"ramfuncs");
#endif

#ifdef __TMS320C28XX_CLA__
#pragma CODE_SECTION(CTRL_getVersion,"Cla1Prog2");
#pragma CODE_SECTION(CTRL_getWaitTimes,"Cla1Prog2");
#pragma CODE_SECTION(CTRL_init,"Cla1Prog2");
#pragma CODE_SECTION(CTRL_reset,"Cla1Prog2");
#pragma CODE_SECTION(CTRL_setParams,"Cla1Prog2");
#pragma CODE_SECTION(CTRL_setVersion,"Cla1Prog2");
#pragma CODE_SECTION(CTRL_setWaitTimes,"Cla1Prog2");
#pragma CODE_SECTION(CTRL_updateState,"Cla1Prog2");
#endif

//*****************************************************************************
//
// CTRL_getVersion
//
//*****************************************************************************
void
CTRL_getVersion(CTRL_Handle handle, CTRL_Version *pVersion)
{
    CTRL_Obj *obj = (CTRL_Obj *)handle;

    pVersion->rsvd = obj->version.rsvd;
    pVersion->targetProc = obj->version.targetProc;
    pVersion->major = obj->version.major;
    pVersion->minor = obj->version.minor;

    return;
} // end of CTRL_getVersion() function

//*****************************************************************************
//
// CTRL_getWaitTimes
//
//*****************************************************************************
void
CTRL_getWaitTimes(CTRL_Handle handle, int_least32_t *pWaitTimes)
{
    CTRL_Obj *obj = (CTRL_Obj *)handle;
    uint_least16_t stateCnt;

    for(stateCnt = 0; stateCnt < CTRL_NUMSTATES; stateCnt++)
    {
        pWaitTimes[stateCnt] = obj->waitTimes[stateCnt];
    }

    return;
} // end of CTRL_getWaitTimes() function

//*****************************************************************************
//
// CTRL_init
//
//*****************************************************************************
CTRL_Handle
CTRL_init(void *pMemory, const size_t numBytes)
{
    CTRL_Handle handle;
    CTRL_Obj *obj;

    if((int16_t)numBytes < (int16_t)sizeof(CTRL_Obj))
    {
        /*LDRA_INSPECTED 95 S MR12 11.3 "Below typecasting to NULL has
        no issues"*/
        return((CTRL_Handle)NULL);
    }

    //
    // Assign the handle
    //
    /*LDRA_INSPECTED 94 S MR12 11.3 "Below typecasting to void * has no 
    issues"*/
    /*LDRA_INSPECTED 95 S MR12 11.3 "Below typecasting to void *has no 
    issues"*/
    handle = (CTRL_Handle)pMemory;

    //
    // Set the version
    //
    /*LDRA_INSPECTED 45 D MR12 D.4.1 "handle points to static object, further
    '0' is also a valid address;So no need to check for NULL"*/
    CTRL_setVersion(handle,CTRL_TARGETPROC_2806X,CTRL_MAJOR_RELEASE_NUMBER,
                    CTRL_MINOR_RELEASE_NUMBER);

    //
    // Assign the object
    //
    obj = (CTRL_Obj *)handle;

    //
    // Initialize the Id PI controller module
    //
    /*LDRA_INSPECTED 45 D MR12 D.4.1 "handle points to static object, further
    '0' is also a valid address;So no need to check for NULL"*/
    obj->piHandle_Id = PI_init(&obj->pi_Id,sizeof(obj->pi_Id));

    //
    // Initialize the Iq PI controller module
    //
    obj->piHandle_Iq = PI_init(&obj->pi_Iq,sizeof(obj->pi_Iq));

    //
    // Initialize the speed PI controller module
    //
    obj->piHandle_spd = PI_init(&obj->pi_spd,sizeof(obj->pi_spd));

    /*LDRA_INSPECTED 71 S MR12 11.3 "Always, address of a static object is
    passed, so ok for wider scope"*/
    return(handle);
} // end of CTRL_init() function

//*****************************************************************************
//
// CTRL_reset
//
//*****************************************************************************
void
CTRL_reset(CTRL_Handle handle)
{
    CTRL_Obj *obj = (CTRL_Obj *)handle;
    MATH_Vec2 null = {
        {(float32_t)0.0,(float32_t)0.0}
    };

    // Reset the integrators
    PI_setUi(obj->piHandle_spd,(float32_t)0.0);
    PI_setUi(obj->piHandle_Id,(float32_t)0.0);
    PI_setUi(obj->piHandle_Iq,(float32_t)0.0);

    //
    // Zero internal values
    //
    CTRL_setSpeed_fb_Hz(handle,(float32_t)0.0);
    CTRL_setSpeed_ref_Hz(handle,(float32_t)0.0);
    CTRL_setSpeed_out_A(handle,(float32_t)0.0);

    CTRL_setIdq_A(handle,&null);
    CTRL_setIdq_offset_A(handle,&null);
    CTRL_setVdq_offset_V(handle,&null);

    CTRL_setVdq_V(handle,&null);

    return;
} // end of CTRL_reset() function

//*****************************************************************************
//
// CTRL_setParams
//
//*****************************************************************************
void
CTRL_setParams(CTRL_Handle handle, USER_Params *pUserParams)
{
    CTRL_Obj *obj = (CTRL_Obj *)handle;

    MATH_Vec2 null = {
        {(float32_t)0.0,(float32_t)0.0}
    };

    float32_t Ki,Kp;
    float32_t outMin,outMax;

    float32_t Rs_d_Ohm = pUserParams->motor_Rs_d_Ohm;
    float32_t Rs_q_Ohm = pUserParams->motor_Rs_q_Ohm;
    float32_t Rr_d_Ohm = pUserParams->motor_Rr_d_Ohm;
    float32_t Rr_q_Ohm = pUserParams->motor_Rr_q_Ohm;
    float32_t Ls_d_H = pUserParams->motor_Ls_d_H;
    float32_t Ls_q_H = pUserParams->motor_Ls_q_H;
    float32_t RoverL_rps;
    float32_t BWc_rps = pUserParams->BWc_rps;
    float32_t BWdelta = pUserParams->BWdelta;
    float32_t Kctrl_Wb_p_kgm2 = pUserParams->Kctrl_Wb_p_kgm2;
    float32_t currentCtrlPeriod_sec =
        (float32_t)pUserParams->numCtrlTicksPerCurrentTick /
        pUserParams->ctrlFreq_Hz;
    float32_t speedCtrlPeriod_sec =
        (float32_t)pUserParams->numCtrlTicksPerSpeedTick /
        pUserParams->ctrlFreq_Hz;

    //
    // Assign the motor type
    //
    CTRL_setMotorParams(handle,pUserParams->motor_type,
                        pUserParams->motor_numPolePairs,
                        pUserParams->motor_ratedFlux_Wb,
                        Ls_d_H,Ls_q_H,Rs_d_Ohm,Rs_q_Ohm,Rr_d_Ohm,Rr_q_Ohm);

    //
    // Assign other controller parameters
    //
    CTRL_setNumIsrTicksPerCtrlTick(handle,pUserParams->numIsrTicksPerCtrlTick);
    CTRL_setNumCtrlTicksPerCurrentTick(handle,
                                       pUserParams->numCtrlTicksPerCurrentTick);
    CTRL_setNumCtrlTicksPerSpeedTick(handle,
                                     pUserParams->numCtrlTicksPerSpeedTick);

    CTRL_setCtrlFreq_Hz(handle,pUserParams->ctrlFreq_Hz);
    CTRL_setCtrlPeriod_sec(handle,pUserParams->ctrlPeriod_sec);
    CTRL_setCurrentCtrlPeriod_sec(handle,currentCtrlPeriod_sec);
    CTRL_setSpeedCtrlPeriod_sec(handle,speedCtrlPeriod_sec);

    CTRL_setIdq_A(handle,&null);
    CTRL_setIdq_offset_A(handle,&null);
    CTRL_setIdq_ref_A(handle,&null);

    CTRL_setVdq_V(handle,&null);
    CTRL_setVdq_offset_V(handle,&null);

    CTRL_setVd_sf(handle,pUserParams->Vd_sf);

    CTRL_setMaxVsMag_V(handle,pUserParams->maxVsMag_V);

    //
    // Set the speed reference
    //
    CTRL_setSpeed_fb_Hz(handle,(float32_t)0.0);
    CTRL_setSpeed_out_A(handle,(float32_t)0.0);
    CTRL_setSpeed_outMax_A(handle,pUserParams->maxCurrent_A);
    CTRL_setSpeed_outMin_A(handle,-pUserParams->maxCurrent_A);
    CTRL_setSpeed_ref_Hz(handle,(float32_t)0.0);

    //
    // Reset the counters
    //
    CTRL_resetCounter_current(handle);
    CTRL_resetCounter_isr(handle);
    CTRL_resetCounter_speed(handle);
    CTRL_resetCounter_state(handle);

    //
    // Set the wait times for each state
    //
    CTRL_setWaitTimes(handle,&pUserParams->ctrlWaitTime[0]);

    //
    // Set flags
    //
    CTRL_setFlag_enable(handle,false);
    CTRL_setFlag_enableCurrentCtrl(handle,false);
    CTRL_setFlag_enableSpeedCtrl(handle,true);
    CTRL_setFlag_resetInt_Id(handle,false);
    CTRL_setFlag_resetInt_Iq(handle,false);
    CTRL_setFlag_resetInt_spd(handle,false);
    CTRL_setFlag_useZeroIq_ref(handle,false);

    //
    // Initialize the controller error code
    //
    CTRL_setErrorCode(handle,CTRL_ERRORCODE_NOERROR);

    //
    // Set the default controller state
    //
    CTRL_setState(handle,CTRL_STATE_IDLE);

    CTRL_setBWc_rps(handle,BWc_rps);
    CTRL_setBWdelta(handle,BWdelta);
    CTRL_setKctrl_Wb_p_kgm2(handle,Kctrl_Wb_p_kgm2);

    // 
    // Configure the default speed controller gains
    //
    Kp = MATH_TWO_PI * BWc_rps / (BWdelta * Kctrl_Wb_p_kgm2);
    Ki = BWc_rps * speedCtrlPeriod_sec / (BWdelta * BWdelta);

    //
    // Set the default speed controller gains
    //
    CTRL_setGains(handle,CTRL_TYPE_PI_SPD,Kp,Ki);

    //
    // Configure the default speed controller output minimum/maximum values
    //
    outMax = pUserParams->maxCurrent_A;
    outMin = -outMax;

    //
    // Set the default speed controller output minimum/maximum values
    //
    PI_setMinMax(obj->piHandle_spd,outMin,outMax);

    //
    // Set the Id current controller gain
    //
    Kp = Ls_d_H * pUserParams->BWc_rps;
    RoverL_rps = Rs_d_Ohm / Ls_d_H;
    Ki = RoverL_rps * currentCtrlPeriod_sec;
    CTRL_setGains(handle,CTRL_TYPE_PI_ID,Kp,Ki);

    //
    // Set the Id current controller gain
    //
    Kp = Ls_q_H * pUserParams->BWc_rps;
    RoverL_rps = Rs_q_Ohm / Ls_q_H;
    CTRL_setGains(handle,CTRL_TYPE_PI_IQ,Kp,Ki);

    //
    // Configure the default current controller output minimum/maximum values
    //
    outMax = pUserParams->dcBus_nominal_V;
    outMin = -outMax;

    //
    // Set the default current controller output minimum/maximum values
    //
    PI_setMinMax(obj->piHandle_Id,outMin,outMax);
    PI_setMinMax(obj->piHandle_Iq,outMin,outMax);

    //
    // Reset the integrators
    //
    CTRL_setUi(handle,CTRL_TYPE_PI_SPD,(float32_t)0.0);
    CTRL_setUi(handle,CTRL_TYPE_PI_ID,(float32_t)0.0);
    CTRL_setUi(handle,CTRL_TYPE_PI_IQ,(float32_t)0.0);

    return;
} // end of CTRL_setParams() function

//*****************************************************************************
//
// CTRL_setVersion
//
//*****************************************************************************
void
CTRL_setVersion(CTRL_Handle handle, const CTRL_TargetProc_e targetProc,
                const uint16_t majorReleaseNumber,
                const uint16_t minorReleaseNumber)
{
    CTRL_Obj *obj = (CTRL_Obj *)handle;

    obj->version.rsvd = 107;
    obj->version.targetProc = (uint16_t)targetProc;
    obj->version.major = (uint16_t)majorReleaseNumber;
    obj->version.minor = (uint16_t)minorReleaseNumber;

    return;
} // end of CTRL_setVersion() function

//*****************************************************************************
//
// CTRL_setWaitTimes
//
//*****************************************************************************
void
CTRL_setWaitTimes(CTRL_Handle handle, const int_least32_t *pWaitTimes)
{
    CTRL_Obj *obj = (CTRL_Obj *)handle;
    uint_least16_t stateCnt;

    for(stateCnt = 0; stateCnt < CTRL_NUMSTATES; stateCnt++)
    {
        obj->waitTimes[stateCnt] = pWaitTimes[stateCnt];
    }

    return;
} // end of CTRL_setWaitTimes() function

//*****************************************************************************
//
// CTRL_updateState
//
//*****************************************************************************
bool
CTRL_updateState(CTRL_Handle handle)
{
    CTRL_State_e state = CTRL_getState(handle);
    bool flag_enable = CTRL_getFlag_enable(handle);
    bool stateChanged = false;

    if(flag_enable)
    {
        int_least32_t counter_state = CTRL_getCount_state(handle);
        int_least32_t waitTime = CTRL_getWaitTime(handle,state);

        //
        // Check for errors
        //
        CTRL_checkForErrors(handle);

        if(counter_state >= waitTime)
        {
            //
            // Reset the counter
            //
            CTRL_resetCounter_state(handle);

            if(state == CTRL_STATE_IDLE)
            {
                //
                // Set the next controller state
                //
                CTRL_setState(handle,CTRL_STATE_ONLINE);
            }
        }
    }
    else
    {
        if(state != CTRL_STATE_ERROR)
        {
            if(CTRL_isNotIdle(handle))
            {
                //
                // Reset the controller
                //
                CTRL_reset(handle);

                //
                // Set the next controller state
                //
                CTRL_setState(handle,CTRL_STATE_IDLE);
            }
        }
    }

    //
    // Check to see if the state changed
    //
    if(state != CTRL_getState(handle))
    {
        stateChanged = true;
    }

    return(stateChanged);
} // end of CTRL_updateState() function

// end of file
