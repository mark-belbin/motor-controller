//#############################################################################
//
// FILE:   user.c
//
// TITLE:  C28x InstaSPIN function for setting initialization data for the
//         CTRL, HAL, and EST modules (floating point)
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

#include "user.h"

#ifdef __TMS320C28XX_CLA__
#pragma CODE_SECTION(USER_setParams,"Cla1Prog2");
#endif

#ifdef _F28002x_
#pragma CODE_SECTION(USER_setParams, "user_code");
#endif

//*****************************************************************************
//
// USER_setParams
//
//*****************************************************************************
void USER_setParams(USER_Params *pUserParams)
{
    pUserParams->dcBus_nominal_V = USER_NOMINAL_DC_BUS_VOLTAGE_V;

    pUserParams->numIsrTicksPerCtrlTick = 1;
    pUserParams->numIsrTicksPerEstTick = 1;
    pUserParams->numIsrTicksPerTrajTick = 1;

    pUserParams->numCtrlTicksPerCurrentTick = USER_NUM_ISR_TICKS_PER_CURRENT_TICK;
    pUserParams->numCtrlTicksPerSpeedTick = USER_NUM_ISR_TICKS_PER_SPEED_TICK;

    pUserParams->numCurrentSensors = USER_NUM_CURRENT_SENSORS;
    pUserParams->numVoltageSensors = USER_NUM_VOLTAGE_SENSORS;

    pUserParams->systemFreq_MHz = USER_SYSTEM_FREQ_MHz;

    pUserParams->voltage_sf = USER_VOLTAGE_SF;

    pUserParams->current_sf = USER_CURRENT_SF;

    pUserParams->dcBusPole_rps = USER_DCBUS_POLE_rps;

    pUserParams->offsetPole_rps = USER_OFFSET_POLE_rps;

    pUserParams->speedPole_rps = USER_SPEED_POLE_rps;

    pUserParams->voltageFilterPole_rps = USER_VOLTAGE_FILTER_POLE_rps;

    pUserParams->RoverL_excFreq_Hz = USER_R_OVER_L_EXC_FREQ_Hz;

    pUserParams->maxVsMag_pu = USER_MAX_VS_MAG_PU;

    pUserParams->motor_type = USER_MOTOR_TYPE;

    pUserParams->motor_numPolePairs = USER_MOTOR_NUM_POLE_PAIRS;
    pUserParams->motor_numEncSlots = USER_MOTOR_NUM_ENC_SLOTS;

    pUserParams->motor_ratedFlux_Wb = USER_MOTOR_RATED_FLUX_VpHz / MATH_TWO_PI;

    pUserParams->motor_Rr_d_Ohm = USER_MOTOR_Rr_Ohm;
    pUserParams->motor_Rr_q_Ohm = USER_MOTOR_Rr_Ohm;

    pUserParams->motor_Rs_a_Ohm = USER_MOTOR_Rs_Ohm;
    pUserParams->motor_Rs_b_Ohm = USER_MOTOR_Rs_Ohm;

    pUserParams->motor_Rs_d_Ohm = USER_MOTOR_Rs_Ohm;
    pUserParams->motor_Rs_q_Ohm = USER_MOTOR_Rs_Ohm;

    pUserParams->motor_Ls_d_H = USER_MOTOR_Ls_d_H;
    pUserParams->motor_Ls_q_H = USER_MOTOR_Ls_q_H;

    pUserParams->maxCurrent_A = USER_MOTOR_MAX_CURRENT_A;

    pUserParams->IdRated_A = USER_MOTOR_MAGNETIZING_CURRENT_A;

    pUserParams->Vd_sf = USER_VD_SF;
    pUserParams->maxVsMag_V = USER_NOMINAL_DC_BUS_VOLTAGE_V;

    /*LDRA_INSPECTED 139 S MR12 14.3 ""Its a compile time flag providing user a
       choice to use or bypass motor identification; so OK"*/
    if((USER_BYPASS_MOTOR_ID == 1) && (pUserParams->flag_bypassMotorId == true))
    {

        pUserParams->BWc_rps = MATH_TWO_PI * 100 * 6;// Factor of 16 after testing, increasing BW
        pUserParams->BWdelta = (float32_t)8.0;

        // 3.0 * pUserParams->motor_numPolePairs * 0.1 / (2.0 * 0.00001);
        pUserParams->Kctrl_Wb_p_kgm2 = (float32_t)3.0 *
                                       pUserParams->motor_numPolePairs *
                                       (float32_t)(0.001) /
                                       (float32_t)(2.0 * 0.000001);

    }
    else
    {
        pUserParams->flag_bypassMotorId = false;

        pUserParams->BWc_rps = MATH_TWO_PI * 100 * 6;// Factor of 16 after testing, increasing BW
        pUserParams->BWdelta = (float32_t)8.0;

        // 3.0 * pUserParams->motor_numPolePairs * 0.1 / (2.0 * 0.00001);
        pUserParams->Kctrl_Wb_p_kgm2 = (float32_t)3.0 *
                                       pUserParams->motor_numPolePairs *
                                       (float32_t)(0.001) /
                                       (float32_t)(2.0 * 0.000001);
    }

    pUserParams->angleDelayed_sf_sec = (float32_t)0.5 * USER_CTRL_PERIOD_sec;

    pUserParams->fluxExcFreq_Hz = USER_MOTOR_FLUX_EXC_FREQ_Hz;

    pUserParams->ctrlWaitTime[CTRL_STATE_ERROR] = 0;
    pUserParams->ctrlWaitTime[CTRL_STATE_IDLE] = 0;
    pUserParams->ctrlWaitTime[CTRL_STATE_ONLINE] = 0;

    pUserParams->estWaitTime[EST_STATE_ERROR] = 0;
    pUserParams->estWaitTime[EST_STATE_IDLE] = 0;
    pUserParams->estWaitTime[EST_STATE_ROVERL] =
        (int32_t)(5.0 * USER_EST_FREQ_Hz);
    pUserParams->estWaitTime[EST_STATE_RS] = 0;
    pUserParams->estWaitTime[EST_STATE_RAMPUP] =
        (int32_t)((USER_MOTOR_FLUX_EXC_FREQ_Hz / USER_MAX_ACCEL_Hzps +
                         (float32_t)1.0) * USER_EST_FREQ_Hz);
    pUserParams->estWaitTime[EST_STATE_CONSTSPEED] =
        (int32_t)(1.0 * USER_EST_FREQ_Hz);
    pUserParams->estWaitTime[EST_STATE_IDRATED] =
        (int32_t)(20.0 * USER_EST_FREQ_Hz);
    pUserParams->estWaitTime[EST_STATE_RATEDFLUX_OL] =
        (int32_t)(1.0 * USER_EST_FREQ_Hz);
    pUserParams->estWaitTime[EST_STATE_RATEDFLUX] = 0;
    pUserParams->estWaitTime[EST_STATE_RAMPDOWN] =
        (int32_t)(0.0 * USER_EST_FREQ_Hz);
    pUserParams->estWaitTime[EST_STATE_LOCKROTOR] = 0;
    pUserParams->estWaitTime[EST_STATE_LS] = 0;
    pUserParams->estWaitTime[EST_STATE_RR] =
        (int32_t)(5.0 * USER_EST_FREQ_Hz);
    pUserParams->estWaitTime[EST_STATE_MOTORIDENTIFIED] = 0;
    pUserParams->estWaitTime[EST_STATE_ONLINE] = 0;

    pUserParams->FluxWaitTime[EST_FLUX_STATE_ERROR] = 0;
    pUserParams->FluxWaitTime[EST_FLUX_STATE_IDLE] = 0;
    pUserParams->FluxWaitTime[EST_FLUX_STATE_CL1] =
        (int32_t)(10.0 * USER_EST_FREQ_Hz);
    pUserParams->FluxWaitTime[EST_FLUX_STATE_CL2] =
        (int32_t)(0.2 * USER_EST_FREQ_Hz);
    pUserParams->FluxWaitTime[EST_FLUX_STATE_FINE] =
        (int32_t)(20.0 * USER_EST_FREQ_Hz);
    pUserParams->FluxWaitTime[EST_FLUX_STATE_DONE] = 0;

    pUserParams->LsWaitTime[EST_LS_STATE_ERROR] = 0;
    pUserParams->LsWaitTime[EST_LS_STATE_IDLE] = 0;
    pUserParams->LsWaitTime[EST_LS_STATE_RAMPUP] =
        (int32_t)(10.0 * USER_EST_FREQ_Hz);
    pUserParams->LsWaitTime[EST_LS_STATE_COARSE] =
        (int32_t)(30.0 * USER_EST_FREQ_Hz);
    pUserParams->LsWaitTime[EST_LS_STATE_FINE] =
        (int32_t)(30.0 * USER_EST_FREQ_Hz);
    pUserParams->LsWaitTime[EST_LS_STATE_DONE] = 0;

    pUserParams->RrWaitTime[EST_RR_STATE_ERROR] = 0;
    pUserParams->RrWaitTime[EST_RR_STATE_IDLE] = 0;
    pUserParams->RrWaitTime[EST_RR_STATE_RAMPUP] =
        (int32_t)(1.0 * USER_EST_FREQ_Hz);
    pUserParams->RrWaitTime[EST_RR_STATE_COARSE] =
        (int32_t)(10.0 * USER_EST_FREQ_Hz);
    pUserParams->RrWaitTime[EST_RR_STATE_FINE] =
        (int32_t)(30.0 * USER_EST_FREQ_Hz);
    pUserParams->RrWaitTime[EST_RR_STATE_DONE] = 0;

    pUserParams->RsWaitTime[EST_RS_STATE_ERROR] = 0;
    pUserParams->RsWaitTime[EST_RS_STATE_IDLE] = 0;
    pUserParams->RsWaitTime[EST_RS_STATE_RAMPUP] =
        (int32_t)(1.0 * USER_EST_FREQ_Hz);
    pUserParams->RsWaitTime[EST_RS_STATE_COARSE] =
        (int32_t)(2.0 * USER_EST_FREQ_Hz);
    pUserParams->RsWaitTime[EST_RS_STATE_FINE] =
        (int32_t)(10.0 * USER_EST_FREQ_Hz);
    pUserParams->RsWaitTime[EST_RS_STATE_DONE] = 0;

    pUserParams->trajWaitTime[EST_TRAJ_STATE_ERROR] = 0;
    pUserParams->trajWaitTime[EST_TRAJ_STATE_IDLE] = 0;
    pUserParams->trajWaitTime[EST_TRAJ_STATE_EST] = 0;
    pUserParams->trajWaitTime[EST_TRAJ_STATE_ONLINE] = 0;

    pUserParams->estFreq_Hz = USER_EST_FREQ_Hz;
    pUserParams->ctrlFreq_Hz = USER_CTRL_FREQ_Hz;
    pUserParams->trajFreq_Hz = USER_TRAJ_FREQ_Hz;

    pUserParams->pwmPeriod_usec = USER_PWM_PERIOD_usec;
    pUserParams->ctrlPeriod_sec = USER_CTRL_PERIOD_sec;

    pUserParams->maxAccel_Hzps = USER_MAX_ACCEL_Hzps;

    pUserParams->maxCurrent_resEst_A = USER_MOTOR_RES_EST_CURRENT_A;
    pUserParams->maxCurrent_indEst_A = USER_MOTOR_IND_EST_CURRENT_A;

    pUserParams->maxCurrentDelta_A = USER_MAX_CURRENT_DELTA_A;
    pUserParams->maxCurrentDelta_pw_A = USER_MAX_CURRENT_DELTA_PW_A;

    pUserParams->IdRated_delta_A = USER_IDRATED_DELTA_A;

    pUserParams->forceAngleFreq_Hz = USER_FORCE_ANGLE_FREQ_Hz;

    pUserParams->indEst_speedMaxFraction = USER_SPEEDMAX_FRACTION_FOR_L_IDENT;

    pUserParams->IdRatedFraction_indEst = USER_IDRATED_FRACTION_FOR_L_IDENT;

    pUserParams->pwGain = USER_PW_GAIN;

    pUserParams->Kp_min_VpA = (float32_t)0.001;
    pUserParams->Kp_max_VpA = (float32_t)1000.0;

    pUserParams->RoverL_Kp_sf = USER_R_OVER_L_KP_SF;
    pUserParams->RoverL_min_rps = MATH_TWO_PI * (float32_t)5.0;
    pUserParams->RoverL_max_rps = MATH_TWO_PI * (float32_t)5000.0;

    pUserParams->oneOverDcBus_min_invV = (float32_t)1.0 / (float32_t)400.0;
    pUserParams->oneOverDcBus_max_invV = (float32_t)1.0 / (float32_t)10.0;

    pUserParams->Ls_d_H = (float32_t)1.0e-6;
    pUserParams->Ls_q_H = (float32_t)1.0e-6;
    pUserParams->Ls_coarseDelta_H = (float32_t)0.0000001;
    pUserParams->Ls_fineDelta_H = (float32_t)0.00000001;
    pUserParams->Ls_min_H = (float32_t)0.000001;
    pUserParams->Ls_max_H = (float32_t)100.0;

    pUserParams->Rr_Ohm = (float32_t)0.0;
    pUserParams->Rr_coarseDelta_Ohm = (float32_t)0.0001;
    pUserParams->Rr_fineDelta_Ohm = (float32_t)0.00001;
    pUserParams->Rr_min_Ohm = (float32_t)0.0;
    pUserParams->Rr_max_Ohm = (float32_t)1000.0;

    pUserParams->Rs_Ohm = (float32_t)0.0;
    pUserParams->Rs_coarseDelta_Ohm = (float32_t)0.01;
    pUserParams->Rs_fineDelta_Ohm = (float32_t)0.00001;
    pUserParams->Rs_min_Ohm = (float32_t)0.001;
    pUserParams->Rs_max_Ohm = (float32_t)1000.0;

    pUserParams->RsOnLine_DeltaInc_Ohm = (float32_t)0.00001;
    pUserParams->RsOnLine_DeltaDec_Ohm = (float32_t)0.00001;
    pUserParams->RsOnLine_min_Ohm = (float32_t)0.001;
    pUserParams->RsOnLine_max_Ohm = (float32_t)1000.0;

    pUserParams->RsOnLine_angleDelta_rad = (float32_t)0.00001;
    pUserParams->RsOnLine_pole_rps = MATH_TWO_PI * (float32_t)0.2;

    pUserParams->flag_bypassMotorId = USER_BYPASS_MOTOR_ID;

    return;
} // end of USER_setParams() function


// end of file
