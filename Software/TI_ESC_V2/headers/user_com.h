//#############################################################################
// $TI Release: MotorControl SDK v3.00.01.00 $
// $Release Date: Tue May 26 19:13:59 CDT 2020 $
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

#ifndef USER_COM_H
#define USER_COM_H

//! \file   solutions/boostxl_drv8320rs/f28004x/drivers/user_com.h
//! \brief  Contains the user related definitions
//!

// **************************************************************************
// the includes

// modules
#include "userParams.h"


//!
//!
//! \defgroup USER USER
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines
//! \brief Defines the system clock frequency, MHz
//!
#define USER_SYSTEM_FREQ_MHz       ((float32_t)(100.0))


//! \brief A flag to bypass motor identification (1/0 : true/false)
//!
#define USER_BYPASS_MOTOR_ID       (1)          // No motor parameters identification
//#define USER_BYPASS_MOTOR_ID        (0)           // Do motor parameters identification

#define USER_ENABLE_MOTOR_ID        0
#define USER_DISABLE_MOTOR_ID       1

//============================================================================================
// Motor defines

//************** Motor Parameters **************

// PMSM motors
#define Estun_EMJ_04APB22_A         100
#define Estun_EMJ_04APB22_B         101
#define Regal_Beloit_5SME39DL0756   111

#define Teknic_M2311SLN02K          121
#define Teknic_M2310PLN04K          122
#define teknic_2310S                123
#define Anaheim_BLY172S_24V         124
#define Anaheim_BLY341S_48V         125
#define Anaheim_BLY341S_24V         126

#define Traxxas_Velineon_380        130
#define Traxxas_Velineon_3500       131
#define Pacific_Scientific          132

#define Anaheim_BLZ362S             141
#define Anaheim_BLWS235D            142
#define tekin_redline_4600KV        143
#define low_voltage_ceiling_fan     144


// ACIM motors
#define Marathon_5K33GN2A           200
#define Marathon_56H17T2011A        201
#define Dayton_3N352C               202


#define my_pm_motor_1               301
#define my_aci_motor_2              302


// **************************************************************************
// the typedefs


// **************************************************************************
// the globals


// **************************************************************************
// the functions
//! \param[in]  pUserParams  The pointer to the user param structure
//! \param[in]  motorNum  The motor number
extern void USER_setDualMotorParams(USER_Params *pUserParams, const uint16_t motorNum);

//! \brief      Sets the private user parameter values
//! \param[in]  pUserParams  The pointer to the user param structure
extern void cla_USER_setParams_priv(USER_Params *pUserParams);

//! \brief      Sets the private user parameter values
//! \param[in]  pUserParams  The pointer to the user param structure
extern void USER_setParams_priv(USER_Params *pUserParams);


#ifdef __cplusplus
}
#endif // extern "C"

//@}  // ingroup
#endif // end of USER_COM_H definition

