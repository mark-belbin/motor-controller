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

#ifndef MOTOR_H
#define MOTOR_H

//! \file   libraries/motor/include/motor.h
//! \brief  Contains motor related definitions
//!


// **************************************************************************
// the includes

#include "types.h"


//!
//!
//! \defgroup MOTOR MOTOR
//!
//! @{

#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines


// **************************************************************************
// the typedefs


//! \brief Enumeration for the motor types
//!
typedef enum
{
  MOTOR_TYPE_INDUCTION = 0,   //!< induction
  MOTOR_TYPE_PM               //!< permanent magnet
} MOTOR_Type_e;

//
//! \brief Enumeration for the Flying Start mode
//
typedef enum
{
    FLYINGSTART_MODE_HALT    = 0,       //!< Halt Mode
    FLYINGSTART_MODE_STANDBY = 1        //!< Standby Mode
} FlyingStart_Mode_e;

//
//! \brief Enumeration for the Motor Control mode
//
typedef enum
{
    MOTORCTRL_MODE_SPEED  = 0,           //!< Speed close loop control
    MOTORCTRL_MODE_TORQUE = 1            //!< Torque close loop control
} MotorCtrl_Mode_e;

//! \brief Defines the motor parameters
//!
typedef struct _MOTOR_Params_
{
  MOTOR_Type_e    type;               //!< Defines the motor type

  uint_least16_t  numPolePairs;       //!< Defines the number of pole pairs

  float32_t         Lmag_H;             //!< Defines the magnetizing inductance, H

  float32_t         Ls_d_H;             //!< Defines the direct stator inductance, H
  float32_t         Ls_q_H;             //!< Defines the quadrature stator inductance, H

  float32_t         Rr_d_Ohm;           //!< Defines the direct rotor resistance, Ohm
  float32_t         Rr_q_Ohm;           //!< Defines the quadrature rotor resistance, Ohm

  float32_t         Rs_d_Ohm;           //!< Defines the direct stator resistance, Ohm
  float32_t         Rs_q_Ohm;           //!< Defines the quadrature stator resistance, Ohm

  float32_t         ratedFlux_Wb;       //!< Defines the rated flux, Wb
} MOTOR_Params;


// **************************************************************************
// the functions


#ifdef __cplusplus
}
#endif // extern "C"

//! @} // ingroup
#endif // end of MOTOR_H definition





