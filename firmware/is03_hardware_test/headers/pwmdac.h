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

#ifndef PWMDAC_H
#define PWMDAC_H

//! \file   solutions/boostxl_drv8320rs/f28004x/drivers/pwmdac.h
//! \brief  Contains public interface to various functions related
//!         to the pulse width modulation digital-to-analog
//!         converter (PWMDAC) object
//!


// **************************************************************************
// the includes

#include "epwm.h"


//!
//!
//! \defgroup PWMDAC PWMDAC
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines

//!  \brief  Defines the pulse width modulation digital-to-analog (PWMDAC) handle
//!
#define   PWMDAC_Handle                             PWM_Handle

//!  \brief  Links the PWMDAC_setPeriod() function to the PWM_setPeriod() function
//!
#define   PWMDAC_getPeriod                          EPWM_getTimeBasePeriod


//!  \brief  Links the PWMDAC_init() function to the PWM_init() function
//!
#define   PWMDAC_init                               PWM_init

// **************************************************************************
// the typedefs

//! \brief Enumeration to define the pulse width modulation digital-to-analog (PWM) numbers
//!
typedef enum
{
  PWMDAC_NUMBER_1=0,
  PWMDAC_NUMBER_2,
  PWMDAC_NUMBER_3,
  PWMDAC_NUMBER_4,
  PWMDAC_NUMBER_5,
  PWMDAC_NUMBER_6,
  PWMDAC_NUMBER_7
} PWMDAC_Number_e;


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif  // end of PWMDAC_H definition

