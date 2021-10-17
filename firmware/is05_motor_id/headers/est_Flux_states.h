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

#ifndef EST_FLUX_STATES_H
#define EST_FLUX_STATES_H

//! \file   libraries/observers/est/include/est_Flux_states.h
//! \brief  Contains the states for the flux
//!         estimator (EST_Flux) module routines
//!


// **************************************************************************
// the includes

//!
//!
//! \defgroup EST_FLUX_STATES EST_FLUX_STATES
//!
//! @{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines


// **************************************************************************
// the typedefs

//! \brief Enumeration for the estimator error codes
//!
typedef enum
{
  EST_FLUX_ERRORCODE_NOERROR=0,      //!< no error error code
  EST_FLUX_ERRORCODE_SHIFTOVERFLOW,  //!< flux shift overflow error code
  EST_FLUX_ERRORCODE_CLIP,           //!< flux clip error code
  EST_FLUX_NUMERRORCODES             //!< the number of estimator error codes
} EST_Flux_ErrorCode_e;


//! \brief Enumeration for the estimator states
//!
typedef enum
{
  EST_FLUX_STATE_ERROR = 0, //!< error state
  EST_FLUX_STATE_IDLE = 1,  //!< idle state
  EST_FLUX_STATE_CL1 = 2,   //!< closed loop control stage 1
  EST_FLUX_STATE_CL2 = 3,   //!< closed loop control stage 2
  EST_FLUX_STATE_FINE = 4,  //!< fine estimate of flux
  EST_FLUX_STATE_DONE = 5,  //!< done state
  EST_FLUX_NUMSTATES = 6    //!< the number of flux estimator states
} EST_Flux_State_e;


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes


#ifdef __cplusplus
}
#endif // extern "C"

//! @} // ingroup
#endif // end of EST_FLUX_STATES_H definition

