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

#ifndef EST_TRAJ_STATES_H
#define EST_TRAJ_STATES_H

//! \file   libraries/observers/est/include/est_Traj_states.h
//! \brief  Contains the states for the flux
//!         estimator (EST_Flux) module routines
//!


// **************************************************************************
// the includes

//!
//!
//! \defgroup EST_TRAJ EST_TRAJ
//!
//! @{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines


// **************************************************************************
// the typedefs

//! \brief Enumeration for the trajectory generator error codes
//!
typedef enum
{
  EST_TRAJ_ERRORCODE_NOERROR=0,        //!< no error error code
  EST_TRAJ_ERRORCODE_IDCLIP,           //!< Id clip error code
  EST_TRAJ_NUMERRORCODES               //!< the number of trajectory generator error codes
} EST_Traj_ErrorCode_e;


//! \brief Enumeration for the trajectory generator states
//!
#ifdef CLA
typedef enum
{
  EST_TRAJ_STATE_ERROR=0,       //!< the trajectory generator error state
  EST_TRAJ_STATE_IDLE = 1,      //!< the trajectory generator idle state
  EST_TRAJ_STATE_EST = 2,       //!< the trajectory generator parameter estimation state
  EST_TRAJ_STATE_ONLINE = 3,    //!< the trajectory generator online state
  EST_TRAJ_NUMSTATES = 4,       //!< the number of trajectory generator states
  EST_TRAJ_RESERVED = 0x10000   //!< reserved to force 32 bit data
} EST_Traj_State_e;
#else
typedef enum
{
  EST_TRAJ_STATE_ERROR=0,       //!< the trajectory generator error state
  EST_TRAJ_STATE_IDLE = 1,      //!< the trajectory generator idle state
  EST_TRAJ_STATE_EST = 2,       //!< the trajectory generator parameter estimation state
  EST_TRAJ_STATE_ONLINE = 3,    //!< the trajectory generator online state
  EST_TRAJ_NUMSTATES = 4,       //!< the number of trajectory generator states
  EST_TRAJ_RESERVED = 0x10000   //!< reserved to force 32 bit data
} EST_Traj_State_e;
#endif

// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes


#ifdef __cplusplus
}
#endif // extern "C"

//! @} // ingroup
#endif // end of EST_TRAJ_STATES_H definition

