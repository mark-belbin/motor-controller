//#############################################################################
//
// FILE:   est.h
//
// TITLE:  C28x InstaSPIN estimator module states
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

#ifndef EST_STATES_H
#define EST_STATES_H

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
//! \addtogroup EST_STATES
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \brief Enumeration for the estimator error codes
//
//*****************************************************************************
typedef enum
{
    EST_ERRORCODE_NOERROR = 0,                //!< no error error code
    EST_ERRORCODE_FLUX_OL_SHIFTOVERFLOW = 1,  //!< flux open loop shift
                                              //!< overflow error code
    EST_ERRORCODE_FLUXERROR = 2,              //!< flux estimator error code
    EST_ERRORCODE_DIR_SHIFTOVERFLOW = 3,      //!< direction shift overflow
                                              //!< error code
    EST_ERRORCODE_IND_SHIFTOVERFLOW = 4,      //!< inductance shift overflow
                                              //!< error code
    EST_NUMERRORCODES = 5                     //!< the number of estimator
                                              //!< error codes
} EST_ErrorCode_e;

//*****************************************************************************
//
//! \brief Enumeration for the estimator states
//
//*****************************************************************************
#ifdef CLA
typedef enum
{
    EST_STATE_ERROR = 0,            //!< error
    EST_STATE_IDLE = 1,             //!< idle
    EST_STATE_ROVERL = 2,           //!< R/L estimation
    EST_STATE_RS = 3,               //!< Rs estimation state
    EST_STATE_RAMPUP = 4,           //!< ramp up the speed
#if !defined(FAST_ROM_V1p6) && !defined(FAST_ROM_V1p7)
    EST_STATE_CONSTSPEED = 5,       //!< constant speed after ramp up
    EST_STATE_IDRATED = 6,          //!< control Id and estimate the rated flux
    EST_STATE_RATEDFLUX_OL = 7,     //!< estimate the open loop rated flux
    EST_STATE_RATEDFLUX = 8,        //!< estimate the rated flux
    EST_STATE_RAMPDOWN = 9,         //!< ramp down the speed
    EST_STATE_LOCKROTOR = 0xA,      //!< lock the rotor
    EST_STATE_LS = 0xB,             //!< stator inductance estimation state
    EST_STATE_RR = 0xC,             //!< rotor resistance estimation state
    EST_STATE_MOTORIDENTIFIED = 0xD,  //!< motor identified state
    EST_STATE_ONLINE = 0xE,         //!< online parameter estimation
    EST_NUMSTATES = 0xF,            //!< the number of estimator states
#else
    EST_STATE_IDRATED = 5,          //!< control Id and estimate the rated flux
    EST_STATE_RATEDFLUX_OL = 6,     //!< estimate the open loop rated flux
    EST_STATE_RATEDFLUX = 7,        //!< estimate the rated flux
    EST_STATE_RAMPDOWN = 8,         //!< ramp down the speed
    EST_STATE_LOCKROTOR = 9,        //!< lock the rotor
    EST_STATE_LS = 0xA,             //!< stator inductance estimation state
    EST_STATE_RR = 0xB,             //!< rotor resistance estimation state
    EST_STATE_MOTORIDENTIFIED = 0xC,  //!< motor identified state
    EST_STATE_ONLINE = 0xD,         //!< online parameter estimation
    EST_NUMSTATES = 0xE,            //!< the number of estimator states
#endif
    EST_RESERVED = 0x1000          //!< reserved to force 32 bit data
} EST_State_e;
#else
typedef enum
{
    EST_STATE_ERROR = 0,            //!< error
    EST_STATE_IDLE = 1,             //!< idle
    EST_STATE_ROVERL = 2,           //!< R/L estimation
    EST_STATE_RS = 3,               //!< Rs estimation state
    EST_STATE_RAMPUP = 4,           //!< ramp up the speed
#if !defined(FAST_ROM_V1p6) && !defined(FAST_ROM_V1p7)
    EST_STATE_CONSTSPEED = 5,       //!< constant speed after ramp up
    EST_STATE_IDRATED = 6,          //!< control Id and estimate the rated flux
    EST_STATE_RATEDFLUX_OL = 7,     //!< estimate the open loop rated flux
    EST_STATE_RATEDFLUX = 8,        //!< estimate the rated flux
    EST_STATE_RAMPDOWN = 9,         //!< ramp down the speed
    EST_STATE_LOCKROTOR = 0xA,      //!< lock the rotor
    EST_STATE_LS = 0xB,             //!< stator inductance estimation state
    EST_STATE_RR = 0xC,             //!< rotor resistance estimation state
    EST_STATE_MOTORIDENTIFIED = 0xD,  //!< motor identified state
    EST_STATE_ONLINE = 0xE,         //!< online parameter estimation
    EST_NUMSTATES = 0xF,            //!< the number of estimator states
#else
    EST_STATE_IDRATED = 5,          //!< control Id and estimate the rated flux
    EST_STATE_RATEDFLUX_OL = 6,     //!< estimate the open loop rated flux
    EST_STATE_RATEDFLUX = 7,        //!< estimate the rated flux
    EST_STATE_RAMPDOWN = 8,         //!< ramp down the speed
    EST_STATE_LOCKROTOR = 9,        //!< lock the rotor
    EST_STATE_LS = 0xA,             //!< stator inductance estimation state
    EST_STATE_RR = 0xB,             //!< rotor resistance estimation state
    EST_STATE_MOTORIDENTIFIED = 0xC,  //!< motor identified state
    EST_STATE_ONLINE = 0xD,         //!< online parameter estimation
    EST_NUMSTATES = 0xE,            //!< the number of estimator states
#endif
    EST_RESERVED = 0x1000          //!< reserved to force 32 bit data
} EST_State_e;
#endif
//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // EST_STATES_H
