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

//! \file   ~/sw/modules/angle_comp/include/angle_comp.h
//! \brief  Contains the public interface to the angle compensation
//! \brief  generator (ANGLE_GEN)
//!         module routines
//!

#ifndef ANGLE_GEN_H
#define ANGLE_GEN_H


//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C" {
#endif


//*****************************************************************************
//
//! \addtogroup ANGLE_GEN
//! @{
//
//*****************************************************************************

//
// the includes
//
#ifdef __TMS320C28XX_CLA__
#include "libraries/math/include/CLAmath.h"
#else
#include <math.h>
#endif // __TMS320C28XX_CLA__

#include "libraries/math/include/math.h"


//! \brief Defines the angle generator (ANGLE_COMP) object
//!
typedef struct _ANGLE_GEN_Obj_
{
    float32_t fs_pu;              //!< the freq input value
    float32_t angleDeltaFactor;   //!< predetermined factor for use in angle compensation calculation
    float32_t angleDelta_pu;      //!< the angle delta value
    float32_t angle_pu;           //!< the angle output value
} ANGLE_GEN_Obj;


//! \brief Defines the ANGLE_GEN handle
//!
typedef struct _ANGLE_GEN_Obj_  *ANGLE_GEN_Handle;


//
// the function prototypes
//

//! \brief     Gets the predicted angle value
//! \param[in] handle  The angle generator (ANGLE_COMP) handle
//! \return    The predicted angle compensation value, rad
static inline float32_t ANGLE_GEN_getAngle_pu(ANGLE_GEN_Handle handle)
{
    ANGLE_GEN_Obj *obj = (ANGLE_GEN_Obj *)handle;

    return(obj->angle_pu);
} // end of ANGLE_GEN_getAngle_pu() function


//! \brief     Initializes the angle generator (ANGLE_GEN) module
//! \param[in] pMemory   A pointer to the memory for the object
//! \param[in] numBytes  The number of bytes allocated for the object, bytes
//! \return    The angle generator (ANGLE_GEN) object handle
extern ANGLE_GEN_Handle ANGLE_GEN_init(void *pMemory, const size_t numBytes);


//! \brief  Compensates for the delay introduced
//! \brief  from the time when the system inputs are sampled to when the PWM
//! \brief  voltages are applied to the motor windings.
//! \param[in] handle     The angle generator (ANGLE_COMP) handle
//! \param[in] fm_pu      The electrical speed in pu
//! \param[in] angleUncomp_pu  The uncompensated angle in pu
static inline void ANGLE_GEN_run(ANGLE_GEN_Handle handle, const float32_t fm_pu)
{
    ANGLE_GEN_Obj *obj = (ANGLE_GEN_Obj *)handle;

    obj->fs_pu = fm_pu;
    obj->angleDelta_pu = obj->fs_pu * obj->angleDeltaFactor;

    float32_t angle_pu = obj->angle_pu;

    //
    // increment the angle
    //
    angle_pu = angle_pu + obj->angleDelta_pu;

    //
    // mask the angle for wrap around
    // note: must account for the sign of the angle
    //
    if(angle_pu > MATH_TWO_PI)
    {
        angle_pu = angle_pu - MATH_TWO_PI;
    }
    else if(angle_pu < -MATH_TWO_PI)
    {
        angle_pu = angle_pu + MATH_TWO_PI;
    }

    //
    // account for sign
    //
    if(fm_pu < 0.0)
    {
        angle_pu = -angle_pu;
    }

    obj->angle_pu = angle_pu;

    return;
} // end of ANGLE_GEN_run()


//! \brief     Sets the parameters
//! \param[in] handle               The angle generator (ANGLE_COMP) handle
//! \param[in] iqFullScaleFreq_Hz   The frequency used to set 1 pu
//! \param[in] pwmPeriod_usec       The the pwmPeriod in usec
//! \param[in] numPWMTicksPerISRTick  The decimation between PWM cycles and the ISR cycle
static inline void ANGLE_GEN_setParams(ANGLE_GEN_Handle handle,
                                       float32_t ctrlPeriod_sec)
{
    ANGLE_GEN_Obj *obj = (ANGLE_GEN_Obj *)handle;

    obj->angleDeltaFactor = ctrlPeriod_sec * MATH_TWO_PI;
    obj->angleDelta_pu = 0.0;
    obj->angle_pu = 0.0;

    return;
} // end of ANGLE_COMP_setParams() function
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
#endif // extern "C"

#endif // end of ANGLE_GEN_H definition

