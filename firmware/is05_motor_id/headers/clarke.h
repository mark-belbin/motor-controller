 //#############################################################################
//
// FILE:   clarke.h
//
// TITLE:  C28x Clarke transform library
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

#ifndef CLARKE_H
#define CLARKE_H

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
//! \addtogroup CLARKE
//! @{
//
//*****************************************************************************

#include "libraries/math/include/math.h"

//*****************************************************************************
//
//! \brief Defines the CLARKE object
//
//*****************************************************************************
typedef struct _CLARKE_Obj_
{
    float32_t alpha_sf;           //!< the scale factor for the alpha component
    float32_t beta_sf;            //!< the scale factor for the beta component
    uint_least8_t numSensors;   //!< the number of sensors
} CLARKE_Obj;

//*****************************************************************************
//
//! \brief Defines the CLARKE handle
//
//*****************************************************************************
typedef struct _CLARKE_Obj_ *CLARKE_Handle;

//*****************************************************************************
//
// Prototypes for the APIs
//
//*****************************************************************************
//*****************************************************************************
//
//! \brief     Gets the number of sensors
//!
//! \param[in] handle  The Clarke transform handle
//!
//! \return    The number of sensors
//
//*****************************************************************************
static inline uint_least8_t
CLARKE_getNumSensors(CLARKE_Handle handle)
{
    CLARKE_Obj *obj = (CLARKE_Obj *)handle;

    return(obj->numSensors);
} // end of CLARKE_getNumSensors() function

//*****************************************************************************
//
//! \brief     Initializes the Clarke transform module
//!
//! \param[in] pMemory   A pointer to the memory for the Clarke object
//!
//! \param[in] numBytes  The number of bytes allocated for the Clarke object
//!
//! \return The Clarke (CLARKE) object handle
//
//*****************************************************************************
extern CLARKE_Handle
CLARKE_init(void *pMemory, const size_t numBytes);

//*****************************************************************************
//
//! \brief     Runs the Clarke transform module for three inputs
//!
//! \param[in] handle  The Clarke transform handle
//!
//! \param[in] pInVec        The pointer to the input vector
//!
//! \param[in] pOutVec       The pointer to the output vector
//!
//! \return    None
//
//*****************************************************************************
static inline void
CLARKE_run(CLARKE_Handle handle, const MATH_Vec3 *pInVec, MATH_Vec2 *pOutVec)
{
    CLARKE_Obj *obj = (CLARKE_Obj *)handle;

    uint_least8_t numSensors = obj->numSensors;

    float32_t alpha_sf = obj->alpha_sf;
    float32_t beta_sf = obj->beta_sf;

    //
    // Perform the Clarke transform for either 2 or 3 sensor setups
    //
    if(numSensors == 3)
    {
        pOutVec->value[0] =
            ((pInVec->value[0] *
              (float32_t)2.0) - (pInVec->value[1] + pInVec->value[2])) * alpha_sf;
        pOutVec->value[1] = (pInVec->value[1] - pInVec->value[2]) * beta_sf;
    }
    else if(numSensors == 2)
    {
        pOutVec->value[0] = pInVec->value[0] * alpha_sf;
        pOutVec->value[1] =
            (pInVec->value[0] + (pInVec->value[1] * (float32_t)2.0)) * beta_sf;
    }
    else
    {
        //
        // Assign value 0 if all conditions fail
        //
        pOutVec->value[0] = (float32_t)0;
        pOutVec->value[1] = (float32_t)0;
    }

    return;
} // end of CLARKE_run() function

//*****************************************************************************
//
//! \brief     Runs the Clarke transform module for two inputs
//!
//! \param[in] handle  The Clarke transform handle
//!
//! \param[in] pInVec        The pointer to the input vector
//!
//! \param[in] pOutVec       The pointer to the output vector
//!
//! \return    None
//
//*****************************************************************************
static inline void
CLARKE_run_twoInput(CLARKE_Handle handle, const MATH_Vec2 *pInVec,
                    MATH_Vec2 *pOutVec)
{
    CLARKE_Obj *obj = (CLARKE_Obj *)handle;

    float32_t beta_sf = obj->beta_sf;

    pOutVec->value[0] = pInVec->value[0];

    pOutVec->value[1] =
        (pInVec->value[0] + (pInVec->value[1] * (float32_t)2.0)) * beta_sf;

    return;
} // end of CLARKE_run_twoInput() function

//*****************************************************************************
//
//! \brief     Sets the number of sensors
//!
//! \param[in] handle  The Clarke transform handle
//!
//! \param[in] numSensors    The number of sensors
//!
//! \return    None
//
//*****************************************************************************
static inline void
CLARKE_setNumSensors(CLARKE_Handle handle, const uint_least8_t numSensors)
{
    CLARKE_Obj *obj = (CLARKE_Obj *)handle;

    obj->numSensors = numSensors;

    return;
} // end of CLARKE_setNumSensors() function

//*****************************************************************************
//
//! \brief     Sets the scale factors
//!
//! \param[in] handle  The Clarke transform handle
//!
//! \param[in] alpha_sf      The scale factor for the alpha voltage
//!
//! \param[in] beta_sf       The scale factor for the beta voltage
//!
//! \return    None
//
//*****************************************************************************
static inline void
CLARKE_setScaleFactors(CLARKE_Handle handle, const float32_t alpha_sf,
                       const float32_t beta_sf)
{
    CLARKE_Obj *obj = (CLARKE_Obj *)handle;

    obj->alpha_sf = alpha_sf;
    obj->beta_sf = beta_sf;

    return;
} // end of CLARKE_setScaleFactors() function

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

#endif // end of CLARKE_H defines
