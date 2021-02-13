//#############################################################################
//
// FILE:   park.h
//
// TITLE:  C28x InstaSPIN park transform library (fixed point)
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

#ifndef PARK_H
#define PARK_H

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
//! \addtogroup PARK
//! @{
//
//*****************************************************************************

#ifdef __TMS320C28XX_CLA__
#include "libraries/math/include/CLAmath.h"
#else
#include <math.h>
#endif // __TMS320C28XX_CLA__

#include "libraries/math/include/math.h"

//*****************************************************************************
//
//! \brief Defines the PARK object
//
//*****************************************************************************
typedef struct _PARK_Obj_
{
    float32_t sinTh;     //!< the sine of the angle between the d,q and the
                       //!< alpha, beta coordinate systems
    float32_t cosTh;     //!< the cosine of the angle between the d,q and the
                       //!< alpha, beta coordinate systems
} PARK_Obj;

//*****************************************************************************
//
//! \brief Defines the PARK handle
//
//*****************************************************************************
typedef struct _PARK_Obj_ *PARK_Handle;

//*****************************************************************************
//
//! \brief     Gets the cosine of the angle between the d,q and the
//!            alpha, beta coordinate systems
//!
//! \param[in] handle  The Park transform handle
//!
//! \return    The cosine of the angle
//
//*****************************************************************************
static inline float32_t
PARK_getCosTh(PARK_Handle handle)
{
    PARK_Obj *obj = (PARK_Obj *)handle;

    return(obj->cosTh);
} // end of PARK_getCosTh() function

//*****************************************************************************
//
//! \brief     Gets the cosine/sine phasor for the Park transform
//!
//! \param[in] handle   The Park transform handle
//!
//! \param[in] pPhasor  The pointer to the cosine/sine phasor
//!
//! \return    None
//
//*****************************************************************************
static inline void
PARK_getPhasor(PARK_Handle handle, MATH_Vec2 *pPhasor)
{
    PARK_Obj *obj = (PARK_Obj *)handle;

    pPhasor->value[0] = obj->cosTh;
    pPhasor->value[1] = obj->sinTh;

    return;
} // end of PARK_getPhasor() function

//*****************************************************************************
//
//! \brief     Gets the sine of the angle between the d,q and the alpha, beta
//!            coordinate systems
//!
//! \param[in] handle  The Park transform handle
//!
//! \return    The sine of the angle
//
//*****************************************************************************
static inline float32_t
PARK_getSinTh(PARK_Handle handle)
{
    PARK_Obj *obj = (PARK_Obj *)handle;

    return(obj->sinTh);
} // end of PARK_getSinTh() function

//*****************************************************************************
//
//! \brief     Initializes the Park transform module
//!
//! \param[in] pMemory    A pointer to the memory for the Park object
//!
//! \param[in] numBytes   The number of bytes allocated for the Park object,
//!                       bytes
//!
//! \return    The Park (PARK) object handle
//
//*****************************************************************************
extern PARK_Handle
PARK_init(void *pMemory, const size_t numBytes);

//*****************************************************************************
//
//! \brief     Runs the Park transform module
//!
//! \param[in] handle  The Park transform handle
//!
//! \param[in] pInVec      The pointer to the input vector
//!
//! \param[in] pOutVec     The pointer to the output vector
//!
//! \return    None
//
//*****************************************************************************
#ifdef __TMS320C28XX_CLA__
#pragma FUNC_ALWAYS_INLINE(PARK_run)
#endif

static inline void
PARK_run(PARK_Handle handle, const MATH_Vec2 *pInVec, MATH_Vec2 *pOutVec)
{
    PARK_Obj *obj = (PARK_Obj *)handle;

    float32_t sinTh = obj->sinTh;
    float32_t cosTh = obj->cosTh;

    float32_t value_0 = pInVec->value[0];
    float32_t value_1 = pInVec->value[1];

    pOutVec->value[0] = (value_0 * cosTh) + (value_1 * sinTh);
    pOutVec->value[1] = (value_1 * cosTh) - (value_0 * sinTh);

    return;
} // end of PARK_run() function

//*****************************************************************************
//
//! \brief     Sets the cosine of the angle between the d,q and the alpha, beta
//!            coordinate systems
//!
//! \param[in] handle  The Park transform handle
//!
//! \param[in] cosTh   The cosine of the angle
//!
//! \return    None
//
//*****************************************************************************
static inline void
PARK_setCosTh(PARK_Handle handle, const float32_t cosTh)
{
    PARK_Obj *obj = (PARK_Obj *)handle;

    obj->cosTh = cosTh;

    return;
} // end of PARK_setCosTh() function

//*****************************************************************************
//
//! \brief     Sets the cosine/sine phasor for the inverse Park transform
//!
//! \param[in] handle   The Park transform handle
//!
//! \param[in] pPhasor  The pointer to the cosine/sine phasor, pu
//!
//! \return    None
//
//*****************************************************************************
static inline void
PARK_setPhasor(PARK_Handle handle, const MATH_Vec2 *pPhasor)
{
    PARK_Obj *obj = (PARK_Obj *)handle;

    obj->cosTh = pPhasor->value[0];
    obj->sinTh = pPhasor->value[1];

    return;
} // end of PARK_setPhasor() function

//*****************************************************************************
//
//! \brief     Sets the sine of the angle between the d,q and the alpha, beta
//!            coordinate systems
//!
//! \param[in] handle  The Park transform handle
//!
//! \param[in] sinTh   The sine of the angle
//!
//! \return    None
//
//*****************************************************************************
static inline void
PARK_setSinTh(PARK_Handle handle, const float32_t sinTh)
{
    PARK_Obj *obj = (PARK_Obj *)handle;

    obj->sinTh = sinTh;

    return;
} // end of PARK_setSinTh() function

//*****************************************************************************
//
//! \brief     Sets up the Park transform module
//!
//! \param[in] handle  The Park transform handle
//!
//! \param[in] Th      The angle between the d,q and the alpha, beta coordinate
//!                    systems, rad
//!
//! \return    None
//
//*****************************************************************************
#ifdef __TMS320C28XX_CLA__
#pragma FUNC_ALWAYS_INLINE(PARK_setup)
#endif

static inline void
PARK_setup(PARK_Handle handle, const float32_t Th)
{
    PARK_Obj *obj = (PARK_Obj *)handle;

#ifdef __TMS320C28XX_CLA__
    obj->sinTh = CLAsin_inline(Th);
    obj->cosTh = CLAcos_inline(Th);
#else
    obj->sinTh = (float32_t)sin((double_t)Th);
    obj->cosTh = (float32_t)cos((double_t)Th);
#endif // __TMS320C28XX_CLA__

    return;
} // end of PARK_setup() function

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

#endif // end of PARK_H definition
