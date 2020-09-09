//#############################################################################
//
// FILE:   filter_fo.h
//
// TITLE:  C28x InstaSPIN filter library, first-order
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

#ifndef FILTER_FO_H
#define FILTER_FO_H

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

#include "types.h"

//!
//!
//! \defgroup FILTER_FO FILTER_FO
//!
//! @{

//*****************************************************************************
//
//! \brief Defines the first-order filter (FILTER_FO) object
//
//*****************************************************************************
typedef struct _FILTER_FO_Obj_
{
    float32_t a1;       //!< the denominator filter coefficient value for z^(-1)
    float32_t b0;       //!< the numerator filter coefficient value for z^0
    float32_t b1;       //!< the numerator filter coefficient value for z^(-1)
    float32_t x1;       //!< the input value at time sample n=-1
    float32_t y1;       //!< the output value at time sample n=-1
} FILTER_FO_Obj;

//*****************************************************************************
//
//! \brief Defines the first-order filter (FILTER_FO) handle
//
//*****************************************************************************
typedef struct _FILTER_FO_Obj_ *FILTER_FO_Handle;

//*****************************************************************************
//
//! \brief     Gets the first-order filter denominator coefficient a1
//!
//! \param[in] handle  The filter handle
//!
//! \return    The filter coefficient value for z^(-1)
//
//*****************************************************************************
static inline float32_t
FILTER_FO_get_a1(FILTER_FO_Handle handle)
{
    FILTER_FO_Obj *obj = (FILTER_FO_Obj *)handle;

    return(obj->a1);
} // end of FILTER_FO_get_a1() function

//*****************************************************************************
//
//! \brief     Gets the first-order filter numerator coefficient b0
//!
//! \param[in] handle  The filter handle
//!
//! \return    The filter coefficient value for z^0
//
//*****************************************************************************
static inline float32_t
FILTER_FO_get_b0(FILTER_FO_Handle handle)
{
    FILTER_FO_Obj *obj = (FILTER_FO_Obj *)handle;

    return(obj->b0);
} // end of FILTER_FO_get_b0() function

//*****************************************************************************
//
//! \brief     Gets the first-order filter numerator coefficient b1
//!
//! \param[in] handle  The filter handle
//!
//! \return    The filter coefficient value for z^(-1)
//
//*****************************************************************************
static inline float32_t
FILTER_FO_get_b1(FILTER_FO_Handle handle)
{
    FILTER_FO_Obj *obj = (FILTER_FO_Obj *)handle;

    return(obj->b1);
} // end of FILTER_FO_get_b1() function

//*****************************************************************************
//
//! \brief     Gets the first-order filter input value at time sample n=-1
//!
//! \param[in] handle  The filter handle
//!
//! \return    The input value at time sample n=-1
//
//*****************************************************************************
static inline float32_t
FILTER_FO_get_x1(FILTER_FO_Handle handle)
{
    FILTER_FO_Obj *obj = (FILTER_FO_Obj *)handle;

    return(obj->x1);
} // end of FILTER_FO_get_x1() function

//*****************************************************************************
//
//! \brief     Gets the first-order filter output value at time sample n=-1
//!
//! \param[in] handle  The filter handle
//!
//! \return    The output value at time sample n=-1
//
//*****************************************************************************
static inline float32_t
FILTER_FO_get_y1(FILTER_FO_Handle handle)
{
    FILTER_FO_Obj *obj = (FILTER_FO_Obj *)handle;

    return(obj->y1);
} // end of FILTER_FO_get_y1() function

//*****************************************************************************
//
//! \brief     Gets the first-order filter denominator coefficients
//!
//! \param[in] handle  The filter handle
//!
//! \param[in] pa1     The pointer to memory for the filter coefficient value
//!                    for z^(-1)
//!
//! \return    None
//
//*****************************************************************************
extern void
FILTER_FO_getDenCoeffs(FILTER_FO_Handle handle, float32_t *pa1);

//*****************************************************************************
//
//! \brief     Gets the initial conditions of the first-order filter
//!
//! \param[in] handle  The filter handle
//!
//! \param[in] px1     The pointer to memory for the input value at time
//!                    sample n=-1
//!
//! \param[in] py1     The pointer to memory for the output value at time
//!                    sample n=-1
//!
//! \return    None
//
//*****************************************************************************
extern void
FILTER_FO_getInitialConditions(FILTER_FO_Handle handle, float32_t *px1,
                               float32_t *py1);

//*****************************************************************************
//
//! \brief     Gets the first-order filter numerator coefficients
//!
//! \param[in] handle  The filter handle
//!
//! \param[in] pb0     The pointer to memory for the filter coefficient
//!                    value for z^0
//!
//! \param[in] pb1     The pointer to memory for the filter coefficient
//!                    value for z^(-1)
//!
//! \return    None
//
//*****************************************************************************
extern void
FILTER_FO_getNumCoeffs(FILTER_FO_Handle handle, float32_t *pb0, float32_t *pb1);

//*****************************************************************************
//
//! \brief     Initializes the first-order filter
//!
//! \param[in] pMemory   A pointer to the memory for the first-order
//!                      filter object
//!
//! \param[in] numBytes  The number of bytes allocated for the first-order
//!                      filter object, bytes
//!
//! \return    The filter (FILTER) object handle
//
//*****************************************************************************
extern FILTER_FO_Handle
FILTER_FO_init(void *pMemory, const size_t numBytes);

//*****************************************************************************
//
//! \brief     Runs a first-order filter of the form
//!            y[n] = b0*x[n] + b1*x[n-1] - a1*y[n-1]
//!
//! \param[in] handle      The filter handle
//!
//! \param[in] inputValue  The input value to filter
//!
//! \return    The output value from the filter
//
//*****************************************************************************
#ifdef __TMS320C28XX_CLA__
#pragma FUNC_ALWAYS_INLINE(FILTER_FO_run)
#endif // __TMS320C28XX_CLA__

static inline float32_t
FILTER_FO_run(FILTER_FO_Handle handle, const float32_t inputValue)
{
    FILTER_FO_Obj *obj = (FILTER_FO_Obj *)handle;

    float32_t a1 = obj->a1;
    float32_t b0 = obj->b0;
    float32_t b1 = obj->b1;
    float32_t x1 = obj->x1;
    float32_t y1 = obj->y1;
    float32_t y0 = 1.0;
    float32_t x0 = inputValue;

    //
    // Compute the output
    //
    y0 = (b0 * x0) + (b1 * x1) - (a1 * y1);
    // y0 = (b0 * inputValue) + (a1 * y1);

    //
    // Store values for next time
    //
    obj->x1 = inputValue;
    obj->y1 = y0;

    return(y0);
} // end of FILTER_FO_run() function

//*****************************************************************************
//
//! \brief     Runs a first-order filter of the form
//!            y[n] = b0*x[n] - a1*y[n-1]
//!
//! \param[in] handle      The filter handle
//!
//! \param[in] inputValue  The input value to filter
//!
//! \return    The output value from the filter
//
//*****************************************************************************
static inline float32_t
FILTER_FO_run_form_0(FILTER_FO_Handle handle, const float32_t inputValue)
{
    FILTER_FO_Obj *obj = (FILTER_FO_Obj *)handle;

    float32_t a1 = obj->a1;
    float32_t b0 = obj->b0;
    float32_t y1 = obj->y1;

    //
    // Compute the output
    //
    float32_t y0 = (b0 * inputValue) - (a1 * y1);

    //
    // Store values for next time
    //
    obj->y1 = y0;

    return(y0);
} // end of FILTER_FO_run_form_0() function

//*****************************************************************************
//
//! \brief     Sets the first-order filter denominator coefficient a1
//!
//! \param[in] handle  The filter handle
//!
//! \param[in] a1      The filter coefficient value for z^(-1)
//!
//! \return    None
//
//*****************************************************************************
static inline void
FILTER_FO_set_a1(FILTER_FO_Handle handle, const float32_t a1)
{
    FILTER_FO_Obj *obj = (FILTER_FO_Obj *)handle;

    obj->a1 = a1;

    return;
} // end of FILTER_FO_set_a1() function

//*****************************************************************************
//
//! \brief     Sets the first-order filter numerator coefficient b0
//!
//! \param[in] handle  The filter handle
//!
//! \param[in] b0      The filter coefficient value for z^0
//!
//! \returnh   None
//
//*****************************************************************************
static inline void
FILTER_FO_set_b0(FILTER_FO_Handle handle, const float32_t b0)
{
    FILTER_FO_Obj *obj = (FILTER_FO_Obj *)handle;

    obj->b0 = b0;

    return;
} // end of FILTER_FO_set_b0() function

//*****************************************************************************
//
//! \brief     Sets the first-order filter numerator coefficient b1
//!
//! \param[in] handle  The filter handle
//!
//! \param[in] b1      The filter coefficient value for z^(-1)
//!
//! \return    None
//
//*****************************************************************************
static inline void
FILTER_FO_set_b1(FILTER_FO_Handle handle, const float32_t b1)
{
    FILTER_FO_Obj *obj = (FILTER_FO_Obj *)handle;

    obj->b1 = b1;

    return;
} // end of FILTER_FO_set_b1() function

//*****************************************************************************
//
//! \brief     Sets the first-order filter input value at time sample n=-1
//!
//! \param[in] handle  The filter handle
//!
//! \param[in] x1      The input value at time sample n=-1
//!
//! \return    None
//
//*****************************************************************************
static inline void
FILTER_FO_set_x1(FILTER_FO_Handle handle, const float32_t x1)
{
    FILTER_FO_Obj *obj = (FILTER_FO_Obj *)handle;

    obj->x1 = x1;

    return;
} // end of FILTER_FO_set_x1() function

//*****************************************************************************
//
//! \brief     Sets the first-order filter output value at time sample n=-1
//!
//! \param[in] handle  The filter handle
//!
//! \param[in] y1      The output value at time sample n=-1
//!
//! \return    None
//
//*****************************************************************************
static inline void
FILTER_FO_set_y1(FILTER_FO_Handle handle, const float32_t y1)
{
    FILTER_FO_Obj *obj = (FILTER_FO_Obj *)handle;

    obj->y1 = y1;

    return;
} // end of FILTER_FO_set_y1() function

//*****************************************************************************
//
//! \brief     Sets the first-order filter denominator coefficients
//!
//! \param[in] handle  The filter handle
//!
//! \param[in] a1      The filter coefficient value for z^(-1)
//!
//! \return    None
//
//*****************************************************************************
extern void
FILTER_FO_setDenCoeffs(FILTER_FO_Handle handle, const float32_t a1);

//*****************************************************************************
//
//! \brief     Sets the initial conditions of the first-order filter
//!
//! \param[in] handle  The filter handle
//!
//! \param[in] x1      The input value at time sample n=-1
//!
//! \param[in] y1      The output value at time sample n=-1
//!
//! \return    None
//
//*****************************************************************************
extern void
FILTER_FO_setInitialConditions(FILTER_FO_Handle handle, const float32_t x1,
                               const float32_t y1);

//*****************************************************************************
//
//! \brief     Sets the first-order filter numerator coefficients
//!
//! \param[in] handle  The filter handle
//!
//! \param[in] b0      The filter coefficient value for z^0
//!
//! \param[in] b1      The filter coefficient value for z^(-1)
//!
//! \return    None
//
//*****************************************************************************
extern void
FILTER_FO_setNumCoeffs(FILTER_FO_Handle handle, const float32_t b0,
                       const float32_t b1);

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

#endif // FILTER_FO_H
