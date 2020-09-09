//#############################################################################
//
// FILE:   math.h
//
// TITLE:  C28x math library (floating point)
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

#ifndef TI_MATH_H
#define TI_MATH_H

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
//! \addtogroup MATH
//! @{
//
//*****************************************************************************

#ifndef __TMS320C28XX_CLA__
#include <math.h>
#endif

#include "types.h"

//*****************************************************************************
//
//! \brief Defines conversion scale factor from N*m to lb*in
//
//*****************************************************************************
#define MATH_Nm_TO_lbin_SF        ((float32_t)(8.8507457913))

//*****************************************************************************
//
//! \brief Defines 2/3
//
//*****************************************************************************
#define MATH_TWO_OVER_THREE       ((float32_t) \
                                   (0.66666666666666666666666666666667))

//*****************************************************************************
//
//! \brief Defines 1/3
//
//*****************************************************************************
#define MATH_ONE_OVER_THREE       ((float32_t) \
                                   (0.33333333333333333333333333333333))

//*****************************************************************************
//
//! \brief Defines 1/(pi)
//
//*****************************************************************************
#define MATH_ONE_OVER_PI          ((float32_t)(0.318309886183791))

//*****************************************************************************
//
//! \brief Defines 1/sqrt(3)
//
//*****************************************************************************
#define MATH_ONE_OVER_SQRT_THREE  ((float32_t) \
                                   (0.57735026918962576450914878050196))

//*****************************************************************************
//
//! \brief Defines 1/(4*pi)
//
//*****************************************************************************
#define MATH_ONE_OVER_FOUR_PI     ((float32_t)(0.07957747154594767))

//*****************************************************************************
//
//! \brief Defines 1/(2*pi)
//
//*****************************************************************************
#define MATH_ONE_OVER_TWO_PI     ((float32_t) (0.1591549430918954))

//*****************************************************************************
//
//! \brief Defines pi
//
//*****************************************************************************
#define MATH_PI                   ((float32_t) \
                                   (3.1415926535897932384626433832795))

//*****************************************************************************
//
//! \brief Defines pi per unit
//
//*****************************************************************************
#define MATH_PI_PU                ((float32_t)(0.5))

//*****************************************************************************
//
//! \brief Defines 2*pi
//
//*****************************************************************************
#define MATH_TWO_PI               ((float32_t)(6.283185307179586))

//*****************************************************************************
//
//! \brief Defines 2*pi per unit
//
//*****************************************************************************
#define MATH_TWO_PI_PU            ((float32_t)(1.0))

//*****************************************************************************
//
//! \brief Defines 4*pi
//
//*****************************************************************************
#define MATH_FOUR_PI               ((float32_t)(12.56637061435917))

//*****************************************************************************
//
//! \brief Defines 4*pi per unit
//
//*****************************************************************************
#define MATH_FOUR_PI_PU            ((float32_t)(2.0))

//*****************************************************************************
//
//! \brief Defines pi/2
//
//*****************************************************************************
#define MATH_PI_OVER_TWO           ((float32_t)(1.570796326794897))

//*****************************************************************************
//
//! \brief Defines pi/2 per unit
//
//*****************************************************************************
#define MATH_PI_OVER_TWO_PU        ((float32_t)(0.25))

//*****************************************************************************
//
//! \brief Defines pi/4
//
//*****************************************************************************
#define MATH_PI_OVER_FOUR          ((float32_t)(0.785398163397448))

//*****************************************************************************
//
//! \brief Defines pi/4 per unit
//
//*****************************************************************************
#define MATH_PI_OVER_FOUR_PU        ((float32_t)(0.125))

//*****************************************************************************
//
//! \brief Defines a two element vector
//
//*****************************************************************************
typedef struct _MATH_Vec2_
{
    float32_t value[2];
} MATH_Vec2;

typedef MATH_Vec2 MATH_vec2;

//*****************************************************************************
//
//! \brief Defines a three element vector
//
//*****************************************************************************
typedef struct _MATH_Vec3_
{
    float32_t value[3];
} MATH_Vec3;

typedef MATH_Vec3 MATH_vec3;

//*****************************************************************************
//
//! \brief     Finds the absolute value
//!
//! \param[in] in   The input value
//!
//! \return    The absolute value
//
//*****************************************************************************
static inline
float32_t MATH_abs(const float32_t in)
{
    float32_t out = in;

    if(in < (float32_t)0.0)
    {
        out = -in;
    }

    return(out);
} // end of MATH_abs() function


//*****************************************************************************
//
//! \brief     Finds the maximum value between the twp input values
//!
//! \param[in] in   The input values
//!
//! \return    The absolute value
//
//*****************************************************************************
static inline
float32_t MATH_max(const float32_t in1, const float32_t in2)
{
  float32_t out = in1;


  if(in1 < in2)
    {
      out = in2;
    }

  return(out);
} // end of MATH_max() function


//*****************************************************************************
//
//! \brief     Finds the minimum value between the twp input values
//!
//! \param[in] in   The input values
//!
//! \return    The absolute value
//
//*****************************************************************************
static inline
float32_t MATH_min(const float32_t in1, const float32_t in2)
{
  float32_t out = in1;


  if(in1 > in2)
    {
      out = in2;
    }

  return(out);
} // end of MATH_min() function

//*****************************************************************************
//
//! \brief     Increments an angle value and handles wrap-around
//!
//! \param[in] angle_rad       The angle value, rad
//!
//! \param[in] angleDelta_rad  The angle increment value, rad
//!
//! \return    The incremented angle value, rad
//
//*****************************************************************************
#ifdef __TMS320C28XX_CLA__
#pragma FUNC_ALWAYS_INLINE(MATH_incrAngle)
#endif

static inline float32_t
MATH_incrAngle(const float32_t angle_rad, const float32_t angleDelta_rad)
{
    float32_t angleNew_rad;

    //
    // Increment the angle
    //
    angleNew_rad = angle_rad + angleDelta_rad;

    //
    // Check for limits
    //
    if(angleNew_rad > MATH_PI)
    {
        angleNew_rad -= MATH_TWO_PI;
    }
    else if(angleNew_rad < (-MATH_PI))
    {
        angleNew_rad += MATH_TWO_PI;
    }
    else
    {
        //
        // Doing nothing as of now
        //
        ;
    }

    return(angleNew_rad);
} // end of MATH_incrAngle() function

//*****************************************************************************
//
//! \brief     Saturates the input value between the minimum and maximum values
//!
//! \param[in] in   The input value
//!
//! \param[in] max  The maximum value allowed
//!
//! \param[in] min  The minimum value allowed
//!
//! \return    The saturated value
//
//*****************************************************************************
static inline float32_t
MATH_sat(const float32_t in, const float32_t max, const float32_t min)
{
    float32_t out = in;

    if(in < min)
    {
        out = min;
    }
    else if(in > max)
    {
        out = max;
    }
    else
    {
        //
        // Do nothing as of now
        //
        ;
    }

    return(out);
} // end of MATH_sat() function

//----------------------------------------------------------------------------
// For Motor Fault Diagnostic
//-----------------------------------------------------------------------------
//*****************************************************************************
//
//! \brief Defines a two element vector
//
//*****************************************************************************
typedef struct _MATH_cplx_vec2_
{

    cplx_float_t  value[2];

} MATH_cplx_vec2;

//*****************************************************************************
//
//! \brief Defines a three element vector
//
//*****************************************************************************
typedef struct _MATH_cplx_vec3_
{

    cplx_float_t  value[3];

} MATH_cplx_vec3;

//*****************************************************************************
//
//! \brief Defines 6*pi
//
//*****************************************************************************
#define MATH_SIX_PI               ((float32_t)(18.8495559215))

//*****************************************************************************
//
//! \brief Defines sqrt(3)/2
//
//*****************************************************************************
#define MATH_SQRTTHREE_OVER_TWO       ((float32_t)(0.8660254038))


//*****************************************************************************
//
//! \brief     Finds the absolute value
//!
//! \param[in] in   The input value
//!
//! \return    The absolute value
//
//*****************************************************************************
#ifdef __TMS320C28XX_CLA__
static inline
float32_t cAbsSq(const cplx_float_t* x1)
{
    float32_t y;

    y = 0;

    return y;
}
#else
static inline
float32_t cAbsSq(const cplx_float_t* x1)
{
    float32_t y;

    #ifdef __TMS320C28XX_TMU__
    y = pow(x1->real,2) + pow(x1->imag,2);
    #else
    y = (float32_t)(pow((double_t)(x1->real),(double_t)(2.0)) +
            pow((double_t)(x1->imag),(double_t)(2.0)));
    #endif  // __TMS320C28XX_TMU__

    return y;
}
#endif // __TMS320C28XX_CLA__


static inline
float32_t MATH_sign(const float32_t in)
{
  float32_t out = 1.0;


  if(in < 0.0)
    {
      out = -1.0;
    }

  return(out);
} // end of MATH_sign() function

static inline void
MATH_mult_cc(cplx_float_t* x1, cplx_float_t* x2, cplx_float_t * y)
{
    y->real = x1->real * x2->real - x1->imag * x2->imag;
    y->imag = x1->real * x2->imag + x1->imag * x2->real;
    return;
}

static inline void
MATH_add_cc(cplx_float_t* x1, cplx_float_t* x2, cplx_float_t * y)
{
    y->real = x1->real  +  x2->real;
    y->imag = x1->imag  +  x2->imag;
    return;
}

static inline void
MATH_sub_cc(cplx_float_t* x1, cplx_float_t* x2, cplx_float_t * y)
{
    y->real = x1->real - x2->real;
    y->imag = x1->imag - x2->imag;
    return;
}

static inline void
MATH_mult_rc(float32_t x1, cplx_float_t* x2, cplx_float_t * y)
{
    y->real = x1 * x2->real;
    y->imag = x1 * x2->imag;
    return;
}

static inline void
MATH_add_rc(float32_t x1, cplx_float_t* x2, cplx_float_t * y)
{
    y->real = x1  +  x2->real;
    y->imag = x2->imag;
    return;
}

static inline void
MATH_sub_rc(float32_t x1, cplx_float_t* x2, cplx_float_t* y)
{
    y->real = x1 - x2->real;
    y->imag = - x2->imag;
    return;
}

static inline void
MATH_sub_cr(cplx_float_t* x1, float32_t x2, cplx_float_t* y)
{
    y->real = x1->real - x2;
    y->imag = x1->imag;

    return;
}



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

#endif // TI_MATH_H
