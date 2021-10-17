//#############################################################################
//
// FILE:   types.h
//
// TITLE:  C28x type definitions library
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

#ifndef TYPES_H
#define TYPES_H

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
//! \addtogroup TYPES
//! @{
//
//*****************************************************************************

#include "stdbool.h"
#include "stdint.h"

#if !defined(__TMS320C28XX_CLA__)
#include "string.h"
#endif

//*****************************************************************************
//
// 32-bit & 64-bit float type
//
//*****************************************************************************
#ifndef C2000_IEEE754_TYPES
#define C2000_IEEE754_TYPES
typedef float               float32_t;
typedef long double         float64_t;
#endif // C2000_IEEE754_TYPES

//*****************************************************************************
//
//! \brief Defines high
//
//*****************************************************************************
#ifndef HIGH
#define   HIGH          1
#endif

//*****************************************************************************
//
//! \brief Defines low
//
//*****************************************************************************
#define   LOW          0

//*****************************************************************************
//
//! \brief Defines off
//
//*****************************************************************************
#define   OFF           0

//*****************************************************************************
//
//! \brief Defines ok
//
//*****************************************************************************
#define   OK            0

//*****************************************************************************
//
//! \brief Defines on
//
//*****************************************************************************
#define   ON            1

//*****************************************************************************
//
//! \brief Defines generic error
//
//*****************************************************************************
#define   ERROR         1

//*****************************************************************************
//
//! \brief Defines pass
//
//*****************************************************************************
#define   PASS          1

//*****************************************************************************
//
//! \brief Defines fails
//
//*****************************************************************************
#define   FAIL          0

//*****************************************************************************
//
//! \brief Defines the portable data type for a status result
//
//*****************************************************************************
typedef unsigned int status;

//*****************************************************************************
//
//! \brief Defines the portable data type for 32 bit, signed floating-point
//!        data
//
//*****************************************************************************
typedef float float_t;

//*****************************************************************************
//
//! \brief Defines the portable data type for 64 bit, signed floating-point
//!        data
//
//*****************************************************************************
//typedef long double double_t;
typedef double double_t;

#ifdef __TMS320C28XX_CLA__
#ifndef NULL
/*LDRA_INSPECTED 218 S MR12 21.2 "NULL is defined in string.h but this header
is not supported by CLA compiler, so defining NULL"*/
/*LDRA_INSPECTED 626 S MR12 20.4 "NULL is defined in string.h but this header
is not supported by CLA compiler, so defining NULL"*/
#define NULL 0
#endif

typedef unsigned int size_t;
#endif // __TMS320C28XX_CLA__

//*****************************************************************************
//
//! \brief Define the complex data type for at least 8 bit signed real and
//!        imaginary components
//
//*****************************************************************************
typedef struct _cplx_int_least8_t
{
    int_least8_t imag;
    int_least8_t real;
} cplx_int_least8_t;

//*****************************************************************************
//
//! \brief Define the complex data type for at least 8 bit unsigned real and
//!        imaginary components
//
//*****************************************************************************
typedef struct _cplx_uint_least8_t
{
    uint_least8_t imag;
    uint_least8_t real;
} cplx_uint_least8_t;

//*****************************************************************************
//
//! \brief Define the complex data type for at least 16 bit signed real and
//!        imaginary components
//
//*****************************************************************************
typedef struct _cplx_least16_t
{
    int_least16_t imag;
    int_least16_t real;
} cplx_int_least16_t;

//*****************************************************************************
//
//! \brief Define the complex data type for at least 16 bit unsigned real and
//!        imaginary components
//
//*****************************************************************************
typedef struct _cplx_uleast16_t
{
    uint_least16_t imag;
    uint_least16_t real;
} cplx_uint_least16_t;

//*****************************************************************************
//
//! \brief Define the complex data type for at least 32 bit signed real and
//!        imaginary components
//
//*****************************************************************************
typedef struct _cplx_int_least32_t_
{
    int_least32_t imag;
    int_least32_t real;
} cplx_int_least32_t;

//*****************************************************************************
//
//! \brief Define the complex data type for at least 32 bit unsigned real and
//!        imaginary components
//
//*****************************************************************************
typedef struct _cplx_uint_least32_t_
{
    uint_least32_t imag;
    uint_least32_t real;
} cplx_uint_least32_t;

//*****************************************************************************
//
//! \brief Define the complex data type for 16 bit signed real and imaginary
//!        components
//
//*****************************************************************************
typedef struct _cplx_int16_t_
{
    int16_t imag;
    int16_t real;
} cplx_int16_t;

//*****************************************************************************
//
//! \brief Define the complex data type for 16 bit unsigned real and imaginary
//!        components
//
//*****************************************************************************
typedef struct _cplx_uint16_t_
{
    uint16_t imag;
    uint16_t real;
} cplx_uint16_t;

//*****************************************************************************
//
//! \brief Define the complex data type for 32 bit signed real and imaginary
//!        components
//
//*****************************************************************************
typedef struct _cplx_int32_t
{
    int32_t imag;
    int32_t real;
} cplx_int32_t;

//*****************************************************************************
//
//! \brief Define the complex data type for 32 bit unsigned real and imaginary
//!        components
//
//*****************************************************************************
typedef struct _cplx_uint32_t
{
    uint32_t imag;
    uint32_t real;
} cplx_uint32_t;


//-----------------------------------------------------------------------------
//*****************************************************************************
//
//! \brief Define the complex data type for float real and imaginary
//!        components
//!
//
//*****************************************************************************
typedef struct _cplx_float_t
{
    float_t  real;
    float_t  imag;
} cplx_float_t;



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

#endif  // TYPES_H
