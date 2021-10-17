//#############################################################################
//
// FILE:   svgen.h
//
// TITLE:  C28x InstaSPIN Space Vector Generator (SVGEN) library
//         (floating point)
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

#ifndef SVGEN_H
#define SVGEN_H

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
//! \addtogroup SVGEN
//! @{
//
//*****************************************************************************

#include "types.h"
#include "libraries/math/include/math.h"

//*****************************************************************************
//
//! \brief Defines sqrt(3)/2
//
//*****************************************************************************
#define SVGEN_SQRT3_OVER_2                    ((float32_t)(0.8660254038))

//*****************************************************************************
//
//! \brief Defines the Space Vector Generator (SVGEN) object
//
//*****************************************************************************
typedef struct _SVGEN_Obj_
{
    float32_t oneOverDcBus_invV;    //!< The inverse DC bus voltage value, 1/V
} SVGEN_Obj;

//*****************************************************************************
//
//! \brief Defines the SVGEN handle
//
//*****************************************************************************
typedef struct _SVGEN_Obj_ *SVGEN_Handle;

//*****************************************************************************
//
//! \brief     Gets the one over DC bus voltage value
//!
//! \param[in] handle  The space vector generator (SVGEN) handle
//!
//! \return    The inverse DC bus voltage value, 1/V
//
//*****************************************************************************
static inline float32_t
SVGEN_getOneOverDcBus_invV(SVGEN_Handle handle)
{
    SVGEN_Obj *obj = (SVGEN_Obj *)handle;

    return(obj->oneOverDcBus_invV);
} // end of SVGEN_getOneOverDcBus_invV() function

//*****************************************************************************
//
//! \brief     Initializes the space vector generator module
//!
//! \param[in] pMemory   A pointer to the space vector generator object memory
//!
//! \param[in] numBytes  The number of bytes allocated for the space vector
//!                      generator object, bytes
//!
//! \return    The state vector generator (SVGEN) object handle
//
//*****************************************************************************
extern SVGEN_Handle
SVGEN_init(void *pMemory, const size_t numBytes);

//*****************************************************************************
//
//! \brief	Implements a SVM that saturates at the level of MaxModulation.
//!
//! \param[in] handle    The space vector generator (SVGEN) handle
//!
//! \param[in] pVab_V    The pointer to the alpha/beta voltages, V
//!
//! \param[in] pVabc_pu  The pointer to the three phase voltages, pu
//!
//! \return    None
//
//*****************************************************************************
static inline void
SVGEN_run(SVGEN_Handle handle, const MATH_Vec2 *pVab_V, MATH_Vec3 *pVabc_pu)
{
    float32_t Vmax_pu = 0,Vmin_pu = 0,Vcom_pu;
    float32_t oneOverDcBus_invV = SVGEN_getOneOverDcBus_invV(handle);

    float32_t Valpha_V = pVab_V->value[0];
    float32_t Vbeta_V = pVab_V->value[1];

    float32_t Va_pu = Valpha_V * oneOverDcBus_invV;
    float32_t Vbeta_pu = Vbeta_V * oneOverDcBus_invV;

    float32_t Va_tmp = (float32_t)(0.5) * (-Va_pu);
    float32_t Vb_tmp = SVGEN_SQRT3_OVER_2 * Vbeta_pu;
    
    //
    // -0.5*Valpha + sqrt(3)/2 * Vbeta
    //
    float32_t Vb_pu = Va_tmp + Vb_tmp;
    
    //
    // -0.5*Valpha - sqrt(3)/2 * Vbeta
    float32_t Vc_pu = Va_tmp - Vb_tmp;

    //
    // Find Vmax and Vmin
    //
    if(Va_pu > Vb_pu)
    {
        Vmax_pu = Va_pu;
        Vmin_pu = Vb_pu;
    }
    else
    {
        Vmax_pu = Vb_pu;
        Vmin_pu = Va_pu;
    }

    if(Vc_pu > Vmax_pu)
    {
        Vmax_pu = Vc_pu;
    }
    else if(Vc_pu < Vmin_pu)
    {
        Vmin_pu = Vc_pu;
    }
    else
    {
        //
        // Do nothing as if now
        //
        ;
    }

    //
    // Compute Vcom = 0.5*(Vmax+Vmin)
    //
    Vcom_pu = (float32_t)0.5 * (Vmax_pu + Vmin_pu);

    //
    // Subtract common-mode term to achieve SV modulation
    //
    pVabc_pu->value[0] = (Va_pu - Vcom_pu);
    pVabc_pu->value[1] = (Vb_pu - Vcom_pu);
    pVabc_pu->value[2] = (Vc_pu - Vcom_pu);

    return;
} // end of SVGEN_run() function

//*****************************************************************************
//
//! \brief     Sets the one over DC bus voltage value
//!
//! \param[in] handle            The space vector generator (SVGEN) handle
//!
//! \param[in] oneOverVbus_invV  The inverse DC bus voltage value, 1/V
//!
//! \return    None
//
//*****************************************************************************
static inline void
SVGEN_setOneOverDcBus_invV(SVGEN_Handle handle,
                           const float32_t oneOverDcBus_invV)
{
    SVGEN_Obj *obj = (SVGEN_Obj *)handle;

    obj->oneOverDcBus_invV = oneOverDcBus_invV;

    return;
} // end of SVGEN_setOneOverDcBus_invV() function

//*****************************************************************************
//
//! \brief     Sets up the space vector generator (SVGEN) module
//!
//! \param[in] handle             The space vector generator (SVGEN) handle
//!
//! \param[in] oneOverDcBus_invV  The inverse DC bus voltage value, 1/V
//!
//! \return    None
//
//*****************************************************************************
static inline void
SVGEN_setup(SVGEN_Handle handle, const float32_t oneOverDcBus_invV)
{
    SVGEN_setOneOverDcBus_invV(handle,oneOverDcBus_invV);

    return;
} // end of SVGEN_setup() function

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

#endif // end of SVGEN_H definition
