//#############################################################################
//
// FILE:   mtpa.h
//
// TITLE:  C28x Maximum torque per ampere (MTPA) (floating point)
//
//#############################################################################
// $TI Release: MotorControl SDK v3.00.01.00 $
// $Release Date: Tue May 26 19:13:58 CDT 2020 $
// $Copyright:
// Copyright (C) 2017-2019 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef MTPA_H
#define MTPA_H

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
//! \addtogroup MTPA
//! @{
//
//*****************************************************************************

#include "libraries/math/include/math.h"

#ifdef __TMS320C28XX_CLA__
#include "libraries/math/include/CLAmath.h"
#else
#include <math.h>
#endif


//! \brief Defines the maximum length of LUT for Ld
#define MTPA_LUT_INDEX_LD_MAX            20


//! \brief Defines the maximum length of LUT for Lq
#define MTPA_LUT_INDEX_LQ_MAX            20

//! \brief Defines the current delta of LUT for Ld, A
#define MTPA_LUT_DELTA_CURRENT_LD_A      0.5

//! \brief Defines the current delta of LUT for Lq, A
#define MTPA_LUT_DELTA_CURRENT_LD_A      0.5

//*****************************************************************************
//
//! \brief Defines the MTPA object
//
//*****************************************************************************
typedef struct _MTPA_Obj_
{
    MATH_Vec2 Idq_ref_A;            //!< the d/q-axis current reference
    float32_t angleCurrent_rad;     //!< the vector stator current reference
    float32_t Is_ref_A;             //!< the vector stator current reference
    float32_t kconst;               //!< the K constant for mtpa
    float32_t gconst;               //!< the G constant for mtpa
    float32_t deltaIs_Ld_A;         //!< the Is detla for Ls lookup table
    float32_t deltaIs_Lq_A;         //!< the Is detla for Ls lookup table
    uint16_t  indexMax_Ld;          //!< the Is detla for Ld lookup table
    uint16_t  indexMax_Lq;          //!< the Is detla for Lq lookup table
    bool      flagEnable;           //!< a flag to enable the controller
} MTPA_Obj;


//! \brief Defines the Ld array
extern const float32_t MTPA_Ld_tableData_H[MTPA_LUT_INDEX_LD_MAX + 1];

//! \brief Defines the Lq array
extern const float32_t MTPA_Lq_tableData_H[MTPA_LUT_INDEX_LQ_MAX + 1];

//*****************************************************************************
//
//! \brief Defines the MTPA handle
//
//*****************************************************************************
typedef struct _MTPA_Obj_ *MTPA_Handle;

//*****************************************************************************
//
// Prototypes for the APIs
//
//*****************************************************************************

//! \brief     Computes the motor constant for MTPA module
//! \param[in] handle      The maximum torque per ampere (MTPA) handle
//! \param[in] Ls_d_H      The direct stator inductance, Henry
//! \param[in] Ls_q_H      The quadrature stator inductance, Henry
//! \param[in] flux_Wb     The rated flux value, Wb
extern void MTPA_computeParameters(MTPA_Handle handle,
                                   const float32_t Ls_d_H,
                                   const float32_t Ls_q_H,
                                   const float32_t flux_Wb);

//! \brief     Disables the MTPA
//! \param[in] handle  The maximum torque per ampere (MTPA) handle
static inline void MTPA_disable(MTPA_Handle handle)
{
    MTPA_Obj *obj = (MTPA_Obj *)handle;

    obj->flagEnable = false;

    return;
} // end of MTPA_disable() function


//! \brief     Enables the MTPA
//! \param[in] handle  The maximum torque per ampere (MTPA) handle
static inline void MTPA_enable(MTPA_Handle handle)
{
    MTPA_Obj *obj = (MTPA_Obj *)handle;

    obj->flagEnable = true;

    return;
} // end of MTPA_enable() function


//! \brief     Gets the stator current phase angle memory address
//! \param[in] handle  The maximum torque per ampere (MTPA) handle
//! \return    The stator current phase angle memory address
static inline float32_t *MTPA_getCurrentAnglePhase_rad_addr(MTPA_Handle handle)
{
    MTPA_Obj *obj = (MTPA_Obj *)handle;

    return(&(obj->angleCurrent_rad));
} // end of MTPA_getCurrentAnglePhase_rad_addr() function


//! \brief     Gets the stator current phase angle value (angleCurrent_rad)
//! \param[in] handle  The maximum torque per ampere (MTPA) handle
//! \return    The stator current phase angle value, rad
static inline float32_t MTPA_getCurrentAngle_rad(MTPA_Handle handle)
{
    MTPA_Obj *obj = (MTPA_Obj *)handle;

    return(obj->angleCurrent_rad);
} // end of MTPA_getCurrentAngle_rad() function


//! \brief     Gets the motor constant of MTPA
//! \param[in] handle  The maximum torque per ampere (MTPA) handle
//! \return    The motor constant of MTPA
static inline float32_t MTPA_getKconst(MTPA_Handle handle)
{
  MTPA_Obj *obj = (MTPA_Obj *)handle;

  return(obj->kconst);
} // end of MTPA_getKconst() function


//! \brief     Gets the stator current reference memory address
//! \param[in] handle  The maximum torque per ampere (MTPA) handle
//! \return    The stator current reference memory address
static inline float32_t *MTPA_getIs_ref_A_addr(MTPA_Handle handle)
{
  MTPA_Obj *obj = (MTPA_Obj *)handle;

  return(&(obj->Is_ref_A));
} // end of CTRL_getId_A_addr() function


//! \brief     Gets the enable controller flag value from the controller
//! \param[in] handle  The maximum torque per ampere (MTPA) handle
//! \return    The enable MTPA flag value
static inline bool MTPA_getFlagEnable(MTPA_Handle handle)
{
    MTPA_Obj *obj = (MTPA_Obj *)handle;

    return(obj->flagEnable);
} // end of MTPA_getFlagEnable() function


//! \brief     Gets the direct current reference value (Id_ref_A)
//! \param[in] handle  The maximum torque per ampere (MTPA) handle
//! \return    The direct reference reference current, A
static inline float32_t MTPA_getId_ref_A(MTPA_Handle handle)
{
    MTPA_Obj *obj = (MTPA_Obj *)handle;

    return(obj->Idq_ref_A.value[0]);
} // end of MTPA_getId_ref_A() function


//! \brief     Gets the quadrature current reference value (Iq_ref_A)
//! \param[in] handle  The maximum torque per ampere (MTPA) handle
//! \return    The quadrature current reference value, A
static inline float32_t MTPA_getIq_ref_A(MTPA_Handle handle)
{
    MTPA_Obj *obj = (MTPA_Obj *)handle;

    return(obj->Idq_ref_A.value[1]);
} // end of MTPA_getIq_ref_A() function


//! \brief     Gets the stator current reference value (Is_ref_A)
//! \param[in] handle  The maximum torque per ampere (MTPA) handle
//! \return    The stator current reference value, A
static inline float32_t MTPA_getIs_ref_A(MTPA_Handle handle)
{
    MTPA_Obj *obj = (MTPA_Obj *)handle;

    return(obj->Is_ref_A);
} // end of MTPA_getIs_ref_A() function


//! \brief     Sets the enable flag
//! \param[in] handle      The maximum torque per ampere (MTPA) handle
//! \param[in] flagEnable  The enable flag
static inline void
MTPA_setFlagEnable(MTPA_Handle handle, const bool flagEnable)
{
    MTPA_Obj *obj = (MTPA_Obj *)handle;

    obj->flagEnable = flagEnable;

    return;
} // end of MTPA_setFlag_enable() function


//! \brief     Sets the stator current phase angle value
//! \param[in] handle      The maximum torque per ampere (MTPA) handle
//! \param[in] angleCurrent_rad  The stator current phase angle value, rad
static inline void
MTPA_setCurrentAngle_rad(MTPA_Handle handle, const float32_t angleCurrent_rad)
{
    MTPA_Obj *obj = (MTPA_Obj *)handle;

    obj->angleCurrent_rad = angleCurrent_rad;

  return;
} // end of MTPA_setCurrentAngle_rad() function


//! \brief     Sets the direct current (Id_ref) reference vector value
//! \param[in] handle    The maximum torque per ampere (MTPA) handle
//! \param[in] Id_ref_A  The direct current reference value, A
static inline void
MTPA_setId_ref_A(MTPA_Handle handle, const float32_t Id_ref_A)
{
    MTPA_Obj *obj = (MTPA_Obj *)handle;

    obj->Idq_ref_A.value[0] = Id_ref_A;

    return;
} // end of MTPA_setId_ref_A() function

//! \brief     Sets the quadrature current (Iq_ref) reference value
//! \param[in] handle    The maximum torque per ampere (MTPA) handle
//! \param[in] Iq_ref_A  The quadrature current reference value, A
static inline void
MTPA_setIq_ref_A(MTPA_Handle handle, const float32_t Iq_ref_A)
{
  MTPA_Obj *obj = (MTPA_Obj *)handle;

  obj->Idq_ref_A.value[1] = Iq_ref_A;

  return;
} // end of MTPA_setIq_ref_A() function


//! \brief     Sets the stator current reference value (Is_ref)
//! \param[in] handle    The maximum torque per ampere (MTPA) handle
//! \param[in] Is_ref_A  The stator current reference value, A
static inline void
MTPA_setIs_ref_A(MTPA_Handle handle, const float32_t Is_ref_A)
{
    MTPA_Obj *obj = (MTPA_Obj *)handle;

    obj->Is_ref_A = Is_ref_A;

    return;
} // end of MTPA_setIs_ref_A() function

//*****************************************************************************
//
//! \brief     Initializes the maximum torque per ampere (MTPA) module
//!
//! \param[in] pMemory   A pointer to the memory for the MTPA object
//!
//! \param[in] numBytes  The number of bytes allocated for the MTPA object
//!
//! \return The Maximum torque per ampere (MTPA) object handle
//
//*****************************************************************************
extern MTPA_Handle     MTPA_init(void *pMemory, const size_t numBytes);
extern MTPA_Handle cla_MTPA_init(void *pMemory, const size_t numBytes);


//*****************************************************************************
//! \brief     Compute the current reference of MTPA
//! \param[in] handle  The maximum torque per ampere (MTPA) handle
//! \param[in] Is_ref_A  The stator current reference value, A
//! \return    None
//*****************************************************************************
static inline void
MTPA_computeCurrentReference(MTPA_Handle handle, const float32_t Is_ref_A)
{
    MTPA_Obj *obj = (MTPA_Obj *)handle;


    obj->Is_ref_A = MATH_abs(Is_ref_A);

    if(MTPA_getFlagEnable(handle) == true)
    {
        // Perform the Maximum torque per ampere (MTPA)
        if((obj->kconst != 0.0) && (obj->Is_ref_A != 0.0))
        {

          #ifdef __TMS320C28XX_CLA__
            obj->Idq_ref_A.value[0] = obj->kconst -
                                      CLAsqrt((obj->kconst * obj->kconst) -
                                      (0.5 * (obj->Is_ref_A * obj->Is_ref_A)));

            obj->Idq_ref_A.value[1] = CLAsqrt((obj->Is_ref_A * obj->Is_ref_A) -
                            (obj->Idq_ref_A.value[0] * obj->Idq_ref_A.value[0]));
          #else
            obj->Idq_ref_A.value[0] = obj->kconst -
                                 (float32_t)sqrt((obj->kconst * obj->kconst) +
                                      (0.5 * (obj->Is_ref_A * obj->Is_ref_A)));

            obj->Idq_ref_A.value[1] = sqrt((obj->Is_ref_A * obj->Is_ref_A) -
                           (obj->Idq_ref_A.value[0] * obj->Idq_ref_A.value[0]));
          #endif

            if(Is_ref_A < 0.0)
            {
                obj->Idq_ref_A.value[1] = -obj->Idq_ref_A.value[1];
            }
        }
        else
        {
            obj->Idq_ref_A.value[0] = 0.0;
            obj->Idq_ref_A.value[1] = Is_ref_A;
        }
    }
    else
    {
        obj->Idq_ref_A.value[0] = 0.0;
        obj->Idq_ref_A.value[1] = Is_ref_A;
    }

    return;
} // end of MTPA_computeCurrentReference() function


//*****************************************************************************
//! \brief     Compute the current angle of MTPA
//! \param[in] handle  The maximum torque per ampere (MTPA) handle
//! \param[in] Is_ref_A  The stator current reference value, A
//! \return    None
//*****************************************************************************
static inline void
MTPA_computeCurrentAngle(MTPA_Handle handle, const float32_t Is_ref_A)
{
    MTPA_Obj *obj = (MTPA_Obj *)handle;

    obj->Is_ref_A = MATH_abs(Is_ref_A);

    if(MTPA_getFlagEnable(handle) == true)
    {
        // Perform the Maximum torque per ampere (MTPA)
        if((obj->kconst != 0.0) && (obj->Is_ref_A != 0.0))
        {
            obj->gconst = obj->kconst / obj->Is_ref_A;

          #ifdef __TMS320C28XX_CLA__
            obj->angleCurrent_rad = CLAacos(obj->gconst -
                                      CLAsqrt(obj->gconst * obj->gconst + 0.5));
          #else
            obj->angleCurrent_rad = acos(obj->gconst -
                              (float32_t)sqrt(obj->gconst * obj->gconst + 0.5));
          #endif

        }
        else
        {
            obj->angleCurrent_rad = MATH_PI_OVER_TWO;
        }
    }
    else
    {
        obj->angleCurrent_rad = MATH_PI_OVER_TWO;
    }

    return;
} // end of MTPA_computeCurrentAngle() function


//*****************************************************************************
//! \brief     Update the motor inductances
//! \param[in] handle  The maximum torque per ampere (MTPA) handle
//! \param[in] Is_A  The stator current value, A
//! \return    The d-axis inductance
//*****************************************************************************
static inline float32_t
MTPA_updateLs_d_withLUT(MTPA_Handle handle, const float32_t Is_A)
{
    MTPA_Obj *obj = (MTPA_Obj *)handle;

    uint16_t indexLdCnt = 0;

    indexLdCnt = (uint16_t)(Is_A/obj->deltaIs_Ld_A);

    if(indexLdCnt > obj->indexMax_Ld)
    {
        indexLdCnt = obj->indexMax_Ld;
    }

    return(MTPA_Ld_tableData_H[indexLdCnt]);
} // end of MTPA_updateLs_d_withLUT() function

//*****************************************************************************
//! \brief     Update the motor inductances
//! \param[in] handle  The maximum torque per ampere (MTPA) handle
//! \param[in] Is_A  The stator current value, A
//! \return    The q-axis inductance
//*****************************************************************************
static inline float32_t
MTPA_updateLs_q_withLUT(MTPA_Handle handle, const float32_t Is_A)
{
    MTPA_Obj *obj = (MTPA_Obj *)handle;

    uint16_t indexLqCnt = 0;

    indexLqCnt = (uint16_t)(Is_A/obj->deltaIs_Lq_A);

    if(indexLqCnt > obj->indexMax_Lq)
    {
        indexLqCnt = obj->indexMax_Lq;
    }

    return(MTPA_Ld_tableData_H[indexLqCnt]);
} // end of MTPA_updateLs_q_withLUT() function

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

#endif // MTPA_H
