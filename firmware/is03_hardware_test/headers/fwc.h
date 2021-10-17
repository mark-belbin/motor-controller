//#############################################################################
//
// FILE:   fwc.h
//
// TITLE:  C28x Field Weakening Control (FWC) library (floating point)
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

#ifndef FWC_H
#define FWC_H

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
//! \addtogroup FWC
//! @{
//
//*****************************************************************************

#include "libraries/math/include/math.h"

#ifdef __TMS320C28XX_CLA__
#include "libraries/math/include/CLAmath.h"
#else
#include <math.h>
#endif

#include "pi.h"

//*****************************************************************************
//
//! \brief Defines the Field Weakening Control (FWC) object
//
//*****************************************************************************
typedef struct _FWC_Obj_
{
    PI_Handle piHandle;          //!< the handle for the fwc angle PI controller
    PI_Obj    pi;                //!< the fwc angle PI controller object

    MATH_Vec2 Idq_ref_A;         //!< the d/q-axis current reference
    float32_t Vs_V;              //!< the stator voltage output value
    float32_t Vs_ref_V;          //!< the stator voltage reference value
    float32_t Is_ref_A;          //!< the stator current reference value
    float32_t angleCurrent_rad;  //!< the stator current phase angle

    bool      flagEnable;        //!< a flag to enable the controller
} FWC_Obj;

//*****************************************************************************
//
//! \brief Defines the FWC handle
//
//*****************************************************************************
typedef struct _FWC_Obj_ *FWC_Handle;

//*****************************************************************************
//
// Prototypes for the APIs
//
//*****************************************************************************
//! \brief     Disables the FWC function
//! \param[in] handle  The Field Weakening Control (FWC) handle
static inline void FWC_disable(FWC_Handle handle)
{
    FWC_Obj *obj = (FWC_Obj *)handle;

    obj->flagEnable = false;

    return;
} // end of FWC_disable() function


//! \brief     Enables the FWC function
//! \param[in] handle  The Field Weakening Control (FWC) handle
static inline void FWC_enable(FWC_Handle handle)
{
    FWC_Obj *obj = (FWC_Obj *)handle;

    obj->flagEnable = true;

    return;
} // end of FWC_enable() function


//! \brief     Gets the stator current phase angle memory address
//! \param[in] handle  The Field Weakening Control (FWC) handle
//! \return    The stator current phase angle memory address
static inline float32_t *FWC_getCurrentAngle_rad_addr(FWC_Handle handle)
{
    FWC_Obj *obj = (FWC_Obj *)handle;

    return(&(obj->angleCurrent_rad));
} // end of FWC_getCurrentAngle_rad_addr() function


//! \brief     Gets the stator current reference (Is_ref_A) memory address
//! \param[in] handle  The Field Weakening Control (FWC) handle
//! \return    The stator current reference memory address
static inline float32_t *FWC_getIs_ref_A_addr(FWC_Handle handle)
{
    FWC_Obj *obj = (FWC_Obj *)handle;

    return(&(obj->Is_ref_A));
} // end of FWC_getIs_ref_A_addr() function


//! \brief     Gets the stator voltage (Vs) memory address
//! \param[in] handle  The Field Weakening Control (FWC) handle
//! \return    The stator voltage memory address
static inline float32_t *FWC_getVs_V_addr(FWC_Handle handle)
{
    FWC_Obj *obj = (FWC_Obj *)handle;

    return(&(obj->Vs_V));
} // end of FWC_getVs_V_addr() function


//! \brief     Gets the stator voltage reference (Vs_ref) memory address
//! \param[in] handle  The Field Weakening Control (FWC) handle
//! \return    The stator voltage reference memory address
static inline float32_t *FWC_getVs_ref_V_addr(FWC_Handle handle)
{
    FWC_Obj *obj = (FWC_Obj *)handle;

    return(&(obj->Vs_ref_V));
} // end of FWC_getVs_ref_V_addr() function


//! \brief     Gets the stator current phase angle value (angleCurrent_rad)
//! \param[in] handle  The Field Weakening Control (FWC) handle
//! \return    The stator current phase angle value, rad
static inline float32_t FWC_getCurrentAngle_rad(FWC_Handle handle)
{
    FWC_Obj *obj = (FWC_Obj *)handle;

    return(obj->angleCurrent_rad);
} // end of FWC_getAnglePhase_rad() function


//! \brief     Gets the enable controller flag value from the FWC
//! \param[in] handle  The Field Weakening Control (FWC) handle
//! \return    The enable FWC flag value
static inline bool FWC_getFlagEnable(FWC_Handle handle)
{
    FWC_Obj *obj = (FWC_Obj *)handle;

    return(obj->flagEnable);
} // end of FWC_getFlagEnable() function


//! \brief     Gets the direct current reference value (Id_ref_A)
//! \param[in] handle  The Field Weakening Control (FWC) handle
//! \return    The direct reference reference current, A
static inline float32_t FWC_getId_ref_A(FWC_Handle handle)
{
    FWC_Obj *obj = (FWC_Obj *)handle;

    return(obj->Idq_ref_A.value[0]);
} // end of FWC_getId_ref_A() function


//! \brief     Gets the quadrature current reference value (Iq_ref_A)
//! \param[in] handle  The Field Weakening Control (FWC) handle
//! \return    The quadrature current reference value, A
static inline float32_t FWC_getIq_ref_A(FWC_Handle handle)
{
    FWC_Obj *obj = (FWC_Obj *)handle;

    return(obj->Idq_ref_A.value[1]);
} // end of FWC_getIq_ref_A() function


//! \brief     Gets the stator current reference value (Is_ref_A)
//! \param[in] handle  The Field Weakening Control (FWC) handle
//! \return    The stator current reference value, A
static inline float32_t FWC_getIs_ref_A(FWC_Handle handle)
{
    FWC_Obj *obj = (FWC_Obj *)handle;

    return(obj->Is_ref_A);
} // end of FWC_getIs_ref_A() function


//! \brief     Gets the stator voltage value (Vs_V)
//! \param[in] handle  The Field Weakening Control (FWC) handle
//! \return    The stator voltage value, V
static inline float32_t FWC_getVs_V(FWC_Handle handle)
{
    FWC_Obj *obj = (FWC_Obj *)handle;

    return(obj->Vs_V);
} // end of FWC_getVs_V() function


//! \brief     Gets the stator voltage reference value (Vs_ref_V)
//! \param[in] handle  The Field Weakening Control (FWC) handle
//! \return    The stator current reference value, V
static inline float32_t FWC_getVs_ref_V(FWC_Handle handle)
{
    FWC_Obj *obj = (FWC_Obj *)handle;

    return(obj->Vs_ref_V);
} // end of FWC_getVs_ref_V() function


//! \brief     Sets the stator current phase angle value
//! \param[in] handle      The Field Weakening Control (FWC) handle
//! \param[in] angleCurrent_rad  The stator current phase angle value, rad
static inline void FWC_setCurrentAngle_rad(FWC_Handle handle,
                                         const float32_t angleCurrent_rad)
{
    FWC_Obj *obj = (FWC_Obj *)handle;

    obj->angleCurrent_rad = angleCurrent_rad;

    return;
} // end of FWC_setCurrentAngle_rad() function

//! \brief     Sets the enable flag
//! \param[in] handle      The Field Weakening Control (FWC) handle
//! \param[in] flagEnable  The enable flag
static inline void FWC_setFlagEnable(FWC_Handle handle, const bool flagEnable)
{
    FWC_Obj *obj = (FWC_Obj *)handle;

    obj->flagEnable = flagEnable;

    return;
} // end of FWC_setFlagEnable() function

//! \brief     Sets the direct current (Id_ref) reference vector value
//! \param[in] handle      The Field Weakening Control (FWC) handle
//! \param[in] pIdq_ref_A  The direct current reference value, A
static inline void FWC_setId_ref_A(FWC_Handle handle, const float32_t Id_ref_A)
{
    FWC_Obj *obj = (FWC_Obj *)handle;

    obj->Idq_ref_A.value[0] = Id_ref_A;

    return;
} // end of FWC_setId_ref_A() function

//! \brief     Sets the quadrature current (Iq_ref) reference value
//! \param[in] handle    The Field Weakening Control (FWC) handle
//! \param[in] Iq_ref_A  The quadrature current reference value, A
static inline void FWC_setIq_ref_A(FWC_Handle handle, const float32_t Iq_ref_A)
{
    FWC_Obj *obj = (FWC_Obj *)handle;

    obj->Idq_ref_A.value[1] = Iq_ref_A;

    return;
} // end of FWC_setIq_ref_A() function


//! \brief     Sets the stator current reference value (Is_ref)
//! \param[in] handle    The Field Weakening Control (FWC) handle
//! \param[in] Is_ref_A  The stator current reference value, A
static inline void FWC_setIs_ref_A(FWC_Handle handle, const float32_t Is_ref_A)
{
    FWC_Obj *obj = (FWC_Obj *)handle;

    obj->Is_ref_A = Is_ref_A;

    return;
} // end of FWC_setIs_ref_A() function


//! \brief     Sets the stator voltage reference value (Vs)
//! \param[in] handle  The Field Weakening Control (FWC) handle
//! \param[in] Vs_V    The stator voltage value, V
static inline void FWC_setVs_V(FWC_Handle handle, const float32_t Vs_V)
{
    FWC_Obj *obj = (FWC_Obj *)handle;

    obj->Vs_V = Vs_V;

    return;
} // end of FWC_setIs_ref_A() function


//! \brief     Sets the stator voltage reference value (Vs_ref)
//! \param[in] handle    The Field Weakening Control (FWC) handle
//! \param[in] Vs_ref_V  The stator voltage reference value, V
static inline void FWC_setVs_ref_V(FWC_Handle handle, const float32_t Vs_ref_V)
{
    FWC_Obj *obj = (FWC_Obj *)handle;

    obj->Vs_ref_V = Vs_ref_V;

    return;
} // end of FWC_setIs_ref_A() function


//! \brief     Sets the integral gain (Ki) value
//! \param[in] handle    The Field Weakening Control (FWC) handle
//! \param[in] Ki        The Ki value
static inline void FWC_setKi(FWC_Handle handle, const float32_t Ki)
{
    FWC_Obj *obj = (FWC_Obj *)handle;

    PI_setKi(obj->piHandle, Ki);

    return;
} // end of FWC_setKi() function


//! \brief     Sets the proportional gain (Kp) value
//! \param[in] handle    The Field Weakening Control (FWC) handle
//! \param[in] Kp        The Kp value
static inline void FWC_setKp(FWC_Handle handle, const float32_t Kp)
{
    FWC_Obj *obj = (FWC_Obj *)handle;

    PI_setKp(obj->piHandle, Kp);

    return;
} // end of FWC_setKp() function

//! \brief     Sets the gain values
//! \param[in] handle    The Field Weakening Control (FWC) handle
//! \param[in] Kp        The Kp gain value
//! \param[in] Ki        The Ki gain value
static inline void FWC_setGains(FWC_Handle handle,
                                const float32_t Kp, const float32_t Ki)
{
    FWC_Obj *obj = (FWC_Obj *)handle;

    PI_setKp(obj->piHandle, Kp);
    PI_setKi(obj->piHandle, Ki);

    return;
} // end of FWC_setGains() function


//! \brief     Sets the minimum and maximum output values of FWC
//! \param[in] handle        The Field Weakening Control (FWC) handle
//! \param[in] angleMin_rad  The angle out minimum value
//! \param[in] angleMax_rad  The angle out maximum value
static inline void FWC_setAngleMinMax(FWC_Handle handle,
                                      const float32_t angleMin_rad,
                                      const float32_t angleMax_rad)
{
    FWC_Obj *obj = (FWC_Obj *)handle;

    // set the minimum and maximum values of PI controller
    PI_setMinMax(obj->piHandle, angleMax_rad, angleMin_rad);

    return;
} // end of FWC_setAngleMinMax() function


//! \brief     Sets the maximum output values of FWC
//! \param[in] handle        The Field Weakening Control (FWC) handle
//! \param[in] angleMax_rad  The angle out maximum value
static inline void FWC_setAngleMax(FWC_Handle handle,
                                   const float32_t angleMax_rad)
{
    FWC_Obj *obj = (FWC_Obj *)handle;

    // set the minimum values of PI controller
    // convert the maximum angle to negative value in PI controller
    // the maximum angle = MATH_PI_OVER_TWO - minimum value of PI output
    PI_setOutMin(obj->piHandle, angleMax_rad);

    return;
} // end of FWC_setAngleMax() function

//! \brief     Resets the integrator start value (Ui)
//! \param[in] handle    The Field Weakening Control (FWC) handle
static inline void FWC_resetUi(FWC_Handle handle)
{
    FWC_Obj *obj = (FWC_Obj *)handle;

    // Sets the integrator start value in the PI controller
    // reset the new integrator value to Zero
    PI_setUi(obj->piHandle, 0.0);

    return;
} // end of FWC_setKi() function

//! \brief     Sets the integrator start value (Ui)
//! \param[in] handle    The Field Weakening Control (FWC) handle
//! \param[in] Ui        The integrator start value
static inline void FWC_setUi(FWC_Handle handle, const float32_t Ui)
{
    FWC_Obj *obj = (FWC_Obj *)handle;

    // set the new integrator value
    PI_setUi(obj->piHandle, Ui);

    return;
} // end of FWC_setKi() function


//! \brief     Initializes the Field Weakening Control (FWC) module
//! \param[in] pMemory   A pointer to the memory for the FWC object
//! \param[in] numBytes  The number of bytes allocated for the FWC object
//! \return The Field Weakening Control (FWC) object handle
extern FWC_Handle     FWC_init(void *pMemory, const size_t numBytes);
extern FWC_Handle cla_FWC_init(void *pMemory, const size_t numBytes);


//! \brief     Sets the Field Weakening Control (FWC) module parmaeters
//! \param[in] handle        The Field Weakening Control (FWC) handle
//! \param[in] Kp            The Kp gain value
//! \param[in] Ki            The Ki gain value
//! \param[in] angleMin_rad  The angle out minimum value
//! \param[in] angleMax_rad  The angle out maximum value
extern void FWC_setParams(FWC_Handle handle,
                          const float32_t Kp, const float32_t Ki,
                          const float32_t angleMin_rad,
                          const float32_t angleMax_rad);


//! \brief     Runs the Field Weakening Control (FWC)
//! \param[in] handle   The Field Weakening Control (FWC) handle
//! \param[in] Is_ref_A The reference vector current, A
//! \return    None
static inline void FWC_computeCurrentReference(FWC_Handle handle,
                                               const float32_t Is_ref_A)
{
    FWC_Obj *obj = (FWC_Obj *)handle;
    MATH_Vec2 phasor;

    // Sets Is reference, A
    FWC_setIs_ref_A(handle, Is_ref_A);

    // compute the sin/cos phasor using fast RTS function, callable assembly
#ifdef __TMS320C28XX_CLA__
    phasor.value[0] = CLAcos_inline(obj->angleCurrent_rad);
    phasor.value[1] = CLAsin_inline(obj->angleCurrent_rad);
#else
    phasor.value[0] = cosf(obj->angleCurrent_rad);
    phasor.value[1] = sinf(obj->angleCurrent_rad);
#endif

    obj->Idq_ref_A.value[0] =  obj->Is_ref_A * phasor.value[0];
    obj->Idq_ref_A.value[1] =  obj->Is_ref_A * phasor.value[1];

    return;
} // end of FWC_computeCurrentReference() function


//! \brief     Runs the Field Weakening Control (FWC)
//! \param[in] handle   The Field Weakening Control (FWC) handle
//! \param[in] Vs_V   The output voltage vector, V
//! \param[in] VsRef_V  The reference voltage vector, V
//! \return    None
static inline void FWC_computeCurrentAngle(FWC_Handle handle,
                            const float32_t Vs_V, const float32_t VsRef_V)
{
    FWC_Obj *obj = (FWC_Obj *)handle;
    float32_t angle_rad = 0.0;

    FWC_setVs_V(handle, Vs_V);
    FWC_setVs_ref_V(handle, VsRef_V);

    if(FWC_getFlagEnable(handle) == true)
    {
        // Perform the Field Weakening Control (FWC)

        PI_run_series(obj->piHandle,
                      obj->Vs_ref_V,
                      obj->Vs_V,
                      0.0,
                      &angle_rad);

        obj->angleCurrent_rad = MATH_PI_OVER_TWO - angle_rad;
    }
    else
    {
        obj->angleCurrent_rad = MATH_PI_OVER_TWO;
    }

    return;
} // end of FWC_computeCurrentAngle() function

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

#endif // FWC_H
