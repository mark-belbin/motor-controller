//#############################################################################
//
// FILE:   svgen_current.h
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

#ifndef SVGEN_CURRENT_H
#define SVGEN_CURRENT_H

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
#ifdef __TMS320C28XX_CLA__
#include "libraries/math/include/CLAmath.h"
#else
#include <math.h>
#endif // __TMS320C28XX_CLA__

#include "libraries/math/include/math.h"

// **************************************************************************
// the typedefs

typedef enum
{
    SVGENCURRENT_USE_ALL = 0,     //!< Use all shunt measurements
    SVGENCURRENT_IGNORE_A,        //!< Ignore the A phase shunt measurement
    SVGENCURRENT_IGNORE_B,        //!< Ignore the B phase shunt measurement
    SVGENCURRENT_IGNORE_C,        //!< Ignore the C phase shunt measurement
    SVGENCURRENT_IGNORE_AB,       //!< Ignore the AB phase shunt measurement
    SVGENCURRENT_IGNORE_AC,       //!< Ignore the AC phase shunt measurement
    SVGENCURRENT_IGNORE_BC,       //!< Ignore the BC phase shunt measurement
    SVGENCURRENT_IGNORE_ALL       //!< Ignore the ABC phase shunt measurement
} SVGENCURRENT_IgnoreShunt_e;

typedef enum
{
    SVGENCURRENT_ALL_PHASE_MEASURABLE = 1,  //!< all shunt measurable
    SVGENCURRENT_TWO_PHASE_MEASURABLE,      //!< just two shunt measurable
    SVGENCURRENT_ONE_PHASE_MEASURABLE,      //!< just one shunt measurable
    SVGENCURRENT_IMMEASUREABLE
} SVGENCURRENT_MeasureShunt_e;

typedef enum
{
    SVGENCURRENT_VMID_A=0,        //!< Middle voltage is A phase
    SVGENCURRENT_VMID_B,          //!< Middle voltage is B phase
    SVGENCURRENT_VMID_C           //!< Middle voltage is C phase
} SVGENCURRENT_VmidShunt_e;

//! \brief Defines the Svgen Current object
//!
typedef struct _SVGENCURRENT_Obj_
{
  int16_t                       minWidth;    //!< The maximum width where a valid measurement cannot be taken
  SVGENCURRENT_IgnoreShunt_e    ignoreShunt; //!< Output of what shunt or shunts to ignore
  SVGENCURRENT_MeasureShunt_e   compMode;    //!< Output phase compensation mode
  SVGENCURRENT_VmidShunt_e      Vmid;        //!< The middle amplitude voltage among the three phase voltages
  float32_t                       Vlimit;      //!< The maximum output voltage duty that current can be sampled
} SVGENCURRENT_Obj;


//! \brief Defines the Svgen Current handle
//!
typedef struct _SVGENCURRENT_Obj_ *SVGENCURRENT_Handle;


// **************************************************************************
// the function prototypes


//! \brief     Initializes the svgen current object
//! \param[in] *pMemory         Pointer in to the svgen current object
//! \param[in] numBytes         Size of the object
extern SVGENCURRENT_Handle SVGENCURRENT_init(void *pMemory,const size_t numBytes);


//! \brief     Sets the minimum Duty Cycle width that the lower switch can be on before
//! \brief     the current data is invalid.
//! \param[in] svgencurrentHandle           The Svgen Current handle
//! \param[in] minwidth                     Integer value of the minimum number of pwm counts
static inline void
SVGENCURRENT_setMinWidth(SVGENCURRENT_Handle svgencurrentHandle,
                         const int16_t minwidth)
{
  SVGENCURRENT_Obj *svgencurrent = (SVGENCURRENT_Obj *)svgencurrentHandle;

  svgencurrent->minWidth = minwidth;

  return;
} // end of SVGENCURRENT_setMinWidth() function


//! \brief     Sets the ignore shunt value
//! \param[in] svgencurrentHandle  The Svgen Current handle
//! \param[in] ignoreShunt         The ignore shunt value
static inline void
SVGENCURRENT_setIgnoreShunt(SVGENCURRENT_Handle svgencurrentHandle,
                            const SVGENCURRENT_IgnoreShunt_e ignoreShunt)
{
  SVGENCURRENT_Obj *svgencurrent = (SVGENCURRENT_Obj *)svgencurrentHandle;

  svgencurrent->ignoreShunt = ignoreShunt;

  return;
} // end of SVGENCURRENT_setIgnoreShunt() function


//! \brief     Sets the compensation mode
//! \param[in] svgencurrentHandle  The Svgen Current handle
//! \param[in] CompMode         CompMode
static inline void
SVGENCURRENT_setMode(SVGENCURRENT_Handle svgencurrentHandle,
                     const SVGENCURRENT_MeasureShunt_e compMode)
{
  SVGENCURRENT_Obj *svgencurrent = (SVGENCURRENT_Obj *)svgencurrentHandle;

  svgencurrent->compMode = compMode;

  return;
} // end of SVGENCURRENT_setMode() function


//! \brief     Sets the output voltage limit value for gurrantee a current sampling
//! \param[in] svgencurrentHandle  The Svgen Current handle
//! \param[in] Vlimit         Vlimit
static inline void
SVGENCURRENT_setVlimit(SVGENCURRENT_Handle svgencurrentHandle,
                       const float32_t Vlimit)
{
  SVGENCURRENT_Obj *svgencurrent = (SVGENCURRENT_Obj *)svgencurrentHandle;

  svgencurrent->Vlimit = Vlimit;

  return;
} // end of SVGENCURRENT_setVlimit() function


//! \brief     Gets the ignore shunt value
//! \param[in] svgencurrentHandle  The Svgen Current handle
//! \return    Ignore shunt value
static inline SVGENCURRENT_IgnoreShunt_e
SVGENCURRENT_getIgnoreShunt(SVGENCURRENT_Handle svgencurrentHandle)
{
  SVGENCURRENT_Obj *svgencurrent = (SVGENCURRENT_Obj *)svgencurrentHandle;

  return(svgencurrent->ignoreShunt);
} // end of SVGENCURRENT_getIgnoreShunt() function


//! \brief     Gets the minimum Duty Cycle width that the lower switch can be on before
//! \brief     the current data is invalid.
//! \param[in] svgencurrentHandle           The Svgen Current handle
//! \return    Integer value of the minimum number of pwm counts
static inline int16_t
SVGENCURRENT_getMinWidth(SVGENCURRENT_Handle svgencurrentHandle)
{
  SVGENCURRENT_Obj *svgencurrent = (SVGENCURRENT_Obj *)svgencurrentHandle;

  return(svgencurrent->minWidth);
} // end of SVGENCURRENT_getMinWidth() function


//! \brief     Gets the Voltage(Duty) Limit value
//! \param[in] svgencurrentHandle  The Svgen Current handle
//! \return    Integer value of the voltage(duty) limit
static inline float32_t
SVGENCURRENT_getVlimit(SVGENCURRENT_Handle svgencurrentHandle)
{
  SVGENCURRENT_Obj *svgencurrent = (SVGENCURRENT_Obj *)svgencurrentHandle;

  return(svgencurrent->Vlimit);

} // end of SVGENCURRENT_getVlimit() function


//! \brief     Gets the current reconstruction mode
//! \param[in] svgencurrentHandle  The Svgen Current handle
//! \return    CompMode
static inline SVGENCURRENT_MeasureShunt_e
SVGENCURRENT_getMode(SVGENCURRENT_Handle svgencurrentHandle)
{
  SVGENCURRENT_Obj *svgencurrent = (SVGENCURRENT_Obj *)svgencurrentHandle;

  return(svgencurrent->compMode);

} // end of SVGENCURRENT_getMode() function


//! \brief     Gets the middle amplitude voltage
//! \param[in] svgencurrentHandle  The Svgen Current handle
//! \return    middle voltage
static inline SVGENCURRENT_VmidShunt_e
SVGENCURRENT_getVmid(SVGENCURRENT_Handle svgencurrentHandle)
{
  SVGENCURRENT_Obj *svgencurrent = (SVGENCURRENT_Obj *)svgencurrentHandle;

  return(svgencurrent->Vmid);

} // end of SVGENCURRENT_getVmiddle() function


//! \brief     Gets the svgen current module ignore shunt
//! \brief     In the pwm structure, the value variable is the on-time of the low fet.
//! \brief     A low value is a small on-time for the low switch of the bridge and thus a short current window.
//! \param[in] svgencurrentHandle  The Svgen Current handle
//! \param[in] cmp1                compare value 1
//! \param[in] cmp2                compare value 2
//! \param[in] cmp3                compare value 3
//! \param[in] cmpM1               active compare value 1, from mirror register
//! \param[in] cmpM2               active compare value 2, from mirror register
//! \param[in] cmpM3               active compare value 3, from mirror register
static inline void
SVGENCURRENT_RunIgnoreShunt(SVGENCURRENT_Handle svgencurrentHandle,
                            uint16_t cmp1, uint16_t cmp2, uint16_t cmp3,
                            uint16_t cmpM1, uint16_t cmpM2, uint16_t cmpM3)
{
  SVGENCURRENT_Obj *svgencurrent = (SVGENCURRENT_Obj *)svgencurrentHandle;
  uint16_t minWidth;

  minWidth = svgencurrent->minWidth;

  uint16_t nextPulse1 = (cmp1 + cmpM1)>>1;
  uint16_t nextPulse2 = (cmp2 + cmpM2)>>1;
  uint16_t nextPulse3 = (cmp3 + cmpM3)>>1;

  if(nextPulse1 < minWidth)
    {
      if((nextPulse2 < minWidth) || ((cmp2 - cmp1) < minWidth))
        {
          svgencurrent->ignoreShunt = SVGENCURRENT_IGNORE_AB;
        }
      else if((nextPulse3 < minWidth) || ((cmp3 - cmp1) < minWidth))
        {
          svgencurrent->ignoreShunt = SVGENCURRENT_IGNORE_AC;
        }
      else
        {
          svgencurrent->ignoreShunt = SVGENCURRENT_IGNORE_A;
        }
    }
  else if(nextPulse2 < minWidth)
    {
      if((nextPulse1 < minWidth) || ((cmp1 - cmp2) < minWidth))
        {
          svgencurrent->ignoreShunt = SVGENCURRENT_IGNORE_AB;
        }
      else if((nextPulse3 < minWidth) || ((cmp3 - cmp2) < minWidth))
        {
          svgencurrent->ignoreShunt = SVGENCURRENT_IGNORE_BC;
        }
      else
        {
          svgencurrent->ignoreShunt = SVGENCURRENT_IGNORE_B;
        }
    }
  else if(nextPulse3 < minWidth)
    {
      if((nextPulse1 < minWidth) || ((cmp1 - cmp3) < minWidth))
        {
          svgencurrent->ignoreShunt = SVGENCURRENT_IGNORE_AC;
        }
      else if((nextPulse2 < minWidth) || ((cmp2 - cmp3) < minWidth))
        {
          svgencurrent->ignoreShunt = SVGENCURRENT_IGNORE_BC;
        }
      else
        {
          svgencurrent->ignoreShunt = SVGENCURRENT_IGNORE_C;
        }
    }
  else
    {
      svgencurrent->ignoreShunt = SVGENCURRENT_USE_ALL;
    }

  return;

} // end of SVGENCURRENT_RunIgnoreShunt() function


//! \brief     Reconstructs the missed measured currents due to a small sampling window
//! \param[in] svgencurrentHandle         The svgen current handle
//! \param[in] pADCData                   Pointer to the shunt currents
static inline void
SVGENCURRENT_RunRegenCurrent(SVGENCURRENT_Handle svgencurrentHandle,
                             MATH_Vec3 *pADCData, MATH_Vec3 *pADCDataPrev)
{
  SVGENCURRENT_Obj *svgencurrent = (SVGENCURRENT_Obj *)svgencurrentHandle;

  float32_t Ia = pADCData->value[0];
  float32_t Ib = pADCData->value[1];
  float32_t Ic = pADCData->value[2];

  // select valid shunts and ignore one when needed
  if (svgencurrent->ignoreShunt == SVGENCURRENT_IGNORE_A)
  {
      // repair a based on b and c
      Ia = -Ib - Ic;       //Ia = -Ib - Ic;
  }
  else if (svgencurrent->ignoreShunt == SVGENCURRENT_IGNORE_B)
  {
      // repair b based on a and c
      Ib = -Ia - Ic;       //Ib = -Ia - Ic;
  }
  else if (svgencurrent->ignoreShunt == SVGENCURRENT_IGNORE_C)
  {
      // repair c based on a and b
      Ic = -Ia - Ib;       //Ic = -Ia - Ib;
  }

  if(svgencurrent->compMode > SVGENCURRENT_TWO_PHASE_MEASURABLE)
  {
      pADCData->value[0] = (Ia + pADCDataPrev->value[0])*0.5;
      pADCData->value[1] = (Ib + pADCDataPrev->value[1])*0.5;
      pADCData->value[2] = (Ic + pADCDataPrev->value[2])*0.5;
  }
  else
  {
      pADCData->value[0] = Ia;
      pADCData->value[1] = Ib;
      pADCData->value[2] = Ic;
  }

  pADCDataPrev->value[0] = pADCData->value[0];
  pADCDataPrev->value[1] = pADCData->value[1];
  pADCDataPrev->value[2] = pADCData->value[2];

  return;
} // end of SVGENCURRENT_RunRegenCurrent() function


//! \brief     output voltage reconsturction to guarantee min duty in two phase at least
//! \param[in] svgencurrentHandle         The svgen current handle
//! \param[in] pPWMData                   The pointer of the PWM data
//! \param[in] pPWMData_old                   The  pointer of old PWM data
static inline void
SVGENCURRENT_compPWMData(SVGENCURRENT_Handle svgencurrentHandle,
                         MATH_Vec3 *pPWMData, MATH_Vec3 *pPWMData_prev)
{
    SVGENCURRENT_Obj *svgencurrent = (SVGENCURRENT_Obj *)svgencurrentHandle;

    float32_t Va_avg = (pPWMData->value[0] + pPWMData_prev->value[0]) * 0.5;
    float32_t Vb_avg = (pPWMData->value[1] + pPWMData_prev->value[1]) * 0.5;
    float32_t Vc_avg = (pPWMData->value[2] + pPWMData_prev->value[2]) * 0.5;

    float32_t Vlimit;
    float32_t Vmiddle, Vmiddle_prev;
    float32_t Voffset;

    Vlimit = svgencurrent->Vlimit;

    //define compensation mode
    if(Va_avg > Vlimit)
    {
        if(Vb_avg > Vlimit)
        {
            svgencurrent->compMode = SVGENCURRENT_ONE_PHASE_MEASURABLE;

            if(Va_avg > Vb_avg)
            {
                Vmiddle = pPWMData->value[1];
                Vmiddle_prev = pPWMData_prev->value[1];
                svgencurrent->Vmid = SVGENCURRENT_VMID_B;
            }
            else
            {
                Vmiddle = pPWMData->value[0];
                Vmiddle_prev = pPWMData_prev->value[0];
                svgencurrent->Vmid = SVGENCURRENT_VMID_A;
            }
        }
        else if(Vc_avg > Vlimit)
        {
            svgencurrent->compMode = SVGENCURRENT_ONE_PHASE_MEASURABLE;

            if(Va_avg > Vc_avg)
            {
                Vmiddle = pPWMData->value[2];
                Vmiddle_prev = pPWMData_prev->value[2];
                svgencurrent->Vmid = SVGENCURRENT_VMID_C;
            }
            else
            {
                Vmiddle = pPWMData->value[0];
                Vmiddle_prev = pPWMData_prev->value[0];
                svgencurrent->Vmid = SVGENCURRENT_VMID_A;
            }
        }
        else
        {
            svgencurrent->compMode = SVGENCURRENT_TWO_PHASE_MEASURABLE;

            if(Vb_avg > Vc_avg)
            {
                svgencurrent->Vmid = SVGENCURRENT_VMID_B;
            }
            else
            {
                svgencurrent->Vmid = SVGENCURRENT_VMID_C;
            }
        }

    }
    else
    {
        if(Vb_avg > Vlimit)
        {
            if(Vc_avg > Vlimit)
            {
                svgencurrent->compMode = SVGENCURRENT_ONE_PHASE_MEASURABLE;

                if(Vb_avg > Vc_avg)
                {
                    Vmiddle = pPWMData->value[2];
                    Vmiddle_prev = pPWMData_prev->value[2];
                    svgencurrent->Vmid = SVGENCURRENT_VMID_C;
                }
                else
                {
                    Vmiddle = pPWMData->value[1];
                    Vmiddle_prev = pPWMData_prev->value[1];
                    svgencurrent->Vmid = SVGENCURRENT_VMID_B;
                }
            }
            else
            {
                svgencurrent->compMode = SVGENCURRENT_TWO_PHASE_MEASURABLE;

                if(Va_avg > Vc_avg)
                {
                    svgencurrent->Vmid = SVGENCURRENT_VMID_A;
                }
                else
                {
                    svgencurrent->Vmid = SVGENCURRENT_VMID_C;
                }
            }
        }
        else
        {
            if(Vc_avg > Vlimit)
            {
                svgencurrent->compMode = SVGENCURRENT_TWO_PHASE_MEASURABLE;

                if(Va_avg > Vb_avg)
                {
                    svgencurrent->Vmid = SVGENCURRENT_VMID_A;
                }
                else
                {
                    svgencurrent->Vmid = SVGENCURRENT_VMID_B;
                }
            }
            else
            {
                svgencurrent->compMode = SVGENCURRENT_ALL_PHASE_MEASURABLE;
            }
        }
    }

    //phase voltage compensator
    if(svgencurrent->compMode == SVGENCURRENT_ONE_PHASE_MEASURABLE)
    {
        Voffset = (Vmiddle + Vmiddle_prev) - (Vlimit*2.0);

        pPWMData->value[0] -= Voffset;
        pPWMData->value[1] -= Voffset;
        pPWMData->value[2] -= Voffset;
    }

    // get ignore current
    if(((pPWMData->value[0] + pPWMData_prev->value[0])*0.5) > Vlimit)
    {
        svgencurrent->ignoreShunt = SVGENCURRENT_IGNORE_A;
    }
    else if(((pPWMData->value[1] + pPWMData_prev->value[1])*0.5) > Vlimit)
    {
        svgencurrent->ignoreShunt = SVGENCURRENT_IGNORE_B;
    }
    else if(((pPWMData->value[2] + pPWMData_prev->value[2])*0.5) > Vlimit)
    {
        svgencurrent->ignoreShunt = SVGENCURRENT_IGNORE_C;
    }
    else
    {
        svgencurrent->ignoreShunt = SVGENCURRENT_USE_ALL;
    }

    pPWMData_prev->value[0] = pPWMData->value[0];
    pPWMData_prev->value[1] = pPWMData->value[1];
    pPWMData_prev->value[2] = pPWMData->value[2];

    return;
} // end of SVGENCURRENT_compPWMData() function

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

#ifdef __cplusplus
}
#endif // extern "C"

#endif // end of SVGEN_CURRENT_H definition
