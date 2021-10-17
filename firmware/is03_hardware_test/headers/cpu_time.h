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

#ifndef CPU_TIME_H
#define CPU_TIME_H

//! \file   libraries/utilities/cpu_time/include/cpu_time.h
//! \brief  Contains the public interface to the 
//!         CPU_time (CPU_time) module routines
//!


// **************************************************************************
// the includes
#include <math.h>

#include "types.h"


//!
//! \defgroup CPU_TIME CPU_TIME

//!
//! \ingroup CPU_TIME
//! @{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines


// **************************************************************************
// the typedefs

//! \brief Defines the CPU usage (CPU_TIME) object
//!
typedef struct _CPU_TIME_Obj_
{
  uint16_t          pwm_period; 	           //!< the number of pwm period

  uint32_t          timer_cnt_now;              //!< the current timer count value, cnts
  uint32_t          timer_cnt_prev;             //!< the previous timer count value, cnts

  uint32_t          timer_delta_now;          	//!< the latest delta count value, cnts
  uint32_t          timer_delta_prev;         	//!< the latest delta count value, cnts

  uint32_t          timer_delta_min;   		 	//!< the minimum delta counts observed, cnts
  uint32_t          timer_delta_max; 	  	 	//!< the maximum delta counts observed, cnts
  uint32_t          timer_delta_avg; 		 	//!< the average delta counts observed, cnts
  uint32_t          timer_band_max; 		 	//!< the maximum bandwidthelta counts observed, cnts

  uint32_t          timer_delta_CntAcc;       	//!< the accumulated delta count values, cnts
  uint32_t          timer_delta_AccNum;       	//!< the number of accumulated delta count values, num
  
  bool              flag_resetStatus;       	//!< a flag to reset all measured data
} CPU_TIME_Obj;


//! \brief Defines the CPU_TIME handle
//!
typedef struct _CPU_TIME_Obj_ *CPU_TIME_Handle;


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes

//! \brief     Initializes the CPU TIME (CPU_TIME) object
//! \param[in] pMemory    A pointer to the base address of the object
//! \param[in] numBytes   The object size, bytes
//! \return    The handle to the object
extern CPU_TIME_Handle CPU_TIME_init(void *pMemory,const size_t numBytes);

//! \brief     Sets the state of the reset stats flag
//! \param[in] handle  The CPU TIME (CPU_TIME) handle
//! \param[in] state   The desired state
static inline void CPU_TIME_setFlag_resetStats(CPU_TIME_Handle handle,const bool state)
{
  CPU_TIME_Obj *obj = (CPU_TIME_Obj *)handle;

  obj->flag_resetStatus = state;

  return;
} // end of CPU_TIME_setFlag_resetStats() function


//! \brief     Sets the CPU TIME module parameters
//! \param[in] handle             The CPU TIME (CPU_TIME) handle
//! \param[in] pwm_period   	  The pwm period
void CPU_TIME_setParams(CPU_TIME_Handle handle, const uint16_t pwm_period);


//! \brief     Runs the CPU TIME module
//! \param[in] handle  The CPU TIME (CPU_TIME) handle
static inline void CPU_TIME_setBandMax(CPU_TIME_Handle handle, const uint32_t band_max)
{
  CPU_TIME_Obj *obj = (CPU_TIME_Obj *)handle;

  obj->timer_band_max = band_max;
}
//! \brief     Runs the CPU TIME module
//! \param[in] handle  The CPU TIME (CPU_TIME) handle
static inline void CPU_TIME_run(CPU_TIME_Handle handle, const uint32_t cnt)
{
  CPU_TIME_Obj *obj = (CPU_TIME_Obj *)handle;

  obj->timer_cnt_prev = obj->timer_cnt_now;
  obj->timer_cnt_now = cnt;

  obj->timer_delta_now = obj->timer_cnt_prev - obj->timer_cnt_now;
  
  if(obj->timer_delta_now > obj->timer_delta_max)
	  obj->timer_delta_max = obj->timer_delta_now;

  if(obj->timer_delta_now < obj->timer_delta_min)
      obj->timer_delta_min = obj->timer_delta_now;

//  if(obj->timer_delta_CntAcc == 256)
//  {
//	  obj->timer_delta_CntAcc = 0;
//	  obj->timer_delta_avg = obj->timer_delta_AccNum>>8;
//	  obj->timer_delta_AccNum = obj->timer_delta_now;
//  }
//  else
//  {
//	  obj->timer_delta_CntAcc++;
//	  obj->timer_delta_AccNum += obj->timer_delta_now;
//  }

  return;
} // end of FEM_run() function


//! \brief     Updates the current and previous count values
//! \param[in] handle    The CPU TIME (CPU_TIME) handle
//! \param[in] timerCnt  The current count value
static inline void CPU_TIME_updateCnts(CPU_TIME_Handle handle, const uint32_t cnt)
{
  CPU_TIME_Obj *obj = (CPU_TIME_Obj *)handle;

  obj->timer_cnt_now = cnt;

  if(obj->flag_resetStatus == true)
  {
	  obj->flag_resetStatus = false;
	  obj->timer_delta_max = 0;
	  obj->timer_delta_min = 0xFFFFFFFF;

//	  obj->timer_delta_CntAcc = 0;
//	  obj->timer_delta_AccNum = 0;
  }

  return;
} // end of CPU_TIME_updateCnts() function


#ifdef __cplusplus
}
#endif // extern "C"

//! @}  // ingroup

#endif // end of CPU_TIME_H definition


