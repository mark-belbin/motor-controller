#ifndef HAL_OBJ_DM_H
#define HAL_OBJ_DM_H
//#############################################################################
// $TI Release: MotorControl SDK v3.00.01.00 $
// $Release Date: Tue May 26 19:13:59 CDT 2020 $
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

//! \file  solutions/boostxl_drv8320rs/f28004x/drivers/include/hal_obj_dm.h
//! \brief Defines the structures for the HAL object 
//!

// drivers
#include "device.h"
#include "driverlib.h"
#include "pwmdac.h"

// modules
#include "hal_data.h"

#include "user_dm.h"

#ifdef DRV8301_SPI
#include "drv8301.h"
#endif

#ifdef DRV8320_SPI
#include "drv8320.h"
#endif
// platforms


//!
//!
//! \defgroup HAL_OBJ HAL_OBJ
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif

//! \brief      Defines the hardware abstraction layer (HAL) data
//! \details    The HAL object contains all handles to peripherals.  When accessing a
//!             peripheral on a processor, use a HAL function along with the HAL handle
//!             for that processor to access its peripherals.
//!
typedef struct _HAL_Obj_
{
  uint32_t      adcHandle[3];       //!< the ADC handles

  uint32_t      adcResult[3];       //!< the ADC results

  uint32_t      claHandle;          //!< the CLA handle

  uint32_t      timerHandle[3];     //<! the timer handles

  uint32_t      sciHandle[2];       //!< the SCI handle

  uint32_t      dmaHandle;          //!< the DMA handle
  uint32_t      dmaChHandle[4];     //!< the DMA Channel handle

  uint32_t      pwmDACHandle[4];    //<! the PWMDAC handles
} HAL_Obj;

//! \brief      Defines the HAL handle
//! \details    The HAL handle is a pointer to a HAL object.  In all HAL functions
//!             the HAL handle is passed so that the function knows what peripherals
//!             are to be accessed.
//!
typedef struct _HAL_Obj_ *HAL_Handle;

//! \brief      Defines the HAL object
//!
extern HAL_Obj      hal;
extern HAL_Handle   halHandle;

typedef struct _HAL_MTR_Obj_
{
  uint32_t       pwmHandle[3];       //<! the PWM handles

  uint32_t       spiHandle;          //!< the SPI handle

  uint32_t       pgaHandle[3];       //!< the PGA handle

  uint32_t       cmpssHandle[3];     //!< the CMPSS handle

  uint32_t       dacHandle;          //!< the DAC handle

  float32_t        current_sf;         //!< the current scale factor, amps/cnt

  float32_t        voltage_sf;         //!< the voltage scale factor, volts/cnt

  float32_t        dcbus_voltage_sf;   //!< the voltage scale factor, volts/cnt

  uint16_t      numCurrentSensors;  //!< the number of current sensors
  uint16_t      numVoltageSensors;  //!< the number of voltage sensors

#ifdef DRV8320_SPI
  DRV8320_Handle drv8320Handle;      //!< the drv8320 interface handle
  DRV8320_Obj    drv8320;            //!< the drv8320 interface object
#endif

#ifdef DRV8301_SPI
  DRV8301_Handle drv8301Handle;      //!< the drv8301 interface handle
  DRV8301_Obj    drv8301;            //!< the drv8301 interface object
#endif

  uint32_t       qepHandle;           //!< the QEP handle

  bool           flagEnablePWM;     //<! the pwm enable flag
} HAL_MTR_Obj;

//! \brief      Defines the HAL_MTR handle
//! \details    The HAL_MTR handle is a pointer to a HAL_MTR object.  In all
//!             HAL_MTR functions, the HAL_MTR handle is passed so that the
//!             function knows what peripherals are to be accessed.
//!
typedef struct _HAL_MTR_Obj_ *HAL_MTR_Handle;


//! \brief      Defines the HAL object
//!
extern HAL_MTR_Obj      halMtr[2];
extern HAL_MTR_Handle   halMtrHandle[2];

#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of HAL_OBJ_H definition

