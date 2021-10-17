/* --COPYRIGHT--,BSD
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//! \file   drivers/drvic/drv8305/src/32b/f28x/f2802x/drv8305.c
//! \brief  Contains the various functions related to the DRV8305 object
//!
//! (C) Copyright 2015, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include <math.h>


// **************************************************************************
// drivers
#include "TI_ESC_V1/drv8305.h"


// **************************************************************************
// modules


// **************************************************************************
// platforms


// **************************************************************************
// the defines


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes


DRV8305_Handle DRV8305_init(void *pMemory,const size_t numBytes)
{
  DRV8305_Handle handle;

  if(numBytes < sizeof(DRV8305_Obj))
    return((DRV8305_Handle)NULL);

  // assign the handle
  handle = (DRV8305_Handle)pMemory;

  DRV8305_resetRxTimeout(handle);
  DRV8305_resetEnableTimeout(handle);

  return(handle);
} // end of DRV8305_init() function


void DRV8305_enable(DRV8305_Handle handle)
{
  DRV8305_Obj *obj = (DRV8305_Obj *)handle;
  static volatile uint16_t enableWaitTimeOut;
  uint16_t n = 0;

  // Enable the DRV8305
  GPIO_setHigh(obj->gpioHandle,obj->gpioNumber);

  enableWaitTimeOut = 0;

  // Make sure the FAULT bit is not set during startup
  while(((DRV8305_readSpi(handle, Address_Status_1) & DRV8305_STATUS01_FAULT_BITS) != 0) && (enableWaitTimeOut < 1000))
  {
      if (++enableWaitTimeOut > 999)
      {
          obj->enableTimeOut = true;
      }
  }

  // Wait for the DRV8305 to go through start up sequence
  for(n=0;n<0xffff;n++)
    asm(" NOP");

  return;
} // end of DRV8305_enable() function


DRV8305_CTRL05_PeakSourCurHS_e DRV8305_getPeakSourCurHS(DRV8305_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_5);

  // mask the bits
  data &= DRV8305_CTRL05_IDRIVEP_HS_BITS;

  return((DRV8305_CTRL05_PeakSourCurHS_e)data);
} // end of DRV8305_getPeakSourCurHS function


DRV8305_CTRL05_PeakSinkCurHS_e DRV8305_getPeakSinkCurHS(DRV8305_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_5);

  // mask the bits
  data &= DRV8305_CTRL05_IDRIVEN_HS_BITS;

  return((DRV8305_CTRL05_PeakSinkCurHS_e)data);
} // end of DRV8305_getPeakSinkCurHS function


DRV8305_CTRL05_PeakSourTime_e DRV8305_getPeakSourTime(DRV8305_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_5);

  // mask the bits
  data &= DRV8305_CTRL05_TDRIVEN_BITS;

  return((DRV8305_CTRL05_PeakSourTime_e)data);
} // end of DRV8305_getPeakSourTime function


DRV8305_CTRL06_PeakSourCurLS_e DRV8305_getPeakSourCurLS(DRV8305_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_6);

  // mask the bits
  data &= DRV8305_CTRL06_IDRIVEP_LS_BITS;

  return((DRV8305_CTRL06_PeakSourCurLS_e)data);
} // end of DRV8305_getPeakSourCurLS function


DRV8305_CTRL06_PeakSinkCurLS_e DRV8305_getPeakSinkCurLS(DRV8305_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_6);

  // mask the bits
  data &= DRV8305_CTRL06_IDRIVEN_LS_BITS;

  return((DRV8305_CTRL06_PeakSinkCurLS_e)data);
} // end of DRV8305_getPeakSinkCurLS function


DRV8305_CTRL06_PeakSinkTime_e DRV8305_getPeakSinkTime(DRV8305_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_6);

  // mask the bits
  data &= DRV8305_CTRL06_TDRIVEP_BITS;

  return((DRV8305_CTRL06_PeakSinkTime_e)data);
} // end of DRV8305_getPeakSinkTime function


DRV8305_CTRL07_VDSDeglitch_e DRV8305_getVDSDeglitch(DRV8305_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_7);

  // mask the bits
  data &= DRV8305_CTRL07_TVDS_BITS;

  return((DRV8305_CTRL07_VDSDeglitch_e)data);
} // end of DRV8305_getVDSDeglitch function


DRV8305_CTRL07_VDSBlanking_e DRV8305_getVDSBlanking(DRV8305_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_7);

  // mask the bits
  data &= DRV8305_CTRL07_TBLANK_BITS;

  return((DRV8305_CTRL07_VDSBlanking_e)data);
} // end of DRV8305_getVDSBlanking function


DRV8305_CTRL07_DeadTime_e DRV8305_getDeadTime(DRV8305_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_7);

  // mask the bits
  data &= DRV8305_CTRL07_DEAD_TIME_BITS;

  return((DRV8305_CTRL07_DeadTime_e)data);
} // end of DRV8305_getDeadTime function


DRV8305_CTRL07_PwmMode_e DRV8305_getPwmMode(DRV8305_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_7);

  // mask the bits
  data &= DRV8305_CTRL07_PWM_MODE_BITS;

  return((DRV8305_CTRL07_PwmMode_e)data);
} // end of DRV8305_getPwmMode function


DRV8305_CTRL07_CommOption_e DRV8305_getCommOption(DRV8305_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_7);

  // mask the bits
  data &= DRV8305_CTRL07_COMM_OPT_BITS;

  return((DRV8305_CTRL07_CommOption_e)data);
} // end of DRV8305_getCommOption function


DRV8305_CTRL09_WatchDelay_e DRV8305_getWatchDelay(DRV8305_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_9);

  // mask the bits
  data &= DRV8305_CTRL09_WD_DLY_BITS;

  return((DRV8305_CTRL09_WatchDelay_e)data);
} // end of DRV8305_getWatchDelay function


DRV8305_CTRL0A_CSGain1_e DRV8305_getCSGain1(DRV8305_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_A);

  // mask the bits
  data &= DRV8305_CTRL0A_GAIN_CS1_BITS;

  return((DRV8305_CTRL0A_CSGain1_e)data);
} // end of DRV8305_getCSGain1 function


DRV8305_CTRL0A_CSGain2_e DRV8305_getCSGain2(DRV8305_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_A);

  // mask the bits
  data &= DRV8305_CTRL0A_GAIN_CS2_BITS;

  return((DRV8305_CTRL0A_CSGain2_e)data);
} // end of DRV8305_getCSGain2 function


DRV8305_CTRL0A_CSGain3_e DRV8305_getCSGain3(DRV8305_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_A);

  // mask the bits
  data &= DRV8305_CTRL0A_GAIN_CS3_BITS;

  return((DRV8305_CTRL0A_CSGain3_e)data);
} // end of DRV8305_getCSGain3 function


DRV8305_CTRL0A_CSBlank_e DRV8305_getCSBlank(DRV8305_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_A);

  // mask the bits
  data &= DRV8305_CTRL0A_CS_BLANK_BITS;

  return((DRV8305_CTRL0A_CSBlank_e)data);
} // end of DRV8305_getCSBlank function


DRV8305_CTRL0A_DcCalMode_e DRV8305_getDcCalMode(DRV8305_Handle handle,const DRV8305_ShuntAmpNumber_e ampNumber)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_A);

  // mask the bits
  if(ampNumber == ShuntAmpNumber_1)
    {
      data &= DRV8305_CTRL0A_DC_CAL_CH1_BITS;
    }
  else if(ampNumber == ShuntAmpNumber_2)
    {
      data &= DRV8305_CTRL0A_DC_CAL_CH2_BITS;
    }
  else if(ampNumber == ShuntAmpNumber_3)
    {
      data &= DRV8305_CTRL0A_DC_CAL_CH3_BITS;
    }
    
  return((DRV8305_CTRL0A_DcCalMode_e)data);
} // end of DRV8305_getDcCalMode() function


DRV8305_CTRL0B_VregUvLevel_e DRV8305_getVregUvLevel(DRV8305_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_B);

  // mask the bits
  data &= DRV8305_CTRL0B_VREG_UV_LEVEL_BITS;

  return((DRV8305_CTRL0B_VregUvLevel_e)data);
} // end of DRV8305_getVregUvLevel function


DRV8305_CTRL0B_SleepDelay_e DRV8305_getSleepDelay(DRV8305_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_B);

  // mask the bits
  data &= DRV8305_CTRL0B_SLP_DLY_BITS;

  return((DRV8305_CTRL0B_SleepDelay_e)data);
} // end of DRV8305_getSleepDelay function


DRV8305_CTRL0B_VrefScaling_e DRV8305_getVrefScaling(DRV8305_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_B);

  // mask the bits
  data &= DRV8305_CTRL0B_VREF_SCALING_BITS;

  return((DRV8305_CTRL0B_VrefScaling_e)data);
} // end of DRV8305_getVrefScaling function


DRV8305_CTRL0C_VDSMode_e DRV8305_getVDSMode(DRV8305_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_C);

  // mask the bits
  data &= DRV8305_CTRL0C_VDS_MODE_BITS;

  return((DRV8305_CTRL0C_VDSMode_e)data);
} // end of DRV8305_getVDSMode function


DRV8305_CTRL0C_VDSLevel_e DRV8305_getVDSLevel(DRV8305_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_C);

  // mask the bits
  data &= DRV8305_CTRL0C_VDS_LEVEL_BITS;

  return((DRV8305_CTRL0C_VDSLevel_e)data);
} // end of DRV8305_getVDSLevel function


void DRV8305_setSpiHandle(DRV8305_Handle handle,SPI_Handle spiHandle)
{
  DRV8305_Obj *obj = (DRV8305_Obj *)handle;

  // initialize the serial peripheral interface object
  obj->spiHandle = spiHandle;

  return;
} // end of DRV8305_setSpiHandle() function


void DRV8305_setGpioHandle(DRV8305_Handle handle,GPIO_Handle gpioHandle)
{
  DRV8305_Obj *obj = (DRV8305_Obj *)handle;

  // initialize the gpio interface object
  obj->gpioHandle = gpioHandle;

  return;
} // end of DRV8305_setGpioHandle() function


void DRV8305_setGpioNumber(DRV8305_Handle handle,GPIO_Number_e gpioNumber)
{
  DRV8305_Obj *obj = (DRV8305_Obj *)handle;

  // initialize the gpio interface object
  obj->gpioNumber = gpioNumber;

  return;
} // end of DRV8305_setGpioNumber() function


void DRV8305_setPeakSourCurHS(DRV8305_Handle handle,const DRV8305_CTRL05_PeakSourCurHS_e level)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_5);

  // clear the bits
  data &= (~DRV8305_CTRL05_IDRIVEP_HS_BITS);

  // set the bits
  data |= level;

  // write the data
  DRV8305_writeSpi(handle,Address_Control_5,data);

  return;
} // end of DRV8305_setPeakSourCurHS() function


void DRV8305_setPeakSinkCurHS(DRV8305_Handle handle,const DRV8305_CTRL05_PeakSinkCurHS_e level)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_5);

  // clear the bits
  data &= (~DRV8305_CTRL05_IDRIVEN_HS_BITS);

  // set the bits
  data |= level;

  // write the data
  DRV8305_writeSpi(handle,Address_Control_5,data);

  return;
} // end of DRV8305_setPeakSinkCurHS() function


void DRV8305_setPeakSourTime(DRV8305_Handle handle,const DRV8305_CTRL05_PeakSourTime_e time)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_5);

  // clear the bits
  data &= (~DRV8305_CTRL05_TDRIVEN_BITS);

  // set the bits
  data |= time;

  // write the data
  DRV8305_writeSpi(handle,Address_Control_5,data);

  return;
} // end of DRV8305_setPeakSourTime() function


void DRV8305_setPeakSinkCurLS(DRV8305_Handle handle,const DRV8305_CTRL06_PeakSinkCurLS_e level)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_6);

  // clear the bits
  data &= (~DRV8305_CTRL06_IDRIVEP_LS_BITS);

  // set the bits
  data |= level;

  // write the data
  DRV8305_writeSpi(handle,Address_Control_6,data);

  return;
} // end of DRV8305_setPeakSinkCurLS() function


void DRV8305_setPeakSinkTime(DRV8305_Handle handle,const DRV8305_CTRL06_PeakSinkTime_e time)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_6);

  // clear the bits
  data &= (~DRV8305_CTRL06_TDRIVEP_BITS);

  // set the bits
  data |= time;

  // write the data
  DRV8305_writeSpi(handle,Address_Control_6,data);

  return;
} // end of DRV8305_setPeakSinkTime() function


void DRV8305_setVDSDeglitch(DRV8305_Handle handle,const DRV8305_CTRL07_VDSDeglitch_e time)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_7);

  // clear the bits
  data &= (~DRV8305_CTRL07_TVDS_BITS);

  // set the bits
  data |= time;

  // write the data
  DRV8305_writeSpi(handle,Address_Control_7,data);

  return;
} // end of DRV8305_setVDSDeglitch() function


void DRV8305_setVDSBlanking(DRV8305_Handle handle,const DRV8305_CTRL07_VDSBlanking_e time)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_7);

  // clear the bits
  data &= (~DRV8305_CTRL07_TBLANK_BITS);

  // set the bits
  data |= time;

  // write the data
  DRV8305_writeSpi(handle,Address_Control_7,data);

  return;
} // end of DRV8305_setVDSBlanking() function


void DRV8305_setDeadTime(DRV8305_Handle handle,const DRV8305_CTRL07_DeadTime_e time)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_7);

  // clear the bits
  data &= (~DRV8305_CTRL07_DEAD_TIME_BITS);

  // set the bits
  data |= time;

  // write the data
  DRV8305_writeSpi(handle,Address_Control_7,data);

  return;
} // end of DRV8305_setDeadTime() function


void DRV8305_setPwmMode(DRV8305_Handle handle,const DRV8305_CTRL07_PwmMode_e mode)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_7);

  // clear the bits
  data &= (~DRV8305_CTRL07_PWM_MODE_BITS);

  // set the bits
  data |= mode;

  // write the data
  DRV8305_writeSpi(handle,Address_Control_7,data);

  return;
} // end of DRV8305_setPwmMode() function


void DRV8305_setCommOption(DRV8305_Handle handle,const DRV8305_CTRL07_CommOption_e mode)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_7);

  // clear the bits
  data &= (~DRV8305_CTRL07_COMM_OPT_BITS);

  // set the bits
  data |= mode;

  // write the data
  DRV8305_writeSpi(handle,Address_Control_7,data);

  return;
} // end of DRV8305_setCommOption() function


void DRV8305_setWatchDelay(DRV8305_Handle handle,const DRV8305_CTRL09_WatchDelay_e time)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_9);

  // clear the bits
  data &= (~DRV8305_CTRL09_WD_DLY_BITS);

  // set the bits
  data |= time;

  // write the data
  DRV8305_writeSpi(handle,Address_Control_9,data);

  return;
} // end of DRV8305_setWatchDelay() function


void DRV8305_setCSGain1(DRV8305_Handle handle,const DRV8305_CTRL0A_CSGain1_e gain)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_A);

  // clear the bits
  data &= (~DRV8305_CTRL0A_GAIN_CS1_BITS);

  // set the bits
  data |= gain;

  // write the data
  DRV8305_writeSpi(handle,Address_Control_A,data);

  return;
} // end of DRV8305_setCSGain1() function


void DRV8305_setCSGain2(DRV8305_Handle handle,const DRV8305_CTRL0A_CSGain2_e gain)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_A);

  // clear the bits
  data &= (~DRV8305_CTRL0A_GAIN_CS2_BITS);

  // set the bits
  data |= gain;

  // write the data
  DRV8305_writeSpi(handle,Address_Control_A,data);

  return;
} // end of DRV8305_setCSGain2() function


void DRV8305_setCSGain3(DRV8305_Handle handle,const DRV8305_CTRL0A_CSGain3_e gain)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_A);

  // clear the bits
  data &= (~DRV8305_CTRL0A_GAIN_CS3_BITS);

  // set the bits
  data |= gain;

  // write the data
  DRV8305_writeSpi(handle,Address_Control_A,data);

  return;
} // end of DRV8305_setCSGain3() function


void DRV8305_setCSBlank(DRV8305_Handle handle,const DRV8305_CTRL0A_CSBlank_e time)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_A);

  // clear the bits
  data &= (~DRV8305_CTRL0A_CS_BLANK_BITS);

  // set the bits
  data |= time;

  // write the data
  DRV8305_writeSpi(handle,Address_Control_A,data);

  return;
} // end of DRV8305_setCSBlank() function


void DRV8305_setDcCalMode(DRV8305_Handle handle,const DRV8305_ShuntAmpNumber_e ampNumber,const DRV8305_CTRL0A_DcCalMode_e mode)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_A);

  // clear the bits
  if(ampNumber == ShuntAmpNumber_1)
    {
      data &= (~DRV8305_CTRL0A_DC_CAL_CH1_BITS);
    }
  else if(ampNumber == ShuntAmpNumber_2)
    {
      data &= (~DRV8305_CTRL0A_DC_CAL_CH2_BITS);
    }
  else if(ampNumber == ShuntAmpNumber_3)
    {
      data &= (~DRV8305_CTRL0A_DC_CAL_CH3_BITS);
    }
    
  // set the bits
  data |= mode;

  // write the data
  DRV8305_writeSpi(handle,Address_Control_A,data);

  return;
} // end of DRV8305_setDcCalMode() function


void DRV8305_setVregUvLevel(DRV8305_Handle handle,const DRV8305_CTRL0B_VregUvLevel_e level)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_B);

  // clear the bits
  data &= (~DRV8305_CTRL0B_VREG_UV_LEVEL_BITS);

  // set the bits
  data |= level;

  // write the data
  DRV8305_writeSpi(handle,Address_Control_B,data);

  return;
} // end of DRV8305_setVregUvLevel() function


void DRV8305_setSleepDelay(DRV8305_Handle handle,const DRV8305_CTRL0B_SleepDelay_e time)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_B);

  // clear the bits
  data &= (~DRV8305_CTRL0B_SLP_DLY_BITS);

  // set the bits
  data |= time;

  // write the data
  DRV8305_writeSpi(handle,Address_Control_B,data);

  return;
} // end of DRV8305_setSleepDelay() function


void DRV8305_setVrefScaling(DRV8305_Handle handle,const DRV8305_CTRL0B_VrefScaling_e mode)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_B);

  // clear the bits
  data &= (~DRV8305_CTRL0B_VREF_SCALING_BITS);

  // set the bits
  data |= mode;

  // write the data
  DRV8305_writeSpi(handle,Address_Control_B,data);

  return;
} // end of DRV8305_setVrefScaling() function


void DRV8305_setVDSMode(DRV8305_Handle handle,const DRV8305_CTRL0C_VDSMode_e mode)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_C);

  // clear the bits
  data &= (~DRV8305_CTRL0C_VDS_MODE_BITS);

  // set the bits
  data |= mode;

  // write the data
  DRV8305_writeSpi(handle,Address_Control_C,data);

  return;
} // end of DRV8305_setVDSMode() function


void DRV8305_setVDSLevel(DRV8305_Handle handle,const DRV8305_CTRL0C_VDSLevel_e level)
{
  uint16_t data;

  // read data
  data = DRV8305_readSpi(handle,Address_Control_C);

  // clear the bits
  data &= (~DRV8305_CTRL0C_VDS_LEVEL_BITS);

  // set the bits
  data |= level;

  // write the data
  DRV8305_writeSpi(handle,Address_Control_C,data);

  return;
} // end of DRV8305_setVDSLevel() function


bool DRV8305_isFault(DRV8305_Handle handle)
{
  DRV8305_Word_t readWord;
  bool status=false;


  // read the data
  readWord = DRV8305_readSpi(handle,Address_Status_1);

  if(readWord & DRV8305_STATUS01_FAULT_BITS)
    {
      status = true;
    }

  return(status);
} // end of DRV8305_isFault() function


void DRV8305_reset(DRV8305_Handle handle)
{
  uint16_t data;


  // read data
  data = DRV8305_readSpi(handle,Address_Control_9);

  // set the bits
  data |= DRV8305_CTRL09_CLR_FLTS_BITS;

  // write the data
  DRV8305_writeSpi(handle,Address_Control_9,data);

  return;
}  // end of DRV8305_reset() function


void DRV8305_setupSpi(DRV8305_Handle handle, DRV_SPI_8305_Vars_t *Spi_8305_Vars)
{
  DRV8305_Address_e drvRegAddr;
  uint16_t drvDataNew;
  
  // Set Default Values
  // Manual Read/Write
  Spi_8305_Vars->ManReadAddr  = 0;
  Spi_8305_Vars->ManReadData  = 0;
  Spi_8305_Vars->ManReadCmd = false;
  Spi_8305_Vars->ManWriteAddr = 0;
  Spi_8305_Vars->ManWriteData = 0;
  Spi_8305_Vars->ManWriteCmd = false;

  // Read/Write
  Spi_8305_Vars->ReadCmd  = false;
  Spi_8305_Vars->WriteCmd = false;

  // Read registers for default values
  // Read Status Register 1
  drvRegAddr = Address_Status_1;
  drvDataNew = DRV8305_readSpi(handle,drvRegAddr);
  Spi_8305_Vars->Stat_Reg_01.OTW         = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS01_OTW_BITS)?1:0;
  Spi_8305_Vars->Stat_Reg_01.TEMP_FLAG3  = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS01_TEMP_FLAG3_BITS)?1:0;
  Spi_8305_Vars->Stat_Reg_01.TEMP_FLAG2  = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS01_TEMP_FLAG2_BITS)?1:0;
  Spi_8305_Vars->Stat_Reg_01.TEMP_FLAG1  = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS01_TEMP_FLAG1_BITS)?1:0;
  Spi_8305_Vars->Stat_Reg_01.VCPH_UVFL   = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS01_VCPH_UVFL_BITS)?1:0;
  Spi_8305_Vars->Stat_Reg_01.VDS_STATUS  = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS01_VDS_STATUS_BITS)?1:0;
  Spi_8305_Vars->Stat_Reg_01.PVDD_OVFL   = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS01_PVDD_OVFL_BITS)?1:0;
  Spi_8305_Vars->Stat_Reg_01.PVDD_UVFL   = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS01_PVDD_UVFL_BITS)?1:0;
  Spi_8305_Vars->Stat_Reg_01.TEMP_FLAG4  = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS01_TEMP_FLAG4_BITS)?1:0;
  Spi_8305_Vars->Stat_Reg_01.STAT01_RSV1 = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS01_RESERVED1_BITS)?1:0;
  Spi_8305_Vars->Stat_Reg_01.FAULT       = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS01_FAULT_BITS)?1:0;

  // Read Status Register 2
  drvRegAddr = Address_Status_2;
  drvDataNew = DRV8305_readSpi(handle,drvRegAddr);
  Spi_8305_Vars->Stat_Reg_02.SNS_A_OCP   = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS02_SNS_A_OCP_BITS)?1:0;
  Spi_8305_Vars->Stat_Reg_02.SNS_B_OCP   = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS02_SNS_B_OCP_BITS)?1:0;
  Spi_8305_Vars->Stat_Reg_02.SNS_C_OCP   = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS02_SNS_C_OCP_BITS)?1:0;
  Spi_8305_Vars->Stat_Reg_02.STAT02_RSV1 = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS02_RESERVED1_BITS)?1:0;
  Spi_8305_Vars->Stat_Reg_02.STAT02_RSV2 = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS02_RESERVED2_BITS)?1:0;
  Spi_8305_Vars->Stat_Reg_02.FETLC_VDS   = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS02_FETLC_VDS_BITS)?1:0;
  Spi_8305_Vars->Stat_Reg_02.FETHC_VDS   = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS02_FETHC_VDS_BITS)?1:0;
  Spi_8305_Vars->Stat_Reg_02.FETLB_VDS   = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS02_FETLB_VDS_BITS)?1:0;
  Spi_8305_Vars->Stat_Reg_02.FETHB_VDS   = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS02_FETHB_VDS_BITS)?1:0;
  Spi_8305_Vars->Stat_Reg_02.FETLA_VDS   = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS02_FETLA_VDS_BITS)?1:0;
  Spi_8305_Vars->Stat_Reg_02.FETHA_VDS   = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS02_FETHA_VDS_BITS)?1:0;

  // Read Status Register 3
  drvRegAddr = Address_Status_3;
  drvDataNew = DRV8305_readSpi(handle,drvRegAddr);
  Spi_8305_Vars->Stat_Reg_03.VCPH_OVLO_ABS = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS03_VCPH_OVLO_ABS_BITS)?1:0;
  Spi_8305_Vars->Stat_Reg_03.VCPH_OVLO     = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS03_VCPH_OVLO_BITS)?1:0;
  Spi_8305_Vars->Stat_Reg_03.VCPH_UVLO2    = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS03_VCPH_UVLO2_BITS)?1:0;
  Spi_8305_Vars->Stat_Reg_03.STAT03_RSV1   = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS03_RESERVED1_BITS)?1:0;
  Spi_8305_Vars->Stat_Reg_03.VCP_LSD_UVLO2 = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS03_VCP_LSD_UVLO2_BITS)?1:0;
  Spi_8305_Vars->Stat_Reg_03.AVDD_UVLO     = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS03_AVDD_UVLO_BITS)?1:0;
  Spi_8305_Vars->Stat_Reg_03.VREG_UV       = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS03_VREG_UV_BITS)?1:0;
  Spi_8305_Vars->Stat_Reg_03.STAT03_RSV2   = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS03_RESERVED2_BITS)?1:0;
  Spi_8305_Vars->Stat_Reg_03.OTS           = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS03_OTS_BITS)?1:0;
  Spi_8305_Vars->Stat_Reg_03.WD_FAULT      = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS03_WD_FAULT_BITS)?1:0;
  Spi_8305_Vars->Stat_Reg_03.PVDD_UVLO2    = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS03_PVDD_UVLO2_BITS)?1:0;

  // Read Status Register 4
  drvRegAddr = Address_Status_4;
  drvDataNew = DRV8305_readSpi(handle,drvRegAddr);
  Spi_8305_Vars->Stat_Reg_04.STAT04_RSV1 = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS04_RESERVED1_BITS)?1:0;
  Spi_8305_Vars->Stat_Reg_04.STAT04_RSV2 = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS04_RESERVED2_BITS)?1:0;
  Spi_8305_Vars->Stat_Reg_04.STAT04_RSV3 = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS04_RESERVED3_BITS)?1:0;
  Spi_8305_Vars->Stat_Reg_04.STAT04_RSV4 = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS04_RESERVED4_BITS)?1:0;
  Spi_8305_Vars->Stat_Reg_04.STAT04_RSV5 = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS04_RESERVED5_BITS)?1:0;
  Spi_8305_Vars->Stat_Reg_04.FETLC_VGS   = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS04_FETLC_VGS_BITS)?1:0;
  Spi_8305_Vars->Stat_Reg_04.FETHC_VGS   = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS04_FETHC_VGS_BITS)?1:0;
  Spi_8305_Vars->Stat_Reg_04.FETLB_VGS   = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS04_FETLB_VGS_BITS)?1:0;
  Spi_8305_Vars->Stat_Reg_04.FETHB_VGS   = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS04_FETHB_VGS_BITS)?1:0;
  Spi_8305_Vars->Stat_Reg_04.FETLA_VGS   = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS04_FETLA_VGS_BITS)?1:0;
  Spi_8305_Vars->Stat_Reg_04.FETHA_VGS   = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS04_FETHA_VGS_BITS)?1:0;
  
    // Read Control Register 5
  drvRegAddr = Address_Control_5;
  drvDataNew = DRV8305_readSpi(handle,drvRegAddr);
  Spi_8305_Vars->Ctrl_Reg_05.IDRIVEP_HS  = (DRV8305_CTRL05_PeakSourCurHS_e)(drvDataNew & (uint16_t)DRV8305_CTRL05_IDRIVEP_HS_BITS);
  Spi_8305_Vars->Ctrl_Reg_05.IDRIVEN_HS  = (DRV8305_CTRL05_PeakSinkCurHS_e)(drvDataNew & (uint16_t)DRV8305_CTRL05_IDRIVEN_HS_BITS);
  Spi_8305_Vars->Ctrl_Reg_05.TDRIVEN     = (DRV8305_CTRL05_PeakSourTime_e)(drvDataNew & (uint16_t)DRV8305_CTRL05_TDRIVEN_BITS);
  Spi_8305_Vars->Ctrl_Reg_05.CTRL05_RSV1 = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL05_RESERVED1_BITS)?1:0;

  // Read Control Register 6
  drvRegAddr = Address_Control_6;
  drvDataNew = DRV8305_readSpi(handle,drvRegAddr);
  Spi_8305_Vars->Ctrl_Reg_06.IDRIVEP_LS  = (DRV8305_CTRL06_PeakSourCurLS_e)(drvDataNew & (uint16_t)DRV8305_CTRL06_IDRIVEP_LS_BITS);
  Spi_8305_Vars->Ctrl_Reg_06.IDRIVEN_LS  = (DRV8305_CTRL06_PeakSinkCurLS_e)(drvDataNew & (uint16_t)DRV8305_CTRL06_IDRIVEN_LS_BITS);
  Spi_8305_Vars->Ctrl_Reg_06.TDRIVEP     = (DRV8305_CTRL06_PeakSinkTime_e)(drvDataNew & (uint16_t)DRV8305_CTRL06_TDRIVEP_BITS);
  Spi_8305_Vars->Ctrl_Reg_06.CTRL06_RSV1 = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL05_RESERVED1_BITS)?1:0;

  // Read Control Register 7
  drvRegAddr = Address_Control_7;
  drvDataNew = DRV8305_readSpi(handle,drvRegAddr);
  Spi_8305_Vars->Ctrl_Reg_07.TVDS        = (DRV8305_CTRL07_VDSDeglitch_e)(drvDataNew & (uint16_t)DRV8305_CTRL07_TVDS_BITS);
  Spi_8305_Vars->Ctrl_Reg_07.TBLANK      = (DRV8305_CTRL07_VDSBlanking_e)(drvDataNew & (uint16_t)DRV8305_CTRL07_TBLANK_BITS);
  Spi_8305_Vars->Ctrl_Reg_07.DEAD_TIME   = (DRV8305_CTRL07_DeadTime_e)(drvDataNew & (uint16_t)DRV8305_CTRL07_DEAD_TIME_BITS);
  Spi_8305_Vars->Ctrl_Reg_07.PWM_MODE    = (DRV8305_CTRL07_PwmMode_e)(drvDataNew & (uint16_t)DRV8305_CTRL07_PWM_MODE_BITS);
  Spi_8305_Vars->Ctrl_Reg_07.COMM_OPT    = (DRV8305_CTRL07_CommOption_e)(drvDataNew & (uint16_t)DRV8305_CTRL07_COMM_OPT_BITS);
  Spi_8305_Vars->Ctrl_Reg_07.CTRL07_RSV1 = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL07_RESERVED1_BITS)?1:0;

  // Read Control Register 8
  //drvRegAddr = Address_Control_8;
  //drvDataNew = DRV8305_readSpi(handle,drvRegAddr);
  //Spi_8305_Vars->Ctrl_Reg_08.CTRL08_RSV1 = ( uint16_t)(drvDataNew & (uint16_t)DRV8305_CTRL08_RESERVED_BITS);

  // Read Control Register 9
  drvRegAddr = Address_Control_9;
  drvDataNew = DRV8305_readSpi(handle,drvRegAddr);
  Spi_8305_Vars->Ctrl_Reg_09.SET_VCPH_UV    = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL09_SET_VCPH_UV_BITS)?1:0;
  Spi_8305_Vars->Ctrl_Reg_09.CLR_FLTS       = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL09_CLR_FLTS_BITS)?1:0;
  Spi_8305_Vars->Ctrl_Reg_09.SLEEP          = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL09_SLEEP_BITS)?1:0;
  Spi_8305_Vars->Ctrl_Reg_09.WD_EN          = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL09_WD_EN_BITS)?1:0;
  Spi_8305_Vars->Ctrl_Reg_09.DIS_SNS_OCP    = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL09_DIS_SNS_OCP_BITS)?1:0;
  Spi_8305_Vars->Ctrl_Reg_09.WD_DLY         = (DRV8305_CTRL09_WatchDelay_e)(drvDataNew & (uint16_t)DRV8305_CTRL09_WD_DLY_BITS);
  Spi_8305_Vars->Ctrl_Reg_09.EN_SNS_CLAMP   = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL09_EN_SNS_CLAMP_BITS)?1:0;
  Spi_8305_Vars->Ctrl_Reg_09.DIS_GDRV_FAULT = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL09_DIS_GDRV_FAULT_BITS)?1:0;
  Spi_8305_Vars->Ctrl_Reg_09.DISABLE        = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL09_DISABLE_BITS)?1:0;
  Spi_8305_Vars->Ctrl_Reg_09.FLIP_OTS       = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL09_FLIP_OTS_BITS)?1:0;

  // Read Control Register A
  drvRegAddr = Address_Control_A;
  drvDataNew = DRV8305_readSpi(handle,drvRegAddr);
  Spi_8305_Vars->Ctrl_Reg_0A.GAIN_CS1   = (DRV8305_CTRL0A_CSGain1_e)(drvDataNew & (uint16_t)DRV8305_CTRL0A_GAIN_CS1_BITS);
  Spi_8305_Vars->Ctrl_Reg_0A.GAIN_CS2   = (DRV8305_CTRL0A_CSGain2_e)(drvDataNew & (uint16_t)DRV8305_CTRL0A_GAIN_CS2_BITS);
  Spi_8305_Vars->Ctrl_Reg_0A.GAIN_CS3   = (DRV8305_CTRL0A_CSGain3_e)(drvDataNew & (uint16_t)DRV8305_CTRL0A_GAIN_CS3_BITS);
  Spi_8305_Vars->Ctrl_Reg_0A.CS_BLANK   = (DRV8305_CTRL0A_CSBlank_e)(drvDataNew & (uint16_t)DRV8305_CTRL0A_CS_BLANK_BITS);
  Spi_8305_Vars->Ctrl_Reg_0A.DC_CAL_CH1 = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL0A_DC_CAL_CH1_BITS)?1:0;
  Spi_8305_Vars->Ctrl_Reg_0A.DC_CAL_CH2 = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL0A_DC_CAL_CH2_BITS)?1:0;
  Spi_8305_Vars->Ctrl_Reg_0A.DC_CAL_CH3 = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL0A_DC_CAL_CH3_BITS)?1:0;

  // Read Control Register B
  drvRegAddr = Address_Control_B;
  drvDataNew = DRV8305_readSpi(handle,drvRegAddr);
  Spi_8305_Vars->Ctrl_Reg_0B.VREG_UV_LEVEL = (DRV8305_CTRL0B_VregUvLevel_e)(drvDataNew & (uint16_t)DRV8305_CTRL0B_VREG_UV_LEVEL_BITS);
  Spi_8305_Vars->Ctrl_Reg_0B.DIS_PWRGD     = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL0B_DIS_PWRGD_BITS)?1:0;
  Spi_8305_Vars->Ctrl_Reg_0B.SLP_DLY       = (DRV8305_CTRL0B_SleepDelay_e)(drvDataNew & (uint16_t)DRV8305_CTRL0B_SLP_DLY_BITS);
  Spi_8305_Vars->Ctrl_Reg_0B.CTRL0B_RSV1   = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL0B_RESERVED1_BITS)?1:0;
  Spi_8305_Vars->Ctrl_Reg_0B.CTRL0B_RSV2   = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL0B_RESERVED2_BITS)?1:0;
  Spi_8305_Vars->Ctrl_Reg_0B.CTRL0B_RSV3   = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL0B_RESERVED3_BITS)?1:0;
  Spi_8305_Vars->Ctrl_Reg_0B.VREF_SCALING  = (DRV8305_CTRL0B_VrefScaling_e)(drvDataNew & (uint16_t)DRV8305_CTRL0B_VREF_SCALING_BITS);
  Spi_8305_Vars->Ctrl_Reg_0B.CTRL0B_RSV4   = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL0B_RESERVED4_BITS)?1:0;

  // Read Control Register C
  drvRegAddr = Address_Control_C;
  drvDataNew = DRV8305_readSpi(handle,drvRegAddr);
  Spi_8305_Vars->Ctrl_Reg_0C.VDS_MODE    = (DRV8305_CTRL0C_VDSMode_e)(drvDataNew & (uint16_t)DRV8305_CTRL0C_VDS_MODE_BITS);
  Spi_8305_Vars->Ctrl_Reg_0C.VDS_LEVEL   = (DRV8305_CTRL0C_VDSLevel_e)(drvDataNew & (uint16_t)DRV8305_CTRL0C_VDS_LEVEL_BITS);
  Spi_8305_Vars->Ctrl_Reg_0C.CTRL0C_RSV1 = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL0C_RESERVED1_BITS)?1:0;
  Spi_8305_Vars->Ctrl_Reg_0C.CTRL0C_RSV2 = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL0C_RESERVED2_BITS)?1:0;
  Spi_8305_Vars->Ctrl_Reg_0C.CTRL0C_RSV3 = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL0C_RESERVED3_BITS)?1:0;

  return;
} // end of DRV8305_setupSpi() function


uint16_t DRV8305_readSpi(DRV8305_Handle handle,const DRV8305_Address_e regAddr)
{
  DRV8305_Obj *obj = (DRV8305_Obj *)handle;
  uint16_t ctrlWord;
  const uint16_t data = 0;
  volatile uint16_t readWord;
  static volatile uint16_t WaitTimeOut = 0;
  volatile SPI_FifoStatus_e RxFifoCnt = SPI_FifoStatus_Empty;

  //Set CS to Low
  GPIO_setLow(obj->gpioHandle,GPIO_Number_19);

  // build the control word
  ctrlWord = (uint16_t)DRV8305_buildCtrlWord(CtrlMode_Read,regAddr,data);

  // reset the Rx fifo pointer to zero
  SPI_resetRxFifo(obj->spiHandle);
  SPI_enableRxFifo(obj->spiHandle);

  // write the command
  SPI_write(obj->spiHandle,ctrlWord);

  // wait for the response to populate the RX fifo, else a wait timeout will occur
  while((RxFifoCnt < SPI_FifoStatus_1_Word) && (WaitTimeOut < 0xffff))
  {
    RxFifoCnt = SPI_getRxFifoStatus(obj->spiHandle);
	if(++WaitTimeOut > 0xfffe)
	{
		obj->RxTimeOut = true;
	}
  }

  // Read the word
  readWord = SPI_readEmu(obj->spiHandle);

  //Set CS to High
  GPIO_setHigh(obj->gpioHandle,GPIO_Number_19);

  return(readWord & DRV8305_DATA_MASK);
} // end of DRV8305_readSpi() function


void DRV8305_writeSpi(DRV8305_Handle handle,const DRV8305_Address_e regAddr,const uint16_t data)
{
  DRV8305_Obj *obj = (DRV8305_Obj *)handle;
  uint16_t ctrlWord;
  uint16_t n;

  //Set CS to Low
  GPIO_setLow(obj->gpioHandle,GPIO_Number_19);

  // build the control word
  ctrlWord = (uint16_t)DRV8305_buildCtrlWord(CtrlMode_Write,regAddr,data);

  // reset the Rx fifo pointer to zero
  SPI_resetRxFifo(obj->spiHandle);
  SPI_enableRxFifo(obj->spiHandle);

  // write the command
  SPI_write(obj->spiHandle,ctrlWord);
  
  // wait for registers to update
  for(n=0;n<0xf;n++)
    asm(" NOP");

  //Set CS to High
  GPIO_setHigh(obj->gpioHandle,GPIO_Number_19);

  return;
}  // end of DRV8305_writeSpi() function


void DRV8305_writeData(DRV8305_Handle handle, DRV_SPI_8305_Vars_t *Spi_8305_Vars)
{
  DRV8305_Address_e drvRegAddr;
  uint16_t drvDataNew;

  if(Spi_8305_Vars->WriteCmd)
  {
    // Write Control Register 5
    drvRegAddr = Address_Control_5;
    drvDataNew = (Spi_8305_Vars->Ctrl_Reg_05.IDRIVEP_HS) | \
    		     (Spi_8305_Vars->Ctrl_Reg_05.IDRIVEN_HS) | \
    		     (Spi_8305_Vars->Ctrl_Reg_05.TDRIVEN)    | \
    		     (Spi_8305_Vars->Ctrl_Reg_05.CTRL05_RSV1 << 10);
    DRV8305_writeSpi(handle,drvRegAddr,drvDataNew);

    // Write Control Register 6
    drvRegAddr = Address_Control_6;
    drvDataNew = (Spi_8305_Vars->Ctrl_Reg_06.IDRIVEP_LS) | \
    		     (Spi_8305_Vars->Ctrl_Reg_06.IDRIVEN_LS) | \
    		     (Spi_8305_Vars->Ctrl_Reg_06.TDRIVEP)    | \
                 (Spi_8305_Vars->Ctrl_Reg_06.CTRL06_RSV1 << 10);
    DRV8305_writeSpi(handle,drvRegAddr,drvDataNew);

    // Write Control Register 7
    drvRegAddr = Address_Control_7;
    drvDataNew = (Spi_8305_Vars->Ctrl_Reg_07.TVDS)      | \
    		     (Spi_8305_Vars->Ctrl_Reg_07.TBLANK)    | \
    		     (Spi_8305_Vars->Ctrl_Reg_07.DEAD_TIME) | \
    		     (Spi_8305_Vars->Ctrl_Reg_07.PWM_MODE)  | \
    		     (Spi_8305_Vars->Ctrl_Reg_07.COMM_OPT)  | \
                 (Spi_8305_Vars->Ctrl_Reg_07.CTRL07_RSV1 << 10);
    DRV8305_writeSpi(handle,drvRegAddr,drvDataNew);

    // Write Control Register 8
    //drvRegAddr = Address_Control_8;
    //drvDataNew = (Spi_8305_Vars->Ctrl_Reg_08.RESERVED);
    //DRV8305_writeSpi(handle,drvRegAddr,drvDataNew);

    // Write Control Register 9
    drvRegAddr = Address_Control_9;
    drvDataNew = (Spi_8305_Vars->Ctrl_Reg_09.SET_VCPH_UV << 0)    | \
    		     (Spi_8305_Vars->Ctrl_Reg_09.CLR_FLTS << 1)       | \
    		     (Spi_8305_Vars->Ctrl_Reg_09.SLEEP << 2)          | \
    		     (Spi_8305_Vars->Ctrl_Reg_09.WD_EN << 3)          | \
    		     (Spi_8305_Vars->Ctrl_Reg_09.DIS_SNS_OCP << 4)    | \
                 (Spi_8305_Vars->Ctrl_Reg_09.WD_DLY)              | \
                 (Spi_8305_Vars->Ctrl_Reg_09.EN_SNS_CLAMP << 7)   | \
                 (Spi_8305_Vars->Ctrl_Reg_09.DIS_GDRV_FAULT << 8) | \
                 (Spi_8305_Vars->Ctrl_Reg_09.DISABLE << 9)        | \
                 (Spi_8305_Vars->Ctrl_Reg_09.FLIP_OTS << 10);
    DRV8305_writeSpi(handle,drvRegAddr,drvDataNew);

    // Write Control Register A
    drvRegAddr = Address_Control_A;
    drvDataNew = (Spi_8305_Vars->Ctrl_Reg_0A.GAIN_CS1)        | \
    		     (Spi_8305_Vars->Ctrl_Reg_0A.GAIN_CS2)        | \
    		     (Spi_8305_Vars->Ctrl_Reg_0A.GAIN_CS3)        | \
    		     (Spi_8305_Vars->Ctrl_Reg_0A.CS_BLANK)        | \
    		     (Spi_8305_Vars->Ctrl_Reg_0A.DC_CAL_CH1 << 8) | \
                 (Spi_8305_Vars->Ctrl_Reg_0A.DC_CAL_CH2 << 9) | \
                 (Spi_8305_Vars->Ctrl_Reg_0A.DC_CAL_CH3 << 10);
    DRV8305_writeSpi(handle,drvRegAddr,drvDataNew);

    // Write Control Register B
    drvRegAddr = Address_Control_B;
    drvDataNew = (Spi_8305_Vars->Ctrl_Reg_0B.VREG_UV_LEVEL)     | \
    		     (Spi_8305_Vars->Ctrl_Reg_0B.DIS_PWRGD << 2)    | \
    		     (Spi_8305_Vars->Ctrl_Reg_0B.SLP_DLY)           | \
    		     (Spi_8305_Vars->Ctrl_Reg_0B.CTRL0B_RSV1 << 5)  | \
    		     (Spi_8305_Vars->Ctrl_Reg_0B.CTRL0B_RSV2 << 6)  | \
                 (Spi_8305_Vars->Ctrl_Reg_0B.CTRL0B_RSV3 << 7)  | \
                 (Spi_8305_Vars->Ctrl_Reg_0B.VREF_SCALING)      | \
                 (Spi_8305_Vars->Ctrl_Reg_0B.CTRL0B_RSV4 << 10);
    DRV8305_writeSpi(handle,drvRegAddr,drvDataNew);

    // Write Control Register C
    drvRegAddr = Address_Control_C;
    drvDataNew = (Spi_8305_Vars->Ctrl_Reg_0C.VDS_MODE)          | \
    		     (Spi_8305_Vars->Ctrl_Reg_0C.VDS_LEVEL)         | \
    		     (Spi_8305_Vars->Ctrl_Reg_0C.CTRL0C_RSV1 << 8)  | \
    		     (Spi_8305_Vars->Ctrl_Reg_0C.CTRL0C_RSV2 << 9)  | \
                 (Spi_8305_Vars->Ctrl_Reg_0C.CTRL0C_RSV3 << 10);
    DRV8305_writeSpi(handle,drvRegAddr,drvDataNew);

    Spi_8305_Vars->WriteCmd = false;
  } 
  
  // Manual write to the DRV8305
  if(Spi_8305_Vars->ManWriteCmd)
  {
	// Custom Write
	drvRegAddr = (DRV8305_Address_e)(Spi_8305_Vars->ManWriteAddr << 11);
	drvDataNew = Spi_8305_Vars->ManWriteData;
    DRV8305_writeSpi(handle,drvRegAddr,drvDataNew);
    
	Spi_8305_Vars->ManWriteCmd = false;
  }
  
  return;
}  // end of DRV8305_writeData() function


void DRV8305_readData(DRV8305_Handle handle, DRV_SPI_8305_Vars_t *Spi_8305_Vars)
{
  DRV8305_Address_e drvRegAddr;
  uint16_t drvDataNew;
  
  if(Spi_8305_Vars->ReadCmd)
  {
    // Read Status Register 1
    drvRegAddr = Address_Status_1;
    drvDataNew = DRV8305_readSpi(handle,drvRegAddr);
    Spi_8305_Vars->Stat_Reg_01.OTW         = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS01_OTW_BITS)?1:0;
    Spi_8305_Vars->Stat_Reg_01.TEMP_FLAG3  = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS01_TEMP_FLAG3_BITS)?1:0;
    Spi_8305_Vars->Stat_Reg_01.TEMP_FLAG2  = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS01_TEMP_FLAG2_BITS)?1:0;
    Spi_8305_Vars->Stat_Reg_01.TEMP_FLAG1  = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS01_TEMP_FLAG1_BITS)?1:0;
    Spi_8305_Vars->Stat_Reg_01.VCPH_UVFL   = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS01_VCPH_UVFL_BITS)?1:0;
    Spi_8305_Vars->Stat_Reg_01.VDS_STATUS  = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS01_VDS_STATUS_BITS)?1:0;
    Spi_8305_Vars->Stat_Reg_01.PVDD_OVFL   = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS01_PVDD_OVFL_BITS)?1:0;
    Spi_8305_Vars->Stat_Reg_01.PVDD_UVFL   = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS01_PVDD_UVFL_BITS)?1:0;
    Spi_8305_Vars->Stat_Reg_01.TEMP_FLAG4  = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS01_TEMP_FLAG4_BITS)?1:0;
    Spi_8305_Vars->Stat_Reg_01.STAT01_RSV1 = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS01_RESERVED1_BITS)?1:0;
    Spi_8305_Vars->Stat_Reg_01.FAULT       = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS01_FAULT_BITS)?1:0;

    // Read Status Register 2
    drvRegAddr = Address_Status_2;
    drvDataNew = DRV8305_readSpi(handle,drvRegAddr);
    Spi_8305_Vars->Stat_Reg_02.SNS_A_OCP   = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS02_SNS_A_OCP_BITS)?1:0;
    Spi_8305_Vars->Stat_Reg_02.SNS_B_OCP   = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS02_SNS_B_OCP_BITS)?1:0;
    Spi_8305_Vars->Stat_Reg_02.SNS_C_OCP   = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS02_SNS_C_OCP_BITS)?1:0;
    Spi_8305_Vars->Stat_Reg_02.STAT02_RSV1 = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS02_RESERVED1_BITS)?1:0;
    Spi_8305_Vars->Stat_Reg_02.STAT02_RSV2 = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS02_RESERVED2_BITS)?1:0;
    Spi_8305_Vars->Stat_Reg_02.FETLC_VDS   = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS02_FETLC_VDS_BITS)?1:0;
    Spi_8305_Vars->Stat_Reg_02.FETHC_VDS   = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS02_FETHC_VDS_BITS)?1:0;
    Spi_8305_Vars->Stat_Reg_02.FETLB_VDS   = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS02_FETLB_VDS_BITS)?1:0;
    Spi_8305_Vars->Stat_Reg_02.FETHB_VDS   = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS02_FETHB_VDS_BITS)?1:0;
    Spi_8305_Vars->Stat_Reg_02.FETLA_VDS   = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS02_FETLA_VDS_BITS)?1:0;
    Spi_8305_Vars->Stat_Reg_02.FETHA_VDS   = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS02_FETHA_VDS_BITS)?1:0;

    // Read Status Register 3
    drvRegAddr = Address_Status_3;
    drvDataNew = DRV8305_readSpi(handle,drvRegAddr);
    Spi_8305_Vars->Stat_Reg_03.VCPH_OVLO_ABS = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS03_VCPH_OVLO_ABS_BITS)?1:0;
    Spi_8305_Vars->Stat_Reg_03.VCPH_OVLO     = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS03_VCPH_OVLO_BITS)?1:0;
    Spi_8305_Vars->Stat_Reg_03.VCPH_UVLO2    = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS03_VCPH_UVLO2_BITS)?1:0;
    Spi_8305_Vars->Stat_Reg_03.STAT03_RSV1   = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS03_RESERVED1_BITS)?1:0;
    Spi_8305_Vars->Stat_Reg_03.VCP_LSD_UVLO2 = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS03_VCP_LSD_UVLO2_BITS)?1:0;
    Spi_8305_Vars->Stat_Reg_03.AVDD_UVLO     = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS03_AVDD_UVLO_BITS)?1:0;
    Spi_8305_Vars->Stat_Reg_03.VREG_UV       = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS03_VREG_UV_BITS)?1:0;
    Spi_8305_Vars->Stat_Reg_03.STAT03_RSV2   = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS03_RESERVED2_BITS)?1:0;
    Spi_8305_Vars->Stat_Reg_03.OTS           = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS03_OTS_BITS)?1:0;
    Spi_8305_Vars->Stat_Reg_03.WD_FAULT      = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS03_WD_FAULT_BITS)?1:0;
    Spi_8305_Vars->Stat_Reg_03.PVDD_UVLO2    = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS03_PVDD_UVLO2_BITS)?1:0;

    // Read Status Register 4
    drvRegAddr = Address_Status_4;
    drvDataNew = DRV8305_readSpi(handle,drvRegAddr);
    Spi_8305_Vars->Stat_Reg_04.STAT04_RSV1 = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS04_RESERVED1_BITS)?1:0;
    Spi_8305_Vars->Stat_Reg_04.STAT04_RSV2 = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS04_RESERVED2_BITS)?1:0;
    Spi_8305_Vars->Stat_Reg_04.STAT04_RSV3 = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS04_RESERVED3_BITS)?1:0;
    Spi_8305_Vars->Stat_Reg_04.STAT04_RSV4 = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS04_RESERVED4_BITS)?1:0;
    Spi_8305_Vars->Stat_Reg_04.STAT04_RSV5 = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS04_RESERVED5_BITS)?1:0;
    Spi_8305_Vars->Stat_Reg_04.FETLC_VGS   = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS04_FETLC_VGS_BITS)?1:0;
    Spi_8305_Vars->Stat_Reg_04.FETHC_VGS   = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS04_FETHC_VGS_BITS)?1:0;
    Spi_8305_Vars->Stat_Reg_04.FETLB_VGS   = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS04_FETLB_VGS_BITS)?1:0;
    Spi_8305_Vars->Stat_Reg_04.FETHB_VGS   = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS04_FETHB_VGS_BITS)?1:0;
    Spi_8305_Vars->Stat_Reg_04.FETLA_VGS   = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS04_FETLA_VGS_BITS)?1:0;
    Spi_8305_Vars->Stat_Reg_04.FETHA_VGS   = (bool)(drvDataNew & (uint16_t)DRV8305_STATUS04_FETHA_VGS_BITS)?1:0;

    // Read Control Register 5
    drvRegAddr = Address_Control_5;
    drvDataNew = DRV8305_readSpi(handle,drvRegAddr);
    Spi_8305_Vars->Ctrl_Reg_05.IDRIVEP_HS  = (DRV8305_CTRL05_PeakSourCurHS_e)(drvDataNew & (uint16_t)DRV8305_CTRL05_IDRIVEP_HS_BITS);
    Spi_8305_Vars->Ctrl_Reg_05.IDRIVEN_HS  = (DRV8305_CTRL05_PeakSinkCurHS_e)(drvDataNew & (uint16_t)DRV8305_CTRL05_IDRIVEN_HS_BITS);
    Spi_8305_Vars->Ctrl_Reg_05.TDRIVEN     = (DRV8305_CTRL05_PeakSourTime_e)(drvDataNew & (uint16_t)DRV8305_CTRL05_TDRIVEN_BITS);
    Spi_8305_Vars->Ctrl_Reg_05.CTRL05_RSV1 = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL05_RESERVED1_BITS)?1:0;

    // Read Control Register 6
    drvRegAddr = Address_Control_6;
    drvDataNew = DRV8305_readSpi(handle,drvRegAddr);
    Spi_8305_Vars->Ctrl_Reg_06.IDRIVEP_LS  = (DRV8305_CTRL06_PeakSourCurLS_e)(drvDataNew & (uint16_t)DRV8305_CTRL06_IDRIVEP_LS_BITS);
    Spi_8305_Vars->Ctrl_Reg_06.IDRIVEN_LS  = (DRV8305_CTRL06_PeakSinkCurLS_e)(drvDataNew & (uint16_t)DRV8305_CTRL06_IDRIVEN_LS_BITS);
    Spi_8305_Vars->Ctrl_Reg_06.TDRIVEP     = (DRV8305_CTRL06_PeakSinkTime_e)(drvDataNew & (uint16_t)DRV8305_CTRL06_TDRIVEP_BITS);
    Spi_8305_Vars->Ctrl_Reg_06.CTRL06_RSV1 = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL05_RESERVED1_BITS)?1:0;

    // Read Control Register 7
    drvRegAddr = Address_Control_7;
    drvDataNew = DRV8305_readSpi(handle,drvRegAddr);
    Spi_8305_Vars->Ctrl_Reg_07.TVDS        = (DRV8305_CTRL07_VDSDeglitch_e)(drvDataNew & (uint16_t)DRV8305_CTRL07_TVDS_BITS);
    Spi_8305_Vars->Ctrl_Reg_07.TBLANK      = (DRV8305_CTRL07_VDSBlanking_e)(drvDataNew & (uint16_t)DRV8305_CTRL07_TBLANK_BITS);
    Spi_8305_Vars->Ctrl_Reg_07.DEAD_TIME   = (DRV8305_CTRL07_DeadTime_e)(drvDataNew & (uint16_t)DRV8305_CTRL07_DEAD_TIME_BITS);
    Spi_8305_Vars->Ctrl_Reg_07.PWM_MODE    = (DRV8305_CTRL07_PwmMode_e)(drvDataNew & (uint16_t)DRV8305_CTRL07_PWM_MODE_BITS);
    Spi_8305_Vars->Ctrl_Reg_07.COMM_OPT    = (DRV8305_CTRL07_CommOption_e)(drvDataNew & (uint16_t)DRV8305_CTRL07_COMM_OPT_BITS);
    Spi_8305_Vars->Ctrl_Reg_07.CTRL07_RSV1 = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL07_RESERVED1_BITS)?1:0;

    // Read Control Register 8
    //drvRegAddr = Address_Control_8;
    //drvDataNew = DRV8305_readSpi(handle,drvRegAddr);
    //Spi_8305_Vars->Ctrl_Reg_08.CTRL08_RSV1 = ( uint16_t)(drvDataNew & (uint16_t)DRV8305_CTRL08_RESERVED_BITS);

    // Read Control Register 9
    drvRegAddr = Address_Control_9;
    drvDataNew = DRV8305_readSpi(handle,drvRegAddr);
    Spi_8305_Vars->Ctrl_Reg_09.SET_VCPH_UV    = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL09_SET_VCPH_UV_BITS)?1:0;
    Spi_8305_Vars->Ctrl_Reg_09.CLR_FLTS       = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL09_CLR_FLTS_BITS)?1:0;
    Spi_8305_Vars->Ctrl_Reg_09.SLEEP          = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL09_SLEEP_BITS)?1:0;
    Spi_8305_Vars->Ctrl_Reg_09.WD_EN          = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL09_WD_EN_BITS)?1:0;
    Spi_8305_Vars->Ctrl_Reg_09.DIS_SNS_OCP    = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL09_DIS_SNS_OCP_BITS)?1:0;
    Spi_8305_Vars->Ctrl_Reg_09.WD_DLY         = (DRV8305_CTRL09_WatchDelay_e)(drvDataNew & (uint16_t)DRV8305_CTRL09_WD_DLY_BITS);
    Spi_8305_Vars->Ctrl_Reg_09.EN_SNS_CLAMP   = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL09_EN_SNS_CLAMP_BITS)?1:0;
    Spi_8305_Vars->Ctrl_Reg_09.DIS_GDRV_FAULT = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL09_DIS_GDRV_FAULT_BITS)?1:0;
    Spi_8305_Vars->Ctrl_Reg_09.DISABLE        = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL09_DISABLE_BITS)?1:0;
    Spi_8305_Vars->Ctrl_Reg_09.FLIP_OTS       = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL09_FLIP_OTS_BITS)?1:0;

    // Read Control Register A
    drvRegAddr = Address_Control_A;
    drvDataNew = DRV8305_readSpi(handle,drvRegAddr);
    Spi_8305_Vars->Ctrl_Reg_0A.GAIN_CS1   = (DRV8305_CTRL0A_CSGain1_e)(drvDataNew & (uint16_t)DRV8305_CTRL0A_GAIN_CS1_BITS);
    Spi_8305_Vars->Ctrl_Reg_0A.GAIN_CS2   = (DRV8305_CTRL0A_CSGain2_e)(drvDataNew & (uint16_t)DRV8305_CTRL0A_GAIN_CS2_BITS);
    Spi_8305_Vars->Ctrl_Reg_0A.GAIN_CS3   = (DRV8305_CTRL0A_CSGain3_e)(drvDataNew & (uint16_t)DRV8305_CTRL0A_GAIN_CS3_BITS);
    Spi_8305_Vars->Ctrl_Reg_0A.CS_BLANK   = (DRV8305_CTRL0A_CSBlank_e)(drvDataNew & (uint16_t)DRV8305_CTRL0A_CS_BLANK_BITS);
    Spi_8305_Vars->Ctrl_Reg_0A.DC_CAL_CH1 = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL0A_DC_CAL_CH1_BITS)?1:0;
    Spi_8305_Vars->Ctrl_Reg_0A.DC_CAL_CH2 = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL0A_DC_CAL_CH2_BITS)?1:0;
    Spi_8305_Vars->Ctrl_Reg_0A.DC_CAL_CH3 = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL0A_DC_CAL_CH3_BITS)?1:0;

    // Read Control Register B
    drvRegAddr = Address_Control_B;
    drvDataNew = DRV8305_readSpi(handle,drvRegAddr);
    Spi_8305_Vars->Ctrl_Reg_0B.VREG_UV_LEVEL = (DRV8305_CTRL0B_VregUvLevel_e)(drvDataNew & (uint16_t)DRV8305_CTRL0B_VREG_UV_LEVEL_BITS);
    Spi_8305_Vars->Ctrl_Reg_0B.DIS_PWRGD     = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL0B_DIS_PWRGD_BITS)?1:0;
    Spi_8305_Vars->Ctrl_Reg_0B.SLP_DLY       = (DRV8305_CTRL0B_SleepDelay_e)(drvDataNew & (uint16_t)DRV8305_CTRL0B_SLP_DLY_BITS);
    Spi_8305_Vars->Ctrl_Reg_0B.CTRL0B_RSV1   = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL0B_RESERVED1_BITS)?1:0;
    Spi_8305_Vars->Ctrl_Reg_0B.CTRL0B_RSV2   = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL0B_RESERVED2_BITS)?1:0;
    Spi_8305_Vars->Ctrl_Reg_0B.CTRL0B_RSV3   = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL0B_RESERVED3_BITS)?1:0;
    Spi_8305_Vars->Ctrl_Reg_0B.VREF_SCALING  = (DRV8305_CTRL0B_VrefScaling_e)(drvDataNew & (uint16_t)DRV8305_CTRL0B_VREF_SCALING_BITS);
    Spi_8305_Vars->Ctrl_Reg_0B.CTRL0B_RSV4   = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL0B_RESERVED4_BITS)?1:0;

    // Read Control Register C
    drvRegAddr = Address_Control_C;
    drvDataNew = DRV8305_readSpi(handle,drvRegAddr);
    Spi_8305_Vars->Ctrl_Reg_0C.VDS_MODE    = (DRV8305_CTRL0C_VDSMode_e)(drvDataNew & (uint16_t)DRV8305_CTRL0C_VDS_MODE_BITS);
    Spi_8305_Vars->Ctrl_Reg_0C.VDS_LEVEL   = (DRV8305_CTRL0C_VDSLevel_e)(drvDataNew & (uint16_t)DRV8305_CTRL0C_VDS_LEVEL_BITS);
    Spi_8305_Vars->Ctrl_Reg_0C.CTRL0C_RSV1 = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL0C_RESERVED1_BITS)?1:0;
    Spi_8305_Vars->Ctrl_Reg_0C.CTRL0C_RSV2 = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL0C_RESERVED2_BITS)?1:0;
    Spi_8305_Vars->Ctrl_Reg_0C.CTRL0C_RSV3 = (bool)(drvDataNew & (uint16_t)DRV8305_CTRL0C_RESERVED3_BITS)?1:0;

    Spi_8305_Vars->ReadCmd = false;
  }
  
  // Manual read from the DRV8305
  if(Spi_8305_Vars->ManReadCmd)
  {
	// Custom Read
	drvRegAddr = (DRV8305_Address_e)(Spi_8305_Vars->ManReadAddr << 11);
    drvDataNew = DRV8305_readSpi(handle,drvRegAddr);
	Spi_8305_Vars->ManReadData = drvDataNew;

	Spi_8305_Vars->ManReadCmd = false;
  }
  
    return;
}  // end of DRV8305_readData() function


// end of file
