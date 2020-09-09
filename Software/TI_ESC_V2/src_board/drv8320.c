//#############################################################################
// $TI Release: MotorControl SDK v3.00.01.00 $
// $Release Date: Tue May 26 19:13:59 CDT 2020 $
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

//! \file   solutions/boostxl_drv8320rs/f28004x/drivers/source/drv8320.c
//! \brief  Contains the various functions related to the DRV8320 object
//!

// **************************************************************************
// the includes

#include <math.h>

// **************************************************************************
// drivers
#include "drv8320.h"

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
void DRV8320_enable(DRV8320_Handle handle)
{
    DRV8320_Obj *obj = (DRV8320_Obj *)handle;
    volatile uint16_t enableWaitTimeOut;
    uint16_t n = 0;

    // Enable the DRV8320
    GPIO_writePin(obj->gpioNumber_EN, 1);

    enableWaitTimeOut = 0;

    // Make sure the FAULT bit is not set during startup
    while(((DRV8320_readSPI(handle, DRV8320_ADDRESS_STATUS_0) &
            DRV8320_STATUS00_FAULT_BITS) != 0) && (enableWaitTimeOut < 1000))
    {
        if(++enableWaitTimeOut > 999)
        {
            obj->enableTimeOut = true;
        }
    }

    // Wait for the DRV8320 to go through start up sequence
    for(n = 0; n < 0xffff; n++)
    {
        __asm(" NOP");
    }

    return;
} // end of DRV8320_enable() function

DRV8320_Handle DRV8320_init(void *pMemory)
{
    DRV8320_Handle handle;

    // assign the handle
    handle = (DRV8320_Handle)pMemory;

    DRV8320_resetRxTimeout(handle);
    DRV8320_resetEnableTimeout(handle);

    return(handle);
} // end of DRV8320_init() function

DRV8320_CTRL03_PeakSourCurHS_e DRV8320_getPeakSourCurHS(DRV8320_Handle handle)
{
    uint16_t data;

    // read data
    data = DRV8320_readSPI(handle, DRV8320_ADDRESS_CONTROL_3);

    // mask the bits
    data &= DRV8320_CTRL03_IDRIVEP_HS_BITS;

    return((DRV8320_CTRL03_PeakSourCurHS_e)data);
} // end of DRV8320_getPeakSourCurHS function

DRV8320_CTRL03_PeakSinkCurHS_e DRV8320_getPeakSinkCurHS(DRV8320_Handle handle)
{
    uint16_t data;

    // read data
    data = DRV8320_readSPI(handle, DRV8320_ADDRESS_CONTROL_3);

    // mask the bits
    data &= DRV8320_CTRL03_IDRIVEN_HS_BITS;

    return((DRV8320_CTRL03_PeakSinkCurHS_e)data);
} // end of DRV8320_getPeakSinkCurHS function

DRV8320_CTRL04_PeakTime_e DRV8320_getPeakSourTime(DRV8320_Handle handle)
{
    uint16_t data;

    // read data
    data = DRV8320_readSPI(handle, DRV8320_ADDRESS_CONTROL_4);

    // mask the bits
    data &= DRV8320_CTRL04_TDRIVE_BITS;

    return((DRV8320_CTRL04_PeakTime_e)data);
} // end of DRV8320_getPeakSourTime function

DRV8320_CTRL04_PeakSourCurLS_e DRV8320_getPeakSourCurLS(DRV8320_Handle handle)
{
    uint16_t data;

    // read data
    data = DRV8320_readSPI(handle, DRV8320_ADDRESS_CONTROL_4);

    // mask the bits
    data &= DRV8320_CTRL04_IDRIVEP_LS_BITS;

    return((DRV8320_CTRL04_PeakSourCurLS_e)data);
} // end of DRV8320_getPeakSourCurLS function

DRV8320_CTRL04_PeakSinkCurLS_e DRV8320_getPeakSinkCurLS(DRV8320_Handle handle)
{
    uint16_t data;

    // read data
    data = DRV8320_readSPI(handle, DRV8320_ADDRESS_CONTROL_4);

    // mask the bits
    data &= DRV8320_CTRL04_IDRIVEN_LS_BITS;

    return((DRV8320_CTRL04_PeakSinkCurLS_e)data);
} // end of DRV8320_getPeakSinkCurLS function

/*
DRV8320_CTRL04_PeakSinkTime_e DRV8320_getPeakSinkTime(DRV8320_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8320_readSPI(handle, DRV8320_ADDRESS_CONTROL_4);

  // mask the bits
  data &= DRV8320_CTRL04_TDRIVE_BITS;

  return((DRV8320_CTRL04_PeakSinkTime_e)data);
} // end of DRV8320_getPeakSinkTime function
*/

DRV8320_CTRL05_OcpDeg_e DRV8320_getVDSDeglitch(DRV8320_Handle handle)
{
    uint16_t data;

    // read data
    data = DRV8320_readSPI(handle, DRV8320_ADDRESS_CONTROL_5);

    // mask the bits
    data &= DRV8320_CTRL05_OCP_DEG_BITS;

    return((DRV8320_CTRL05_OcpDeg_e)data);
} // end of DRV8320_getVDSDeglitch function

/*
DRV8320_CTRL07_VDSBlanking_e DRV8320_getVDSBlanking(DRV8320_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8320_readSPI(handle, DRV8320_ADDRESS_CONTROL_7);

  // mask the bits
  data &= DRV8320_CTRL07_TBLANK_BITS;

  return((DRV8320_CTRL07_VDSBlanking_e)data);
} // end of DRV8320_getVDSBlanking function
*/

DRV8320_CTRL05_DeadTime_e DRV8320_getDeadTime(DRV8320_Handle handle)
{
    uint16_t data;

    // read data
    data = DRV8320_readSPI(handle, DRV8320_ADDRESS_CONTROL_5);

    // mask the bits
    data &= DRV8320_CTRL05_DEAD_TIME_BITS;

    return((DRV8320_CTRL05_DeadTime_e)data);
} // end of DRV8320_getDeadTime function

DRV8320_CTRL02_PWMMode_e DRV8320_getPWMMode(DRV8320_Handle handle)
{
    uint16_t data;

    // read data
    data = DRV8320_readSPI(handle, DRV8320_ADDRESS_CONTROL_2);

    // mask the bits
    data &= DRV8320_CTRL02_PWM_MODE_BITS;

    return((DRV8320_CTRL02_PWMMode_e)data);
} // end of DRV8320_getPWMMode function

void DRV8320_setSPIHandle(DRV8320_Handle handle, uint32_t spiHandle)
{
    DRV8320_Obj *obj = (DRV8320_Obj *)handle;

    // initialize the serial peripheral interface object
    obj->spiHandle = spiHandle;

    return;
} // end of DRV8320_setSPIHandle() function

void DRV8320_setGPIOCSNumber(DRV8320_Handle handle, uint32_t gpioNumber)
{
    DRV8320_Obj *obj = (DRV8320_Obj *)handle;

    // initialize the gpio interface object
    obj->gpioNumber_CS = gpioNumber;

    return;
} // end of DRV8320_setGPIOCSNumber() function

void DRV8320_setGPIONumber(DRV8320_Handle handle, uint32_t gpioNumber)
{
    DRV8320_Obj *obj = (DRV8320_Obj *)handle;

    // initialize the gpio interface object
    obj->gpioNumber_EN = gpioNumber;

    return;
} // end of DRV8320_setGPIONumber() function

void DRV8320_setupSPI(DRV8320_Handle handle,
                      DRV8320_SPIVars_t *drv8320SPIVars)
{
    DRV8320_Address_e drvRegAddr;
    uint16_t drvDataNew;

    // Set Default Values
    // Manual Read/Write
    drv8320SPIVars->manReadAddr  = 0;
    drv8320SPIVars->manReadData  = 0;
    drv8320SPIVars->manReadCmd = false;
    drv8320SPIVars->manWriteAddr = 0;
    drv8320SPIVars->manWriteData = 0;
    drv8320SPIVars->manWriteCmd = false;

    // Read/Write
    drv8320SPIVars->readCmd  = false;
    drv8320SPIVars->writeCmd = false;

    // Read registers for default values
    // Read Status Register 0
    drvRegAddr = DRV8320_ADDRESS_STATUS_0;
    drvDataNew = DRV8320_readSPI(handle, drvRegAddr);
    drv8320SPIVars->Stat_Reg_00.VDS_LC  = (bool)(drvDataNew & (uint16_t)DRV8320_STATUS00_VDS_LC_BITS)?1:0;
    drv8320SPIVars->Stat_Reg_00.VDS_HC  = (bool)(drvDataNew & (uint16_t)DRV8320_STATUS00_VDS_HC_BITS)?1:0;
    drv8320SPIVars->Stat_Reg_00.VDS_LB  = (bool)(drvDataNew & (uint16_t)DRV8320_STATUS00_VDS_LB_BITS)?1:0;
    drv8320SPIVars->Stat_Reg_00.VDS_HB  = (bool)(drvDataNew & (uint16_t)DRV8320_STATUS00_VDS_HB_BITS)?1:0;
    drv8320SPIVars->Stat_Reg_00.VDS_LA  = (bool)(drvDataNew & (uint16_t)DRV8320_STATUS00_VDS_LA_BITS)?1:0;
    drv8320SPIVars->Stat_Reg_00.VDS_HA  = (bool)(drvDataNew & (uint16_t)DRV8320_STATUS00_VDS_HA_BITS)?1:0;
    drv8320SPIVars->Stat_Reg_00.OTSD    = (bool)(drvDataNew & (uint16_t)DRV8320_STATUS00_OTSD_BITS)?1:0;
    drv8320SPIVars->Stat_Reg_00.UVLO    = (bool)(drvDataNew & (uint16_t)DRV8320_STATUS00_UVLO_BITS)?1:0;
    drv8320SPIVars->Stat_Reg_00.GDF     = (bool)(drvDataNew & (uint16_t)DRV8320_STATUS00_GDF_BITS)?1:0;
    drv8320SPIVars->Stat_Reg_00.VDS_OCP = (bool)(drvDataNew & (uint16_t)DRV8320_STATUS00_VDS_OCP_BITS)?1:0;
    drv8320SPIVars->Stat_Reg_00.FAULT   = (bool)(drvDataNew & (uint16_t)DRV8320_STATUS00_FAULT_BITS)?1:0;

    // Read Status Register 1
    drvRegAddr = DRV8320_ADDRESS_STATUS_1;
    drvDataNew = DRV8320_readSPI(handle, drvRegAddr);
    drv8320SPIVars->Stat_Reg_01.VGS_LC  = (bool)(drvDataNew & (uint16_t)DRV8320_STATUS01_VGS_LC_BITS)?1:0;
    drv8320SPIVars->Stat_Reg_01.VGS_HC  = (bool)(drvDataNew & (uint16_t)DRV8320_STATUS01_VGS_HC_BITS)?1:0;
    drv8320SPIVars->Stat_Reg_01.VGS_LB  = (bool)(drvDataNew & (uint16_t)DRV8320_STATUS01_VGS_LB_BITS)?1:0;
    drv8320SPIVars->Stat_Reg_01.VGS_HB  = (bool)(drvDataNew & (uint16_t)DRV8320_STATUS01_VGS_HB_BITS)?1:0;
    drv8320SPIVars->Stat_Reg_01.VGS_LA  = (bool)(drvDataNew & (uint16_t)DRV8320_STATUS01_VGS_LA_BITS)?1:0;
    drv8320SPIVars->Stat_Reg_01.VGS_HA  = (bool)(drvDataNew & (uint16_t)DRV8320_STATUS01_VGS_HA_BITS)?1:0;
    drv8320SPIVars->Stat_Reg_01.CPUV    = (bool)(drvDataNew & (uint16_t)DRV8320_STATUS01_CPUV_BITS)?1:0;
    drv8320SPIVars->Stat_Reg_01.OTW     = (bool)(drvDataNew & (uint16_t)DRV8320_STATUS01_OTW_BITS)?1:0;
    drv8320SPIVars->Stat_Reg_01.SC_OC   = (bool)(drvDataNew & (uint16_t)DRV8320_STATUS01_SC_OC_BITS)?1:0;
    drv8320SPIVars->Stat_Reg_01.SB_OC   = (bool)(drvDataNew & (uint16_t)DRV8320_STATUS01_SB_OC_BITS)?1:0;
    drv8320SPIVars->Stat_Reg_01.SA_OC   = (bool)(drvDataNew & (uint16_t)DRV8320_STATUS01_SA_OC_BITS)?1:0;

      // Read Control Register 2
    drvRegAddr = DRV8320_ADDRESS_CONTROL_2;
    drvDataNew = DRV8320_readSPI(handle, drvRegAddr);
    drv8320SPIVars->Ctrl_Reg_02.CLR_FLT  = (bool)(drvDataNew & (uint16_t)DRV8320_CTRL02_CLR_FLT_BITS);
    drv8320SPIVars->Ctrl_Reg_02.BRAKE    = (bool)(drvDataNew & (uint16_t)DRV8320_CTRL02_BRAKE_BITS);
    drv8320SPIVars->Ctrl_Reg_02.COAST    = (bool)(drvDataNew & (uint16_t)DRV8320_CTRL02_COAST_BITS);
    drv8320SPIVars->Ctrl_Reg_02.PWM1_DIR = (bool)(drvDataNew & (uint16_t)DRV8320_CTRL02_PWM1_DIR_BITS);
    drv8320SPIVars->Ctrl_Reg_02.PWM1_COM = (bool)(drvDataNew & (uint16_t)DRV8320_CTRL02_PWM1_COM_BITS);
    drv8320SPIVars->Ctrl_Reg_02.PWM_MODE = (DRV8320_CTRL02_PWMMode_e)(drvDataNew & (uint16_t)DRV8320_CTRL02_PWM_MODE_BITS);
    drv8320SPIVars->Ctrl_Reg_02.OTW_REP  = (bool)(drvDataNew & (uint16_t)DRV8320_CTRL02_OTW_REP_BITS);
    drv8320SPIVars->Ctrl_Reg_02.DIS_GDF  = (bool)(drvDataNew & (uint16_t)DRV8320_CTRL02_DIS_GDF_BITS);
    drv8320SPIVars->Ctrl_Reg_02.DIS_CPUV = (bool)(drvDataNew & (uint16_t)DRV8320_CTRL02_DIS_CPUV_BITS);
    drv8320SPIVars->Ctrl_Reg_02.CTRL02_RSV1 = (bool)(drvDataNew & (uint16_t)DRV8320_CTRL02_RESERVED1_BITS)?1:0;

    // Read Control Register 3
    drvRegAddr = DRV8320_ADDRESS_CONTROL_3;
    drvDataNew = DRV8320_readSPI(handle, drvRegAddr);
    drv8320SPIVars->Ctrl_Reg_03.IDRIVEN_HS  = (DRV8320_CTRL03_PeakSinkCurHS_e)(drvDataNew & (uint16_t)DRV8320_CTRL03_IDRIVEN_HS_BITS);
    drv8320SPIVars->Ctrl_Reg_03.IDRIVEP_HS  = (DRV8320_CTRL03_PeakSourCurHS_e)(drvDataNew & (uint16_t)DRV8320_CTRL03_IDRIVEP_HS_BITS);
    drv8320SPIVars->Ctrl_Reg_03.LOCK        = (DRV8320_CTRL03_Lock_e)(drvDataNew & (uint16_t)DRV8320_CTRL03_LOCK_BITS);

    // Read Control Register 4
    drvRegAddr = DRV8320_ADDRESS_CONTROL_4;
    drvDataNew = DRV8320_readSPI(handle, drvRegAddr);
    drv8320SPIVars->Ctrl_Reg_04.IDRIVEN_LS  = (DRV8320_CTRL04_PeakSinkCurLS_e)(drvDataNew & (uint16_t)DRV8320_CTRL04_IDRIVEN_LS_BITS);
    drv8320SPIVars->Ctrl_Reg_04.IDRIVEP_LS  = (DRV8320_CTRL04_PeakSourCurLS_e)(drvDataNew & (uint16_t)DRV8320_CTRL04_IDRIVEP_LS_BITS);
    drv8320SPIVars->Ctrl_Reg_04.TDRIVE      = (DRV8320_CTRL04_PeakTime_e)(drvDataNew & (uint16_t)DRV8320_CTRL04_TDRIVE_BITS);
    drv8320SPIVars->Ctrl_Reg_04.CBC         = (bool)(drvDataNew & (uint16_t)DRV8320_CTRL04_CBC_BITS)?1:0;

    // Read Control Register 5
    drvRegAddr = DRV8320_ADDRESS_CONTROL_5;
    drvDataNew = DRV8320_readSPI(handle, drvRegAddr);
    drv8320SPIVars->Ctrl_Reg_05.VDS_LVL     = (DRV8320_CTRL05_VDSLVL_e)(drvDataNew & (uint16_t)DRV8320_CTRL05_VDS_LVL_BITS);
    drv8320SPIVars->Ctrl_Reg_05.OCP_DEG     = (DRV8320_CTRL05_OcpDeg_e)(drvDataNew & (uint16_t)DRV8320_CTRL05_OCP_DEG_BITS);
    drv8320SPIVars->Ctrl_Reg_05.OCP_MODE    = (DRV8320_CTRL05_OcpMode_e)(drvDataNew & (uint16_t)DRV8320_CTRL05_OCP_MODE_BITS);
    drv8320SPIVars->Ctrl_Reg_05.DEAD_TIME   = (DRV8320_CTRL05_DeadTime_e)(drvDataNew & (uint16_t)DRV8320_CTRL05_DEAD_TIME_BITS);
    drv8320SPIVars->Ctrl_Reg_05.TRETRY      = (bool)(drvDataNew & (uint16_t)DRV8320_CTRL05_TRETRY_BITS);

    return;
} // end of DRV8320_setupSPI() function

uint16_t DRV8320_readSPI(DRV8320_Handle handle,
                         const DRV8320_Address_e regAddr)
{
    DRV8320_Obj *obj = (DRV8320_Obj *)handle;
    uint16_t ctrlWord;
    uint16_t n;
    const uint16_t data = 0;
    volatile uint16_t readWord;
    volatile uint16_t WaitTimeOut = 0;

    volatile SPI_RxFIFOLevel RxFifoCnt = SPI_FIFO_RXEMPTY;

    // build the control word
    ctrlWord = (uint16_t)DRV8320_buildCtrlWord(DRV8320_CTRLMODE_READ, regAddr, data);

    // reset the Rx fifo pointer to zero
    SPI_resetRxFIFO(obj->spiHandle);
    SPI_enableFIFO(obj->spiHandle);

//  GPIO_writePin(obj->gpioNumber_CS, 0);

    // wait for registers to update
    for(n = 0; n < 0x06; n++)
    {
        __asm(" NOP");
    }

    // write the command
    SPI_writeDataBlockingNonFIFO(obj->spiHandle, ctrlWord);

    // wait for two words to populate the RX fifo, or a wait timeout will occur
    while(RxFifoCnt < SPI_FIFO_RX1)
    {
        RxFifoCnt = SPI_getRxFIFOStatus(obj->spiHandle);

        if(++WaitTimeOut > 0xfffe)
        {
            obj->rxTimeOut = true;
        }
    }

    WaitTimeOut = 0xffff;

//  GPIO_writePin(obj->gpioNumber_CS, 1);

    // Read the word
    readWord = SPI_readDataNonBlocking(obj->spiHandle);

    return(readWord & DRV8320_DATA_MASK);
} // end of DRV8320_readSPI() function


void DRV8320_writeSPI(DRV8320_Handle handle, const DRV8320_Address_e regAddr,
                      const uint16_t data)
{
    DRV8320_Obj *obj = (DRV8320_Obj *)handle;
    uint16_t ctrlWord;
    uint16_t n;

    // build the control word
    ctrlWord = (uint16_t)DRV8320_buildCtrlWord(DRV8320_CTRLMODE_WRITE, regAddr, data);

    // reset the Rx fifo pointer to zero
    SPI_resetRxFIFO(obj->spiHandle);
    SPI_enableFIFO(obj->spiHandle);

    //  GPIO_writePin(obj->gpioNumber_CS, 0);

    // wait for GPIO
    for(n = 0; n < 0x06; n++)
    {
        __asm(" NOP");
    }

    // write the command
    SPI_writeDataBlockingNonFIFO(obj->spiHandle, ctrlWord);

    // wait for registers to update
    for(n = 0; n < 0x40; n++)
    {
        __asm(" NOP");
    }

    //  GPIO_writePin(obj->gpioNumber_CS, 1);

    return;
}  // end of DRV8320_writeSPI() function


void DRV8320_writeData(DRV8320_Handle handle, DRV8320_SPIVars_t *drv8320SPIVars)
{
    DRV8320_Address_e drvRegAddr;
    uint16_t drvDataNew;

    if(drv8320SPIVars->writeCmd)
    {
        // Write Control Register 2
        drvRegAddr = DRV8320_ADDRESS_CONTROL_2;
        drvDataNew = (drv8320SPIVars->Ctrl_Reg_02.CLR_FLT << 0)  | \
                     (drv8320SPIVars->Ctrl_Reg_02.BRAKE << 1)    | \
                     (drv8320SPIVars->Ctrl_Reg_02.COAST <<2)     | \
                     (drv8320SPIVars->Ctrl_Reg_02.PWM1_DIR << 3) | \
                     (drv8320SPIVars->Ctrl_Reg_02.PWM1_COM << 4) | \
                     (drv8320SPIVars->Ctrl_Reg_02.PWM_MODE)      | \
                     (drv8320SPIVars->Ctrl_Reg_02.OTW_REP << 7)  | \
                     (drv8320SPIVars->Ctrl_Reg_02.DIS_GDF << 8)  | \
                     (drv8320SPIVars->Ctrl_Reg_02.DIS_CPUV <<9)  | \
                     (drv8320SPIVars->Ctrl_Reg_02.CTRL02_RSV1 << 10);
        DRV8320_writeSPI(handle, drvRegAddr, drvDataNew);

        // Write Control Register 3
        drvRegAddr = DRV8320_ADDRESS_CONTROL_3;
        drvDataNew = (drv8320SPIVars->Ctrl_Reg_03.IDRIVEN_HS) | \
                     (drv8320SPIVars->Ctrl_Reg_03.IDRIVEP_HS) | \
                     (drv8320SPIVars->Ctrl_Reg_03.LOCK);
        DRV8320_writeSPI(handle, drvRegAddr, drvDataNew);

        // Write Control Register 4
        drvRegAddr = DRV8320_ADDRESS_CONTROL_4;
        drvDataNew = (drv8320SPIVars->Ctrl_Reg_04.IDRIVEN_LS) | \
                     (drv8320SPIVars->Ctrl_Reg_04.IDRIVEP_LS) | \
                     (drv8320SPIVars->Ctrl_Reg_04.TDRIVE) | \
                     (drv8320SPIVars->Ctrl_Reg_04.CBC << 10);
        DRV8320_writeSPI(handle, drvRegAddr, drvDataNew);

        // Write Control Register 5
        drvRegAddr = DRV8320_ADDRESS_CONTROL_5;
        drvDataNew = (drv8320SPIVars->Ctrl_Reg_05.VDS_LVL)      | \
                     (drv8320SPIVars->Ctrl_Reg_05.OCP_DEG)      | \
                     (drv8320SPIVars->Ctrl_Reg_05.OCP_MODE)     | \
                     (drv8320SPIVars->Ctrl_Reg_05.DEAD_TIME)    | \
                     (drv8320SPIVars->Ctrl_Reg_05.TRETRY << 10);
        DRV8320_writeSPI(handle, drvRegAddr, drvDataNew);

        drv8320SPIVars->writeCmd = false;
    }

    // Manual write to the DRV8320
    if(drv8320SPIVars->manWriteCmd)
    {
        // Custom Write
        drvRegAddr = (DRV8320_Address_e)(drv8320SPIVars->manWriteAddr << 11);
        drvDataNew = drv8320SPIVars->manWriteData;
        DRV8320_writeSPI(handle, drvRegAddr, drvDataNew);

        drv8320SPIVars->manWriteCmd = false;
    }

    return;
}  // end of DRV8320_writeData() function

void DRV8320_readData(DRV8320_Handle handle, DRV8320_SPIVars_t *drv8320SPIVars)
{
    DRV8320_Address_e drvRegAddr;
    uint16_t drvDataNew;

    if(drv8320SPIVars->readCmd)
    {
        // Read registers for default values
        // Read Status Register 0
        drvRegAddr = DRV8320_ADDRESS_STATUS_0;
        drvDataNew = DRV8320_readSPI(handle, drvRegAddr);
        drv8320SPIVars->Stat_Reg_00.VDS_LC  = (bool)(drvDataNew & (uint16_t)DRV8320_STATUS00_VDS_LC_BITS)?1:0;
        drv8320SPIVars->Stat_Reg_00.VDS_HC  = (bool)(drvDataNew & (uint16_t)DRV8320_STATUS00_VDS_HC_BITS)?1:0;
        drv8320SPIVars->Stat_Reg_00.VDS_LB  = (bool)(drvDataNew & (uint16_t)DRV8320_STATUS00_VDS_LB_BITS)?1:0;
        drv8320SPIVars->Stat_Reg_00.VDS_HB  = (bool)(drvDataNew & (uint16_t)DRV8320_STATUS00_VDS_HB_BITS)?1:0;
        drv8320SPIVars->Stat_Reg_00.VDS_LA  = (bool)(drvDataNew & (uint16_t)DRV8320_STATUS00_VDS_LA_BITS)?1:0;
        drv8320SPIVars->Stat_Reg_00.VDS_HA  = (bool)(drvDataNew & (uint16_t)DRV8320_STATUS00_VDS_HA_BITS)?1:0;
        drv8320SPIVars->Stat_Reg_00.OTSD    = (bool)(drvDataNew & (uint16_t)DRV8320_STATUS00_OTSD_BITS)?1:0;
        drv8320SPIVars->Stat_Reg_00.UVLO    = (bool)(drvDataNew & (uint16_t)DRV8320_STATUS00_UVLO_BITS)?1:0;
        drv8320SPIVars->Stat_Reg_00.GDF     = (bool)(drvDataNew & (uint16_t)DRV8320_STATUS00_GDF_BITS)?1:0;
        drv8320SPIVars->Stat_Reg_00.VDS_OCP = (bool)(drvDataNew & (uint16_t)DRV8320_STATUS00_VDS_OCP_BITS)?1:0;
        drv8320SPIVars->Stat_Reg_00.FAULT   = (bool)(drvDataNew & (uint16_t)DRV8320_STATUS00_FAULT_BITS)?1:0;

        // Read Status Register 1
        drvRegAddr = DRV8320_ADDRESS_STATUS_1;
        drvDataNew = DRV8320_readSPI(handle, drvRegAddr);
        drv8320SPIVars->Stat_Reg_01.VGS_LC  = (bool)(drvDataNew & (uint16_t)DRV8320_STATUS01_VGS_LC_BITS)?1:0;
        drv8320SPIVars->Stat_Reg_01.VGS_HC  = (bool)(drvDataNew & (uint16_t)DRV8320_STATUS01_VGS_HC_BITS)?1:0;
        drv8320SPIVars->Stat_Reg_01.VGS_LB  = (bool)(drvDataNew & (uint16_t)DRV8320_STATUS01_VGS_LB_BITS)?1:0;
        drv8320SPIVars->Stat_Reg_01.VGS_HB  = (bool)(drvDataNew & (uint16_t)DRV8320_STATUS01_VGS_HB_BITS)?1:0;
        drv8320SPIVars->Stat_Reg_01.VGS_LA  = (bool)(drvDataNew & (uint16_t)DRV8320_STATUS01_VGS_LA_BITS)?1:0;
        drv8320SPIVars->Stat_Reg_01.VGS_HA  = (bool)(drvDataNew & (uint16_t)DRV8320_STATUS01_VGS_HA_BITS)?1:0;
        drv8320SPIVars->Stat_Reg_01.CPUV    = (bool)(drvDataNew & (uint16_t)DRV8320_STATUS01_CPUV_BITS)?1:0;
        drv8320SPIVars->Stat_Reg_01.OTW     = (bool)(drvDataNew & (uint16_t)DRV8320_STATUS01_OTW_BITS)?1:0;
        drv8320SPIVars->Stat_Reg_01.SC_OC   = (bool)(drvDataNew & (uint16_t)DRV8320_STATUS01_SC_OC_BITS)?1:0;
        drv8320SPIVars->Stat_Reg_01.SB_OC   = (bool)(drvDataNew & (uint16_t)DRV8320_STATUS01_SB_OC_BITS)?1:0;
        drv8320SPIVars->Stat_Reg_01.SA_OC   = (bool)(drvDataNew & (uint16_t)DRV8320_STATUS01_SA_OC_BITS)?1:0;

        // Read Control Register 2
        drvRegAddr = DRV8320_ADDRESS_CONTROL_2;
        drvDataNew = DRV8320_readSPI(handle, drvRegAddr);
        drv8320SPIVars->Ctrl_Reg_02.CLR_FLT  = (bool)(drvDataNew & (uint16_t)DRV8320_CTRL02_CLR_FLT_BITS);
        drv8320SPIVars->Ctrl_Reg_02.BRAKE    = (bool)(drvDataNew & (uint16_t)DRV8320_CTRL02_BRAKE_BITS);
        drv8320SPIVars->Ctrl_Reg_02.COAST    = (bool)(drvDataNew & (uint16_t)DRV8320_CTRL02_COAST_BITS);
        drv8320SPIVars->Ctrl_Reg_02.PWM1_DIR = (bool)(drvDataNew & (uint16_t)DRV8320_CTRL02_PWM1_DIR_BITS);
        drv8320SPIVars->Ctrl_Reg_02.PWM1_COM = (bool)(drvDataNew & (uint16_t)DRV8320_CTRL02_PWM1_COM_BITS);
        drv8320SPIVars->Ctrl_Reg_02.PWM_MODE = (DRV8320_CTRL02_PWMMode_e)(drvDataNew & (uint16_t)DRV8320_CTRL02_PWM_MODE_BITS);
        drv8320SPIVars->Ctrl_Reg_02.OTW_REP  = (bool)(drvDataNew & (uint16_t)DRV8320_CTRL02_OTW_REP_BITS);
        drv8320SPIVars->Ctrl_Reg_02.DIS_GDF  = (bool)(drvDataNew & (uint16_t)DRV8320_CTRL02_DIS_GDF_BITS);
        drv8320SPIVars->Ctrl_Reg_02.DIS_CPUV = (bool)(drvDataNew & (uint16_t)DRV8320_CTRL02_DIS_CPUV_BITS);
        drv8320SPIVars->Ctrl_Reg_02.CTRL02_RSV1 = (bool)(drvDataNew & (uint16_t)DRV8320_CTRL02_RESERVED1_BITS)?1:0;

        // Read Control Register 3
        drvRegAddr = DRV8320_ADDRESS_CONTROL_3;
        drvDataNew = DRV8320_readSPI(handle, drvRegAddr);
        drv8320SPIVars->Ctrl_Reg_03.IDRIVEN_HS  = (DRV8320_CTRL03_PeakSinkCurHS_e)(drvDataNew & (uint16_t)DRV8320_CTRL03_IDRIVEN_HS_BITS);
        drv8320SPIVars->Ctrl_Reg_03.IDRIVEP_HS  = (DRV8320_CTRL03_PeakSourCurHS_e)(drvDataNew & (uint16_t)DRV8320_CTRL03_IDRIVEP_HS_BITS);
        drv8320SPIVars->Ctrl_Reg_03.LOCK        = (DRV8320_CTRL03_Lock_e)(drvDataNew & (uint16_t)DRV8320_CTRL03_LOCK_BITS);

        // Read Control Register 4
        drvRegAddr = DRV8320_ADDRESS_CONTROL_4;
        drvDataNew = DRV8320_readSPI(handle, drvRegAddr);
        drv8320SPIVars->Ctrl_Reg_04.IDRIVEN_LS  = (DRV8320_CTRL04_PeakSinkCurLS_e)(drvDataNew & (uint16_t)DRV8320_CTRL04_IDRIVEN_LS_BITS);
        drv8320SPIVars->Ctrl_Reg_04.IDRIVEP_LS  = (DRV8320_CTRL04_PeakSourCurLS_e)(drvDataNew & (uint16_t)DRV8320_CTRL04_IDRIVEP_LS_BITS);
        drv8320SPIVars->Ctrl_Reg_04.TDRIVE      = (DRV8320_CTRL04_PeakTime_e)(drvDataNew & (uint16_t)DRV8320_CTRL04_TDRIVE_BITS);
        drv8320SPIVars->Ctrl_Reg_04.CBC         = (bool)(drvDataNew & (uint16_t)DRV8320_CTRL04_CBC_BITS)?1:0;

        // Read Control Register 5
        drvRegAddr = DRV8320_ADDRESS_CONTROL_5;
        drvDataNew = DRV8320_readSPI(handle, drvRegAddr);
        drv8320SPIVars->Ctrl_Reg_05.VDS_LVL     = (DRV8320_CTRL05_VDSLVL_e)(drvDataNew & (uint16_t)DRV8320_CTRL05_VDS_LVL_BITS);
        drv8320SPIVars->Ctrl_Reg_05.OCP_DEG     = (DRV8320_CTRL05_OcpDeg_e)(drvDataNew & (uint16_t)DRV8320_CTRL05_OCP_DEG_BITS);
        drv8320SPIVars->Ctrl_Reg_05.OCP_MODE    = (DRV8320_CTRL05_OcpMode_e)(drvDataNew & (uint16_t)DRV8320_CTRL05_OCP_MODE_BITS);
        drv8320SPIVars->Ctrl_Reg_05.DEAD_TIME   = (DRV8320_CTRL05_DeadTime_e)(drvDataNew & (uint16_t)DRV8320_CTRL05_DEAD_TIME_BITS);
        drv8320SPIVars->Ctrl_Reg_05.TRETRY      = (bool)(drvDataNew & (uint16_t)DRV8320_CTRL05_TRETRY_BITS);

        drv8320SPIVars->readCmd = false;
    }

    // Manual read from the DRV8320
    if(drv8320SPIVars->manReadCmd)
    {
        // Custom Read
        drvRegAddr = (DRV8320_Address_e)(drv8320SPIVars->manReadAddr << 11);
        drvDataNew = DRV8320_readSPI(handle, drvRegAddr);
        drv8320SPIVars->manReadData = drvDataNew;

        drv8320SPIVars->manReadCmd = false;
    }

    return;
}  // end of DRV8320_readData() function

// end of file
