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

//! \file   solutions/boostxl_drv8320rs/f28004x/drivers/hal.c
//! \brief  Contains the various functions related to the HAL object
//!


// **************************************************************************
// the includes

// drivers

// modules
#include "user.h"

// platforms
#include "hal.h"
#include "hal_obj.h"

// libraries
#include "datalog.h"

#ifdef _FLASH
#pragma CODE_SECTION(Flash_initModule, ".TI.ramfunc");
#endif

// **************************************************************************
// the defines


// **************************************************************************
// the globals


// **************************************************************************
// the functions
void HAL_cal(HAL_Handle handle)
{
  SysCtl_deviceCal();
  return;
} // end of HAL_cal() function

void HAL_disableGlobalInts(HAL_Handle handle)
{

  // disable global interrupts
  Interrupt_disableMaster();

  return;
} // end of HAL_disableGlobalInts() function

void HAL_disableWdog(HAL_Handle halHandle)
{

  // disable watchdog
  SysCtl_disableWatchdog();

  return;
} // end of HAL_disableWdog() function

void HAL_enableADCInts(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    // enable the PIE interrupts associated with the ADC interrupts
    Interrupt_enable(INT_ADCB2);        //RB2

    // enable the ADC interrupts
    ADC_enableInterrupt(obj->adcHandle[1], ADC_INT_NUMBER2);

    // enable the cpu interrupt for ADC interrupts
    Interrupt_enableInCPU(INTERRUPT_CPU_INT1);


#ifdef _VSF_EN_
    // enable the cpu timer 0 interrupt for FAST estimator
    Interrupt_enable(INT_TIMER0);
#endif  // _VSF_EN_

    return;
} // end of HAL_enableADCInts() function

void HAL_enableADCIntsToTriggerCLA(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    // clear the ADC interrupt flag
    ADC_clearInterruptStatus(obj->adcHandle[1], ADC_INT_NUMBER2);

    // Acknowledge interrupt from PIE group 10
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP10);

    // set up trigger source for CLA task 2
    CLA_setTriggerSource(CLA_TASK_2, CLA_TRIGGER_ADCB2);

    // enable the ADC interrupts
    ADC_enableInterrupt(obj->adcHandle[1], ADC_INT_NUMBER2);

    return;
} // end of HAL_enableADCIntsToTriggerCLA() function


void HAL_enableDebugInt(HAL_Handle handle)
{

    // enable debug events
    ERTM;

    return;
} // end of HAL_enableDebugInt() function


void HAL_enableDRV(HAL_Handle handle)
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  DRV8320_enable(obj->drv8320Handle);

  return;
}  // end of HAL_enableDRV() function

HAL_Handle HAL_init(void *pMemory,const size_t numBytes)
{
    HAL_Handle handle;
    HAL_Obj *obj;

    if(numBytes < sizeof(HAL_Obj))
    {
        return((HAL_Handle)NULL);
    }

    // assign the handle
    handle = (HAL_Handle)pMemory;

    // assign the object
    obj = (HAL_Obj *)handle;

    // disable watchdog
    SysCtl_disableWatchdog();

    // initialize the ADC handles
    obj->adcHandle[0] = ADCA_BASE;
    obj->adcHandle[1] = ADCB_BASE;
    obj->adcHandle[2] = ADCC_BASE;

    // initialize the ADC results
    obj->adcResult[0] = ADCARESULT_BASE;
    obj->adcResult[1] = ADCBRESULT_BASE;
    obj->adcResult[2] = ADCCRESULT_BASE;

    // initialize CLA handle
    obj->claHandle = CLA1_BASE;

    // initialize SCI handle
    obj->sciHandle[0] = SCIA_BASE;        //!< the SCIA handle
    obj->sciHandle[1] = SCIB_BASE;        //!< the SCIB handle

    // initialize SPI handle
    obj->spiHandle[0] = SPIA_BASE;        //!< the SPIA handle
    obj->spiHandle[1] = SPIB_BASE;        //!< the SPIB handle

    // initialize CAN handle
    obj->canHandle[0] = CANA_BASE;        //!< the CANA handle
    obj->canHandle[1] = CANB_BASE;        //!< the CANA handle

    // initialize DMA handle
    obj->dmaHandle = DMA_BASE;            //!< the DMA handle

    // initialize DMA channel handle
    obj->dmaChHandle[0] = DMA_CH1_BASE;   //!< the DMA Channel handle
    obj->dmaChHandle[1] = DMA_CH2_BASE;   //!< the DMA Channel handle
    obj->dmaChHandle[2] = DMA_CH3_BASE;   //!< the DMA Channel handle
    obj->dmaChHandle[3] = DMA_CH4_BASE;   //!< the DMA Channel handle

    // For T200 Controller ************

    // initialize PWM handles for Motor 1
    obj->pwmHandle[0] = EPWM1_BASE;       //!< the PWM handle
    obj->pwmHandle[1] = EPWM2_BASE;       //!< the PWM handle
    obj->pwmHandle[2] = EPWM3_BASE;       //!< the PWM handle

    // initialize PGA handle
    obj->pgaHandle[0] = PGA1_BASE;        //!< the PGA handle
    obj->pgaHandle[1] = PGA2_BASE;        //!< the PGA handle
    obj->pgaHandle[2] = PGA3_BASE;        //!< the PGA handle

    // initialize CMPSS handle
    obj->cmpssHandle[0] = CMPSS1_BASE;    //!< the CMPSS handle
    obj->cmpssHandle[1] = CMPSS2_BASE;    //!< the CMPSS handle
    obj->cmpssHandle[2] = CMPSS3_BASE;    //!< the CMPSS handle

    //*********************************

    // initialize DAC handle
    obj->dacHandle[0] = DACB_BASE;        //!< the DAC handle
    obj->dacHandle[1] = DACA_BASE;        //!< the DAC handle

    // initialize timer handles
    obj->timerHandle[0] = CPUTIMER0_BASE;
    obj->timerHandle[1] = CPUTIMER1_BASE;
    obj->timerHandle[2] = CPUTIMER2_BASE;

    // initialize pwmdac handles
    obj->pwmDACHandle[0] = EPWM7_BASE;
    obj->pwmDACHandle[1] = EPWM7_BASE;
    obj->pwmDACHandle[2] = EPWM8_BASE;
    obj->pwmDACHandle[3] = EPWM8_BASE;

    // initialize drv8320 interface
    obj->drv8320Handle = DRV8320_init(&obj->drv8320);

#ifdef _EQEP_EN_
    // initialize QEP driver
    obj->qepHandle[0] = EQEP1_BASE;         // EQEP1
    obj->qepHandle[1] = EQEP2_BASE;         // EQEP2
#endif

  return(handle);
} // end of HAL_init() function


void HAL_setParams(HAL_Handle handle)
{
    HAL_setNumCurrentSensors(handle,USER_NUM_CURRENT_SENSORS);
    HAL_setNumVoltageSensors(handle,USER_NUM_VOLTAGE_SENSORS);

    // disable global interrupts
    Interrupt_disableMaster();

    // Disable the watchdog
    SysCtl_disableWatchdog();

#ifdef _FLASH
    //
    // Copy time critical code and flash setup code to RAM. This includes the
    // following functions: InitFlash();
    //
    // The RamfuncsLoadStart, RamfuncsLoadSize, and RamfuncsRunStart symbols
    // are created by the linker. Refer to the device .cmd file.
    //
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
#endif

    // Disable DC-DC controller
    ASysCtl_disableDCDC();

    // Enable temperature sensor
    ASysCtl_enableTemperatureSensor();

    // initialize the interrupt controller
    Interrupt_initModule();

    // init vector table
    Interrupt_initVectorTable();

    // Set up PLL control and clock dividers
    // PLLSYSCLK = 20MHz (XTAL_OSC) * 10 (IMULT) * 1 (FMULT) / 2 (PLLCLK_BY_2)
    SysCtl_setClock(SYSCTL_OSCSRC_XTAL |
                    SYSCTL_IMULT(10) |
                    SYSCTL_FMULT_NONE |
                    SYSCTL_SYSDIV(2) |
                    SYSCTL_PLL_ENABLE);

    // These asserts will check that the #defines for the clock rates in
    // device.h match the actual rates that have been configured. If they do
    // not match, check that the calculations of DEVICE_SYSCLK_FREQ and
    // DEVICE_LSPCLK_FREQ are accurate. Some examples will not perform as
    // expected if these are not correct.
    //
    ASSERT(SysCtl_getClock(DEVICE_OSCSRC_FREQ) == DEVICE_SYSCLK_FREQ);
    ASSERT(SysCtl_getLowSpeedClock(DEVICE_OSCSRC_FREQ) == DEVICE_LSPCLK_FREQ);


    // run the device calibration
    HAL_cal(handle);

    // setup the peripheral clocks
    HAL_setupPeripheralClks(handle);

#ifdef CLA
    // setup CLA
    HAL_setupCLA(handle);
#endif

    // setup the GPIOs
    HAL_setupGPIOs(handle);

#ifdef _FLASH
    //
    // Call Flash Initialization to setup flash waitstates. This function must
    // reside in RAM.
    Flash_initModule(FLASH0CTRL_BASE, FLASH0ECC_BASE, DEVICE_FLASH_WAITSTATES);
#endif

    // setup the ADCs
    HAL_setupADCs(handle);

    // setup the PGAs
    HAL_setupPGAs(handle);

    // setup the Dacs
    HAL_setupDACs(handle);

    // setup the CMPSSs
    //HAL_setupCMPSSs(handle);

    // setup the PWMs
    HAL_setupPWMs(handle,
                  USER_SYSTEM_FREQ_MHz,
                  USER_PWM_PERIOD_usec,
                  USER_NUM_PWM_TICKS_PER_ISR_TICK);

    // set the current scale factor
    HAL_setCurrentScaleFactor(handle,USER_CURRENT_SF);

    // set the voltage scale factor
    HAL_setVoltageScaleFactor(handle,USER_VOLTAGE_SF);

    // setup the timers
    HAL_setupTimers(handle, USER_SYSTEM_FREQ_MHz);

    // setup the sci
    HAL_setupSCIA(handle);

    // setup the spiB for DRV8320_Kit_RevD
    HAL_setupSPIA(handle);

    // setup the spiB for DRV8301_Kit_RevD
    HAL_setupSPIB(handle);

    // setup the drv8320 interface
    HAL_setupGate(handle);

    // setup the CANB interface
    HAL_setupCANB(handle);

#ifdef _EQEP_EN_
    // setup the eqep
    HAL_setupQEP(handle, HAL_QEP_QEP1);
#endif

#ifdef _VSF_EN_
    // setup the timer for estimator
    HAL_setupEstTimer(handle, USER_SYSTEM_FREQ_MHz, USER_EST_FREQ_Hz);
#endif  // _VSF_EN_

  return;
} // end of HAL_setParams() function

void HAL_setupADCs(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    SysCtl_delay(100U);
    ADC_setVREF(obj->adcHandle[2], ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    ADC_setVREF(obj->adcHandle[1], ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    ADC_setVREF(obj->adcHandle[0], ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    SysCtl_delay(100U);

    // Configure internal reference as 1.65V*2 = 3.3V
    ASysCtl_setAnalogReference1P65(ASYSCTL_VREFHIA |
                                   ASYSCTL_VREFHIB |
                                   ASYSCTL_VREFHIC);

    // Enable internal voltage reference
    ASysCtl_setAnalogReferenceInternal(ASYSCTL_VREFHIA |
                                       ASYSCTL_VREFHIB |
                                       ASYSCTL_VREFHIC);

    // Set main clock scaling factor (50MHz max clock for the ADC module)
    ADC_setPrescaler(obj->adcHandle[0], ADC_CLK_DIV_2_0);
    ADC_setPrescaler(obj->adcHandle[1], ADC_CLK_DIV_2_0);
    ADC_setPrescaler(obj->adcHandle[2], ADC_CLK_DIV_2_0);

    // set the ADC interrupt pulse generation to end of conversion
    ADC_setInterruptPulseMode(obj->adcHandle[0], ADC_PULSE_END_OF_CONV);
    ADC_setInterruptPulseMode(obj->adcHandle[1], ADC_PULSE_END_OF_CONV);
    ADC_setInterruptPulseMode(obj->adcHandle[2], ADC_PULSE_END_OF_CONV);

    // enable the ADCs
    ADC_enableConverter(obj->adcHandle[0]);
    ADC_enableConverter(obj->adcHandle[1]);
    ADC_enableConverter(obj->adcHandle[2]);

    // set priority of SOCs
    ADC_setSOCPriority(obj->adcHandle[0], ADC_PRI_ALL_HIPRI);
    ADC_setSOCPriority(obj->adcHandle[1], ADC_PRI_ALL_HIPRI);
    ADC_setSOCPriority(obj->adcHandle[2], ADC_PRI_ALL_HIPRI);

    // delay to allow ADCs to power up
    SysCtl_delay(1000U);

    // configure the interrupt sources
    // configure the ample window to 15 system clock cycle wide by assigning 14
    // to the ACQPS of ADCSOCxCTL Register.
    // RB2/B2
    ADC_setInterruptSource(obj->adcHandle[1], ADC_INT_NUMBER2, ADC_SOC_NUMBER2);

    // configure the SOCs for T200 Controller
    // ISENA - PGA1->A11->RA0
    ADC_setupSOC(obj->adcHandle[0], ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN11, HAL_ADC_SAMPLE_WINDOW);

    // ISENB - PGA2->B9->RB0
    ADC_setupSOC(obj->adcHandle[1], ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN9, HAL_ADC_SAMPLE_WINDOW);

    // ISENC - PGA3->C7->RC0
    ADC_setupSOC(obj->adcHandle[2], ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN7, HAL_ADC_SAMPLE_WINDOW);

    // VSENA - A2->RA1
    ADC_setupSOC(obj->adcHandle[0], ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN2, HAL_ADC_SAMPLE_WINDOW);

    // VSENB - B3->RB1
    ADC_setupSOC(obj->adcHandle[1], ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN3, HAL_ADC_SAMPLE_WINDOW);

    // VSENC - C3->RC1
    ADC_setupSOC(obj->adcHandle[2], ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN3, HAL_ADC_SAMPLE_WINDOW);

    // VSENVM - B2->RB2
    ADC_setupSOC(obj->adcHandle[1], ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN2, HAL_ADC_SAMPLE_WINDOW);


  return;
} // end of HAL_setupADCs() function


void HAL_runADCZeroOffsetCalibration(const uint32_t base)
{
    uint16_t adcOffsetMean;
    uint32_t adcSum;
    uint16_t index, sampleSize;

    // Adc Zero Offset Calibration
    // This is not typically necessary
    //      to achieve datasheet specified performance
    ADC_setupSOC(base, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN13, 10);
    ADC_setupSOC(base, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN13, 10);
    ADC_setupSOC(base, ADC_SOC_NUMBER2, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN13, 10);
    ADC_setupSOC(base, ADC_SOC_NUMBER3, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN13, 10);
    ADC_setupSOC(base, ADC_SOC_NUMBER4, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN13, 10);
    ADC_setupSOC(base, ADC_SOC_NUMBER5, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN13, 10);
    ADC_setupSOC(base, ADC_SOC_NUMBER6, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN13, 10);
    ADC_setupSOC(base, ADC_SOC_NUMBER7, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN13, 10);

    EALLOW;
    HWREGH(base + ADC_O_OFFTRIM) = 96;
    EDIS;

    //
    // Set SOC1 to set the interrupt 1 flag. Enable the interrupt and make
    // sure its flag is cleared.
    //
    ADC_setInterruptSource(base, ADC_INT_NUMBER1, ADC_SOC_NUMBER7);
    ADC_enableInterrupt(base, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(base, ADC_INT_NUMBER1);

    adcOffsetMean = 0;
    adcSum = 0;
    index = 0;
    sampleSize = 512;

    while(index < sampleSize)
    {
        ADC_forceSOC(base, ADC_SOC_NUMBER0);
        ADC_forceSOC(base, ADC_SOC_NUMBER1);
        ADC_forceSOC(base, ADC_SOC_NUMBER2);
        ADC_forceSOC(base, ADC_SOC_NUMBER3);
        ADC_forceSOC(base, ADC_SOC_NUMBER4);
        ADC_forceSOC(base, ADC_SOC_NUMBER5);
        ADC_forceSOC(base, ADC_SOC_NUMBER6);
        ADC_forceSOC(base, ADC_SOC_NUMBER7);

        SysCtl_delay(2000U);

        while(ADC_getInterruptStatus(base, ADC_INT_NUMBER1) == false)
        {
        }

        SysCtl_delay(100U);

        adcSum += ADC_readResult(base, ADC_SOC_NUMBER0);
        adcSum += ADC_readResult(base, ADC_SOC_NUMBER1);
        adcSum += ADC_readResult(base, ADC_SOC_NUMBER2);
        adcSum += ADC_readResult(base, ADC_SOC_NUMBER3);
        adcSum += ADC_readResult(base, ADC_SOC_NUMBER4);
        adcSum += ADC_readResult(base, ADC_SOC_NUMBER5);
        adcSum += ADC_readResult(base, ADC_SOC_NUMBER6);
        adcSum += ADC_readResult(base, ADC_SOC_NUMBER7);

        index += 8;

        ADC_clearInterruptStatus(base, ADC_INT_NUMBER1);
    }

    ADC_clearInterruptStatus(base, ADC_INT_NUMBER1);

    //Calculate average ADC sample value
    adcOffsetMean = adcSum / sampleSize;

    // delay to allow ADCs to power up
    SysCtl_delay(100U);

    EALLOW;
    HWREGH(base + ADC_O_OFFTRIM) = 96 - adcOffsetMean;
    EDIS;
//    ADC_setOffTrim(base, 96-adcOffsetMean);

    // delay to allow ADCs to power up
    SysCtl_delay(100U);

    return;
}

void HAL_setupPGAs(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    uint16_t  cnt;

    // For Motor_1/Motor_2
    for(cnt = 0; cnt < 3; cnt++)
    {
        // Set a gain of 12 to PGA1/2/3
        PGA_setGain(obj->pgaHandle[cnt], PGA_GAIN_12);

        // No filter resistor for output
        PGA_setFilterResistor(obj->pgaHandle[cnt], PGA_LOW_PASS_FILTER_DISABLED);

        // Enable PGA1/2/3
        PGA_enable(obj->pgaHandle[cnt]);
    }

    return;
} // end of HAL_setupPGAs() function

// HAL_setupCMPSSs
void HAL_setupCMPSSs(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    uint16_t  cnt;

    // Set the initial value to half of ADC range
    uint16_t cmpsaDACH = 2048 + 1024;
    uint16_t cmpsaDACL = 2048 - 1024;

    for(cnt = 0; cnt < 3; cnt++)
    {
        // Enable CMPSS and configure the negative input signal to come
        // from the DAC
        CMPSS_enableModule(obj->cmpssHandle[cnt]);

        CMPSS_configHighComparator(obj->cmpssHandle[cnt], CMPSS_INSRC_DAC);
        CMPSS_configLowComparator(obj->cmpssHandle[cnt], CMPSS_INSRC_DAC);

        // Use VDDA as the reference for the DAC and set DAC value to midpoint
        // for arbitrary reference
        CMPSS_configDAC(obj->cmpssHandle[cnt], CMPSS_DACREF_VDDA |
                        CMPSS_DACVAL_SYSCLK | CMPSS_DACSRC_SHDW);

        // define these numbers in hal.h
        //
        CMPSS_setDACValueHigh(obj->cmpssHandle[cnt], cmpsaDACH);
        CMPSS_setDACValueLow(obj->cmpssHandle[cnt], cmpsaDACL);

        // Configure digital filter. For this example, the maxiumum values will
        // be used for the clock prescale, sample window size, and threshold.
        CMPSS_configFilterHigh(obj->cmpssHandle[cnt], 32, 32, 30);
        CMPSS_configFilterLow(obj->cmpssHandle[cnt], 32, 32, 30);

        // Initialize the filter logic and start filtering
        CMPSS_initFilterHigh(obj->cmpssHandle[cnt]);
        CMPSS_initFilterLow(obj->cmpssHandle[cnt]);

        // Configure the output signals. Both CTRIPH and CTRIPOUTH will be fed
        // by the asynchronous comparator output. CMPSS_INV_INVERTED |
        CMPSS_configOutputsHigh(obj->cmpssHandle[cnt], CMPSS_TRIP_FILTER |
                                CMPSS_TRIPOUT_FILTER |
                                CMPSS_OR_ASYNC_OUT_W_FILT);

        CMPSS_configOutputsLow(obj->cmpssHandle[cnt], CMPSS_TRIP_FILTER |
                               CMPSS_TRIPOUT_FILTER |
                               CMPSS_INV_INVERTED);

        // Clear any high comparator digital filter output latch
        CMPSS_clearFilterLatchHigh(obj->cmpssHandle[cnt]);

        // Clear any low comparator digital filter output latch
        CMPSS_clearFilterLatchLow(obj->cmpssHandle[cnt]);
    }

    //
    // Refer to the Table 9-2 in Chapter 9 of TMS320F28004x
    // Technical Reference Manual (SPRUI33B), to configure the ePWM X-Bar
    //

    // For T200 Controller

    ASysCtl_selectCMPHPMux(ASYSCTL_CMPHPMUX_SELECT_1, 4);
    ASysCtl_selectCMPLPMux(ASYSCTL_CMPLPMUX_SELECT_1, 4);

    ASysCtl_selectCMPHPMux(ASYSCTL_CMPHPMUX_SELECT_2, 4);
    ASysCtl_selectCMPLPMux(ASYSCTL_CMPLPMUX_SELECT_2, 4);

    ASysCtl_selectCMPHPMux(ASYSCTL_CMPHPMUX_SELECT_3, 4);
    ASysCtl_selectCMPLPMux(ASYSCTL_CMPLPMUX_SELECT_3, 4);

    //Phase A / PGA1
    // Configure TRIP7 to be CTRIP1H and CTRIP1L using the ePWM X-BAR
    XBAR_setEPWMMuxConfig(XBAR_TRIP7, XBAR_EPWM_MUX00_CMPSS1_CTRIPH_OR_L);
    XBAR_enableEPWMMux(XBAR_TRIP7, XBAR_MUX00);

    //Phase B / PGA2
    // Configure TRIP8 to be CTRIP1H and CTRIP1L using the ePWM X-BAR
    XBAR_setEPWMMuxConfig(XBAR_TRIP8, XBAR_EPWM_MUX02_CMPSS2_CTRIPH_OR_L);
    XBAR_enableEPWMMux(XBAR_TRIP8, XBAR_MUX02);

    //Phase C / PGA3
    // Configure TRIP9 to be CTRIP1H and CTRIP1L using the ePWM X-BAR
    XBAR_setEPWMMuxConfig(XBAR_TRIP9, XBAR_EPWM_MUX04_CMPSS3_CTRIPH_OR_L);
    XBAR_enableEPWMMux(XBAR_TRIP9, XBAR_MUX04);


    return;
} // end of HAL_setupCMPSSs() function

void HAL_setupDACs(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    //DACA for T200 controller
    // Set the DAC gain to 2
    DAC_setGainMode(obj->dacHandle[1], DAC_GAIN_TWO);

    // Set ADC voltage reference
    DAC_setReferenceVoltage(obj->dacHandle[1], DAC_REF_ADC_VREFHI);

    // Set load mode for DAC on next SYSCLK
    DAC_setLoadMode(obj->dacHandle[1], DAC_LOAD_SYSCLK);

    // Enable DAC output
    DAC_enableOutput(obj->dacHandle[1]);

    // Set the DAC Shadow Output Value
    // Set the initial value to half of ADC range for 1.65V output
    DAC_setShadowValue(obj->dacHandle[1], 2048U);

    return;
} // end of HAL_setupDACs() function


void HAL_writeDRVData(HAL_Handle handle, DRV8320_SPIVars_t *drv8320SPIVars)
{
  HAL_Obj  *obj = (HAL_Obj *)handle;

  DRV8320_writeData(obj->drv8320Handle,drv8320SPIVars);

  return;
}  // end of HAL_writeDRVData() function


void HAL_readDRVData(HAL_Handle handle, DRV8320_SPIVars_t *drv8320SPIVars)
{
  HAL_Obj  *obj = (HAL_Obj *)handle;

  DRV8320_readData(obj->drv8320Handle,drv8320SPIVars);

  return;
}  // end of HAL_readDRVData() function

void HAL_setupDRVSPI(HAL_Handle handle, DRV8320_SPIVars_t *drv8320SPIVars)
{
  HAL_Obj  *obj = (HAL_Obj *)handle;

  DRV8320_setupSPI(obj->drv8320Handle, drv8320SPIVars);

  return;
}  // end of HAL_setupDRVSPI() function


void HAL_setupFaults(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;
    uint16_t cnt;

    // Configure Trip Mechanism for the Motor control software
    // -Cycle by cycle trip on CPU halt
    // -One shot fault trip zone
    // These trips need to be repeated for EPWM1 ,2 & 3

    // configure the input x bar for TZ2 to GPIO, where Over Current is connected
    XBAR_setInputPin(XBAR_INPUT2, HAL_PM_nFAULT_GPIO);
    XBAR_lockInput(XBAR_INPUT2);

    for(cnt=0;cnt<3;cnt++)
    {
        EPWM_enableTripZoneSignals(obj->pwmHandle[cnt],
                                   EPWM_TZ_SIGNAL_CBC6);

        EPWM_enableTripZoneSignals(obj->pwmHandle[cnt],
                                   EPWM_TZ_SIGNAL_OSHT2);

        //enable DC TRIP combinational input
        EPWM_enableDigitalCompareTripCombinationInput(obj->pwmHandle[cnt],
               (EPWM_DC_COMBINATIONAL_TRIPIN7 |
                EPWM_DC_COMBINATIONAL_TRIPIN8 |
                EPWM_DC_COMBINATIONAL_TRIPIN9),
                EPWM_DC_TYPE_DCAH);
        EPWM_enableDigitalCompareTripCombinationInput(obj->pwmHandle[cnt],
               (EPWM_DC_COMBINATIONAL_TRIPIN7 |
                EPWM_DC_COMBINATIONAL_TRIPIN8 |
                EPWM_DC_COMBINATIONAL_TRIPIN9),
                EPWM_DC_TYPE_DCAL);
        EPWM_enableDigitalCompareTripCombinationInput(obj->pwmHandle[cnt],
               (EPWM_DC_COMBINATIONAL_TRIPIN7 |
                EPWM_DC_COMBINATIONAL_TRIPIN8 |
                EPWM_DC_COMBINATIONAL_TRIPIN9),
               EPWM_DC_TYPE_DCBH);
        EPWM_enableDigitalCompareTripCombinationInput(obj->pwmHandle[cnt],
               (EPWM_DC_COMBINATIONAL_TRIPIN7 |
                EPWM_DC_COMBINATIONAL_TRIPIN8 |
                EPWM_DC_COMBINATIONAL_TRIPIN9),
               EPWM_DC_TYPE_DCBL);

        // Trigger event when DCAH is High
        EPWM_setTripZoneDigitalCompareEventCondition(obj->pwmHandle[cnt],
                                                     EPWM_TZ_DC_OUTPUT_A1,
                                                     EPWM_TZ_EVENT_DCXH_HIGH);

        // Trigger event when DCBH is High
        EPWM_setTripZoneDigitalCompareEventCondition(obj->pwmHandle[cnt],
                                                     EPWM_TZ_DC_OUTPUT_B1,
                                                     EPWM_TZ_EVENT_DCXL_HIGH);

        // Enable DCA as OST
        EPWM_enableTripZoneSignals(obj->pwmHandle[cnt], EPWM_TZ_SIGNAL_DCAEVT1);

        // Enable DCB as OST
        EPWM_enableTripZoneSignals(obj->pwmHandle[cnt], EPWM_TZ_SIGNAL_DCBEVT1);

        // Configure the DCA path to be un-filtered and asynchronous
        EPWM_setDigitalCompareEventSource(obj->pwmHandle[cnt],
                                          EPWM_DC_MODULE_A,
                                          EPWM_DC_EVENT_1,
                                          EPWM_DC_EVENT_SOURCE_FILT_SIGNAL);

        // Configure the DCB path to be un-filtered and asynchronous
        EPWM_setDigitalCompareEventSource(obj->pwmHandle[cnt],
                                          EPWM_DC_MODULE_B,
                                          EPWM_DC_EVENT_1,
                                          EPWM_DC_EVENT_SOURCE_FILT_SIGNAL);

        // What do we want the OST/CBC events to do?
        // TZA events can force EPWMxA
        // TZB events can force EPWMxB
        EPWM_setTripZoneAction(obj->pwmHandle[cnt],
                               EPWM_TZ_ACTION_EVENT_TZA,
                               EPWM_TZ_ACTION_LOW);
        EPWM_setTripZoneAction(obj->pwmHandle[cnt],
                               EPWM_TZ_ACTION_EVENT_TZB,
                               EPWM_TZ_ACTION_LOW);

        // Clear any high comparator digital filter output latch
        CMPSS_clearFilterLatchHigh(obj->cmpssHandle[cnt]);

        // Clear any low comparator digital filter output latch
        CMPSS_clearFilterLatchLow(obj->cmpssHandle[cnt]);

        // Clear any spurious fault
        EPWM_clearTripZoneFlag(obj->pwmHandle[cnt],
                                        HAL_TZ_INTERRUPT_ALL);
    }

    return;
} // end of HAL_setupFaults() function


void HAL_setupGate(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    DRV8320_setSPIHandle(obj->drv8320Handle, obj->spiHandle[0]);
    DRV8320_setGPIOCSNumber(obj->drv8320Handle, HAL_DRV_SPI_CS_GPIO);
    DRV8320_setGPIONumber(obj->drv8320Handle, HAL_DRV_EN_GATE_GPIO);

    return;
} // HAL_setupGate() function


void HAL_setupGPIOs(HAL_Handle handle)
{
    // EPWM1A->UH for T200 Controller
    GPIO_setMasterCore(0, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_0_EPWM1A);
    GPIO_setDirectionMode(0, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);

    // EPWM1B->UL for T200 Controller
    GPIO_setMasterCore(1, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_1_EPWM1B);
    GPIO_setDirectionMode(1, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_STD);

    // EPWM2A->WH for T200 Controller
    GPIO_setMasterCore(2, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_2_EPWM2A);
    GPIO_setDirectionMode(2, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(2, GPIO_PIN_TYPE_STD);

    // EPWM2B->WL for T200 Controller
    GPIO_setMasterCore(3, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_3_EPWM2B);
    GPIO_setDirectionMode(3, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(3, GPIO_PIN_TYPE_STD);

    // EPWM3A->WH for T200 Controller
    GPIO_setMasterCore(4, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_4_EPWM3A);
    GPIO_setDirectionMode(4, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(4, GPIO_PIN_TYPE_STD);

    // EPWM3B->WL for T200 Controller
    GPIO_setMasterCore(5, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_5_EPWM3B);
    GPIO_setDirectionMode(5, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(5, GPIO_PIN_TYPE_STD);

    // GPIO8->DRV_EN for T200 Controller
    GPIO_setMasterCore(8, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_8_GPIO8);
    GPIO_setDirectionMode(8, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(8, GPIO_PIN_TYPE_PULLUP);
    GPIO_writePin(8, 1);

    // GPIO29->nFAULT for T200 Controller
    GPIO_setMasterCore(7, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_7_GPIO7);
    GPIO_setDirectionMode(7, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(7, GPIO_PIN_TYPE_PULLUP);
    GPIO_writePin(7, 1);

    // GPIO16->SPIA-MOSI for T200 Controller
    GPIO_setMasterCore(16, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_16_SPISIMOA);
    GPIO_setDirectionMode(16, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(16, GPIO_PIN_TYPE_STD);

    // GPIO17->SPIA-MISO for T200 Controller
    GPIO_setMasterCore(17, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_17_SPISOMIA);
    GPIO_setDirectionMode(17, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(17, GPIO_PIN_TYPE_STD);

    // GPIO9->SPIA-CLK for T200 Controller
    GPIO_setMasterCore(9, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_9_SPICLKA);
    GPIO_setDirectionMode(9, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(9, GPIO_PIN_TYPE_STD);

    // GPIO27->SPIA-CS for T200 Controller
    GPIO_setMasterCore(11, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_11_SPISTEA);
    GPIO_setDirectionMode(1, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(11, GPIO_PIN_TYPE_STD);

    // GPIO6->Test LED for T200 Controller
    GPIO_setMasterCore(6, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_6_GPIO6);
    GPIO_writePin(6, 0);
    GPIO_setDirectionMode(6, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(6, GPIO_PIN_TYPE_STD);

    // TDI
    GPIO_setMasterCore(35, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_35_TDI);

    // TDO
    GPIO_setMasterCore(37, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_37_TDO);

    // CANB TX for T200 Controller (GPIO 12)
    GPIO_setMasterCore(12, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_12_CANTXB);
    GPIO_setDirectionMode(12, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(12, GPIO_PIN_TYPE_STD);

    // CANB RX for T200 Controller (GPIO 13)
    GPIO_setMasterCore(13, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_13_CANRXB);
    GPIO_setDirectionMode(13, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(13, GPIO_PIN_TYPE_STD);

    // GPIO33->Reserve (N/A)
    GPIO_setMasterCore(33, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_33_GPIO33);
    GPIO_setDirectionMode(33, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(33, GPIO_PIN_TYPE_STD);

    // GPIO22->Reserve (N/A)
    GPIO_setMasterCore(22, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_22_GPIO22);
    GPIO_setDirectionMode(22, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(22, GPIO_PIN_TYPE_STD);

    // GPIO23->Reserve (N/A)
    GPIO_setMasterCore(23, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_23_GPIO23);
    GPIO_setDirectionMode(23, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(23, GPIO_PIN_TYPE_STD);

    return;
}  // end of HAL_setupGPIOs() function

#ifdef CLA
void HAL_setupCLA(HAL_Handle handle)
{
    HAL_Obj  *obj = (HAL_Obj *)handle;
    uint32_t tmp_vec;

    // configure LS memory through LSxMSEL register to allow sharing between CPU and CLA
    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS0, MEMCFG_LSRAMMASTER_CPU_CLA1);
    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS1, MEMCFG_LSRAMMASTER_CPU_CLA1);
    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS2, MEMCFG_LSRAMMASTER_CPU_CLA1);
    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS3, MEMCFG_LSRAMMASTER_CPU_CLA1);
    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS4, MEMCFG_LSRAMMASTER_CPU_CLA1);
    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS5, MEMCFG_LSRAMMASTER_CPU_CLA1);
    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS6, MEMCFG_LSRAMMASTER_CPU_CLA1);
    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS7, MEMCFG_LSRAMMASTER_CPU_CLA1);

    // configure what memory is for CLA program through the LSxCLAPGM register
    MemCfg_setCLAMemType(MEMCFG_SECT_LS0, MEMCFG_CLA_MEM_DATA);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS1, MEMCFG_CLA_MEM_DATA);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS2, MEMCFG_CLA_MEM_PROGRAM);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS3, MEMCFG_CLA_MEM_PROGRAM);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS4, MEMCFG_CLA_MEM_PROGRAM);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS5, MEMCFG_CLA_MEM_PROGRAM);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS6, MEMCFG_CLA_MEM_PROGRAM);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS7, MEMCFG_CLA_MEM_DATA);

    tmp_vec = (uint32_t)(&task_initModules);
    CLA_mapTaskVector(obj->claHandle, CLA_MVECT_1, (uint16_t)tmp_vec);

    tmp_vec = (uint32_t)(&task_mainISR);
    CLA_mapTaskVector(obj->claHandle, CLA_MVECT_2, (uint16_t)tmp_vec);

    tmp_vec = (uint32_t)(&task_mainLoop);
    CLA_mapTaskVector(obj->claHandle, CLA_MVECT_3, (uint16_t)tmp_vec);

    tmp_vec = (uint32_t)(&cla1Task4);
    CLA_mapTaskVector(obj->claHandle, CLA_MVECT_4, (uint16_t)tmp_vec);

    tmp_vec = (uint32_t)(&cla1Task5);
    CLA_mapTaskVector(obj->claHandle, CLA_MVECT_5, (uint16_t)tmp_vec);

    tmp_vec = (uint32_t)(&cla1Task6);
    CLA_mapTaskVector(obj->claHandle, CLA_MVECT_6, (uint16_t)tmp_vec);

    tmp_vec = (uint32_t)(&cla1Task7);
    CLA_mapTaskVector(obj->claHandle, CLA_MVECT_7, (uint16_t)tmp_vec);

    tmp_vec = (uint32_t)(&cla1Task8);
    CLA_mapTaskVector(obj->claHandle, CLA_MVECT_8, (uint16_t)tmp_vec);

    CLA_enableTasks(obj->claHandle, CLA_TASKFLAG_ALL);
    CLA_clearTaskFlags(obj->claHandle, CLA_TASKFLAG_ALL);

    CLA_enableBackgroundTask(obj->claHandle);

    CLA_disableHardwareTrigger(obj->claHandle);

    tmp_vec = (uint32_t)(&cla_EST_run_BackgroundTask);
    CLA_mapBackgroundTaskVector(obj->claHandle, (uint16_t)tmp_vec);

    CLA_setTriggerSource(CLA_TASK_1, CLA_TRIGGER_SOFTWARE);
    CLA_setTriggerSource(CLA_TASK_2, CLA_TRIGGER_SOFTWARE);
    CLA_setTriggerSource(CLA_TASK_3, CLA_TRIGGER_SOFTWARE);

    return;
} // end of HAL_setupCLA() function
#endif

void HAL_setupPeripheralClks(HAL_Handle handle)
{

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CLA1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_DMA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TIMER0);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TIMER1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TIMER2);
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_HRPWM);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM3);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM4);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM5);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM6);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM7);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM8);

    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_ECAP1);
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_ECAP2);
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_ECAP3);
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_ECAP4);
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_ECAP5);
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_ECAP6);
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_ECAP7);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EQEP1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EQEP2);

    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_SD1);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SCIA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SCIB);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SPIA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SPIB);

    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_I2CA);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CANA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CANB);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCB);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCC);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS3);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS4);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS5);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS6);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS7);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_PGA1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_PGA2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_PGA3);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_PGA4);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_PGA5);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_PGA6);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_PGA7);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_DACA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_DACB);

    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_FSITXA);
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_FSIRXA);

    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_LINA);

    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_PMBUSA);

    return;
} // end of HAL_setupPeripheralClks() function

void HAL_setupPWMs(HAL_Handle handle,
                   const float32_t systemFreq_MHz,
                   const float32_t pwmPeriod_usec,
                   const uint_least16_t numPWMTicksPerISRTick)
{
    HAL_Obj   *obj = (HAL_Obj *)handle;
    uint16_t  halfPeriod_cycles = (uint16_t)(systemFreq_MHz *
                                  pwmPeriod_usec / (float32_t)2.0);
    uint16_t  cnt;

    // disable the ePWM module time base clock sync signal
    // to synchronize all of the PWMs
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    // turns off the outputs of the EPWM peripherals which will put the power
    // switches into a high impedance state.
    EPWM_forceTripZoneEvent(obj->pwmHandle[0], EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(obj->pwmHandle[1], EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(obj->pwmHandle[2], EPWM_TZ_FORCE_EVENT_OST);

    for(cnt=0;cnt<3;cnt++)
    {
        // setup the Time-Base Control Register (TBCTL)
        EPWM_setTimeBaseCounterMode(obj->pwmHandle[cnt],
                                    EPWM_COUNTER_MODE_UP_DOWN);
        EPWM_disablePhaseShiftLoad(obj->pwmHandle[cnt]);
        EPWM_setPeriodLoadMode(obj->pwmHandle[cnt], EPWM_PERIOD_DIRECT_LOAD);
        EPWM_setSyncOutPulseMode(obj->pwmHandle[cnt],
                                 EPWM_SYNC_OUT_PULSE_ON_SOFTWARE);
        EPWM_setClockPrescaler(obj->pwmHandle[cnt], EPWM_CLOCK_DIVIDER_1,
                                 EPWM_HSCLOCK_DIVIDER_1);
        EPWM_setCountModeAfterSync(obj->pwmHandle[cnt],
                                   EPWM_COUNT_MODE_UP_AFTER_SYNC);
        EPWM_setEmulationMode(obj->pwmHandle[cnt], EPWM_EMULATION_FREE_RUN);

        // setup the Timer-Based Phase Register (TBPHS)
        EPWM_setPhaseShift(obj->pwmHandle[cnt], 0);

        // setup the Time-Base Counter Register (TBCTR)
        EPWM_setTimeBaseCounter(obj->pwmHandle[cnt], 0);

        // setup the Time-Base Period Register (TBPRD)
        // set to zero initially
        EPWM_setTimeBasePeriod(obj->pwmHandle[cnt], 0);

        // setup the Counter-Compare Control Register (CMPCTL)
        EPWM_setCounterCompareShadowLoadMode(obj->pwmHandle[cnt],
                                             EPWM_COUNTER_COMPARE_A,
                                             EPWM_COMP_LOAD_ON_CNTR_ZERO);

        EPWM_disableCounterCompareShadowLoadMode(obj->pwmHandle[cnt],
                                             EPWM_COUNTER_COMPARE_B);

                                                   //
        EPWM_disableCounterCompareShadowLoadMode(obj->pwmHandle[cnt],
                                             EPWM_COUNTER_COMPARE_C);

        //
        EPWM_disableCounterCompareShadowLoadMode(obj->pwmHandle[cnt],
                                             EPWM_COUNTER_COMPARE_D);

        // setup the Action-Qualifier Output A Register (AQCTLA)
        EPWM_setActionQualifierAction(obj->pwmHandle[cnt],
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

        EPWM_setActionQualifierAction(obj->pwmHandle[cnt],
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

        // setup the Action-qualifier Continuous Software Force Register
        // (AQCSFRC)
        EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[cnt],
                                                 EPWM_AQ_OUTPUT_B,
                                                 EPWM_AQ_SW_OUTPUT_HIGH);

        // setup the Dead-Band Generator Control Register (DBCTL)
        EPWM_setDeadBandDelayMode(obj->pwmHandle[cnt], EPWM_DB_RED, true);
        EPWM_setDeadBandDelayMode(obj->pwmHandle[cnt], EPWM_DB_FED, true);

        // select EPWMA as the input to the dead band generator
        EPWM_setRisingEdgeDeadBandDelayInput(obj->pwmHandle[cnt],
                                             EPWM_DB_INPUT_EPWMA);

        // configure the right polarity for active high complementary config.
        EPWM_setDeadBandDelayPolarity(obj->pwmHandle[cnt],
                                      EPWM_DB_RED,
                                      EPWM_DB_POLARITY_ACTIVE_HIGH);
        EPWM_setDeadBandDelayPolarity(obj->pwmHandle[cnt],
                                      EPWM_DB_FED,
                                      EPWM_DB_POLARITY_ACTIVE_LOW);

        // setup the Dead-Band Rising Edge Delay Register (DBRED)
        EPWM_setRisingEdgeDelayCount(obj->pwmHandle[cnt],HAL_PWM_DBRED_CNT);

        // setup the Dead-Band Falling Edge Delay Register (DBFED)
        EPWM_setFallingEdgeDelayCount(obj->pwmHandle[cnt],HAL_PWM_DBFED_CNT);

        // setup the PWM-Chopper Control Register (PCCTL)
        EPWM_disableChopper(obj->pwmHandle[cnt]);

        // setup the Trip Zone Select Register (TZSEL)
        EPWM_disableTripZoneSignals(obj->pwmHandle[cnt],
                                    EPWM_TZ_SIGNAL_CBC1 |
                                    EPWM_TZ_SIGNAL_CBC2 |
                                    EPWM_TZ_SIGNAL_CBC3 |
                                    EPWM_TZ_SIGNAL_CBC4 |
                                    EPWM_TZ_SIGNAL_CBC5 |
                                    EPWM_TZ_SIGNAL_CBC6 |
                                    EPWM_TZ_SIGNAL_DCAEVT2 |
                                    EPWM_TZ_SIGNAL_DCBEVT2 |
                                    EPWM_TZ_SIGNAL_OSHT1 |
                                    EPWM_TZ_SIGNAL_OSHT2 |
                                    EPWM_TZ_SIGNAL_OSHT3 |
                                    EPWM_TZ_SIGNAL_OSHT4 |
                                    EPWM_TZ_SIGNAL_OSHT5 |
                                    EPWM_TZ_SIGNAL_OSHT6 |
                                    EPWM_TZ_SIGNAL_DCAEVT1 |
                                    EPWM_TZ_SIGNAL_DCBEVT1);
    }

    // setup the Event Trigger Selection Register (ETSEL)
    EPWM_disableInterrupt(obj->pwmHandle[0]);
    EPWM_setADCTriggerSource(obj->pwmHandle[0],
                             EPWM_SOC_A,
                             EPWM_SOC_TBCTR_D_CMPC);

    EPWM_enableADCTrigger(obj->pwmHandle[0], EPWM_SOC_A);

    // setup the Event Trigger Prescale Register (ETPS)
    if(numPWMTicksPerISRTick > 15)
    {
        EPWM_setInterruptEventCount(obj->pwmHandle[0], 15);
        EPWM_setADCTriggerEventPrescale(obj->pwmHandle[0], EPWM_SOC_A, 15);
    }
    else if(numPWMTicksPerISRTick < 1)
    {
        EPWM_setInterruptEventCount(obj->pwmHandle[0], 1);
        EPWM_setADCTriggerEventPrescale(obj->pwmHandle[0], EPWM_SOC_A, 1);
    }
    else
    {
        EPWM_setInterruptEventCount(obj->pwmHandle[0], numPWMTicksPerISRTick);
        EPWM_setADCTriggerEventPrescale(obj->pwmHandle[0],
                                        EPWM_SOC_A,
                                        numPWMTicksPerISRTick);
    }

    // setup the Event Trigger Clear Register (ETCLR)
    EPWM_clearEventTriggerInterruptFlag(obj->pwmHandle[0]);
    EPWM_clearADCTriggerFlag(obj->pwmHandle[0], EPWM_SOC_A);

    // since the PWM is configured as an up/down counter, the period register is
    // set to one-half of the desired PWM period
    EPWM_setTimeBasePeriod(obj->pwmHandle[0], halfPeriod_cycles);
    EPWM_setTimeBasePeriod(obj->pwmHandle[1], halfPeriod_cycles);
    EPWM_setTimeBasePeriod(obj->pwmHandle[2], halfPeriod_cycles);

      // write the PWM data value  for ADC trigger
    EPWM_setCounterCompareValue(obj->pwmHandle[0],
                                EPWM_COUNTER_COMPARE_C,
                                2);

    // enable the ePWM module time base clock sync signal
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    return;
}  // end of HAL_setupPWMs() function

void HAL_setupPWMDACs(HAL_Handle handle,
                   const float32_t systemFreq_MHz)
{
  HAL_Obj  *obj = (HAL_Obj *)handle;

  // PWMDAC frequency = 100kHz, calculate the period for pwm
  uint16_t halfPeriod_cycles = (uint16_t)(systemFreq_MHz *
                               (float32_t)(1000.0/100.0/2.0));
  uint16_t  cnt;

  // disable the ePWM module time base clock sync signal
  // to synchronize all of the PWMs
  SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

  for(cnt=0;cnt<4;cnt++)
    {
      // setup the Time-Base Control Register (TBCTL)
      EPWM_setTimeBaseCounterMode(obj->pwmDACHandle[cnt],
                                  EPWM_COUNTER_MODE_UP_DOWN);
      EPWM_disablePhaseShiftLoad(obj->pwmDACHandle[cnt]);
      EPWM_setPeriodLoadMode(obj->pwmDACHandle[cnt], EPWM_PERIOD_DIRECT_LOAD);
      EPWM_setSyncOutPulseMode(obj->pwmDACHandle[cnt],
                               EPWM_SYNC_OUT_PULSE_ON_SOFTWARE);
      EPWM_setClockPrescaler(obj->pwmDACHandle[cnt], EPWM_CLOCK_DIVIDER_1,
                               EPWM_HSCLOCK_DIVIDER_1);
      EPWM_setCountModeAfterSync(obj->pwmDACHandle[cnt],
                                 EPWM_COUNT_MODE_UP_AFTER_SYNC);
      EPWM_setEmulationMode(obj->pwmDACHandle[cnt], EPWM_EMULATION_FREE_RUN);

      // setup the Timer-Based Phase Register (TBPHS)
      EPWM_setPhaseShift(obj->pwmDACHandle[cnt], 0);

      // setup the Time-Base Counter Register (TBCTR)
      EPWM_setTimeBaseCounter(obj->pwmDACHandle[cnt], 0);

      // setup the Time-Base Period Register (TBPRD)
      // set to zero initially
      EPWM_setTimeBasePeriod(obj->pwmDACHandle[cnt], 0);

      // setup the Counter-Compare Control Register (CMPCTL)
      EPWM_setCounterCompareShadowLoadMode(obj->pwmDACHandle[cnt],
                                           EPWM_COUNTER_COMPARE_A,
                                           EPWM_COMP_LOAD_ON_CNTR_ZERO);

      // setup the Action-Qualifier Output A Register (AQCTLA)
      EPWM_setActionQualifierAction(obj->pwmDACHandle[cnt],
                                    EPWM_AQ_OUTPUT_A,
                                    EPWM_AQ_OUTPUT_HIGH,
                                    EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

      EPWM_setActionQualifierAction(obj->pwmDACHandle[cnt],
                                    EPWM_AQ_OUTPUT_A,
                                    EPWM_AQ_OUTPUT_LOW,
                                    EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);


      // setup the Action-Qualifier Output B Register (AQCTLB)
      EPWM_setActionQualifierAction(obj->pwmDACHandle[cnt],
                                    EPWM_AQ_OUTPUT_B,
                                    EPWM_AQ_OUTPUT_HIGH,
                                    EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);

      EPWM_setActionQualifierAction(obj->pwmDACHandle[cnt],
                                    EPWM_AQ_OUTPUT_B,
                                    EPWM_AQ_OUTPUT_LOW,
                                    EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);


      // setup the Dead-Band Generator Control Register (DBCTL)
      EPWM_setDeadBandDelayMode(obj->pwmDACHandle[cnt], EPWM_DB_RED, false);
      EPWM_setDeadBandDelayMode(obj->pwmDACHandle[cnt], EPWM_DB_FED, false);

      // setup the PWM-Chopper Control Register (PCCTL)
      EPWM_disableChopper(obj->pwmDACHandle[cnt]);

      // setup the Trip Zone Select Register (TZSEL)
      EPWM_disableTripZoneSignals(obj->pwmDACHandle[cnt],
                                  EPWM_TZ_SIGNAL_CBC1 |
                                  EPWM_TZ_SIGNAL_CBC2 |
                                  EPWM_TZ_SIGNAL_CBC3 |
                                  EPWM_TZ_SIGNAL_CBC4 |
                                  EPWM_TZ_SIGNAL_CBC5 |
                                  EPWM_TZ_SIGNAL_CBC6 |
                                  EPWM_TZ_SIGNAL_DCAEVT2 |
                                  EPWM_TZ_SIGNAL_DCBEVT2 |
                                  EPWM_TZ_SIGNAL_OSHT1 |
                                  EPWM_TZ_SIGNAL_OSHT2 |
                                  EPWM_TZ_SIGNAL_OSHT3 |
                                  EPWM_TZ_SIGNAL_OSHT4 |
                                  EPWM_TZ_SIGNAL_OSHT5 |
                                  EPWM_TZ_SIGNAL_OSHT6 |
                                  EPWM_TZ_SIGNAL_DCAEVT1 |
                                  EPWM_TZ_SIGNAL_DCBEVT1);

      // since the PWM is configured as an up/down counter, the period register
      // is set to one-half of the desired PWM period
      EPWM_setTimeBasePeriod(obj->pwmDACHandle[cnt], halfPeriod_cycles);
    }

  // enable the ePWM module time base clock sync signal
  SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

  return;
}  // end of HAL_setupPWMs() function


#ifdef _EQEP_EN_
void HAL_setupQEP(HAL_Handle handle, HAL_QEPSelect_e qep)
{
  HAL_Obj   *obj = (HAL_Obj *)handle;

  //
  // Configure the decoder for quadrature count mode
  //
  EQEP_setDecoderConfig(obj->qepHandle[qep], (EQEP_CONFIG_1X_RESOLUTION |
                                     EQEP_CONFIG_QUADRATURE |
                                     EQEP_CONFIG_NO_SWAP));

  EQEP_setEmulationMode(obj->qepHandle[qep], EQEP_EMULATIONMODE_RUNFREE);

  //
  // Configure the position counter to reset on an index event
  //
  EQEP_setPositionCounterConfig(obj->qepHandle[qep], EQEP_POSITION_RESET_IDX,
                                0xFFFFFFFF);

  //
  // Enable the unit timer, setting the frequency to 100 Hz
  //
  EQEP_enableUnitTimer(obj->qepHandle[qep], (DEVICE_SYSCLK_FREQ / 100));

  //
  // Configure the position counter to be latched on a unit time out
  //
  EQEP_setLatchMode(obj->qepHandle[qep], EQEP_LATCH_UNIT_TIME_OUT);

  //
  // Enable the eQEP module
  //
  EQEP_enableModule(obj->qepHandle[qep]);

  //
  // Configure and enable the edge-capture unit. The capture clock divider is
  // SYSCLKOUT/64. The unit-position event divider is QCLK/32.
  //
  EQEP_setCaptureConfig(obj->qepHandle[qep], EQEP_CAPTURE_CLK_DIV_64,
                        EQEP_UNIT_POS_EVNT_DIV_32);

  EQEP_enableCapture(obj->qepHandle[qep]);

  return;
}
#endif


void HAL_setupSCIA(HAL_Handle halHandle)
{
  HAL_Obj *obj = (HAL_Obj *)halHandle;

  // Initialize SCIA and its FIFO.
  SCI_performSoftwareReset(obj->sciHandle[0]);

  // Configure SCIA for echoback.
  SCI_setConfig(obj->sciHandle[0], DEVICE_LSPCLK_FREQ, 9600,
                                                      (SCI_CONFIG_WLEN_8 |
                                                      SCI_CONFIG_STOP_ONE |
                                                      SCI_CONFIG_PAR_NONE));
  SCI_resetChannels(obj->sciHandle[0]);

  SCI_resetRxFIFO(obj->sciHandle[0]);

  SCI_resetTxFIFO(obj->sciHandle[0]);

  SCI_clearInterruptStatus(obj->sciHandle[0], SCI_INT_TXFF | SCI_INT_RXFF);

  SCI_enableFIFO(obj->sciHandle[0]);

  SCI_enableModule(obj->sciHandle[0]);

  SCI_performSoftwareReset(obj->sciHandle[0]);

}  // end of DRV_setupSci() function


void HAL_setupSPIA(HAL_Handle handle)
{
  HAL_Obj   *obj = (HAL_Obj *)handle;

  // Must put SPI into reset before configuring it
  SPI_disableModule(obj->spiHandle[0]);

  // SPI configuration. Use a 1MHz SPICLK and 16-bit word size. 500kbps
  SPI_setConfig(obj->spiHandle[0], DEVICE_LSPCLK_FREQ, SPI_PROT_POL0PHA0,
                SPI_MODE_MASTER, 400000, 16);

  SPI_disableLoopback(obj->spiHandle[0]);

  SPI_setEmulationMode(obj->spiHandle[0], SPI_EMULATION_FREE_RUN);

  SPI_enableFIFO(obj->spiHandle[0]);

  HWREGH((obj->spiHandle[0])+SPI_O_FFCT) = 0x0018;

  SPI_clearInterruptStatus(obj->spiHandle[0], SPI_INT_TXFF);

  // Configuration complete. Enable the module.
  SPI_enableModule(obj->spiHandle[0]);

  return;
}  // end of HAL_setupSPIA() function


void HAL_setupSPIB(HAL_Handle handle)
{
  HAL_Obj   *obj = (HAL_Obj *)handle;

  // Must put SPI into reset before configuring it
  SPI_disableModule(obj->spiHandle[1]);

  // SPI configuration. Use a 1MHz SPICLK and 16-bit word size.
  SPI_setConfig(obj->spiHandle[1], DEVICE_LSPCLK_FREQ, SPI_PROT_POL0PHA0,
                SPI_MODE_MASTER, 400000, 16);

  SPI_disableLoopback(obj->spiHandle[1]);

  SPI_setEmulationMode(obj->spiHandle[1], SPI_EMULATION_FREE_RUN);

  SPI_enableFIFO(obj->spiHandle[1]);
  HWREGH((obj->spiHandle[1])+SPI_O_FFCT) = 0x0018;

  SPI_clearInterruptStatus(obj->spiHandle[1], SPI_INT_TXFF);

  // Configuration complete. Enable the module.
  SPI_enableModule(obj->spiHandle[1]);

  return;
}  // end of HAL_setupSPIB() function

void HAL_setupCANB(HAL_Handle handle)
{

   HAL_Obj   *obj = (HAL_Obj *)handle;

   // Initilize CANB
   CAN_initModule(obj->canHandle[1]);

   // Set CANB bitrate
   CAN_setBitRate(obj->canHandle[1], DEVICE_SYSCLK_FREQ, 500000, 20);

   // Disable CANB Test Mode
   CAN_disableTestMode(obj->canHandle[1]);

   //*************************************
   // Setup CAN Message Objects

   // Arm Message Object
   CAN_setupMessageObject(obj->canHandle[1], arm_id, ((board_id << 6) | arm_id),
                          CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_RX, 0,
                          CAN_MSG_OBJ_NO_FLAGS, 1);

   // Abort Message Object
   CAN_setupMessageObject(obj->canHandle[1], abort_id, ((board_id << 6) | abort_id),
                          CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_RX, 0,
                          CAN_MSG_OBJ_NO_FLAGS, 1);

   // MotorONOFF Message Object
   CAN_setupMessageObject(obj->canHandle[1], motor_onoff_id, ((board_id << 6) | motor_onoff_id),
                          CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_RX, 0,
                          CAN_MSG_OBJ_NO_FLAGS, 1);

   // SetRPM Message Object
   CAN_setupMessageObject(CANB_BASE, setRPM_id, ((board_id << 6) | setRPM_id),
                          CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_RX, 0,
                          CAN_MSG_OBJ_NO_FLAGS, 3);

   // SetAccel Message Object
   CAN_setupMessageObject(obj->canHandle[1], setAccel_id, ((board_id << 6) | setAccel_id),
                          CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_RX, 0,
                          CAN_MSG_OBJ_NO_FLAGS, 2);

   // MeasuredRPM Message Object
   CAN_setupMessageObject(obj->canHandle[1], measuredRPM_id, ((board_id << 6) | measuredRPM_id),
                          CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0,
                          CAN_MSG_OBJ_NO_FLAGS, 3);

   // MeasuredVoltage Message Object
   CAN_setupMessageObject(obj->canHandle[1], measuredVoltage_id, ((board_id << 6) | measuredVoltage_id),
                          CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0,
                          CAN_MSG_OBJ_NO_FLAGS, 2);

   // MeasuredTorque Message Object
   CAN_setupMessageObject(obj->canHandle[1], measuredTorque_id, ((board_id << 6) | measuredTorque_id),
                          CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0,
                          CAN_MSG_OBJ_NO_FLAGS, 2);

   // Board State Message Object
   CAN_setupMessageObject(obj->canHandle[1], boardState_id, ((board_id << 6) | boardState_id),
                          CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0,
                          CAN_MSG_OBJ_NO_FLAGS, 1);

   // faultStatus Message Object
   CAN_setupMessageObject(obj->canHandle[1], faultStatus_id, ((board_id << 6) | faultStatus_id),
                          CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0,
                          CAN_MSG_OBJ_NO_FLAGS, 2);


   // Start CANB module operations
   CAN_startModule(obj->canHandle[1]);

  return;
}


void HAL_setupEstTimer(HAL_Handle handle,
                       const float32_t systemFreq_MHz,
                       const float32_t estFreq_Hz)
{
  HAL_Obj  *obj = (HAL_Obj *)handle;

  uint32_t temp;

  //
  // calculate timer period value
  //
  temp = ((uint32_t)(systemFreq_MHz *1000000.0 / estFreq_Hz)) - 1;

  //
  // use CPU timer 0 for estimator ISR in variable pwm frequency project
  //
  CPUTimer_setPreScaler(obj->timerHandle[0], 0);
  CPUTimer_setEmulationMode(obj->timerHandle[0],
                            CPUTIMER_EMULATIONMODE_RUNFREE);
  CPUTimer_setPeriod(obj->timerHandle[0], temp);

  CPUTimer_enableInterrupt(obj->timerHandle[0]);

  // Starts CPU-Timer 0.
  CPUTimer_startTimer(obj->timerHandle[0]);

  return;
}  // end of HAL_setupEstTimer() function


void HAL_setupTimers(HAL_Handle handle, const float32_t systemFreq_MHz)
{
  HAL_Obj  *obj = (HAL_Obj *)handle;

  //
  // 1ms calculation
  //
  uint32_t timerPeriod_1ms = (uint32_t)(systemFreq_MHz *
                              (float32_t)1000.0) - 1;

  //
  // use CPU timer 0 for estimator ISR in variable pwm frequency project
  //
  CPUTimer_setPreScaler(obj->timerHandle[0], 0);
  CPUTimer_setEmulationMode(obj->timerHandle[0],
                            CPUTIMER_EMULATIONMODE_RUNFREE);
  CPUTimer_setPeriod(obj->timerHandle[0], timerPeriod_1ms);

  //
  // 1ms, use CPU timer 1 as a software timer for system control
  //
  CPUTimer_setPreScaler(obj->timerHandle[1], 0);
  CPUTimer_setEmulationMode(obj->timerHandle[1],
                            CPUTIMER_EMULATIONMODE_RUNFREE);
  CPUTimer_setPeriod(obj->timerHandle[1], timerPeriod_1ms);

  //
  // use CPU timer 2 for CPU usage diagnostics
  //
  CPUTimer_setPreScaler(obj->timerHandle[2], 0);
  CPUTimer_setEmulationMode(obj->timerHandle[2],
                            CPUTIMER_EMULATIONMODE_RUNFREE);
  CPUTimer_setPeriod(obj->timerHandle[2], 0xFFFFFFFF);

  return;
}  // end of HAL_setupTimers() function


void HAL_setupVSFPWMMode(HAL_Handle halHandle)
{
    HAL_Obj *halObj = (HAL_Obj *)halHandle;

    // setup the Counter-Compare Control Register (CMPCTL)
    uint16_t cnt;

    for(cnt = 0; cnt < 3; cnt++)
    {
        // setup the Counter-Compare Control Register (CMPCTL)
        EPWM_setCounterCompareShadowLoadMode(halObj->pwmHandle[cnt],
                                             EPWM_COUNTER_COMPARE_A,
                                             EPWM_COMP_LOAD_ON_CNTR_ZERO);

        EPWM_setCounterCompareShadowLoadMode(halObj->pwmHandle[cnt],
                                             EPWM_COUNTER_COMPARE_B,
                                             EPWM_COMP_LOAD_ON_CNTR_ZERO);

        EPWM_setPeriodLoadMode(halObj->pwmHandle[cnt],
                               EPWM_PERIOD_DIRECT_LOAD);
    }

    return;
}  // end of HAL_setupVSFPWMMode() function


void HAL_setPWMDACParameters(HAL_Handle handle, HAL_PWMDACData_t *pPWMDACData)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    pPWMDACData->periodMax =
            PWMDAC_getPeriod(obj->pwmDACHandle[PWMDAC_NUMBER_1]);

    pPWMDACData->offset[0] = 0.0;
    pPWMDACData->offset[1] = 0.0;
    pPWMDACData->offset[2] = 0.0;
    pPWMDACData->offset[3] = 0.0;

    pPWMDACData->gain[0] = MATH_ONE_OVER_TWO_PI;
    pPWMDACData->gain[1] = MATH_ONE_OVER_TWO_PI;
    pPWMDACData->gain[2] = MATH_ONE_OVER_TWO_PI;
    pPWMDACData->gain[3] = MATH_ONE_OVER_TWO_PI;

    return;
}   //end of HAL_setPWMDACParameters() function


void HAL_setupDlogWithDMA(HAL_Handle handle,
                          const uint16_t dmaChannel,
                          const void *dlogDestAddr,
                          const void *dlogSrcAddr)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    const void *destAddr;
    const void *srcAddr;
    destAddr = (const void *)dlogDestAddr;
    srcAddr  = (const void *)dlogSrcAddr;

    //
    // configure DMA Channel
    //
    DMA_configAddresses(obj->dmaChHandle[dmaChannel], destAddr, srcAddr);
    DMA_configBurst(obj->dmaChHandle[dmaChannel], DLOG_BURST, 2, 2);
    DMA_configTransfer(obj->dmaChHandle[dmaChannel], DLOG_TRANSFER, 1, 1);
    DMA_configMode(obj->dmaChHandle[dmaChannel],DMA_TRIGGER_SOFTWARE,
                   (DMA_CFG_ONESHOT_ENABLE +
                    DMA_CFG_CONTINUOUS_ENABLE +
                    DMA_CFG_SIZE_32BIT));
    DMA_setInterruptMode(obj->dmaChHandle[dmaChannel],DMA_INT_AT_END);
    DMA_enableTrigger(obj->dmaChHandle[dmaChannel]);
    DMA_disableInterrupt(obj->dmaChHandle[dmaChannel]);

    return;
}    //end of HAL_initDlogDMA() function


void HAL_clearDataRAM(void *pMemory, const uint16_t lengthMemory)
{
    uint16_t *pMemoryStart;
    uint16_t loopCount, loopLength;

    pMemoryStart = pMemory;
    loopLength = lengthMemory;

    for(loopCount = 0; loopCount < loopLength; loopCount++)
    {
        *(pMemoryStart + loopCount) = 0x0000;
    }

    return;
}   //end of HAL_clearDataRAM() function


//*****************************************************************************
//
// Error handling function to be called when an ASSERT is violated
//
//*****************************************************************************
void __error__(char *filename, uint32_t line)
{
    //
    // An ASSERT condition was evaluated as false. You can use the filename and
    // line parameters to determine what went wrong.
    //
    ESTOP0;
}
// end of file
