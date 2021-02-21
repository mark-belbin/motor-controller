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
#ifndef HAL_H
#define HAL_H

//! \file   solutions/boostxl_drv8320rs/f28004x/drivers/hal.h
//! \brief  Contains public interface to various functions related
//!         to the HAL object
//!


// **************************************************************************
// the includes


// platforms
#include "hal_obj.h"

#include "svgen_current.h"

//!
//!
//! \defgroup HAL HAL
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines

// CAN Info
//***********************
//Specify board ID for CAN ID differentiation (8 to 15)
#define board_id    8

//Specify function IDs

// Commands (Recieving)
#define arm_id          0
#define abort_id        1
#define motor_onoff_id  2
#define setRPM_id       3
#define setAccel_id     4

// Telemetry (Sending)
#define measuredRPM_id      5
#define measuredVoltage_id  6
#define measuredTorque_id   7
#define boardState_id       8
#define faultStatus_id      9

// CAN_ID = (board_id << 6) | function_id




//! Trip Zones all interrupt
//!
#define HAL_TZ_INTERRUPT_ALL      EPWM_TZ_INTERRUPT_DCBEVT2 \
                                 + EPWM_TZ_INTERRUPT_DCBEVT1 \
                                 + EPWM_TZ_INTERRUPT_DCAEVT2 \
                                 + EPWM_TZ_INTERRUPT_DCAEVT1 \
                                 + EPWM_TZ_INTERRUPT_OST \
                                 + EPWM_TZ_INTERRUPT_CBC

//! configure the ample window to 15 system clock cycle wide by assigning 14
//! to the ACQPS of ADCSOCxCTL Register for correct ADC operation
#define HAL_ADC_SAMPLE_WINDOW           14

//! \brief Defines the comparator number for current prection
//!
#define HAL_NUM_CMPSS_CURRENT           3

//! \brief Defines the gpio for start command
//!
#define HAL_START_CMD_GPIO              25

//! \brief Defines the gpio for stop command
//!
#define HAL_STOP_CMD_GPIO               25

//! \brief Defines the gpio for the nFAULT of Power Module device
//!
#define HAL_PM_nFAULT_GPIO              7

//! \brief Defines the gpio for the OCTW of Power Module device
//!
#define HAL_PM_nOCTW_GPIO               40

//! \brief Defines the gpio for the SPI_CS of DRV device
//!
#define HAL_DRV_SPI_CS_GPIO             11

//! \brief Defines the gpio for the enable gate of DRV device
//!
#define HAL_DRV_EN_GATE_GPIO            8

#define HAL_TZ_SIGNAL1            EPWM_TZ_SIGNAL_OSHT1
#define HAL_TZ_SIGNAL2            EPWM_TZ_SIGNAL_OSHT2

#define HAL_XBAR_INPUT1           XBAR_INPUT1
#define HAL_XBAR_INPUT2           XBAR_INPUT2

#define HAL_DCTRIPIN              EPWM_DC_COMBINATIONAL_TRIPIN7                \
                                | EPWM_DC_COMBINATIONAL_TRIPIN8                \
                                | EPWM_DC_COMBINATIONAL_TRIPIN9


//! \brief Defines the PWM deadband falling edge delay count (system clocks)
//!
#define HAL_PWM_DBFED_CNT         1     // the final deadband is set by DRV IC

//! \brief Defines the PWM deadband rising edge delay count (system clocks)
//!
#define HAL_PWM_DBRED_CNT         1     // the final deadband is set by DRV IC

//! \brief Defines the PWM deadband rising edge delay count (system clocks)
//!
#define HAL_PM_PWM_DB_CNT         10

//! \brief Defines the bypassed delay for PWM on/off noise
//!
#define HAL_PM_NOISE_WINDOW_SET      10

//! \brief Defines the function to turn LEDs off
//!
#define HAL_turnLEDOff            HAL_setGPIOHigh

//! \brief Defines the function to turn LEDs on
//!
#define HAL_turnLEDOn             HAL_setGPIOLow

//! \brief Defines the function to toggle LEDs
//!
#define HAL_toggleLED             HAL_toggleGPIO

// **************************************************************************
// the typedefs

//! \brief Enumeration for the LED numbers
//!
typedef enum
{
    HAL_GPIO_LED1 = 23,   //!< GPIO pin number for LaunchPad LED 4
    HAL_GPIO_LED2 = 34,   //!< GPIO pin number for LaunchPad LED 5
    HAL_GPIO_ISR  = 30,   //!< GPIO pin number for ISR Executing Time
    HAL_GPIO_BML  = 31,    //!< GPIO pin number for Main Loop Executing Time
    HAL_GPIO_TESTLED = 6   //!< GPIO pin number for T200 Controller Test LED
} HAL_LEDNumber_e;


//! \brief Enumeration for the sensor types
//!
typedef enum
{
    HAL_SENSORTYPE_CURRENT = 0,  //!< Enumeration for current sensor
    HAL_SENSORTYPE_VOLTAGE = 1   //!< Enumeration for voltage sensor
} HAL_SensorType_e;


//! \brief Enumeration for the QEP setup
//!
typedef enum
{
    HAL_QEP_QEP1=0,  //!< Select QEP1
    HAL_QEP_QEP2=1   //!< Select QEP2
} HAL_QEPSelect_e;


//! \brief Enumeration for the CPU Timer
//!
typedef enum
{
    HAL_CPU_TIMER0 = 0,  //!< Select CPU Timer0
    HAL_CPU_TIMER1 = 1,  //!< Select CPU Timer1
    HAL_CPU_TIMER2 = 2,  //!< Select CPU Timer2
} HAL_CPUTimerNum_e;


// **************************************************************************
// the globals

extern __interrupt void mainISR(void);
extern __interrupt void estISR(void);

// CLA Tasks
extern __interrupt void task_initModules(void);
extern __interrupt void task_mainISR(void);
extern __interrupt void task_mainLoop(void);
extern __interrupt void cla1Task4(void);
extern __interrupt void cla1Task5(void);
extern __interrupt void cla1Task6(void);
extern __interrupt void cla1Task7(void);
extern __interrupt void cla1Task8(void);
extern __interrupt void cla_EST_run_BackgroundTask(void);


// **************************************************************************
// the function prototypes

//! \brief     Acknowledges an interrupt from the ADC so that another ADC
//!            interrupt can happen again.
//! \param[in] handle     The hardware abstraction layer (HAL) handle
//! \param[in] adcIntNum  The interrupt number
static inline void
HAL_ackADCInt(HAL_Handle handle,const ADC_IntNumber adcIntNum)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    // clear the ADC interrupt flag
    ADC_clearInterruptStatus(obj->adcHandle[1], adcIntNum);         // ADCB/RB2

    // Acknowledge interrupt from PIE group 10
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP10);

    return;
} // end of HAL_ackADCInt() function


//! \brief     Acknowledges an interrupt that is set for estimator
//! \param[in] handle     The hardware abstraction layer (HAL) handle
static inline void HAL_ackEstInt(HAL_Handle handle)
{
    // Acknowledge this interrupt to receive more interrupts from group 1
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);

    return;
} // end of HAL_ackEstInt() function


//! \brief      Sets up the ADCs (Analog to Digital Converters)
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_runADCZeroOffsetCalibration(const uint32_t base);


//! \brief      Executes calibration routines
//! \details    Values for offset and gain are programmed into OTP memory at
//!             the TI factory.  This calls and internal function that programs
//!             these offsets and gains into the ADC registers.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_cal(HAL_Handle handle);


//! \brief      Forces a CLA task
//! \param[in]  handle     The hardware abstraction layer (HAL) handle
//! \param[in]  taskFlags  The task to be forced
static inline void
HAL_forceCLATasks(HAL_Handle handle, const uint16_t taskFlags)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    // force task
    CLA_forceTasks(obj->claHandle, taskFlags);

    return;
} // end of HAL_forceCLATasks() function


//! \brief      Gets the run status of the specified CLA task
//! \param[in]  handle      The hardware abstraction layer (HAL) handle
//! \param[in]  taskNumber  The task to check the run status on
static inline bool
HAL_getCLATaskRunStatus(HAL_Handle handle, const CLA_TaskNumber taskNumber)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    // return the run status of the specified task
    return(CLA_getTaskRunStatus(obj->claHandle, taskNumber));
} // end of HAL_getCLATaskRunStatus() function


//! \brief     Sets up the timers
//! \param[in] handle          The hardware abstraction layer (HAL) handle
//! \param[in] cpuTimerNumber  The CPU timer number
static inline void
HAL_clearTimerFlag(HAL_Handle halHandle, const uint16_t cpuTimerNumber)
{
    HAL_Obj   *obj = (HAL_Obj *)halHandle;

    CPUTimer_clearOverflowFlag(obj->timerHandle[cpuTimerNumber]);

    return;
}   // end of HAL_clearTimerFlag() function


//! \brief      Disables global interrupts
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_disableGlobalInts(HAL_Handle handle);


//! \brief      Enables the ADC interrupts
//! \details    Enables the ADC interrupt in the PIE, and CPU.  Enables the
//!             interrupt to be sent from the ADC peripheral.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_enableADCInts(HAL_Handle handle);


//! \brief      Enables the ADC interrupts without CPU interrupts
//! \details    Enables the ADC interrupts to only trigger CLA, and without
//!             interrupting the CPU
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_enableADCIntsToTriggerCLA(HAL_Handle handle);


//! \brief      Enables the debug interrupt
//! \details    The debug interrupt is used for the real-time debugger.  It is
//!             not needed if the real-time debugger is not used.  Clears
//!             bit 1 of ST1.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_enableDebugInt(HAL_Handle handle);


//! \brief      Enables the 8320 device
//! \details    Provides the correct timing to enable the drv8320
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_enableDRV(HAL_Handle handle);


//! \brief     Enables global interrupts
//! \param[in] handle  The hardware abstraction layer (HAL) handle
static inline void HAL_enableGlobalInts(HAL_Handle handle)
{

    // enable global interrupts
    Interrupt_enableMaster();

    return;
} // end of HAL_enableGlobalInts() function


//! \brief      Forces a CLA task
//! \param[in]  handle     The hardware abstraction layer (HAL) handle
//! \param[in]  taskFlags  The task to be forced
static inline void
HAL_CLA_forceTasks(HAL_Handle handle, const uint16_t taskFlags)
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  // force task
  CLA_forceTasks(obj->claHandle, taskFlags);

  return;
} // end of HAL_CLA_forceTasks() function

//! \brief      Gets the run status of the specified CLA task
//! \param[in]  handle      The hardware abstraction layer (HAL) handle
//! \param[in]  taskNumber  The task to check the run status on
static inline bool
HAL_CLA_getTaskRunStatus(HAL_Handle handle, const CLA_TaskNumber taskNumber)
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  // return the run status of the specified task
  return(CLA_getTaskRunStatus(obj->claHandle, taskNumber));
} // end of HAL_CLA_getTaskRunStatus() function

//! \brief     Force trig the DMA channel for datalog
//! \param[in] handle          The hardware abstraction layer (HAL) handle
//! \param[in] dmaChNumber     The DMC Channel Number
static inline void
HAL_trigDlogWithDMA(HAL_Handle handle, const uint16_t dmaChannel)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    DMA_startChannel(obj->dmaChHandle[dmaChannel]);
    DMA_forceTrigger(obj->dmaChHandle[dmaChannel]);

    return;
} // end of HAL_trigDlogWithDMA() function


//! \brief     Gets the current scale factor
//! \param[in] handle  The hardware abstraction layer (HAL) handle
//! \return    The current scale factor
static inline float32_t HAL_getCurrentScaleFactor(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    return(obj->current_sf);
} // end of HAL_getCurrentScaleFactor() function

//! \brief     Gets the PWM duty cycle times
//! \param[in] handle       The hardware abstraction layer (HAL) handle
//! \param[in] pDutyCycles  A pointer to memory for the duty cycle durations
static inline void
HAL_getDutyCycles(HAL_Handle handle, uint16_t *pDutyCycles)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    pDutyCycles[0] = EPWM_getCounterCompareValue(obj->pwmHandle[0],
                                                 EPWM_COUNTER_COMPARE_A);
    pDutyCycles[1] = EPWM_getCounterCompareValue(obj->pwmHandle[1],
                                                 EPWM_COUNTER_COMPARE_A);
    pDutyCycles[2] = EPWM_getCounterCompareValue(obj->pwmHandle[2],
                                                 EPWM_COUNTER_COMPARE_A);
    return;
} // end of HAL_getDutyCycles() function


//! \brief     Gets the number of current sensors
//! \param[in] handle  The hardware abstraction layer (HAL) handle
//! \return    The number of current sensors
static inline uint16_t HAL_getNumCurrentSensors(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;


    return(obj->numCurrentSensors);
} // end of HAL_getNumCurrentSensors() function


//! \brief     Gets the number of voltage sensors
//! \param[in] handle  The hardware abstraction layer (HAL) handle
//! \return    The number of voltage sensors
static inline uint16_t HAL_getNumVoltageSensors(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;


    return(obj->numVoltageSensors);
} // end of HAL_getNumVoltageSensors() function


//! \brief     Gets the pwm enable status
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
//! \return    The pwm enable
static inline bool HAL_getPwmEnableStatus(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    return(obj->flagEnablePWM);
} // end of HAL_getPwmStatus() function


//! \brief     Sets up the timers
//! \param[in] handle          The hardware abstraction layer (HAL) handle
//! \param[in] cpuTimerNumber  The CPU timer number
static inline bool HAL_getTimerStatus(HAL_Handle halHandle,
                                      const uint16_t cpuTimerNumber)
{
    HAL_Obj   *obj = (HAL_Obj *)halHandle;

    return(CPUTimer_getTimerOverflowStatus(obj->timerHandle[cpuTimerNumber]));
}   // end of HAL_getTimerStatus() function


//! \brief     Gets the voltage scale factor
//! \param[in] handle  The hardware abstraction layer (HAL) handle
//! \return    The voltage scale factor
static inline float32_t HAL_getVoltageScaleFactor(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    return(obj->voltage_sf);
} // end of HAL_getVoltageScaleFactor() function


//! \brief      Configures the fault protection logic
//! \details    Sets up the trip zone inputs so that when a comparator
//!             signal from outside the micro-controller trips a fault,
//!             the EPWM peripheral blocks will force the
//!             power switches into a high impedance state.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupFaults(HAL_Handle handle);


//! \brief      Initializes the hardware abstraction layer (HAL) object
//! \details    Initializes all handles to the microcontroller peripherals.
//!             Returns a handle to the HAL object.
//! \param[in]  pMemory   A pointer to the memory for the HAL object
//! \param[in]  numBytes  The number of bytes allocated for the HAL object
//! \return     The hardware abstraction layer (HAL) object handle
extern HAL_Handle HAL_init(void *pMemory, const size_t numBytes);


//! \brief      Initializes the interrupt vector table
//! \details    Points the ISR to the function mainISR.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
static inline void HAL_initIntVectorTable(HAL_Handle handle)
 {

Interrupt_register(INT_ADCB2, &mainISR); // For T200 Controller

#ifdef _VSF_EN_
    Interrupt_register(INT_TIMER0, &estISR);
#endif  // _VSF_EN_

    return;
 } // end of HAL_initIntVectorTable() function


//! \brief      Reads the ADC data
//! \details    Reads in the ADC result registers and scales the values
//!             according to the settings in user.h.  The structure gAdcData
//!             holds three phase voltages, three line currents, and one DC bus
//!             voltage.
//! \param[in]  handle    The hardware abstraction layer (HAL) handle
//! \param[in]  pADCData  A pointer to the ADC data buffer
static inline void
HAL_readADCDataWithOffsets(HAL_Handle handle, HAL_ADCData_t *pADCData)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    float32_t value;

    float32_t current_sf = -HAL_getCurrentScaleFactor(handle);
    float32_t voltage_sf = HAL_getVoltageScaleFactor(handle);

    // For T200 Controller

    // convert phase A current PGA1       ->RA0/A11
    value = (float32_t)ADC_readResult(obj->adcResult[0], ADC_SOC_NUMBER0);
    pADCData->I_A.value[0] = value * current_sf;

    // convert phase B current PGA2       ->RB0/B9
    value = (float32_t)ADC_readResult(obj->adcResult[1], ADC_SOC_NUMBER0);
    pADCData->I_A.value[1] = value * current_sf;

    // convert phase C current PGA3       ->RC0/C7
    value = (float32_t)ADC_readResult(obj->adcResult[2], ADC_SOC_NUMBER0);
    pADCData->I_A.value[2] = value * current_sf;

    // convert phase A voltage        ->RA1/A2
    value = (float32_t)ADC_readResult(obj->adcResult[0], ADC_SOC_NUMBER1);
    pADCData->V_V.value[0] = value * voltage_sf;

    // convert phase B voltage        ->RB1/B3
    value = (float32_t)ADC_readResult(obj->adcResult[1], ADC_SOC_NUMBER1);
    pADCData->V_V.value[1] = value * voltage_sf;

    // convert phase C voltage        ->RC1/C3
    value = (float32_t)ADC_readResult(obj->adcResult[2], ADC_SOC_NUMBER1);
    pADCData->V_V.value[2] = value * voltage_sf;

    // convert dcBus voltage          ->RB2/B2
    value = (float32_t)ADC_readResult(obj->adcResult[1], ADC_SOC_NUMBER2);
    pADCData->dcBus_V = value * voltage_sf;


    return;
} // end of HAL_readADCDataWithOffsets() function


//! \brief      Reads the ADC data
//! \details    Reads in the ADC result registers and scales the values
//!             according to the settings in user.h.  The structure gAdcData
//!             holds three phase voltages, three line currents, and one DC bus
//!             voltage.
//! \param[in]  handle    The hardware abstraction layer (HAL) handle
//! \param[in]  pADCData  A pointer to the ADC data buffer
static inline void
HAL_readADCDataWithoutOffsets(HAL_Handle handle, HAL_ADCData_t *pADCData)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    float32_t value;

    float32_t current_sf = -HAL_getCurrentScaleFactor(handle);
    float32_t voltage_sf = HAL_getVoltageScaleFactor(handle);

    // convert phase A current
    value = (float32_t)ADC_readResult(obj->adcResult[1], ADC_SOC_NUMBER0);
    pADCData->I_A.value[0] = value * current_sf;

    // convert phase B current
    value = (float32_t)ADC_readResult(obj->adcResult[2], ADC_SOC_NUMBER1);
    pADCData->I_A.value[1] = value * current_sf;

    // convert phase C current
    value = (float32_t)ADC_readResult(obj->adcResult[0], ADC_SOC_NUMBER2);
    pADCData->I_A.value[2] = value * current_sf;

    // convert phase A voltage
    value = (float32_t)ADC_readResult(obj->adcResult[2], ADC_SOC_NUMBER3);
    pADCData->V_V.value[0] = value * voltage_sf;

    // convert phase B voltage
    value = (float32_t)ADC_readResult(obj->adcResult[0], ADC_SOC_NUMBER4);
    pADCData->V_V.value[1] = value * voltage_sf;

    // convert phase C voltage
    value = (float32_t)ADC_readResult(obj->adcResult[1], ADC_SOC_NUMBER5);
    pADCData->V_V.value[2] = value * voltage_sf;

    // convert dcBus voltage
    value = (float32_t)ADC_readResult(obj->adcResult[1], ADC_SOC_NUMBER6);
    pADCData->dcBus_V = value * voltage_sf;

    return;
} // end of HAL_readADCDataWithOffsets() function


//! \brief     Reads the timer count
//! \param[in] handle       The hardware abstraction layer (HAL) handle
//! \param[in] timerNumber  The timer number, 0,1 or 2
//! \return    The timer count
static inline uint32_t
HAL_readTimerCnt(HAL_Handle handle, const HAL_CPUTimerNum_e timerNumber)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    return(CPUTimer_getTimerCount(obj->timerHandle[timerNumber]));
} // end of HAL_readTimerCnt() function


//! \brief     Sets the GPIO pin high
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \param[in] gpioNumber  The GPIO number
static inline void
HAL_setGPIOHigh(HAL_Handle handle, const uint32_t gpioNumber)
{
    // set GPIO high
    GPIO_writePin(gpioNumber, 1);

    return;
} // end of HAL_setGPIOHigh() function


//! \brief     Read the GPIO pin
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \param[in] gpioNumber  The GPIO number
//! \return    The GPIO pin
static inline uint32_t
HAL_readGPIOData(HAL_Handle handle, const uint32_t gpioNumber)
{
    // read GPIO data
    return(GPIO_readPin(gpioNumber));
} // end of HAL_readGPIOData() function


//! \brief     Sets the GPIO pin low
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \param[in] gpioNumber  The GPIO number
static inline void
HAL_setGPIOLow(HAL_Handle handle, const uint32_t gpioNumber)
{
    // set GPIO low
    GPIO_writePin(gpioNumber, 0);

    return;
} // end of HAL_setGPIOLow() function


//! \brief     Sets the DAC Shadow Output Value
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \param[in] dacNumber   The DAC number
//! \param[in] dacValue    The DAC value
static inline void HAL_setDACValue(HAL_Handle handle,
                                   const uint16_t dacNumber,
                                   const uint16_t dacValue)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    // set DAC Shadow Output Value
    DAC_setShadowValue(obj->dacHandle[dacNumber], dacValue);

    return;
} // end of HAL_setDACValue() function


//! \brief     Sets the value of the internal DAC of the high comparator
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \param[in] cmpssNumber The CMPSS number
//! \param[in] dacValue    The DAC value of the high comparator
static inline void HAL_setCMPSSDACValueHigh(HAL_Handle handle,
                                            const uint16_t cmpssNumber,
                                            const uint16_t dacValue)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    // set GPIO low
    CMPSS_setDACValueHigh(obj->cmpssHandle[cmpssNumber], dacValue);

    return;
} // end of HAL_setCMPSSDACValueHigh() function


//! \brief     Sets the value of the internal DAC of the low comparator
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \param[in] cmpssNumber The CMPSS number
//! \param[in] dacValue    The DAC value of the low comparator
static inline void HAL_setCMPSSDACValueLow(HAL_Handle handle,
                                           const uint16_t cmpssNumber,
                                           const uint16_t dacValue)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    // set GPIO low
    CMPSS_setDACValueLow(obj->cmpssHandle[cmpssNumber], dacValue);

    return;
} // end of HAL_setCMPSSDACValueLow() function


//! \brief     Sets the number of voltage sensors
//! \param[in] handle             The hardware abstraction layer (HAL) handle
//! \param[in] numVoltageSensors  The number of voltage sensors
static inline void
HAL_setNumVoltageSensors(HAL_Handle handle, const uint16_t numVoltageSensors)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    obj->numVoltageSensors = numVoltageSensors;

    return;
} // end of HAL_setNumVoltageSensors() function

//! \brief     Sets the number of current sensors
//! \param[in] handle             The hardware abstraction layer (HAL) handle
//! \param[in] numCurrentSensors  The number of current sensors
static inline void
HAL_setNumCurrentSensors(HAL_Handle handle, const uint16_t numCurrentSensors)
{
    HAL_Obj *obj = (HAL_Obj *)handle;


    obj->numCurrentSensors = numCurrentSensors;

    return;
} // end of HAL_setNumCurrentSensors() function


//! \brief      Sets the hardware abstraction layer parameters
//! \details    Sets up the microcontroller peripherals.  Creates all of the
//!             scale factors for the ADC voltage and current conversions. Sets
//!             the initial offset values for voltage and current measurements.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_setParams(HAL_Handle handle);


//! \brief      Sets up the ADCs (Analog to Digital Converters)
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupADCs(HAL_Handle handle);


//! \brief      Sets up the PGAs (Programmable Gain Amplifiers)
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupPGAs(HAL_Handle handle);


//! \brief      Sets up the CMPSSs (Comparator Subsystems)
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupCMPSSs(HAL_Handle handle);


//! \brief      Sets up the DACs (Buffered Digital-to-Analog Converter)
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupDACs(HAL_Handle handle);


//! \brief      Sets up the clocks
//! \details    Sets up the micro-controller's main oscillator
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupClks(HAL_Handle handle);


//! \brief     Sets up the faults
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupFaults(HAL_Handle handle);


//! \brief     Sets up the GATE object
//! \param[in] handle       The hardware abstraction layer (HAL) handle
extern void HAL_setupGate(HAL_Handle handle);


//! \brief     Sets up the GPIO (General Purpose I/O) pins
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupGPIOs(HAL_Handle handle);


//! \brief     Sets up the FLASH.
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupFlash(HAL_Handle handle);


//! \brief     Sets up the CLA
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupCLA(HAL_Handle handle);


//! \brief     Sets up the peripheral clocks
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupPeripheralClks(HAL_Handle handle);


//! \brief     Sets up the PIE (Peripheral Interrupt Expansion)
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupPIE(HAL_Handle handle);


//! \brief     Sets up the PWMs (Pulse Width Modulators)
//! \param[in] handle          The hardware abstraction layer (HAL) handle
//! \param[in] systemFreq_MHz  The system frequency, MHz
extern void HAL_setupPWMDACs(HAL_Handle handle,
                   const float32_t systemFreq_MHz);


//! \brief     Sets up the QEP peripheral
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupQEP(HAL_Handle handle,HAL_QEPSelect_e qep);


//! \brief     Sets up the SCIA
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupSCIA(HAL_Handle handle);


//! \brief     Sets up the SPIA
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupSPIA(HAL_Handle handle);


//! \brief     Sets up the SPIB
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupSPIB(HAL_Handle handle);

//! \brief     Sets up the CANB
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupCANB(HAL_Handle handle);


//! \brief     Set the timer for estimator
//! \param[in] handle    The hardware abstraction layer (HAL) handle
//! \param[in] handle          The hardware abstraction layer (HAL) handle
//! \param[in] systemFreq_MHz  The system frequency, MHz
//! \param[in] estFreq_Hz      The estimator frequency, Hz
void HAL_setupEstTimer(HAL_Handle handle,
                       const float32_t systemFreq_MHz,
                       const float32_t estFreq_Hz);

//! \brief     Sets up the timers
//! \param[in] handle          The hardware abstraction layer (HAL) handle
//! \param[in] systemFreq_MHz  The system frequency, MHz
extern void HAL_setupTimers(HAL_Handle handle, const float32_t systemFreq_MHz);


//! \brief     Sets up the DMA for datalog
//! \param[in] handle          The hardware abstraction layer (HAL) handle
//! \param[in] dmaChNumber     The DMC Channel Number
//! \param[in] dlogDestAddr    The Datalog buffer dest address
//! \param[in] dlogSrcAddr     The Datalog buffer src address
void HAL_setupDlogWithDMA(HAL_Handle handle, const uint16_t dmaChNumber,
                          const void *dlogDestAddr, const void *dlogSrcAddr);

//! \brief     reset the DMA for datalog
static inline void HAL_resetDlogWithDMA(void)
{
    DMA_initController();

    return;
}


//! \brief     Toggles the GPIO pin
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \param[in] gpioNumber  The GPIO number
static inline void HAL_toggleGPIO(HAL_Handle handle, const uint32_t gpioNumber)
{
    // toggle GPIO
    GPIO_togglePin(gpioNumber);

    return;
} // end of HAL_toggleGPIO() function


//! \brief     Writes DAC data to the PWM comparators for DAC output
//! \param[in] handle    The hardware abstraction layer (HAL) handle
//! \param[in] pPWMDACData  The pointer to the DAC data
static inline void
HAL_writePWMDACData(HAL_Handle handle, HAL_PWMDACData_t *pPWMDACData)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    uint16_t cnt;
    float32_t period;
    float32_t dacData_sat_dc;
    float32_t dacData;
    float32_t value;

    period = (float32_t)pPWMDACData->periodMax;

    for(cnt=0;cnt<4;cnt++)
    {
        dacData = (*pPWMDACData->ptrData[cnt]);
        dacData_sat_dc = dacData * pPWMDACData->gain[cnt] +
                pPWMDACData->offset[cnt];

        value = dacData_sat_dc * period;

        pPWMDACData->cmpValue[cnt] = (int16_t)MATH_sat(value, period, 0);
    }

    // write the DAC data
    if(obj->pwmDACHandle[PWMDAC_NUMBER_1])
    {
        // write the PWM data value
        EPWM_setCounterCompareValue(obj->pwmDACHandle[PWMDAC_NUMBER_1],
                                    EPWM_COUNTER_COMPARE_A,
                                    pPWMDACData->cmpValue[0]);
    }

    if(obj->pwmDACHandle[PWMDAC_NUMBER_2])
    {
        // write the PWM data value
        EPWM_setCounterCompareValue(obj->pwmDACHandle[PWMDAC_NUMBER_2],
                                    EPWM_COUNTER_COMPARE_B,
                                    pPWMDACData->cmpValue[1]);
    }

    if(obj->pwmDACHandle[PWMDAC_NUMBER_3])
    {
        // write the PWM data value
        EPWM_setCounterCompareValue(obj->pwmDACHandle[PWMDAC_NUMBER_3],
                                    EPWM_COUNTER_COMPARE_A,
                                    pPWMDACData->cmpValue[2]);
    }


    if(obj->pwmDACHandle[PWMDAC_NUMBER_4])
    {
        // write the PWM data value
        EPWM_setCounterCompareValue(obj->pwmDACHandle[PWMDAC_NUMBER_4],
                                    EPWM_COUNTER_COMPARE_A,
                                    pPWMDACData->cmpValue[3]);
    }

    return;
} // end of HAL_writeDACData() function


//! \brief     Writes DAC data to the PWM comparators for DAC output
//! \param[in] handle    The hardware abstraction layer (HAL) handle
//! \param[in] pPWMDACData  The pointer to the DAC data
void HAL_setPWMDACParameters(HAL_Handle handle, HAL_PWMDACData_t *pPWMDACData);


//! \brief      Clear the assigned RAM memory to zero
//! \param[in]  pMemory       The pointer of RAM memory address
//! \param[in]  lengthMemory  The size of RAM memory
void HAL_clearDataRAM(void *pMemory, const uint16_t lengthMemory);


//! \brief     Reads PWM period register
//! \param[in] handle     The hardware abstraction layer (HAL) handle
//! \param[in] pwmNumber  The PWM number
//! \return    The PWM period value
static inline uint16_t
HAL_readPWMPeriod(HAL_Handle handle, const uint16_t pwmNumber)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    // the period value to be returned
    return(EPWM_getTimeBasePeriod(obj->pwmHandle[pwmNumber]));
} // end of HAL_readPWMPeriod() function


//! \brief     Writes PWM data to the PWM comparators for motor control
//! \param[in] handle    The hardware abstraction layer (HAL) handle
//! \param[in] pPWMData  The pointer to the PWM data
static inline void
HAL_writePWMData(HAL_Handle handle, HAL_PWMData_t *pPWMData)
{
    HAL_Obj *obj = (HAL_Obj *)handle;
    uint16_t pwmCnt;

    for(pwmCnt=0;pwmCnt<3;pwmCnt++)
    {
        // compute the value
        float32_t period =
                (float32_t)(EPWM_getTimeBasePeriod(obj->pwmHandle[pwmCnt]));

        float32_t V_pu = -pPWMData->Vabc_pu.value[pwmCnt];
        float32_t V_sat_pu = MATH_sat(V_pu, 0.5, -0.5);
        float32_t V_sat_dc_pu = V_sat_pu + 0.5;
        int16_t pwmValue  = (int16_t)(V_sat_dc_pu * period);

        // Save current CMP value for OVM
        pPWMData->cmpValue[pwmCnt] = pwmValue;

        // write the PWM data value
        EPWM_setCounterCompareValue(obj->pwmHandle[pwmCnt],
                                    EPWM_COUNTER_COMPARE_A,
                                    pwmValue);
    }

    // write the PWM data value  for ADC trigger
    EPWM_setCounterCompareValue(obj->pwmHandle[0],
                                EPWM_COUNTER_COMPARE_C,
                                1);
    return;
} // end of HAL_writePWMData() function

//! \brief     Set the variable pwm frequency epwm reload mode
//! \param[in] halHandle The hardware abstraction layer (HAL) handle
extern void HAL_setupVSFPWMMode(HAL_Handle handle);


//! \brief     Writes PWM data to the PWM comparator & Period for motor control
//! \param[in] handle    The hardware abstraction layer (HAL) handle
//! \param[in] pPWMData  The pointer to the PWM data
static inline void
HAL_writePWMAllData(HAL_Handle handle, HAL_PWMData_t *pPWMData)
{
    HAL_Obj *obj = (HAL_Obj *)handle;
    uint16_t pwmCnt;

    for(pwmCnt=0;pwmCnt<3;pwmCnt++)
    {
        // compute the value
        float32_t period = (float32_t)pPWMData->period;

        float32_t V_pu = -pPWMData->Vabc_pu.value[pwmCnt];
        float32_t V_sat_pu = MATH_sat(V_pu, 0.5, -0.5);
        float32_t V_sat_dc_pu = V_sat_pu + 0.5;
        int16_t pwmValue  = (int16_t)(V_sat_dc_pu * period);

        // Save current CMP value for OVM
        pPWMData->cmpValue[pwmCnt] = pwmValue;

        // Write the Time-Base Period Register (TBPRD)
        EPWM_setTimeBasePeriod(obj->pwmHandle[pwmCnt], pPWMData->period);

        // write the PWM data value
        EPWM_setCounterCompareValue(obj->pwmHandle[pwmCnt],
                                    EPWM_COUNTER_COMPARE_A,
                                    pwmValue);
    }

    // write the PWM data value  for ADC trigger
    EPWM_setCounterCompareValue(obj->pwmHandle[0],
                                EPWM_COUNTER_COMPARE_C,
                                1);

    return;
} // end of HAL_writePWMData() function

//! \brief      Enables the PWM devices
//! \details    Turns on the outputs of the EPWM peripheral which will allow
//!             the power switches to be controlled.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
static inline void HAL_enablePWM(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    // Clear comparator digital filter output latch
    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[0]);
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[0]);

    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[1]);
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[1]);

    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[2]);
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[2]);

    EPWM_clearTripZoneFlag(obj->pwmHandle[0], HAL_TZ_INTERRUPT_ALL);
    EPWM_clearTripZoneFlag(obj->pwmHandle[1], HAL_TZ_INTERRUPT_ALL);
    EPWM_clearTripZoneFlag(obj->pwmHandle[2], HAL_TZ_INTERRUPT_ALL);

    obj->flagEnablePWM = true;

    return;
} // end of HAL_enablePWM() function

//! \brief      Disables the PWM device
//! \details    Turns off the outputs of the EPWM peripherals which will put
//!             the power switches into a high impedance state.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
static inline void HAL_disablePWM(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    EPWM_forceTripZoneEvent(obj->pwmHandle[0], EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(obj->pwmHandle[1], EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(obj->pwmHandle[2], EPWM_TZ_FORCE_EVENT_OST);

    obj->flagEnablePWM = false;


    return;
} // end of HAL_disablePWM() function


//! \brief     Sets up the PWMs (Pulse Width Modulators)
//! \param[in] handle          The hardware abstraction layer (HAL) handle
//! \param[in] systemFreq_MHz  The system frequency, MHz
//! \param[in] pwmPeriod_usec  The PWM period, usec
//! \param[in] numPWMTicksPerISRTick The number of PWM clock ticks per ISR clock tick
extern void HAL_setupPWMs(HAL_Handle handle,
                          const float32_t systemFreq_MHz,
                          const float32_t pwmPeriod_usec,
                          const uint16_t numPWMTicksPerISRTick);

//! \brief     Writes data to the driver
//! \param[in] handle         The hardware abstraction layer (HAL) handle
//! \param[in] drv8320SPIVars  SPI variables
void HAL_writeDRVData(HAL_Handle handle, DRV8320_SPIVars_t *drv8320SPIVars);


//! \brief     Reads data from the driver
//! \param[in] handle         The hardware abstraction layer (HAL) handle
//! \param[in] drv8320SPIVars  SPI variables
void HAL_readDRVData(HAL_Handle handle, DRV8320_SPIVars_t *drv8320SPIVars);


//! \brief     Sets up the SPI interface for the driver
//! \param[in] handle         The hardware abstraction layer (HAL) handle
//! \param[in] drv8320SPIVars  SPI variables
extern void
HAL_setupDRVSPI(HAL_Handle handle, DRV8320_SPIVars_t *drv8320SPIVars);


//! \brief     Sets the current scale factor in the hal
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \param[in] current_sf  The current scale factor
static inline void
HAL_setCurrentScaleFactor(HAL_Handle handle, const float32_t current_sf)
{
  HAL_Obj *obj = (HAL_Obj *)handle;


  obj->current_sf = current_sf;

  return;
} // end of HAL_setCurrentScaleFactor() function


//! \brief     Sets the voltage scale factor in the hal
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \param[in] voltage_sf  The voltage scale factor
static inline void
HAL_setVoltageScaleFactor(HAL_Handle handle,const float32_t voltage_sf)
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  obj->voltage_sf = voltage_sf;

  return;
} // end of HAL_setVoltageScaleFactor() function


static inline uint16_t HAL_getTripFaults(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;
    uint16_t tripFault = 0;

    tripFault = (EPWM_getTripZoneFlagStatus(obj->pwmHandle[0]) &
            (EPWM_TZ_FLAG_OST | EPWM_TZ_FLAG_DCAEVT1 | EPWM_TZ_FLAG_DCAEVT2)) |
                (EPWM_getTripZoneFlagStatus(obj->pwmHandle[1]) &
            (EPWM_TZ_FLAG_OST | EPWM_TZ_FLAG_DCAEVT1 | EPWM_TZ_FLAG_DCAEVT2)) |
                (EPWM_getTripZoneFlagStatus(obj->pwmHandle[2]) &
             (EPWM_TZ_FLAG_OST | EPWM_TZ_FLAG_DCAEVT1 | EPWM_TZ_FLAG_DCAEVT2));

    return(tripFault);
}


//! \brief     Set the parameters for overmodulation function
//! \param[in] handle    The hardware abstraction layer (HAL) handle
//! \param[in] pPWMData  The pointer to the PWM data
static inline void HAL_setOvmParams(HAL_Handle handle, HAL_PWMData_t *pPWMData)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    pPWMData->period = EPWM_getTimeBasePeriod(obj->pwmHandle[0]);

    pPWMData->deadband[0] = HAL_PM_PWM_DB_CNT;
    pPWMData->deadband[1] = HAL_PM_PWM_DB_CNT;
    pPWMData->deadband[2] = HAL_PM_PWM_DB_CNT;

    pPWMData->noiseWindow = HAL_PM_NOISE_WINDOW_SET;

    return;
}


//! \brief     Set trigger point in the middle of the low side pulse
//! \param[in] handle    The hardware abstraction layer (HAL) handle
//! \param[in] ignoreShunt  The low side shunt that should be ignored
//! \param[in] midVolShunt  The middle length of output voltage
static inline void HAL_setTrigger(HAL_Handle handle, HAL_PWMData_t *pPWMData,
                                  const SVGENCURRENT_IgnoreShunt_e ignoreShunt,
                                  const SVGENCURRENT_VmidShunt_e midVolShunt)
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  uint16_t pwmCMPA[3];

  pwmCMPA[0] = EPWM_getCounterCompareValue(obj->pwmHandle[0],
                                           EPWM_COUNTER_COMPARE_A);

  pwmCMPA[1] = EPWM_getCounterCompareValue(obj->pwmHandle[1],
                                           EPWM_COUNTER_COMPARE_A);

  pwmCMPA[2] = EPWM_getCounterCompareValue(obj->pwmHandle[2],
                                           EPWM_COUNTER_COMPARE_A);

  int16_t pwmNum = 0;
  int16_t pwmSOCCMP = 1;

  if(ignoreShunt == SVGENCURRENT_USE_ALL)
  {
      pwmSOCCMP = 2;

      // Set up event source for ADC trigger
      EPWM_setADCTriggerSource(obj->pwmHandle[0],
                               EPWM_SOC_A,
                               EPWM_SOC_TBCTR_D_CMPC);
  }
  else
  {
      if(midVolShunt == SVGENCURRENT_VMID_A)
      {
          pwmNum = 0;
      }
      else if(midVolShunt == SVGENCURRENT_VMID_B)
      {
          pwmNum = 1;
      }
      else      // (midVolShunt == SVGENCURRENT_VMID_C)
      {
          pwmNum = 2;
      }

      pwmSOCCMP = pwmCMPA[pwmNum] -
              pPWMData->deadband[pwmNum] -
              pPWMData->noiseWindow;

      if(pwmSOCCMP < 0)
      {
          pwmSOCCMP = -pwmSOCCMP;

          // Set up event source for ADC trigger
          EPWM_setADCTriggerSource(obj->pwmHandle[0],
                                   EPWM_SOC_A,
                                   EPWM_SOC_TBCTR_U_CMPC);
      }
      else
      {
          // Set up event source for ADC trigger
          EPWM_setADCTriggerSource(obj->pwmHandle[0],
                                   EPWM_SOC_A,
                                   EPWM_SOC_TBCTR_D_CMPC);
      }

  }

  //
  pPWMData->socCMP = pwmSOCCMP;

  // write the PWM data value  for ADC trigger
  EPWM_setCounterCompareValue(obj->pwmHandle[0],
                              EPWM_COUNTER_COMPARE_C,
                              pwmSOCCMP);
  return;
} // end of HAL_setTrigger() function

#ifdef __cplusplus
}
#endif // extern "C"

//@}  // ingroup


#endif // end of HAL_H definition

