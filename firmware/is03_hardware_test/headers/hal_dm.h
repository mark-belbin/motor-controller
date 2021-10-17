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

#ifndef HAL_DM_H
#define HAL_DM_H

//! \file   solutions/boostxl_drv8320rs/f28004x/drivers/hal_dm.h
//! \brief  Contains public interface to various functions related
//!         to the HAL object
//!


// **************************************************************************
// the includes


// platforms
#include "hal_obj_dm.h"

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

//! Trip Zones all interrupt
//!
#define HAL_TZ_INTERRUPT_ALL      EPWM_TZ_INTERRUPT_DCBEVT2 \
                                 + EPWM_TZ_INTERRUPT_DCBEVT1 \
                                 + EPWM_TZ_INTERRUPT_DCAEVT2 \
                                 + EPWM_TZ_INTERRUPT_DCAEVT1 \
                                 + EPWM_TZ_INTERRUPT_OST \
                                 + EPWM_TZ_INTERRUPT_CBC

//! \brief Defines the comparator number for current prection
//!
#define HAL_NUM_CMPSS_CURRENT       3

#define MTR_1_PGA_GAIN              PGA_GAIN_12
#define MTR_1_DAC_VALUE             2048
#define MTR_1_CMPSS_DACH_VALUE      2048+1024+512
#define MTR_1_CMPSS_DACL_VALUE      2048-1024-512


#define MTR_2_PGA_GAIN              PGA_GAIN_12
#define MTR_2_DAC_VALUE             2048
#define MTR_2_CMPSS_DACH_VALUE      2048+1024+512
#define MTR_2_CMPSS_DACL_VALUE      2048-1024-512

#define MTR_1_TZ_SIGNAL             EPWM_TZ_SIGNAL_OSHT2
#define MTR_1_XBAR_IMPUT            XBAR_INPUT2
#define MTR_1_DCTRIPIN              EPWM_DC_COMBINATIONAL_TRIPIN7 \
                                    | EPWM_DC_COMBINATIONAL_TRIPIN8 \
                                    | EPWM_DC_COMBINATIONAL_TRIPIN9

#define MTR_2_TZ_SIGNAL             EPWM_TZ_SIGNAL_OSHT3
#define MTR_2_XBAR_IMPUT            XBAR_INPUT3
#define MTR_2_DCTRIPIN              EPWM_DC_COMBINATIONAL_TRIPIN10 \
                                    | EPWM_DC_COMBINATIONAL_TRIPIN11 \
                                    | EPWM_DC_COMBINATIONAL_TRIPIN12


//! \brief Defines the gpio for start command
//!
#define M1_HAL_START_CMD_GPIO            25

//! \brief Defines the gpio for stop command
//!
#define M1_HAL_STOP_CMD_GPIO             25

//! \brief Defines the gpio for the nFAULT of Power Module device
//!
#define M1_HAL_PM_nFAULT_GPIO            40

//! \brief Defines the gpio for the OCTW of Power Module device
//!
#define M1_HAL_PM_nOCTW_GPIO             40

//! \brief Defines the gpio for the SPI_CS of DRV device
//!
#define M1_HAL_DRV_SPI_CS_GPIO          57

//! \brief Defines the gpio for the enable gate of DRV device
//!
#define M1_HAL_DRV_EN_GATE_GPIO         13

//! \brief Defines the PWM deadband falling edge delay count (system clocks)
//!
#define M1_HAL_PWM_DBFED_CNT            1

//! \brief Defines the PWM deadband rising edge delay count (system clocks)
//!
#define M1_HAL_PWM_DBRED_CNT            1

//! \brief Defines the gpio for start command
//!
#define M2_HAL_START_CMD_GPIO            32

//! \brief Defines the gpio for stop command
//!
#define M2_HAL_STOP_CMD_GPIO             32

//! \brief Defines the gpio for the nFAULT of Power Module device
//!
#define M2_HAL_PM_nFAULT_GPIO            29

//! \brief Defines the gpio for the OCTW of Power Module device
//!
#define M2_HAL_PM_nOCTW_GPIO             29

//! \brief Defines the gpio for the SPI_CS of DRV device
//!
#define M2_HAL_DRV_SPI_CS_GPIO          27

//! \brief Defines the gpio for the enable gate of DRV device
//!
#define M2_HAL_DRV_EN_GATE_GPIO         28

//! \brief Defines the PWM deadband falling edge delay count (system clocks)
//!
#define M2_HAL_PWM_DBFED_CNT            1

//! \brief Defines the PWM deadband rising edge delay count (system clocks)
//!
#define M2_HAL_PWM_DBRED_CNT            1


//! \brief Defines the function to turn LEDs off
//!
#define HAL_turnLEDOff              HAL_setGPIOHigh

//! \brief Defines the function to turn LEDs on
//!
#define HAL_turnLEDOn               HAL_setGPIOLow

//! \brief Defines the function to toggle LEDs
//!
#define HAL_toggleLED               HAL_toggleGPIO

// **************************************************************************
// the typedefs

//! \brief Enumeration for the LED numbers
//!
typedef enum
{
  HAL_GPIO_LED1 = 23,   //!< GPIO pin number for LaunchPad LED 4
  HAL_GPIO_LED2 = 34,   //!< GPIO pin number for LaunchPad LED 5
  HAL_GPIO_ISR  = 30,   //!< GPIO pin number for ISR Executing Time
  HAL_GPIO_BML  = 31    //!< GPIO pin number for Main Loop Executing Time
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
  HAL_CPU_TIMER2 = 2   //!< Select CPU Timer2
} HAL_CPUTimerNum_e;


//! \brief Enumeration for the Motor numbers
//!
typedef enum
{
  HAL_MTR_1 = 0,
  HAL_MTR_2 = 1
} HAL_MotorNum_e;

// **************************************************************************
// the globals
extern __interrupt void mainISR(void);

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

//! \brief     Acknowledges an interrupt from the ADC so that another ADC interrupt can 
//!            happen again.
//! \param[in] handle     The hardware abstraction layer (HAL) handle
//! \param[in] adcIntNum  The interrupt number
static inline void HAL_ackADCInt(HAL_Handle handle,const ADC_IntNumber adcIntNum)
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  // clear the ADC interrupt flag
  // RB2
  ADC_clearInterruptStatus(obj->adcHandle[2], adcIntNum);         // ADCC

  // Acknowledge interrupt from PIE group 1
  Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);

  return;
} // end of HAL_ackADCInt() function

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

//! \brief      Enables the 8320/8301 device
//! \details    Provides the correct timing to enable the drv8320
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_enableDRV(HAL_MTR_Handle handle);

//! \brief     Enables global interrupts
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_enableGlobalInts(HAL_Handle handle);

//! \brief     Gets the current scale factor
//! \param[in] handle  The hardware abstraction layer (HAL) handle
//! \return    The current scale factor
static inline float32_t HAL_getCurrentScaleFactor(HAL_MTR_Handle handle)
{
  HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

  return(obj->current_sf);
} // end of HAL_getCurrentScaleFactor() function

//! \brief     Gets the PWM duty cycle times
//! \param[in] handle       The hardware abstraction layer (HAL) handle
//! \param[in] pDutyCycles  A pointer to memory for the duty cycle durations
static inline void
HAL_getDutyCycles(HAL_MTR_Handle handle,uint16_t *pDutyCycles)
{
  HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

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
static inline uint16_t HAL_getNumCurrentSensors(HAL_MTR_Handle handle)
{
  HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;
  
  return(obj->numCurrentSensors);
} // end of HAL_getNumCurrentSensors() function


//! \brief     Gets the number of voltage sensors
//! \param[in] handle  The hardware abstraction layer (HAL) handle
//! \return    The number of voltage sensors
static inline uint16_t HAL_getNumVoltageSensors(HAL_MTR_Handle handle)
{
  HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

  return(obj->numVoltageSensors);
} // end of HAL_getNumVoltageSensors() function

//! \brief     Gets the pwm enable status
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
//! \return    The pwm enable
static inline bool HAL_getPwmEnableStatus(HAL_MTR_Handle handle)
{
  HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

  return(obj->flagEnablePWM);
} // end of HAL_getPwmStatus() function

//! \brief     Gets the voltage scale factor
//! \param[in] handle  The hardware abstraction layer (HAL) handle
//! \return    The voltage scale factor
static inline float32_t HAL_getVoltageScaleFactor(HAL_MTR_Handle handle)
{
  HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

  return(obj->voltage_sf);
} // end of HAL_getVoltageScaleFactor() function

//! \brief      Configures the fault protection logic
//! \details    Sets up the trip zone inputs so that when a comparator
//!             signal from outside the micro-controller trips a fault,
//!             the EPWM peripheral blocks will force the
//!             power switches into a high impedance state.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupFaults(HAL_MTR_Handle handle,
                            const HAL_MotorNum_e motorNum);


//! \brief      Initializes the hardware abstraction layer (HAL) object
//! \details    Initializes all handles to the microcontroller peripherals.
//!             Returns a handle to the HAL object.
//! \param[in]  pMemory   A pointer to the memory for the hardware abstraction layer object
//! \param[in]  numBytes  The number of bytes allocated for the hardware abstraction layer object, bytes
//! \return     The hardware abstraction layer (HAL) object handle
extern HAL_Handle HAL_init(void *pMemory,const size_t numBytes);


//! \brief      Initializes the hardware abstraction layer (HAL) object
//! \details    Initializes all handles to the microcontroller peripherals.
//!             Returns a handle to the HAL_MTR object.
//! \param[in]  pMemory   A pointer to the memory for the hardware abstraction layer object
//! \param[in]  numBytes  The number of bytes allocated for the hardware abstraction layer object, bytes
//! \return     The hardware abstraction layer (HAL_MTR) object handle
extern HAL_MTR_Handle HAL_MTR_init(void *pMemory,
                                   const size_t numBytes,
                                   const HAL_MotorNum_e motorNum);


//! \brief      Initializes the interrupt vector table
//! \details    Points the ISR to the function mainISR.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
static inline void HAL_initIntVectorTable(HAL_Handle handle)
 {

    Interrupt_register(INT_ADCC1, &mainISR);

    return;
 } // end of HAL_initIntVectorTable() function


//! \brief      Reads the ADC data with offset
//! \details    Reads in the ADC result registers and scales the values
//!             according to the settings in user_m1.h or user_m2.h.
//!             The structure gAdcData holds three phase voltages,
//!             three line currents, and one DC bus voltage.
//! \param[in]  handle    The hardware abstraction layer (HAL) handle
//! \param[in]  pADCData  A pointer to the ADC data buffer
static inline void HAL_readADCDataWithOffsets(HAL_Handle handle,
                                              HAL_MTR_Handle mtrHandle,
                                              HAL_ADCData_t *pADCData,
                                              const HAL_MotorNum_e motorNum)
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  float32_t value;

  float32_t current_sf = -HAL_getCurrentScaleFactor(mtrHandle);
  float32_t voltage_sf = HAL_getVoltageScaleFactor(mtrHandle);

  if(motorNum == HAL_MTR_1)
  {
      // convert phase A current        ->RA0/A14
      value = (float32_t)ADC_readResult(obj->adcResult[0], ADC_SOC_NUMBER0);
      pADCData->I_A.value[0] = value * current_sf;

      // convert phase B current        ->RC0/C7
      value = (float32_t)ADC_readResult(obj->adcResult[2], ADC_SOC_NUMBER0);
      pADCData->I_A.value[1] = value * current_sf;

      // convert phase C current        ->RB0/B7
      value = (float32_t)ADC_readResult(obj->adcResult[1], ADC_SOC_NUMBER0);
      pADCData->I_A.value[2] = value * current_sf;

      // convert phase A voltage        ->RA1/A5
      value = (float32_t)ADC_readResult(obj->adcResult[0], ADC_SOC_NUMBER1);
      pADCData->V_V.value[0] = value * voltage_sf;

      // convert phase B voltage        ->RB1/B0
      value = (float32_t)ADC_readResult(obj->adcResult[1], ADC_SOC_NUMBER1);
      pADCData->V_V.value[1] = value * voltage_sf;

      // convert phase C voltage        ->RC1/C2
      value = (float32_t)ADC_readResult(obj->adcResult[2], ADC_SOC_NUMBER1);
      pADCData->V_V.value[2] = value * voltage_sf;

      // convert dcBus voltage          ->RB2/B1
      value = (float32_t)ADC_readResult(obj->adcResult[1], ADC_SOC_NUMBER2);
      pADCData->dcBus_V = value * voltage_sf;
  }
  else if(motorNum == HAL_MTR_2)
  {
      // convert phase A current        ->RB3/B9
      value = (float32_t)ADC_readResult(obj->adcResult[1], ADC_SOC_NUMBER3);
      pADCData->I_A.value[0] = value * current_sf;

      // convert phase B current        ->RA2/A15
      value = (float32_t)ADC_readResult(obj->adcResult[0], ADC_SOC_NUMBER2);
      pADCData->I_A.value[1] = value * current_sf;

      // convert phase C current        ->RC2/C9
      value = (float32_t)ADC_readResult(obj->adcResult[2], ADC_SOC_NUMBER2);
      pADCData->I_A.value[2] = value * current_sf;

      // convert phase A voltage        ->RA3/A6
      value = (float32_t)ADC_readResult(obj->adcResult[0], ADC_SOC_NUMBER3);
      pADCData->V_V.value[0] = value * voltage_sf;

      // convert phase B voltage        ->RB4/B6
      value = (float32_t)ADC_readResult(obj->adcResult[1], ADC_SOC_NUMBER4);
      pADCData->V_V.value[1] = value * voltage_sf;

      // convert phase C voltage        ->RC3/C14
      value = (float32_t)ADC_readResult(obj->adcResult[2], ADC_SOC_NUMBER3);
      pADCData->V_V.value[2] = value * voltage_sf;

      // convert dcBus voltage          ->RC4/C1
      value = (float32_t)ADC_readResult(obj->adcResult[2], ADC_SOC_NUMBER4);
      pADCData->dcBus_V = value * voltage_sf;
  }

  return;
} // end of HAL_readADCDataWithOffsets() function


//! \brief      Reads the ADC data without offsets
//! \details    Reads in the ADC result registers and scales the values
//!             according to the settings in user_m1.h or user_m2.h.
//!             The structure gAdcData holds three phase voltages,
//!             three line currents, and one DC bus voltage.
//! \param[in]  handle    The hardware abstraction layer (HAL) handle
//! \param[in]  pADCData  A pointer to the ADC data buffer
static inline void HAL_readADCDataWithoutOffsets(HAL_Handle handle,
                                                 HAL_MTR_Handle mtrHandle,
                                                 HAL_ADCData_t *pADCData)
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  float32_t value;

  float32_t current_sf = -HAL_getCurrentScaleFactor(mtrHandle);
  float32_t voltage_sf = HAL_getVoltageScaleFactor(mtrHandle);

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
HAL_readTimerCnt(HAL_Handle handle,const uint16_t timerNumber)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    uint32_t timerCnt = CPUTimer_getTimerCount(obj->timerHandle[timerNumber]);

    return(timerCnt);
} // end of HAL_readTimerCnt() function

//! \brief     Sets the GPIO pin high
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \param[in] gpioNumber  The GPIO number
static inline void HAL_setGPIOHigh(HAL_Handle handle,const uint32_t gpioNumber)
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
HAL_readGPIOData(HAL_Handle handle,const uint32_t gpioNumber)
{
  uint32_t gpioPinData;

  // set GPIO high
  gpioPinData = GPIO_readPin(gpioNumber);

  return(gpioPinData);
} // end of HAL_readGPIOData() function


//! \brief     Sets the GPIO pin low
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \param[in] gpioNumber  The GPIO number
static inline void HAL_setGPIOLow(HAL_Handle handle,const uint32_t gpioNumber)
{
  // set GPIO low
  GPIO_writePin(gpioNumber, 0);

  return;
} // end of HAL_setGPIOLow() function


//! \brief     Sets the value of the internal DAC of the high comparator
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \param[in] cmpssNumber The CMPSS number
//! \param[in] dacValue    The DAC value of the high comparator
static inline void
HAL_setCMPSSDACValueHigh(HAL_MTR_Handle handle,
                         const uint16_t cmpssNumber, uint16_t dacValue)
{
  HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

  // set GPIO low
  CMPSS_setDACValueHigh(obj->cmpssHandle[cmpssNumber], dacValue);

  return;
} // end of HAL_setCMPSSDACValueHigh() function


//! \brief     Sets the value of the internal DAC of the low comparator
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \param[in] cmpssNumber The CMPSS number
//! \param[in] dacValue    The DAC value of the low comparator
static inline void
HAL_setCMPSSDACValueLow(HAL_MTR_Handle handle,
                        const uint16_t cmpssNumber, uint16_t dacValue)
{
  HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

  // set GPIO low
  CMPSS_setDACValueLow(obj->cmpssHandle[cmpssNumber], dacValue);

  return;
} // end of HAL_setCMPSSDACValueLow() function

//! \brief     Sets the number of voltage sensors
//! \param[in] handle             The hardware abstraction layer (HAL) handle
//! \param[in] numVoltageSensors  The number of voltage sensors
static inline void
HAL_setNumVoltageSensors(HAL_MTR_Handle handle,const uint16_t numVoltageSensors)
{
  HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

  obj->numVoltageSensors = numVoltageSensors;

  return;
} // end of HAL_setNumVoltageSensors() function

//! \brief     Sets the number of current sensors
//! \param[in] handle             The hardware abstraction layer (HAL) handle
//! \param[in] numCurrentSensors  The number of current sensors
static inline void
HAL_setNumCurrentSensors(HAL_MTR_Handle handle,const uint16_t numCurrentSensors)
{
  HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

  obj->numCurrentSensors = numCurrentSensors;

  return;
} // end of HAL_setNumCurrentSensors() function

//! \brief      Sets the hardware abstraction layer parameters
//! \details    Sets up the microcontroller peripherals.  Creates all of the scale
//!             factors for the ADC voltage and current conversions.  Sets the initial
//!             offset values for voltage and current measurements.
//! \param[in]  handle       The hardware abstraction layer (HAL) handle
extern void HAL_setParams(HAL_Handle handle);

//! \brief      Sets the hardware abstraction layer parameters
//! \details    Sets up the microcontroller peripherals.  Creates all of the scale
//!             factors for the ADC voltage and current conversions.  Sets the initial
//!             offset values for voltage and current measurements.
//! \param[in]  handle       The hardware abstraction layer (HAL) handle
extern void HAL_MTR_setParams(HAL_MTR_Handle handle,
                              const HAL_MotorNum_e motorNum);

//! \brief      Sets up the ADCs (Analog to Digital Converters)
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupADCs(HAL_Handle handle);

//! \brief      Sets up the ADCs (Analog to Digital Converters)
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_runADCZeroOffsetCalibration(uint32_t base);


//! \brief      Sets up the PGAs (Programmable Gain Amplifiers)
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupPGAs(HAL_MTR_Handle handle,
                          const HAL_MotorNum_e motorNum);


//! \brief      Sets up the CMPSSs (Comparator Subsystems)
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupCMPSSs(HAL_MTR_Handle handle,
                            const HAL_MotorNum_e motorNum);


//! \brief      Sets up the DACs (Buffered Digital-to-Analog Converter)
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupDACs(HAL_MTR_Handle handle,
                          const HAL_MotorNum_e motorNum);


//! \brief      Sets up the clocks
//! \details    Sets up the micro-controller's main oscillator
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupClks(HAL_Handle handle);

//! \brief     Sets up the GATE object
//! \param[in] handle       The hardware abstraction layer (HAL) handle
extern void HAL_setupGate(HAL_MTR_Handle handle,
                          const HAL_MotorNum_e motorNum);

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
extern void HAL_setupQEP(HAL_MTR_Handle handle);

//! \brief     Sets up the SCIA
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupSCIA(HAL_Handle handle);


//! \brief     Sets up the SPI
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupSPI(HAL_MTR_Handle handle);

//! \brief     Sets up the timers
//! \param[in] handle          The hardware abstraction layer (HAL) handle
//! \param[in] systemFreq_MHz  The system frequency, MHz
extern void HAL_setupTimers(HAL_Handle handle,const float32_t systemFreq_MHz);


//! \brief     Sets up the timers
//! \param[in] handle          The hardware abstraction layer (HAL) handle
//! \param[in] cpuTimerNumber  The CPU timer number
bool HAL_getTimerStatus(HAL_Handle halHandle, const uint16_t cpuTimerNumber);


//! \brief     Sets up the timers
//! \param[in] handle          The hardware abstraction layer (HAL) handle
//! \param[in] cpuTimerNumber  The CPU timer number
void HAL_clearTimerFlag(HAL_Handle halHandle, const uint16_t cpuTimerNumber);


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

//! \brief     Force trig the DMA channel for datalog
//! \param[in] handle          The hardware abstraction layer (HAL) handle
//! \param[in] dmaChNumber     The DMC Channel Number
static inline void
HAL_trigDlogWithDMA(HAL_Handle handle, const uint16_t DMAChannel)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    DMA_startChannel(obj->dmaChHandle[DMAChannel]);
    DMA_forceTrigger(obj->dmaChHandle[DMAChannel]);

    return;
} // end of HAL_trigDlogWithDMA() function

//! \brief     Toggles the GPIO pin
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \param[in] gpioNumber  The GPIO number
static inline void HAL_toggleGPIO(HAL_Handle handle,const uint32_t gpioNumber)
{

    // set GPIO high
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

    // convert values from float to unit16
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
} // end of HAL_writeDacData() function


//! \brief     Writes DAC data to the PWM comparators for DAC output
//! \param[in] handle    The hardware abstraction layer (HAL) handle
//! \param[in] pPWMDACData  The pointer to the DAC data
void HAL_setPWMDACParameters(HAL_Handle handle, HAL_PWMDACData_t *pPWMDACData);

//! \brief
//! \param[in]
//! \param[in]
void HAL_clearDataRAM(void *pMemory, uint16_t lengthMemory);

//! \brief     Reads PWM period register
//! \param[in] handle     The hardware abstraction layer (HAL) handle
//! \param[in] pwmNumber  The PWM number
//! \return    The PWM period value
static inline uint16_t
HAL_readPWMPeriod(HAL_MTR_Handle handle,const uint16_t pwmNumber)
{
  HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

  // the period value to be returned
  uint16_t pwmPeriodValue;

  pwmPeriodValue = EPWM_getTimeBasePeriod(obj->pwmHandle[pwmNumber]);

  return(pwmPeriodValue);
} // end of HAL_readPWMPeriod() function


//! \brief     Writes PWM data to the PWM comparators for motor control
//! \param[in] handle    The hardware abstraction layer (HAL) handle
//! \param[in] pPWMData  The pointer to the PWM data
static inline void
HAL_writePWMData(HAL_MTR_Handle handle,const HAL_PWMData_t *pPWMData)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;
    uint16_t pwmCnt;

    for(pwmCnt=0;pwmCnt<3;pwmCnt++)
    {
      // compute the value
        float32_t period =
                (float32_t)(EPWM_getTimeBasePeriod(obj->pwmHandle[pwmCnt]));

        float32_t V_pu = -pPWMData->Vabc_pu.value[pwmCnt];      // Negative
        float32_t V_sat_pu = MATH_sat(V_pu,0.5,-0.5);           // -0.5~0.5
        float32_t V_sat_dc_pu = V_sat_pu + 0.5;                 // 0~1.0
        int16_t pwmValue  = (int16_t)(V_sat_dc_pu * period);  //

        // write the PWM data value
        EPWM_setCounterCompareValue(obj->pwmHandle[pwmCnt],
                                    EPWM_COUNTER_COMPARE_A,
                                    pwmValue);
    }

    return;
} // end of HAL_writePWMData() function


//! \brief      Enables the PWM devices
//! \details    Turns on the outputs of the EPWM peripheral which will allow
//!             the power switches to be controlled.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
static inline void HAL_enablePWM(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

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
static inline void HAL_disablePWM(HAL_MTR_Handle handle)
{
  HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

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
//! \param[in] numPWMTicksPerISRTick  The number of PWM clock ticks per ISR clock tick
extern void HAL_setupPWMs(HAL_MTR_Handle handle,
                          const HAL_MotorNum_e motorNum);

#ifdef DRV8320_SPI
//! \brief     Writes data to the driver
//! \param[in] handle         The hardware abstraction layer (HAL) handle
//! \param[in] drv8320SPIVars  SPI variables
void HAL_writeDRVData(HAL_MTR_Handle handle, DRV8320_SPIVars_t *drv8320SPIVars);

//! \brief     Reads data from the driver
//! \param[in] handle         The hardware abstraction layer (HAL) handle
//! \param[in] drv8320SPIVars  SPI variables
void HAL_readDRVData(HAL_MTR_Handle handle, DRV8320_SPIVars_t *drv8320SPIVars);

//! \brief     Sets up the SPI interface for the driver
//! \param[in] handle         The hardware abstraction layer (HAL) handle
//! \param[in] drv8320SPIVars  SPI variables
extern void HAL_setupDRVSPI(HAL_MTR_Handle handle,
                            DRV8320_SPIVars_t *drv8320SPIVars);
#endif  // DRV8320_SPI

#ifdef DRV8301_SPI
//! \brief     Writes data to the driver
//! \param[in] handle         The hardware abstraction layer (HAL) handle
//! \param[in] Spi_8301_Vars  SPI variables
void HAL_writeDRVData(HAL_MTR_Handle handle, DRV8301_SPIVars_t *Spi_8301_Vars);

//! \brief     Reads data from the driver
//! \param[in] handle         The hardware abstraction layer (HAL) handle
//! \param[in] Spi_8301_Vars  SPI variables
void HAL_readDRVData(HAL_MTR_Handle handle, DRV8301_SPIVars_t *Spi_8301_Vars);

//! \brief     Sets up the SPI interface for the driver
//! \param[in] handle         The hardware abstraction layer (HAL) handle
//! \param[in] Spi_8301_Vars  SPI variables
extern void HAL_setupDRVSPI(HAL_MTR_Handle handle, DRV8301_SPIVars_t *Spi_8301_Vars);
#endif

//! \brief     Sets the current scale factor in the hal
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \param[in] current_sf  The current scale factor
static inline void
HAL_setCurrentScaleFactor(HAL_MTR_Handle handle, const float32_t current_sf)
{
  HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

  obj->current_sf = current_sf;

  return;
} // end of HAL_setCurrentScaleFactor() function

//! \brief     Sets the voltage scale factor in the hal
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \param[in] voltage_sf  The voltage scale factor
static inline void
HAL_setVoltageScaleFactor(HAL_MTR_Handle handle, const float32_t voltage_sf)
{
  HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

  obj->voltage_sf = voltage_sf;

  return;
} // end of HAL_setVoltageScaleFactor() function


static inline uint16_t HAL_getTripFaults(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;
    uint16_t tripFault = 0;

    tripFault = (EPWM_getTripZoneFlagStatus(obj->pwmHandle[0]) &
            (EPWM_TZ_FLAG_OST | EPWM_TZ_FLAG_DCAEVT1 | EPWM_TZ_FLAG_DCAEVT2)) |
                    (EPWM_getTripZoneFlagStatus(obj->pwmHandle[1]) &
            (EPWM_TZ_FLAG_OST | EPWM_TZ_FLAG_DCAEVT1 | EPWM_TZ_FLAG_DCAEVT2)) |
                    (EPWM_getTripZoneFlagStatus(obj->pwmHandle[2]) &
            (EPWM_TZ_FLAG_OST | EPWM_TZ_FLAG_DCAEVT1 | EPWM_TZ_FLAG_DCAEVT2));

    return(tripFault);
}


#ifdef __cplusplus
}
#endif // extern "C"

//@}  // ingroup


#endif // end of HAL_DM_H definition

