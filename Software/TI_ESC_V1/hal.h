#ifndef _HAL_H_
#define _HAL_H_
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

//! \file   sw\modules\hal\boards\boostxldrv8305evm_revA\f28x\f2802x\src\hal.h
//! \brief  Contains public interface to various functions related
//!         to the HAL object
//!
//! (C) Copyright 2015, Texas Instruments, Inc.


// **************************************************************************
// the includes


// modules


// platforms
#include "TI_ESC_V1/hal_obj.h"
#include "sw/modules/svgen/src/32b/svgen_current.h"


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


//! \brief Defines LAUNCHPAD which is needed to blink LED
#define LAUNCHPAD


//! \brief Defines that a DRV8305 chip SPI port is used on the board.
#define DRV8305_SPI


#define Device_cal (void   (*)(void))0x3D7C80

//! \brief Defines used in oscillator calibration functions
//! \brief Defines the scale factor for Q15 fixed point numbers (2^15)
#define FP_SCALE 32768

//! \brief Defines the quantity added to Q15 numbers before converting to integer to round the number
#define FP_ROUND FP_SCALE/2

//! \brief Defines the amount to add to Q16.15 fixed point number to shift from a fine trim range of
//! \brief (-31 to 31) to (1 to 63).  This guarantees that the trim is positive and can
//! \brief therefore be efficiently rounded
#define OSC_POSTRIM 32
#define OSC_POSTRIM_OFF FP_SCALE*OSC_POSTRIM

//! \brief The following functions return reference values stored in OTP.

//! \brief Defines the slope used to compensate oscillator 1 (fine trim steps / ADC code). Stored in fixed point Q15 format
#define getOsc1FineTrimSlope() (*(int16_t (*)(void))0x3D7E90)()

//! \brief Defines the oscillator 1 fine trim at high temp
#define getOsc1FineTrimOffset() (*(int16_t (*)(void))0x3D7E93)()

//! \brief Defines the oscillator 1 coarse trim
#define getOsc1CoarseTrim() (*(int16_t (*)(void))0x3D7E96)()

//! \brief Defines the slope used to compensate oscillator 2 (fine trim steps / ADC code). Stored
//! \brief in fixed point Q15 format.
#define getOsc2FineTrimSlope() (*(int16_t (*)(void))0x3D7E99)()

//! \brief Defines the oscillator 2 fine trim at high temp
#define getOsc2FineTrimOffset() (*(int16_t (*)(void))0x3D7E9C)()

//! \brief Defines the oscillator 2 coarse trim
#define getOsc2CoarseTrim() (*(int16_t (*)(void))0x3D7E9F)()

//! \brief Defines the ADC reading of temperature sensor at reference temperature for compensation
#define getRefTempOffset() (*(int16_t (*)(void))0x3D7EA2)()

//! \brief Defines the PWM deadband falling edge delay count (system clocks)
//!
#define HAL_PWM_DBFED_CNT         1


//! \brief Defines the PWM deadband rising edge delay count (system clocks)
//!
#define HAL_PWM_DBRED_CNT         1


//! \brief Defines the function to turn LEDs off
//!
#define HAL_turnLedOff            HAL_setGpioHigh


//! \brief Defines the function to turn LEDs on
//!
#define HAL_turnLedOn             HAL_setGpioLow


//! \brief Defines the function to turn LEDs on
//!
#define HAL_toggleLed             HAL_toggleGpio

// **************************************************************************
// the typedefs


//! \brief Enumeration for the LED numbers
//!
typedef enum
{
  HAL_Gpio_LED2=GPIO_Number_0   //!< GPIO pin number. LaunchPad uses PWM pins for LEDs
} HAL_LedNumber_e;
  

//! \brief Enumeration for the sensor types
//!
typedef enum
{
  HAL_SensorType_Current=0,      //!< Enumeration for current sensor
  HAL_SensorType_Voltage         //!< Enumeration for voltage sensor
} HAL_SensorType_e;


// **************************************************************************
// the globals

extern interrupt void mainISR(void);

extern interrupt void timer0ISR(void);


// **************************************************************************
// the function prototypes


//! \brief     Acknowledges an interrupt from the ADC so that another ADC interrupt can 
//!            happen again.
//! \param[in] handle     The hardware abstraction layer (HAL) handle
//! \param[in] intNumber  The interrupt number
static inline void HAL_acqAdcInt(HAL_Handle handle,const ADC_IntNumber_e intNumber)
{
  HAL_Obj *obj = (HAL_Obj *)handle;


  // clear the ADC interrupt flag
  ADC_clearIntFlag(obj->adcHandle,intNumber);


  // Acknowledge interrupt from PIE group 10 
  PIE_clearInt(obj->pieHandle,PIE_GroupNumber_10);

  return;
} // end of HAL_acqAdcInt() function


//! \brief     Acknowledges an interrupt from the PWM so that another PWM interrupt can
//!            happen again.
//! \param[in] handle     The hardware abstraction layer (HAL) handle
//! \param[in] pwmNumber  The PWM number
static inline void HAL_acqPwmInt(HAL_Handle handle,const PWM_Number_e pwmNumber)
{
  HAL_Obj *obj = (HAL_Obj *)handle;


  // clear the PWM interrupt flag
  PWM_clearIntFlag(obj->pwmHandle[pwmNumber]);


  // clear the SOCA flag
  PWM_clearSocAFlag(obj->pwmHandle[pwmNumber]);


  // Acknowledge interrupt from PIE group 3
  PIE_clearInt(obj->pieHandle,PIE_GroupNumber_3);

  return;
} // end of HAL_acqPwmInt() function

//! \brief      Acknowledges an interrupt from Timer0 so that another Timer0 interrupt can occur
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
static inline void HAL_acqTimer0Int(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    // clear the Timer0 interrupt flag
    TIMER_clearFlag(obj->timerHandle[0]);

    // Acknowledge interrupt from PIE group 1
    PIE_clearInt(obj->pieHandle, PIE_GroupNumber_1);

    return;
} // end of the HAL_acqTimer0Int() function



//! \brief      Executes calibration routines
//! \details    Values for offset and gain are programmed into OTP memory at
//!             the TI factory.  This calls and internal function that programs
//!             these offsets and gains into the ADC registers.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_cal(HAL_Handle handle);


//! \brief      Disables global interrupts
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_disableGlobalInts(HAL_Handle handle);


//! \brief      Disables the watch dog
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_disableWdog(HAL_Handle handle);


//! \brief      Disables the PWM device
//! \details    Turns off the outputs of the EPWM peripherals which will put
//!             the power switches into a high impedance state.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
static inline void HAL_disablePwm(HAL_Handle handle)
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  PWM_setOneShotTrip(obj->pwmHandle[PWM_Number_1]);
  PWM_setOneShotTrip(obj->pwmHandle[PWM_Number_2]);
  PWM_setOneShotTrip(obj->pwmHandle[PWM_Number_3]);

  return;
} // end of HAL_disablePwm() function


//! \brief      Enables the ADC interrupts
//! \details    Enables the ADC interrupt in the PIE, and CPU.  Enables the 
//!             interrupt to be sent from the ADC peripheral.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_enableAdcInts(HAL_Handle handle);


//! \brief      Enables the debug interrupt
//! \details    The debug interrupt is used for the real-time debugger.  It is
//!             not needed if the real-time debugger is not used.  Clears
//!             bit 1 of ST1.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_enableDebugInt(HAL_Handle handle);


//! \brief     Enables global interrupts
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_enableGlobalInts(HAL_Handle handle);


//! \brief      Enables the 8305 device
//! \details    Provides the correct timing to enable the drv8305
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_enableDrv(HAL_Handle handle);


//! \brief      Enables the PWM devices
//! \details    Turns on the outputs of the EPWM peripheral which will allow 
//!             the power switches to be controlled.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
static inline void HAL_enablePwm(HAL_Handle handle)
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  PWM_clearOneShotTrip(obj->pwmHandle[PWM_Number_1]);
  PWM_clearOneShotTrip(obj->pwmHandle[PWM_Number_2]);
  PWM_clearOneShotTrip(obj->pwmHandle[PWM_Number_3]);

  return;
} // end of HAL_enablePwm() function


//! \brief     Enables the PWM interrupt
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_enablePwmInt(HAL_Handle handle);


//! \brief     Gets the ADC delay value
//! \param[in] handle     The hardware abstraction layer (HAL) handle
//! \param[in] socNumber  The ADC SOC number
//! \return    The ADC delay value
static inline ADC_SocSampleDelay_e HAL_getAdcSocSampleDelay(HAL_Handle handle,
                                                            const ADC_SocNumber_e socNumber)
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  return(ADC_getSocSampleDelay(obj->adcHandle,socNumber));
} // end of HAL_getAdcSocSampleDelay() function


//! \brief      Gets the ADC bias value
//! \details    The ADC bias contains the feedback circuit's offset and bias.
//!             Bias is the mathematical offset used when a bi-polar signal
//!             is read into a uni-polar ADC.
//! \param[in]  handle        The hardware abstraction layer (HAL) handle
//! \param[in]  sensorType    The sensor type
//! \param[in]  sensorNumber  The sensor number
//! \return     The ADC bias value
static inline _iq HAL_getBias(HAL_Handle handle,
                              const HAL_SensorType_e sensorType,
                              uint_least8_t sensorNumber)
{
  HAL_Obj *obj = (HAL_Obj *)handle;
  _iq bias = _IQ(0.0);

  if(sensorType == HAL_SensorType_Current)
    {
      bias = obj->adcBias.I.value[sensorNumber];
    }
  else if(sensorType == HAL_SensorType_Voltage)
    {
      bias = obj->adcBias.V.value[sensorNumber];
    }

  return(bias);
} // end of HAL_getBias() function


//! \brief      Gets the current scale factor
//! \details    The current scale factor is defined as
//!             USER_ADC_FULL_SCALE_CURRENT_A/USER_IQ_FULL_SCALE_CURRENT_A.
//!             This scale factor is not used when converting between PU amps
//!             and real amps.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
//! \return     The current scale factor
static inline _iq HAL_getCurrentScaleFactor(HAL_Handle handle)
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  return(obj->current_sf);
} // end of HAL_getCurrentScaleFactor() function


//! \brief     Gets the number of current sensors
//! \param[in] handle  The hardware abstraction layer (HAL) handle
//! \return    The number of current sensors
static inline uint_least8_t HAL_getNumCurrentSensors(HAL_Handle handle)
{
  HAL_Obj *obj = (HAL_Obj *)handle;
  

  return(obj->numCurrentSensors);
} // end of HAL_getNumCurrentSensors() function


//! \brief     Gets the number of voltage sensors
//! \param[in] handle  The hardware abstraction layer (HAL) handle
//! \return    The number of voltage sensors
static inline uint_least8_t HAL_getNumVoltageSensors(HAL_Handle handle)
{
  HAL_Obj *obj = (HAL_Obj *)handle;
  

  return(obj->numVoltageSensors);
} // end of HAL_getNumVoltageSensors() function


//! \brief      Gets the value used to set the low pass filter pole for offset estimation
//! \details    An IIR single pole low pass filter is used to find the feedback circuit's
//!             offsets.  This function returns the value of that pole.
//! \param[in]  handle        The hardware abstraction layer (HAL) handle
//! \param[in]  sensorType    The sensor type
//! \param[in]  sensorNumber  The sensor number
//! \return     The value used to set the low pass filter pole, pu
static inline _iq HAL_getOffsetBeta_lp_pu(HAL_Handle handle,
                                          const HAL_SensorType_e sensorType,
                                          const uint_least8_t sensorNumber)
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  _iq beta_lp_pu = _IQ(0.0);
  
  if(sensorType == HAL_SensorType_Current)
    {
      beta_lp_pu = OFFSET_getBeta(obj->offsetHandle_I[sensorNumber]);
    }
  else if(sensorType == HAL_SensorType_Voltage)
    {
      beta_lp_pu = OFFSET_getBeta(obj->offsetHandle_V[sensorNumber]);
    }

  return(beta_lp_pu);
} // end of HAL_getOffsetBeta_lp_pu() function


//! \brief      Gets the offset value
//! \details    The offsets that are calculated during the feedback circuits calibrations
//!             are returned from the IIR filter object.
//! \param[in]  handle        The hardware abstraction layer (HAL) handle
//! \param[in]  sensorType    The sensor type
//! \param[in]  sensorNumber  The sensor number
//! \return     The offset value
static inline _iq HAL_getOffsetValue(HAL_Handle handle,
                                     const HAL_SensorType_e sensorType,
                                     const uint_least8_t sensorNumber)
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  _iq offset = _IQ(0.0);
  
  if(sensorType == HAL_SensorType_Current)
    {
      offset = OFFSET_getOffset(obj->offsetHandle_I[sensorNumber]);
    }
  else if(sensorType == HAL_SensorType_Voltage)
    {
      offset = OFFSET_getOffset(obj->offsetHandle_V[sensorNumber]);
    }

  return(offset);
} // end of HAL_getOffsetValue() function


//! \brief      Gets the voltage scale factor
//! \details    The voltage scale factor is defined as
//!             USER_ADC_FULL_SCALE_VOLTAGE_V/USER_IQ_FULL_SCALE_VOLTAGE_V.
//!             This scale factor is not used when converting between PU volts
//!             and real volts.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
//! \return     The voltage scale factor
static inline _iq HAL_getVoltageScaleFactor(HAL_Handle handle)
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
//! \param[in]  pMemory   A pointer to the memory for the hardware abstraction layer object
//! \param[in]  numBytes  The number of bytes allocated for the hardware abstraction layer object, bytes
//! \return     The hardware abstraction layer (HAL) object handle
extern HAL_Handle HAL_init(void *pMemory,const size_t numBytes);


//! \brief      Initializes the interrupt vector table
//! \details    Points the TINT0 to timer0ISR and ADCINT1 to mainISR
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
static inline void HAL_initIntVectorTable(HAL_Handle handle)
 {
  HAL_Obj *obj = (HAL_Obj *)handle;
  PIE_Obj *pie = (PIE_Obj *)obj->pieHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  pie->ADCINT1 = &mainISR;

  pie->TINT0 = &timer0ISR;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
 } // end of HAL_initIntVectorTable() function


//! \brief      Reads the ADC data
//! \details    Reads in the ADC result registers, adjusts for offsets, and
//!             scales the values according to the settings in user.h.  The
//!             structure gAdcData holds three phase voltages, three line
//!             currents, and one DC bus voltage.
//! \param[in]  handle    The hardware abstraction layer (HAL) handle
//! \param[in]  pAdcData  A pointer to the ADC data buffer
static inline void HAL_readAdcData(HAL_Handle handle,HAL_AdcData_t *pAdcData)
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  _iq value;
  _iq current_sf = HAL_getCurrentScaleFactor(handle);
  _iq voltage_sf = HAL_getVoltageScaleFactor(handle);


  // convert current A
  // sample the first sample twice due to errata sprz342f, ignore the first sample
  value = (_iq)ADC_readResult(obj->adcHandle,ADC_ResultNumber_1);
  value = _IQ12mpy(value,current_sf) - obj->adcBias.I.value[0];      // divide by 2^numAdcBits = 2^12
  pAdcData->I.value[0] = value;

  // convert current B
  value = (_iq)ADC_readResult(obj->adcHandle,ADC_ResultNumber_2);
  value = _IQ12mpy(value,current_sf) - obj->adcBias.I.value[1];      // divide by 2^numAdcBits = 2^12
  pAdcData->I.value[1] = value;

  // convert current C
  value = (_iq)ADC_readResult(obj->adcHandle,ADC_ResultNumber_3);
  value = _IQ12mpy(value,current_sf) - obj->adcBias.I.value[2];      // divide by 2^numAdcBits = 2^12
  pAdcData->I.value[2] = value;

  // convert voltage A
  value = (_iq)ADC_readResult(obj->adcHandle,ADC_ResultNumber_4);
  value = _IQ12mpy(value,voltage_sf) - obj->adcBias.V.value[0];      // divide by 2^numAdcBits = 2^12
  pAdcData->V.value[0] = value;

  // convert voltage B
  value = (_iq)ADC_readResult(obj->adcHandle,ADC_ResultNumber_5);
  value = _IQ12mpy(value,voltage_sf) - obj->adcBias.V.value[1];      // divide by 2^numAdcBits = 2^12
  pAdcData->V.value[1] = value;

  // convert voltage C
  value = (_iq)ADC_readResult(obj->adcHandle,ADC_ResultNumber_6);
  value = _IQ12mpy(value,voltage_sf) - obj->adcBias.V.value[2];      // divide by 2^numAdcBits = 2^12
  pAdcData->V.value[2] = value;

  // read the dcBus voltage value
  value = (_iq)ADC_readResult(obj->adcHandle,ADC_ResultNumber_7);     // divide by 2^numAdcBits = 2^12
  value = _IQ12mpy(value,voltage_sf);
  pAdcData->dcBus = value;

  return;
} // end of HAL_readAdcData() function

//! \brief Reads the Potentiometer
//! \param[in] handle The hardware abstraction layer (HAL) handle
//! \return The potentiometer value from _IQ(-1.0) to _IQ(1.0)
static inline _iq HAL_readPotentiometerData(HAL_Handle handle)
{
 HAL_Obj *obj = (HAL_Obj *)handle;
 _iq value;
 // convert potentiometer from IQ12 to IQ24.
 value = _IQ12toIQ((_iq)ADC_readResult(obj->adcHandle,ADC_ResultNumber_8));
 return(value);
} // end of HAL_readPotentiometerData() function

//! \brief      Reads the ADC data
//! \details    Reads in the ADC result registers, and
//!             scales the values according to the settings in user.h.  The
//!             structure gAdcData holds three phase voltages, three line
//!             currents, and one DC bus voltage.
//! \param[in]  handle    The hardware abstraction layer (HAL) handle
//! \param[in]  pAdcData  A pointer to the ADC data buffer
static inline void HAL_readAdcDataWithOffsets(HAL_Handle handle,HAL_AdcData_t *pAdcData)
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  _iq value;
  _iq current_sf = HAL_getCurrentScaleFactor(handle);
  _iq voltage_sf = HAL_getVoltageScaleFactor(handle);


  // convert current A
  // sample the first sample twice due to errata sprz342f, ignore the first sample
  value = (_iq)ADC_readResult(obj->adcHandle,ADC_ResultNumber_1);
  value = _IQ12mpy(value,current_sf);
  pAdcData->I.value[0] = value;

  // convert current B
  value = (_iq)ADC_readResult(obj->adcHandle,ADC_ResultNumber_2);
  value = _IQ12mpy(value,current_sf);
  pAdcData->I.value[1] = value;

  // convert current C
  value = (_iq)ADC_readResult(obj->adcHandle,ADC_ResultNumber_3);
  value = _IQ12mpy(value,current_sf);
  pAdcData->I.value[2] = value;

  // convert voltage A
  value = (_iq)ADC_readResult(obj->adcHandle,ADC_ResultNumber_4);
  value = _IQ12mpy(value,voltage_sf);
  pAdcData->V.value[0] = value;

  // convert voltage B
  value = (_iq)ADC_readResult(obj->adcHandle,ADC_ResultNumber_5);
  value = _IQ12mpy(value,voltage_sf);
  pAdcData->V.value[1] = value;

  // convert voltage C
  value = (_iq)ADC_readResult(obj->adcHandle,ADC_ResultNumber_6);
  value = _IQ12mpy(value,voltage_sf);
  pAdcData->V.value[2] = value;

  // read the dcBus voltage value
  value = (_iq)ADC_readResult(obj->adcHandle,ADC_ResultNumber_7);
  value = _IQ12mpy(value,voltage_sf);
  pAdcData->dcBus = value;

  return;
} // end of HAL_readAdcDataWithOffsets() function


//! \brief     Reads the timer count
//! \param[in] handle       The hardware abstraction layer (HAL) handle
//! \param[in] timerNumber  The timer number, 0,1 or 2
//! \return    The timer count
static inline uint32_t HAL_readTimerCnt(HAL_Handle handle,const uint_least8_t timerNumber)
{
  HAL_Obj *obj = (HAL_Obj *)handle;
  uint32_t timerCnt = TIMER_getCount(obj->timerHandle[timerNumber]);

  return(timerCnt);
} // end of HAL_readTimerCnt() function


//! \brief     Reloads the timer
//! \param[in] handle       The hardware abstraction layer (HAL) handle
//! \param[in] timerNumber  The timer number, 0,1 or 2
static inline void HAL_reloadTimer(HAL_Handle handle,const uint_least8_t timerNumber)
{
  HAL_Obj  *obj = (HAL_Obj *)handle;

  // reload the specified timer
  TIMER_reload(obj->timerHandle[timerNumber]);

  return;
}  // end of HAL_reloadTimer() function


//! \brief     Sets up the GATE object
//! \param[in] handle       The hardware abstraction layer (HAL) handle
void HAL_setupGate(HAL_Handle handle);


//! \brief     Sets up the MCP2515 object
//! \param[in] handle       The hardware abstraction layer (HAL) handle
void HAL_setupMCP2515(HAL_Handle handle);


//! \brief     Starts the timer
//! \param[in] handle       The hardware abstraction layer (HAL) handle
//! \param[in] timerNumber  The timer number, 0,1 or 2
static inline void HAL_startTimer(HAL_Handle handle,const uint_least8_t timerNumber)
{
  HAL_Obj  *obj = (HAL_Obj *)handle;

  // start the specified timer
  TIMER_start(obj->timerHandle[timerNumber]);

  return;
}  // end of HAL_startTimer() function


//! \brief     Stops the timer
//! \param[in] handle       The hardware abstraction layer (HAL) handle
//! \param[in] timerNumber  The timer number, 0,1 or 2
static inline void HAL_stopTimer(HAL_Handle handle,const uint_least8_t timerNumber)
{
  HAL_Obj  *obj = (HAL_Obj *)handle;

  // stop the specified timer
  TIMER_stop(obj->timerHandle[timerNumber]);

  return;
}  // end of HAL_stopTimer() function


//! \brief     Sets the timer period
//! \param[in] handle       The hardware abstraction layer (HAL) handle
//! \param[in] timerNumber  The timer number, 0,1 or 2
//! \param[in] period       The timer period
static inline void HAL_setTimerPeriod(HAL_Handle handle,const uint_least8_t timerNumber, const uint32_t period)
{
  HAL_Obj  *obj = (HAL_Obj *)handle;

  // set the period
  TIMER_setPeriod(obj->timerHandle[timerNumber], period);

  return;
}  // end of HAL_setTimerPeriod() function


//! \brief     Gets the timer period
//! \param[in] handle       The hardware abstraction layer (HAL) handle
//! \param[in] timerNumber  The timer number, 0,1 or 2
//! \return    The timer period
static inline uint32_t HAL_getTimerPeriod(HAL_Handle handle,const uint_least8_t timerNumber)
{
  HAL_Obj  *obj = (HAL_Obj *)handle;

  uint32_t timerPeriod = TIMER_getPeriod(obj->timerHandle[timerNumber]);

  return(timerPeriod);
}  // end of HAL_getTimerPeriod() function


//! \brief     Sets the ADC SOC sample delay value
//! \param[in] handle       The hardware abstraction layer (HAL) handle
//! \param[in] socNumber    The SOC number
//! \param[in] sampleDelay  The delay value for the ADC
static inline void HAL_setAdcSocSampleDelay(HAL_Handle handle,
                                            const ADC_SocNumber_e socNumber,
                                            const ADC_SocSampleDelay_e sampleDelay)
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  ADC_setSocSampleDelay(obj->adcHandle,socNumber,sampleDelay);

  return;
} // end of HAL_setAdcSocSampleDelay() function


//! \brief     Sets the ADC bias value
//! \param[in] handle        The hardware abstraction layer (HAL) handle
//! \param[in] sensorType    The sensor type
//! \param[in] sensorNumber  The sensor number
//! \param[in] bias          The ADC bias value
static inline void HAL_setBias(HAL_Handle handle,
                               const HAL_SensorType_e sensorType,
                               uint_least8_t sensorNumber,
                               const _iq bias)
{
  HAL_Obj *obj = (HAL_Obj *)handle;


  if(sensorType == HAL_SensorType_Current)
    {
      obj->adcBias.I.value[sensorNumber] = bias;
    }
  else if(sensorType == HAL_SensorType_Voltage)
    {
      obj->adcBias.V.value[sensorNumber] = bias;
    }

  return;
} // end of HAL_setBias() function


//! \brief      Sets the GPIO pin high
//! \details    Takes in the enumeration GPIO_Number_e and sets that GPIO
//!             pin high.
//! \param[in]  handle      The hardware abstraction layer (HAL) handle
//! \param[in]  gpioNumber  The GPIO number
static inline void HAL_setGpioHigh(HAL_Handle handle,const GPIO_Number_e gpioNumber)
{
  HAL_Obj *obj = (HAL_Obj *)handle;
  

  // set GPIO high
  GPIO_setHigh(obj->gpioHandle,gpioNumber);

  return;
} // end of HAL_setGpioHigh() function


//! \brief      Toggles the GPIO pin
//! \details    Takes in the enumeration GPIO_Number_e and toggles that GPIO
//!             pin.
//! \param[in]  handle      The hardware abstraction layer (HAL) handle
//! \param[in]  gpioNumber  The GPIO number
static inline void HAL_toggleGpio(HAL_Handle handle,const GPIO_Number_e gpioNumber)
{
  HAL_Obj *obj = (HAL_Obj *)handle;


  // set GPIO high
  GPIO_toggle(obj->gpioHandle,gpioNumber);

  return;
} // end of HAL_setGpioHigh() function


//! \brief      Sets the GPIO pin low
//! \details    Takes in the enumeration GPIO_Number_e and clears that GPIO
//!             pin low.
//! \param[in]  handle      The hardware abstraction layer (HAL) handle
//! \param[in]  gpioNumber  The GPIO number
static inline void HAL_setGpioLow(HAL_Handle handle,const GPIO_Number_e gpioNumber)
{
  HAL_Obj *obj = (HAL_Obj *)handle;
  

  // set GPIO low
  GPIO_setLow(obj->gpioHandle,gpioNumber);

  return;
} // end of HAL_setGpioLow() function


//! \brief     Sets the current scale factor in the hardware abstraction layer
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \param[in] current_sf  The current scale factor
static inline void HAL_setCurrentScaleFactor(HAL_Handle handle,const _iq current_sf)
{
  HAL_Obj *obj = (HAL_Obj *)handle;
  

  obj->current_sf = current_sf;

  return;
} // end of HAL_setCurrentScaleFactor() function


//! \brief     Sets the number of current sensors
//! \param[in] handle             The hardware abstraction layer (HAL) handle
//! \param[in] numCurrentSensors  The number of current sensors
static inline void HAL_setNumCurrentSensors(HAL_Handle handle,const uint_least8_t numCurrentSensors)
{
  HAL_Obj *obj = (HAL_Obj *)handle;
  

  obj->numCurrentSensors = numCurrentSensors;

  return;
} // end of HAL_setNumCurrentSensors() function


//! \brief     Sets the number of voltage sensors
//! \param[in] handle             The hardware abstraction layer (HAL) handle
//! \param[in] numVoltageSensors  The number of voltage sensors
static inline void HAL_setNumVoltageSensors(HAL_Handle handle,const uint_least8_t numVoltageSensors)
{
  HAL_Obj *obj = (HAL_Obj *)handle;
  

  obj->numVoltageSensors = numVoltageSensors;

  return;
} // end of HAL_setNumVoltageSensors() function


//! \brief     Sets the value used to set the low pass filter pole for offset estimation
//! \param[in] handle        The hardware abstraction layer (HAL) handle
//! \param[in] sensorType    The sensor type
//! \param[in] sensorNumber  The sensor number
//! \param[in] beta_lp_pu    The value used to set the low pass filter pole, pu
static inline void HAL_setOffsetBeta_lp_pu(HAL_Handle handle,
                                           const HAL_SensorType_e sensorType,
                                           const uint_least8_t sensorNumber,
                                           const _iq beta_lp_pu)
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  if(sensorType == HAL_SensorType_Current)
    {
      OFFSET_setBeta(obj->offsetHandle_I[sensorNumber],beta_lp_pu);
    }
  else if(sensorType == HAL_SensorType_Voltage)
    {
      OFFSET_setBeta(obj->offsetHandle_V[sensorNumber],beta_lp_pu);
    }

  return;
} // end of HAL_setOffsetBeta_lp_pu() function


//! \brief     Sets the offset initial condition value for offset estimation
//! \param[in] handle        The hardware abstraction layer (HAL) handle
//! \param[in] sensorType    The sensor type
//! \param[in] sensorNumber  The sensor number
//! \param[in] initCond      The initial condition value
static inline void HAL_setOffsetInitCond(HAL_Handle handle,
                                         const HAL_SensorType_e sensorType,
                                         const uint_least8_t sensorNumber,
                                         const _iq initCond)
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  if(sensorType == HAL_SensorType_Current)
    {
      OFFSET_setInitCond(obj->offsetHandle_I[sensorNumber],initCond);
    }
  else if(sensorType == HAL_SensorType_Voltage)
    {
      OFFSET_setInitCond(obj->offsetHandle_V[sensorNumber],initCond);
    }

  return;
} // end of HAL_setOffsetInitCond() function


//! \brief     Sets the initial offset value for offset estimation
//! \param[in] handle        The hardware abstraction layer (HAL) handle
//! \param[in] sensorType    The sensor type
//! \param[in] sensorNumber  The sensor number
//! \param[in] value         The initial offset value
static inline void HAL_setOffsetValue(HAL_Handle handle,
                                      const HAL_SensorType_e sensorType,
                                      const uint_least8_t sensorNumber,
                                      const _iq value)
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  if(sensorType == HAL_SensorType_Current)
    {
      OFFSET_setOffset(obj->offsetHandle_I[sensorNumber],value);
    }
  else if(sensorType == HAL_SensorType_Voltage)
    {
      OFFSET_setOffset(obj->offsetHandle_V[sensorNumber],value);
    }

  return;
} // end of HAL_setOffsetValue() function


//! \brief     Sets the voltage scale factor in the hardware abstraction layer
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \param[in] voltage_sf  The voltage scale factor
static inline void HAL_setVoltageScaleFactor(HAL_Handle handle,const _iq voltage_sf)
{
  HAL_Obj *obj = (HAL_Obj *)handle;
  
  obj->voltage_sf = voltage_sf;

  return;
} // end of HAL_setVoltageScaleFactor() function


//! \brief      Sets the hardware abstraction layer parameters
//! \details    Sets up the microcontroller peripherals.  Creates all of the scale
//!             factors for the ADC voltage and current conversions.  Sets the initial
//!             offset values for voltage and current measurements.
//! \param[in]  handle       The hardware abstraction layer (HAL) handle
//! \param[in]  pUserParams  The pointer to the user parameters
extern void HAL_setParams(HAL_Handle handle,const USER_Params *pUserParams);


//! \brief      Sets up the ADCs (Analog to Digital Converters)
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupAdcs(HAL_Handle handle);


//! \brief      Sets up the clocks
//! \details    Sets up the micro-controller's main oscillator
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupClks(HAL_Handle handle);


//! \brief     Sets up the FLASH.
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupFlash(HAL_Handle handle);


//! \brief     Sets up the GPIO (General Purpose I/O) pins
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupGpios(HAL_Handle handle);


//! \brief          Setup GPIO 0 and 1 as outputs for the LaunchPad project lab 1 experiment
//! \details        Since the LaunchPad uses the same GPIO pins for LEDs as PWM pins,
//!                 the GPIO 0 and 1 are setup as an extra step.  GPIO 1 will be
//!                 kept low so that there is no possibility of shoot-through conduction
//!                 with the high side switch that is controlled by GPIO0.
//! \param[in]      handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupLaunchPadGpio0and1(HAL_Handle handle);


//! \brief     Sets up the peripheral clocks
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupPeripheralClks(HAL_Handle handle);


//! \brief     Sets up the PIE (Peripheral Interrupt Expansion)
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupPie(HAL_Handle handle);


//! \brief     Sets up the PLL (Phase Lock Loop)
//! \param[in] handle   The hardware abstraction layer (HAL) handle
//! \param[in] clkFreq  The clock frequency
extern void HAL_setupPll(HAL_Handle handle,const PLL_ClkFreq_e clkFreq);


//! \brief     Sets up the PWMs (Pulse Width Modulators)
//! \param[in] handle          The hardware abstraction layer (HAL) handle
//! \param[in] systemFreq_MHz  The system frequency, MHz
//! \param[in] pwmPeriod_usec  The PWM period, usec
//! \param[in] numPwmTicksPerIsrTick  The number of PWM clock ticks per ISR clock tick
extern void HAL_setupPwms(HAL_Handle handle,
                   const float_t systemFreq_MHz,
                   const float_t pwmPeriod_usec,
                   const uint_least16_t numPwmTicksPerIsrTick);


//! \brief     Sets up the spiA peripheral
//! \param[in] handle   The hardware abstraction layer (HAL) handle
extern void HAL_setupSpiA(HAL_Handle handle);


//! \brief     Sets up the timers
//! \param[in] handle          The hardware abstraction layer (HAL) handle
//! \param[in] systemFreq_MHz  The system frequency, MHz
extern void HAL_setupTimers(HAL_Handle handle,const float_t systemFreq_MHz);


//! \brief      Updates the ADC bias values
//! \details    This function is called before the motor is started.  It sets the voltage
//!             and current measurement offsets.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
static inline void HAL_updateAdcBias(HAL_Handle handle)
{
  uint_least8_t cnt;
  HAL_Obj *obj = (HAL_Obj *)handle;
  _iq bias;


  // update the current bias
  for(cnt=0;cnt<HAL_getNumCurrentSensors(handle);cnt++)
    {
      bias = HAL_getBias(handle,HAL_SensorType_Current,cnt);
      
      bias += OFFSET_getOffset(obj->offsetHandle_I[cnt]);

      HAL_setBias(handle,HAL_SensorType_Current,cnt,bias);
    }


  // update the voltage bias
  for(cnt=0;cnt<HAL_getNumVoltageSensors(handle);cnt++)
    {
      bias = HAL_getBias(handle,HAL_SensorType_Voltage,cnt);

      bias += OFFSET_getOffset(obj->offsetHandle_V[cnt]);

      HAL_setBias(handle,HAL_SensorType_Voltage,cnt,bias);
    }

  return;
} // end of HAL_updateAdcBias() function


//! \brief     Writes DAC data to the PWM comparators for DAC (digital-to-analog conversion) output
//! \param[in] handle    The hardware abstraction layer (HAL) handle
//! \param[in] pDacData  The pointer to the DAC data
static inline void HAL_writeDacData(HAL_Handle handle,HAL_DacData_t *pDacData)
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  // convert values from _IQ to _IQ15
  uint_least8_t cnt;
  _iq period;
  _iq dacData_sat_dc;
  _iq value;
  uint16_t cmpValue[4];

  period = (_iq)pDacData->PeriodMax;

  for(cnt=0;cnt<4;cnt++)
	{
	  dacData_sat_dc = _IQmpy(pDacData->value[cnt], pDacData->gain[cnt]) + pDacData->offset[cnt];
	  value = _IQmpy(dacData_sat_dc, period);
	  cmpValue[cnt] = (uint16_t)_IQsat(value, period, 0);
	}

  // write the DAC data
  if(obj->pwmDacHandle[PWMDAC_Number_1])
  {
	  PWMDAC_write_CmpA(obj->pwmDacHandle[PWMDAC_Number_1], cmpValue[0]);
	  PWMDAC_write_CmpB(obj->pwmDacHandle[PWMDAC_Number_1], cmpValue[1]);
  }

  if(obj->pwmDacHandle[PWMDAC_Number_2])
  {
	  PWMDAC_write_CmpA(obj->pwmDacHandle[PWMDAC_Number_2], cmpValue[2]);
	  PWMDAC_write_CmpB(obj->pwmDacHandle[PWMDAC_Number_2], cmpValue[3]);
  }

  return;
} // end of HAL_writeDacData() function


//! \brief     Writes PWM data to the PWM comparators for motor control
//! \param[in] handle    The hardware abstraction layer (HAL) handle
//! \param[in] pPwmData  The pointer to the PWM data
static inline void HAL_writePwmData(HAL_Handle handle,HAL_PwmData_t *pPwmData)
{
  uint_least8_t cnt;
  HAL_Obj *obj = (HAL_Obj *)handle;
  PWM_Obj *pwm;
  _iq period;
  _iq pwmData_neg;
  _iq pwmData_sat;
  _iq pwmData_sat_dc;
  _iq value;
  uint16_t value_sat;

  for(cnt=0;cnt<3;cnt++)
    {
      pwm = (PWM_Obj *)obj->pwmHandle[cnt];
      period = (_iq)pwm->TBPRD;
      pwmData_neg = _IQmpy(pPwmData->Tabc.value[cnt],_IQ(-1.0));
      pwmData_sat = _IQsat(pwmData_neg,_IQ(0.5),_IQ(-0.5));
      pwmData_sat_dc = pwmData_sat + _IQ(0.5);
      value = _IQmpy(pwmData_sat_dc, period);
      value_sat = (uint16_t)_IQsat(value, period, _IQ(0.0));

      // write the PWM data
      PWM_write_CmpA(obj->pwmHandle[cnt],value_sat);
    }

  return;
} // end of HAL_writePwmData() function


//! \brief     Reads PWM compare register A
//! \param[in] handle    The hardware abstraction layer (HAL) handle
//! \param[in] pwmNumber  The PWM number
//! \return    The PWM compare value
static inline uint16_t HAL_readPwmCmpA(HAL_Handle handle,const PWM_Number_e pwmNumber)
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  // the compare value to be returned
  uint16_t pwmValue;

  pwmValue = PWM_get_CmpA(obj->pwmHandle[pwmNumber]);

  return(pwmValue);
} // end of HAL_readPwmCmpA() function


//! \brief     Reads PWM compare mirror register A
//! \param[in] handle    The hardware abstraction layer (HAL) handle
//! \param[in] pwmNumber  The PWM number
//! \return    The PWM compare value
static inline uint16_t HAL_readPwmCmpAM(HAL_Handle handle,const PWM_Number_e pwmNumber)
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  // the compare value to be returned
  uint16_t pwmValue;

  pwmValue = PWM_get_CmpAM(obj->pwmHandle[pwmNumber]);

  return(pwmValue);
} // end of HAL_readPwmCmpAM() function


//! \brief     Reads PWM compare register B
//! \param[in] handle    The hardware abstraction layer (HAL) handle
//! \param[in] pwmNumber  The PWM number
//! \return    The PWM compare value
static inline uint16_t HAL_readPwmCmpB(HAL_Handle handle,const PWM_Number_e pwmNumber)
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  // the compare value to be returned
  uint16_t pwmValue;

  pwmValue = PWM_get_CmpB(obj->pwmHandle[pwmNumber]);

  return(pwmValue);
} // end of HAL_readPwmCmpB() function


//! \brief     Reads PWM period register
//! \param[in] handle    The hardware abstraction layer (HAL) handle
//! \param[in] pwmNumber  The PWM number
//! \return    The PWM period value
static inline uint16_t HAL_readPwmPeriod(HAL_Handle handle,const PWM_Number_e pwmNumber)
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  // the period value to be returned
  uint16_t pwmPeriodValue;

  pwmPeriodValue = PWM_getPeriod(obj->pwmHandle[pwmNumber]);

  return(pwmPeriodValue);
} // end of HAL_readPwmPeriod() function


//! \brief     Set trigger point in the middle of the low side pulse
//! \param[in] handle    The hardware abstraction layer (HAL) handle
//! \param[in] ignoreShunt  The low side shunt that should be ignored
//! \param[in] midVolShunt  The middle length of output voltage
static inline void HAL_setTrigger(HAL_Handle handle,const SVGENCURRENT_IgnoreShunt_e ignoreShunt,
									const SVGENCURRENT_VmidShunt_e midVolShunt)
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  PWM_Obj *pwm1 = (PWM_Obj *)obj->pwmHandle[PWM_Number_1];
  PWM_Obj *pwm2 = (PWM_Obj *)obj->pwmHandle[PWM_Number_2];
  PWM_Obj *pwm3 = (PWM_Obj *)obj->pwmHandle[PWM_Number_3];
  PWM_Obj *pwm;

  uint16_t nextPulse1 = (pwm1->CMPA + pwm1->CMPAM) / 2;
  uint16_t nextPulse2 = (pwm2->CMPA + pwm2->CMPAM) / 2;
  uint16_t nextPulse3 = (pwm3->CMPA + pwm3->CMPAM) / 2;
  uint16_t pwmCMPA1 = pwm1->CMPA;
  uint16_t pwmCMPA2 = pwm2->CMPA;
  uint16_t pwmCMPA3 = pwm3->CMPA;

  if(ignoreShunt == use_all)
    {
      if((nextPulse1 <= nextPulse2) && (nextPulse1 <= nextPulse3))
        {
          pwm = pwm1;
        }
      else if((nextPulse2 <= nextPulse1) && (nextPulse2 <= nextPulse3))
        {
          pwm = pwm2;
        }
      else
        {
          pwm = pwm3;
        }
    }
  else
  {
	  if(midVolShunt == Vmid_a)
	  {
		  pwm = pwm1;
	  }
	  else if(midVolShunt == Vmid_b)
	  {
		  pwm = pwm2;
	  }
	  else
	  {
		  pwm = pwm3;
	  }
  }

  pwmCMPA1 = pwm->CMPA;
  pwmCMPA2 = pwm->CMPAM;

  if(pwmCMPA2 >= (pwmCMPA1 + pwm->DBFED))
  {
	  pwmCMPA3 = (pwmCMPA2 - (pwmCMPA1 + pwm->DBFED)) / 2 + 1;
	  if(pwmCMPA3 < (pwm1->TBPRD>>1))
	  {
		  pwm1->CMPB = pwmCMPA3;
	  }
	  else
	  {
		  pwm1->CMPB = (pwm1->TBPRD>>1);
	  }
	  PWM_setSocAPulseSrc(obj->pwmHandle[PWM_Number_1],PWM_SocPulseSrc_CounterEqualCmpBDecr);
  }
  else
  {
	  pwmCMPA3 = ((pwmCMPA1 + pwm->DBFED) - pwmCMPA2 ) / 2 + 1;
	  if(pwmCMPA3 < (pwm1->TBPRD>>1))
	  {
		  pwm1->CMPB = pwmCMPA3;
	  }
	  else
	  {
		  pwm1->CMPB = (pwm1->TBPRD>>1);
	  }
	  PWM_setSocAPulseSrc(obj->pwmHandle[PWM_Number_1],PWM_SocPulseSrc_CounterEqualCmpBIncr);
  }

  return;
} // end of HAL_setTrigger() function


//! \brief     Selects the analog channel used for calibration
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \param[in] chanNumber  The channel number
void HAL_AdcCalChanSelect(HAL_Handle handle, const ADC_SocChanNumber_e chanNumber);


//! \brief     Reads the converted value from the selected calibration channel
//! \param[in] handle     The hardware abstraction layer (HAL) handle
//! \return    The converted value
uint16_t HAL_AdcCalConversion(HAL_Handle handle);


//! \brief     Executes the offset calibration of the ADC
//! \param[in] handle     The hardware abstraction layer (HAL) handle
void HAL_AdcOffsetSelfCal(HAL_Handle handle);


//! \brief     Converts coarse and fine oscillator trim values into a single 16bit word value
//! \param[in] handle     The hardware abstraction layer (HAL) handle
//! \param[in] coarse     The coarse trim portion of the oscillator trim
//! \param[in] fine       The fine trim portion of the oscillator trim
//! \return    The combined trim value
uint16_t HAL_getOscTrimValue(int16_t coarse, int16_t fine);


//! \brief     Executes the oscillator 1 and 2 calibration functions
//! \param[in] handle     The hardware abstraction layer (HAL) handle
void HAL_OscTempComp(HAL_Handle handle);


//! \brief     Executes the oscillator 1 calibration based on input sample
//! \param[in] handle     The hardware abstraction layer (HAL) handle
void HAL_osc1Comp(HAL_Handle handle, const int16_t sensorSample);


//! \brief     Executes the oscillator 2 calibration based on input sample
//! \param[in] handle     The hardware abstraction layer (HAL) handle
void HAL_osc2Comp(HAL_Handle handle, const int16_t sensorSample);


//! \brief     Writes data to the driver
//! \param[in] handle         The hardware abstraction layer (HAL) handle
//! \param[in] Spi_8305_Vars  SPI variables
void HAL_writeDrvData(HAL_Handle handle, DRV_SPI_8305_Vars_t *Spi_8305_Vars);


//! \brief     Reads data from the driver
//! \param[in] handle         The hardware abstraction layer (HAL) handle
//! \param[in] Spi_8305_Vars  SPI variables
void HAL_readDrvData(HAL_Handle handle, DRV_SPI_8305_Vars_t *Spi_8305_Vars);


//! \brief     Sets up the SPI interface for the driver
//! \param[in] handle         The hardware abstraction layer (HAL) handle
//! \param[in] Spi_8305_Vars  SPI variables
void HAL_setupDrvSpi(HAL_Handle handle, DRV_SPI_8305_Vars_t *Spi_8305_Vars);

//! \brief     Writes DAC data to the PWM comparators for DAC (digital-to-analog conversion) output
//! \param[in] handle    The hardware abstraction layer (HAL) handle
//! \param[in] pDacData  The pointer to the DAC data
void HAL_setDacParameters(HAL_Handle handle, HAL_DacData_t *pDacData);

//! \brief     Sets up the SciA peripheral
//! \param[in] handle    The hardware abstraction layer (HAL) handle
extern void HAL_setupSciA(HAL_Handle handle);

//! \brief     Enables the Timer0 interrupt
//! \param[in] handle    The hardware abstraction layer (HAL) handle
extern void HAL_enableTimer0Int(HAL_Handle handle);



#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _HAL_H_ definition


