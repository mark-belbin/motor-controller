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

#ifndef EST_H
#define EST_H

//! \file   /libraries/observers/est/include/est.h
//! \brief  Contains the public interface to the
//!         estimator (EST) module routines
//!


// **************************************************************************
// the includes

// modules
#include "libraries/math/include/math.h"

#include "ctrl_obj.h"

#include "est_Flux_states.h"
#include "est_Traj_states.h"
#include "est_states.h"

//!
//!
//! \defgroup EST EST
//!
//! @{

#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines


// **************************************************************************
// the typedefs

//! \brief Enumeration for the Rs online filter types
//!
typedef enum
{
  EST_RSONLINEFILTERTYPE_CURRENT=0,        //!< Current Filter
  EST_RSONLINEFILTERTYPE_VOLTAGE           //!< Voltage Filter
} EST_RsOnLineFilterType_e;


//! \brief Defines the estimator (EST) input data
//!
typedef struct _EST_InputData_
{
  int32_t    timeStamp;        //!< a time stamp for the input data buffer

  MATH_Vec2        Iab_A;            //!< the alpha/beta current values, A
  MATH_Vec2        Vab_V;            //!< the alpha/beta current values, V
  float32_t          dcBus_V;          //!< the DC bus voltage value, V
  float32_t          speed_ref_Hz;     //!< the speed reference value, Hz
  float32_t          speed_int_Hz;     //!< the speed int value, Hz
} EST_InputData_t;


//! \brief Defines the estimator (EST) output data
//!
typedef struct _EST_OutputData_
{
  int32_t      timeStamp;                 //!< a time stamp for the output data buffer

  float32_t            angle_rad;                 //!< the estimated angle value at t = m+1, rad

  float32_t            fe_rps;                    //!< the electrical frequency estimate, rad/sec
  float32_t            fm_rps;                    //!< the mechanical frequency estimate, rad/sec
  float32_t            fm_lp_rps;                 //!< the low pass filtered mechanical frequency estimate, rad/sec
  float32_t            fmDot_rps2;                //!< the mechanical acceleration estimate, rad/sec^2
  float32_t            fslip_rps;                 //!< the slip frequency estimate, rad/sec

  MATH_Vec2          Eab_V;                     //!< the alpha/beta back-EMF estimates, V
  MATH_Vec2          Edq_V;                     //!< the direction/quadrature back-EMF estimates, V

  int16_t       direction;                 //!< the rotational direction estimate, unitless

  float32_t            oneOverDcBus_invV;         //!< the DC Bus inverse, 1/V
} EST_OutputData_t;


//! \brief Defines the estimator (EST) handle
//!
typedef struct _EST_Obj_ *EST_Handle;


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes

//! \brief     Computes the magnetizing inductance in Henries (H)
//! \param[in] handle   The estimator (EST) handle
//! \param[in] current  The current in the rotor, A
//! \return    The magnetizing inductance, H
extern float32_t     EST_computeLmag_H(EST_Handle handle,const float32_t current_A);
extern float32_t cla_EST_computeLmag_H(EST_Handle handle,const float32_t current_A);


//! \brief     Computes the power in Watts (W)
//! \param[in] handle  The estimator (EST) handle
//! \return    The power value, W
extern float32_t     EST_computePower_W(EST_Handle handle);
extern float32_t cla_EST_computePower_W(EST_Handle handle);


//! \brief     Computes the torque value in per Newton-meter (Nm)
//! \param[in] handle  The estimator (EST) handle
//! \return    The torque value, N*m
extern float32_t     EST_computeTorque_Nm(EST_Handle handle);
extern float32_t cla_EST_computeTorque_Nm(EST_Handle handle);


//! \brief     Configures the controller for each of the estimator states
//! \param[in] handle      The estimator (EST) handle
//! \param[in] ctrlHandle  The controller (CTRL) handle
extern void     EST_configureCtrl(EST_Handle handle,CTRL_Handle ctrlHandle);
extern void cla_EST_configureCtrl(EST_Handle handle,CTRL_Handle ctrlHandle);


//! \brief     Configures the trajectory generator for each of the estimator states
//! \param[in] handle  The estimator (EST) handle
extern void     EST_configureTraj(EST_Handle handle);
extern void cla_EST_configureTraj(EST_Handle handle);


//! \brief     Disables the estimator
//! \param[in] handle  The estimator (EST) handle
extern void     EST_disable(EST_Handle handle);
extern void cla_EST_disable(EST_Handle handle);


//! \brief     Disables the estimator trajectory generator
//! \param[in] handle  The estimator (EST) handle
extern void     EST_disableTraj(EST_Handle handle);
extern void cla_EST_disableTraj(EST_Handle handle);


//! \brief     Determines if current control should be performed during motor identification
//! \param[in] handle  The estimator (EST) handle
//! \return    A boolean value denoting whether (true) or not (false) to perform current control
extern bool     EST_doCurrentCtrl(EST_Handle handle);
extern bool cla_EST_doCurrentCtrl(EST_Handle handle);


//! \brief     Determines if speed control should be performed during motor identification
//! \param[in] handle  The estimator (EST) handle
//! \return    A boolean value denoting whether (true) or not (false) to perform speed control
extern bool     EST_doSpeedCtrl(EST_Handle handle);
extern bool cla_EST_doSpeedCtrl(EST_Handle handle);


//! \brief     Enables the estimator
//! \param[in] handle  The estimator (EST) handle
extern void     EST_enable(EST_Handle handle);
extern void cla_EST_enable(EST_Handle handle);


//! \brief     Enables the estimator trajectory generator
//! \param[in] handle  The estimator (EST) handle
extern void     EST_enableTraj(EST_Handle handle);
extern void cla_EST_enableTraj(EST_Handle handle);


//! \brief     Gets the rpm to rad/sec scale factor
//! \param[in] handle  The estimator (EST) handle
//! \return    The rpm to rad/sec scale factor
extern float32_t     EST_get_rpm_to_rps_sf(EST_Handle handle);
extern float32_t cla_EST_get_rpm_to_rps_sf(EST_Handle handle);


//! \brief     Gets the rad/sec to rpm scale factor
//! \param[in] handle  The estimator (EST) handle
//! \return    The rad/sec to rpm scale factor
extern float32_t     EST_get_rps_to_rpm_sf(EST_Handle handle);
extern float32_t cla_EST_get_rps_to_rpm_sf(EST_Handle handle);


//! \brief     Gets the mechanical acceleration from the estimator
//! \param[in] handle  The estimator (EST) handle
//! \return    The mechanical acceleration, rad/sec^2
extern float32_t     EST_getAccel_rps2(EST_Handle handle);
extern float32_t cla_EST_getAccel_rps2(EST_Handle handle);


//! \brief     Gets the estimated angle from the estimator
//! \param[in] handle  The estimator (EST) handle
//! \return    The estimated angle, rad
extern float32_t     EST_getAngle_est_rad(EST_Handle handle);
extern float32_t cla_EST_getAngle_est_rad(EST_Handle handle);


//! \brief     Gets the angle estimate for t = n+1 from the estimator in radians (rad).
//! \details   This function returns the angle value in units of radians.  This value wraps around
//!            at 2*pi, so the return value is between -pi and pi.
//!            An example of using this angle is shown:
//! \code
//! float32_t rotorFluxAngle_rad = EST_getAngle_rad(handle);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The angle value for t = n+1, rad
extern float32_t     EST_getAngle_rad(EST_Handle handle);
extern float32_t cla_EST_getAngle_rad(EST_Handle handle);


//! \brief     Gets the trajectory angle from the estimator
//! \param[in] handle  The estimator (EST) handle
//! \return    The trajectory angle, rad
extern float32_t     EST_getAngle_traj_rad(EST_Handle handle);
extern float32_t cla_EST_getAngle_traj_rad(EST_Handle handle);


//! \brief     Gets the angle delta estimate for t = n+1 from the estimator
//! \brief     or trajectory depending on state in radians (rad).
//! \param[in] handle  The estimator (EST) handle
//! \return    The angle delta value for t = n+1, rad
extern float32_t     EST_getAngleDelta_rad(EST_Handle handle);
extern float32_t cla_EST_getAngleDelta_rad(EST_Handle handle);


//! \brief     Gets the angle delta estimate for t = n+1 from the estimator in radians (rad).
//! \param[in] handle  The estimator (EST) handle
//! \return    The angle delta value for t = n+1, rad
extern float32_t     EST_getAngleDelta_est_rad(EST_Handle handle);
extern float32_t cla_EST_getAngleDelta_est_rad(EST_Handle handle);


//! \brief     Gets the angle delta estimate for t = n+1 from the trajectory in radians (rad).
//! \param[in] handle  The estimator (EST) handle
//! \return    The angle delta value for t = n+1, rad
extern float32_t     EST_getAngleDelta_traj_rad(EST_Handle handle);
extern float32_t cla_EST_getAngleDelta_traj_rad(EST_Handle handle);


//! \brief     Gets the ISR count from the estimator
//! \param[in] handle  The estimator (EST) handle
//! \return    The ISR count
extern int16_t     EST_getCount_isr(EST_Handle handle);
extern int16_t cla_EST_getCount_isr(EST_Handle handle);


//! \brief     Gets the DC bus value from the estimator in volts (V)
//! \param[in] handle  The estimator (EST) handle
//! \return    The DC bus value, V
extern float32_t     EST_getDcBus_V(EST_Handle handle);
extern float32_t cla_EST_getDcBus_V(EST_Handle handle);


//! \brief     Gets the alpha/beta back EMF voltage vector from the estimator
//! \param[in] handle  The estimator (EST) handle
//! \param[in] pEab_V  The pointer to memory for the Eab vector, V
extern void     EST_getEab_V(EST_Handle handle,MATH_Vec2 *pEab_V);
extern void cla_EST_getEab_V(EST_Handle handle,MATH_Vec2 *pEab_V);


//! \brief     Gets the direct/quadrature back EMF voltage vector from the estimator
//! \param[in] handle  The estimator (EST) handle
//! \param[in] pEdq_V  The pointer to memory for the Edq vector, V
extern void     EST_getEdq_V(EST_Handle handle,MATH_Vec2 *pEdq_V);
extern void cla_EST_getEdq_V(EST_Handle handle,MATH_Vec2 *pEdq_V);


//! \brief     Gets the error code from the estimator
//! \param[in] handle  The estimator (EST) handle
//! \return    The error code
extern EST_ErrorCode_e     EST_getErrorCode(EST_Handle handle);
extern EST_ErrorCode_e cla_EST_getErrorCode(EST_Handle handle);


//! \brief     Gets the electrical frequency of the motor in Hertz (Hz).
//! \details   This frequency, in Hz, is the frequency of currents and voltages going into the motor.
//!            In order to get the speed of the motor, it is better to use EST_getFm_Hz().
//! \param[in] handle  The estimator (EST) handle
//! \return    The electrical frequency, Hz
extern float32_t     EST_getFe_Hz(EST_Handle handle);
extern float32_t cla_EST_getFe_Hz(EST_Handle handle);


//! \brief     Gets the absolute maximum electrical frequency of the motor in rad/sec
//! \param[in] handle  The estimator (EST) handle
//! \return    The absolute maximum electrical frequency, rad/sec
extern float32_t     EST_getFe_abs_max_rps(EST_Handle handle);
extern float32_t cla_EST_getFe_abs_max_rps(EST_Handle handle);


//! \brief     Gets the electrical frequency of the motor in rad/sec.
//! \details   This frequency, in Hz, is the frequency of currents and voltages going into the motor.
//!            In order to get the speed of the motor, it is better to use EST_getFm_rps().
//! \param[in] handle  The estimator (EST) handle
//! \return    The electrical frequency, rad/sec
extern float32_t     EST_getFe_rps(EST_Handle handle);
extern float32_t cla_EST_getFe_rps(EST_Handle handle);


//! \brief     Gets the bypass lock rotor flag value
//! \param[in] handle  The estimator (EST) handle
//! \return    The bypass lock rotor flag value
extern bool     EST_getFlag_bypassLockRotor(EST_Handle handle);
extern bool cla_EST_getFlag_bypassLockRotor(EST_Handle handle);


//! \brief     Gets the enable estimator flag value
//! \param[in] handle  The estimator (EST) handle
//! \return    The enable estimator flag value
extern bool     EST_getFlag_enable(EST_Handle handle);
extern bool cla_EST_getFlag_enable(EST_Handle handle);


//! \brief     Gets the enable force angle flag value from the estimator.
//! \param[in] handle  The estimator (EST) handle
//! \return    The value of the flag, in boolean type, bool
extern bool     EST_getFlag_enableForceAngle(EST_Handle handle);
extern bool cla_EST_getFlag_enableForceAngle(EST_Handle handle);


//! \brief     Gets the enable power warp flag value from the trajectory generator
//! \param[in] handle  The estimator (EST) handle
//! \return    The enable power warp flag value
extern bool     EST_getFlag_enablePowerWarp(EST_Handle handle);
extern bool cla_EST_getFlag_enablePowerWarp(EST_Handle handle);


//! \brief     Gets the value of the flag which enables online stator resistance (Rs) estimation
//! \param[in] handle  The estimator (EST) handle
//! \return    The enable online Rs flag value
//! \retval
//!            true   Rs online recalibration algorithm is enabled. The estimator will run a set of
//!                   functions related to rs online which recalculates the stator resistance while the
//!                   motor is rotating. This algorithm is useful when motor heats up, and hence stator
//!                   resistance increases.
//! \retval
//!            false  Rs online recalibration algorithm is disabled, and no updates to Rs will be made
//!                   even if the motor heats up. Low speed performace, and startup performance with
//!                   full torque might be affected if stator resistance changes due to motor heating
//!                   up. The stator resistance will be fixed, and equal to the value returned by:
//!                   EST_getRs_Ohm().
extern bool     EST_getFlag_enableRsOnLine(EST_Handle handle);
extern bool cla_EST_getFlag_enableRsOnLine(EST_Handle handle);


//! \brief     Gets the enable stator resistance re-calibrate flag value from the estimator
//! \param[in] handle  The estimator (EST) handle
//! \return    The value of the enable stator resistance re-calibrate flag
//! \retval
//!            true   Rs recalibration is enabled. The estimator will inject a DC current to the D-axis
//!                   to recalibrate the stator resistance before the motor rotates. It is required that
//!                   the motor is at standstill to perform Rs recalibration.
//! \retval
//!            false  Rs recalibration is disabled. The estimator will start the motor with the resistance
//!                   value that was used before the motor was stopped, or what is returned by function:
//!                   EST_getRs_Ohm().
extern bool     EST_getFlag_enableRsRecalc(EST_Handle handle);
extern bool cla_EST_getFlag_enableRsRecalc(EST_Handle handle);


//! \brief     Gets the enable trajectory flag value from the trajectory generator
//! \param[in] handle  The estimator (EST) handle
//! \return    The enable trajectory flag value
extern bool     EST_getFlag_enableTraj(EST_Handle handle);
extern bool cla_EST_getFlag_enableTraj(EST_Handle handle);


//! \brief     Gets the value of the flag which denotes when the estimation is complete
//! \details   This flag is set to true every time the EST_run() function is run.
//!            This flag can be reset to false by using the following example:
//! \code
//! bool estComplete_Flag = EST_getFlag_estComplete(handle);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The estimation complete flag value
//! \retval
//!            true   The estimator has been run at least once since last time.
//!                   EST_setFlag_estComplete(handle, false) was called.
//! \retval
//!            false  The estimator has not been run since last time EST_setFlag_estComplete(handle, false)
//!                   was called.
extern bool     EST_getFlag_estComplete(EST_Handle handle);
extern bool cla_EST_getFlag_estComplete(EST_Handle handle);


//! \brief     Gets the motor identified flag state in the estimator
//! \param[in] handle  The estimator (EST) handle
//! \return    The state of the motor identified flag
extern bool     EST_getFlag_motorIdentified(EST_Handle handle);
extern bool cla_EST_getFlag_motorIdentified(EST_Handle handle);


//! \brief     Gets the near zero speed flag state
//! \param[in] handle  The estimator (EST) handle
//! \return    The state of the near zero speed flag
extern bool     EST_getFlag_nearZeroSpeed(EST_Handle handle);
extern bool cla_EST_getFlag_nearZeroSpeed(EST_Handle handle);


//! \brief     Gets the value of the flag which enables the updating of the stator resistance (Rs) value
//! \details   When the online resistance estimator is enabled, the update flag allows the online resistance
//!            to be copied to the resistance used by the estimator model. If the update flag is not set to true,
//!            the online resistance estimation will not be used by the estimator model, and if the resistance
//!            changes too much due to temperature increase, the model may not work as expected.
//! \code
//! bool update_Flag = EST_getFlag_updateRs(handle);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The update Rs flag value
//! \retval
//!            true   The stator resistance estimated by the Rs OnLine module will be copied to
//!                   the stator resistance used by the module, so of the motor's temperature changes,
//!                   the estimated angle will be calculated based on the most up to date stator
//!                   resistance
//! \retval
//!            false  The stator resistance estimated by the Rs OnLine module may or may not be updated
//!                   depending on the enable flag, but will not be used in the motor's model used to generate
//!                   the estimated speed and angle.
extern bool     EST_getFlag_updateRs(EST_Handle handle);
extern bool cla_EST_getFlag_updateRs(EST_Handle handle);


//! \brief     Gets the flux value in Weber (Wb).
//! \details   The estimator continuously calculates the flux linkage between the rotor and stator, which is the
//!            portion of the flux that produces torque. This function returns the flux linkage, ignoring the
//!            number of turns, between the rotor and stator coils, in Weber, or Wb, or Volts * Seconds / rad (V.sec/rad).
//!            This functions returns a precise value only after the motor has been identified, which can be
//!            checked by the following code example:
//! \code
//! if(EST_isMotorIdentified(handle))
//!   {
//!     // once the motor has been identified, get the flux
//!     float32_t Flux_Wb = EST_getFlux_Wb(handle);
//!   }
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The flux value, Weber or V*sec/rad, in floating point
extern float32_t     EST_getFlux_Wb(EST_Handle handle);
extern float32_t cla_EST_getFlux_Wb(EST_Handle handle);


//! \brief     Gets the flux estimator state
//! \param[in] handle  The estimator (EST) handle
//! \return    The flux estimator state
extern EST_Flux_State_e     EST_getFluxState(EST_Handle handle);
extern EST_Flux_State_e cla_EST_getFluxState(EST_Handle handle);


//! \brief     Gets the mechanical frequency of the motor in Hertz (Hz).
//! \details   This frequency, in Hz, is the mechanical frequency of the motor. If the motor is a permanent
//!            magnet motor, the mechanical frequency will be equal to the electrical frequency, since it is
//!            a synchronous motor. In the case of AC induction motors, the mechanical frequency will be equal
//!            to the electrical frequency minus the slip frequency. The following code example shows how to
//!            use this function to calculate revolutions per minute (RPM) in floating point:
//! \code
//! #define USER_MOTOR_NUM_POLE_PAIRS  (2)
//!
//! float32_t mechFreq_Hz = EST_getFm_Hz(handle);
//! float32_t hz_to_rpm_sf = 60.0/(float32_t)USER_MOTOR_NUM_POLE_PAIRS;
//! float32_t speed_rpm = mechFreq_Hz * hz_to_rpm_sf;
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The mechanical frequency, Hz
extern float32_t     EST_getFm_Hz(EST_Handle handle);
extern float32_t cla_EST_getFm_Hz(EST_Handle handle);


//! \brief     Gets the low pass filtered mechanical frequency of the motor in Hz
//! \param[in] handle  The estimator (EST) handle
//! \return    The low pass filtered mechanical frequency, Hz
extern float32_t     EST_getFm_lp_Hz(EST_Handle handle);
extern float32_t cla_EST_getFm_lp_Hz(EST_Handle handle);


//! \brief     Gets the low pass filtered mechanical frequency of the motor in rad/sec.
//! \param[in] handle  The estimator (EST) handle
//! \return    The low pass filtered mechanical frequency, rad/sec
extern float32_t     EST_getFm_lp_rps(EST_Handle handle);
extern float32_t cla_EST_getFm_lp_rps(EST_Handle handle);


//! \brief     Gets the mechanical frequency of the motor in rad/sec.
//! \details   This frequency, in rad/sec, is the mechanical frequency of the motor. If the motor is a permanent
//!            magnet motor, the mechanical frequency will be equal to the electrical frequency, since it is
//!            a synchronous motor. In the case of AC induction motors, the mechanical frequency will be equal
//!            to the electrical frequency minus the slip frequency. The following code example shows how to
//!            use this function to calculate revolutions per minute (RPM) in floating point:
//! \code
//! #define USER_MOTOR_NUM_POLE_PAIRS  (2)
//!
//! float32_t mechFreq_rps = EST_getFm_rps(handle);
//! float32_t rps_to_rpm_sf = (float32_t)60.0/(MATH_TWO_PI * (float32_t)USER_MOTOR_NUM_POLE_PAIRS);
//! float32_t speed_rpm = mechFreq_rps * rps_to_rpm_sf;
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The mechanical frequency, rad/sec
extern float32_t     EST_getFm_rps(EST_Handle handle);
extern float32_t cla_EST_getFm_rps(EST_Handle handle);


//! \brief     Gets the force angle delta value from the estimator in radians (rad).
//! \param[in] handle  The estimator (EST) handle
//! \return    The force angle delta, rad
extern float32_t     EST_getForceAngleDelta_rad(EST_Handle handle);
extern float32_t cla_EST_getForceAngleDelta_rad(EST_Handle handle);


//! \brief     Gets the status of the force angle operation in the estimator
//! \param[in] handle  The estimator (EST) handle
//! \return    A boolean value denoting whether the angle has been forced (true) or not (false)
//! \retval
//!            true   The last iteration of the estimator used a forced angle to run the park and inverse park
//!                   transforms. The estimator was also run in parallel to the forced angle, but the estimator
//!                   output was not used.
//! \retval
//!            false  Forced angle mode is either disabled, or the electrical frequency did not fall below
//!                   the predetermined threshold. The estimator output was used to run the park and
//!                   inverse park transforms.
extern bool     EST_getForceAngleStatus(EST_Handle handle);
extern bool cla_EST_getForceAngleStatus(EST_Handle handle);


//! \brief     Gets the value used to set the pole location in the low-pass filter of the frequency estimator
//!            in radians per second (rps).
//! \param[in] handle  The estimator (EST) handle
//! \return    The value used to set the filter pole location, rad
extern float32_t     EST_getFreqBeta_lp(EST_Handle handle);
extern float32_t cla_EST_getFreqBeta_lp(EST_Handle handle);


//! \brief     Gets the slip frequency of the motor in Hertz (Hz).
//! \details   When running a permanent magnet motor, the slip frequency returned by this function will be zero.
//!            If an induction motor is used, this function will return the slip frequency. This frequency, in Hz,
//!            will be the difference between the electrical frequency and the mechanical frequency.\n
//!            \f[F_{slip}=F_{electrical}-F_{mechanical}\f]
//!
//! \param[in] handle  The estimator (EST) handle
//! \return    The slip frequency, Hz
extern float32_t     EST_getFslip_Hz(EST_Handle handle);
extern float32_t cla_EST_getFslip_Hz(EST_Handle handle);


//! \brief     Gets the slip frequency of the motor in rad/sec.
//! \details   When running a permanent magnet motor, the slip frequency returned by this function will be zero.
//!            If an induction motor is used, this function will return the slip frequency. This frequency, in rad/sec,
//!            will be the difference between the electrical frequency and the mechanical frequency.\n
//!            \f[F_{slip}=F_{electrical}-F_{mechanical}\f]
//!
//! \param[in] handle  The estimator (EST) handle
//! \return    The slip frequency, rad/sec
extern float32_t     EST_getFslip_rps(EST_Handle handle);
extern float32_t cla_EST_getFslip_rps(EST_Handle handle);


//! \brief     Gets the Iab current vector in Ampere (A).
//! \param[in] handle  The estimator (EST) handle
//! \param[in] pIab_A  The pointer to memory for the Iab vector, A
extern void     EST_getIab_A(EST_Handle handle,MATH_Vec2 *pIab_A);
extern void cla_EST_getIab_A(EST_Handle handle,MATH_Vec2 *pIab_A);


//! \brief     Gets the low pass filtered Iab current vector in Ampere (A).
//! \param[in] handle     The estimator (EST) handle
//! \param[in] pIab_lp_A  The pointer to memory for the low pass filtered Iab vector, A
extern void     EST_getIab_lp_A(EST_Handle handle,MATH_Vec2 *pIab_lp_A);
extern void cla_EST_getIab_lp_A(EST_Handle handle,MATH_Vec2 *pIab_lp_A);


//! \brief     Gets the beta value for the Iab low pass filter
//! \param[in] handle  The estimator (EST) handle
//! \return    The beta value, rad
extern float32_t     EST_getIab_beta_lp(EST_Handle handle);
extern float32_t cla_EST_getIab_beta_lp(EST_Handle handle);


//! \brief     Gets the Idq current vector in Ampere (A)
//! \param[in] handle  The estimator (EST) handle
//! \param[in] pIdq_A  The pointer to memory for the Idq vector, A
extern void     EST_getIdq_A(EST_Handle handle,MATH_Vec2 *pIdq_A);
extern void cla_EST_getIdq_A(EST_Handle handle,MATH_Vec2 *pIdq_A);


//! \brief     Gets the Idq low pass filtered current vector in Ampere (A).
//! \param[in] handle     The estimator (EST) handle
//! \param[in] pIdq_lp_A  The pointer to memory for the Idq low pass filtered vector, A
extern void     EST_getIdq_lp_A(EST_Handle handle,MATH_Vec2 *pIdq_lp_A);
extern void cla_EST_getIdq_lp_A(EST_Handle handle,MATH_Vec2 *pIdq_lp_A);


//! \brief     Gets the Idq current vector reference in Ampere (A).
//! \param[in] handle      The estimator (EST) handle
//! \param[in] pIdq_ref_A  The pointer to memory for the Idq vector reference, A
extern void     EST_getIdq_ref_A(EST_Handle handle,MATH_Vec2 *pIdq_ref_A);
extern void cla_EST_getIdq_ref_A(EST_Handle handle,MATH_Vec2 *pIdq_ref_A);


//! \brief     Gets the Id rated current value from the estimator in Ampere (A).
//! \param[in] handle  The estimator (EST) handle
//! \return    The Id rated current value, A
extern float32_t     EST_getIdRated_A(EST_Handle handle);
extern float32_t cla_EST_getIdRated_A(EST_Handle handle);


//! \brief     Gets the Id rated delta current value from the estimator in Ampere (A)
//! \param[in] handle  The estimator (EST) handle
//! \return    The delta Id rated current value, A
extern float32_t     EST_getIdRated_delta_A(EST_Handle handle);
extern float32_t cla_EST_getIdRated_delta_A(EST_Handle handle);


//! \brief     Gets the Id current value used for inductance estimation of induction motors
//!            in Ampere (A).
//! \param[in] handle  The estimator (EST) handle
//! \return    The Id rated value, A
extern float32_t     EST_getIdRated_indEst_A(EST_Handle handle);
extern float32_t cla_EST_getIdRated_indEst_A(EST_Handle handle);


//! \brief     Gets the intermediate value from the Id trajectory generator
//! \param[in] handle  The estimator (EST) handle
//! \return    The intermediate value, A
extern float32_t     EST_getIntValue_Id_A(EST_Handle handle);
extern float32_t cla_EST_getIntValue_Id_A(EST_Handle handle);


//! \brief     Gets the intermediate value from the Iq trajectory generator
//! \param[in] handle  The estimator (EST) handle
//! \return    The intermediate value, A
extern float32_t     EST_getIntValue_Iq_A(EST_Handle handle);
extern float32_t cla_EST_getIntValue_Iq_A(EST_Handle handle);


//! \brief     Gets the intermediate value from the speed trajectory generator
//! \param[in] handle  The estimator (EST) handle
//! \return    The intermediate value, Hz
extern float32_t     EST_getIntValue_spd_Hz(EST_Handle handle);
extern float32_t cla_EST_getIntValue_spd_Hz(EST_Handle handle);


//! \brief     Gets the Lhf value from the estimator
//! \param[in] handle  The estimator (EST) handle
//! \return    The Lhf value, H
extern float32_t     EST_getLhf_H(EST_Handle handle);
extern float32_t cla_EST_getLhf_H(EST_Handle handle);


//! \brief     Gets the magnetizing inductance value in Henry (H).
//! \param[in] handle  The estimator (EST) handle
//! \return    The magnetizing inductance value, H
extern float32_t     EST_getLmag_H(EST_Handle handle);
extern float32_t cla_EST_getLmag_H(EST_Handle handle);


//! \brief     Gets the delta stator inductance value during coarse estimation
//! \param[in] handle  The estimator (EST) handle
//! \return    The delta stator inductance value for coarse estimation, H
extern float32_t     EST_getLs_coarseDelta_H(EST_Handle handle);
extern float32_t cla_EST_getLs_coarseDelta_H(EST_Handle handle);


//! \brief     Gets the direct stator inductance value in Henry (H).
//! \param[in] handle  The estimator (EST) handle
//! \return    The direct stator inductance value, H
extern float32_t     EST_getLs_d_H(EST_Handle handle);
extern float32_t cla_EST_getLs_d_H(EST_Handle handle);


//! \brief     Gets the direct/quadrature stator inductance values from the estimator in Henry (H).
//! \details   Both direct and quadrature stator inductances can be read from the estimator by using
//!            this function call and passing a pointer to a structure where these two values will be
//!            stored.
//! \param[in] handle     The estimator (EST) handle
//! \param[out] pLs_dq_H  The pointer to the vector of direct/quadrature stator inductance values, H
extern void     EST_getLs_dq_H(EST_Handle handle,MATH_Vec2 *pLs_dq_H);
extern void cla_EST_getLs_dq_H(EST_Handle handle,MATH_Vec2 *pLs_dq_H);


//! \brief     Gets the delta stator inductance value during fine estimation
//! \param[in] handle  The estimator (EST) handle
//! \return    The delta stator inductance value for fine estimation, H
extern float32_t     EST_getLs_fineDelta_H(EST_Handle handle);
extern float32_t cla_EST_getLs_fineDelta_H(EST_Handle handle);


//! \brief     Gets the stator inductance value in Henry (H)
//! \param[in] handle  The estimator (EST) handle
//! \return    The stator inductance value, H
extern float32_t     EST_getLs_H(EST_Handle handle);
extern float32_t cla_EST_getLs_H(EST_Handle handle);


//! \brief     Gets the stator inductance value in the quadrature coordinate direction in Henry (H).
//! \param[in] handle  The estimator (EST) handle
//! \return    The stator inductance value, H
extern float32_t     EST_getLs_q_H(EST_Handle handle);
extern float32_t cla_EST_getLs_q_H(EST_Handle handle);


//! \brief     Gets the maximum current slope value used in the estimator in Amperes (A).
//! \param[in] handle  The estimator (EST) handle
//! \return    The maximum current delta value, A
extern float32_t     EST_getMaxCurrentDelta_A(EST_Handle handle);
extern float32_t cla_EST_getMaxCurrentDelta_A(EST_Handle handle);


//! \brief     Gets the maximum power warp current slope value used in the estimator
//!            in Amperes (A).
//! \param[in] handle  The estimator (EST) handle
//! \return    The maximum power warp current delta value, A
extern float32_t     EST_getMaxCurrentDelta_pw_A(EST_Handle handle);
extern float32_t cla_EST_getMaxCurrentDelta_pw_A(EST_Handle handle);


//! \brief     Gets the maximum delta value from the Id trajectory generator
//! \param[in] handle  The estimator (EST) handle
//! \return    The maximum delta value, A
extern float32_t     EST_getMaxDelta_Id_A(EST_Handle handle);
extern float32_t cla_EST_getMaxDelta_Id_A(EST_Handle handle);


//! \brief     Gets the maximum delta value from the Iq trajectory generator
//! \param[in] handle  The estimator (EST) handle
//! \return    The maximum delta value, A
extern float32_t     EST_getMaxDelta_Iq_A(EST_Handle handle);
extern float32_t cla_EST_getMaxDelta_Iq_A(EST_Handle handle);


//! \brief     Gets the maximum delta value from the speed trajectory generator
//! \param[in] handle  The estimator (EST) handle
//! \return    The maximum delta value, Hz
extern float32_t     EST_getMaxDelta_spd_Hz(EST_Handle handle);
extern float32_t cla_EST_getMaxDelta_spd_Hz(EST_Handle handle);


//! \brief     Gets the maximum acceleration value used in the estimator in Hertz (Hz).
//! \param[in] handle  The estimator (EST) handle
//! \return    The maximum speed delta value, Hz
extern float32_t     EST_getMaxSpeedDelta_Hz(EST_Handle handle);
extern float32_t cla_EST_getMaxSpeedDelta_Hz(EST_Handle handle);


//! \brief     Gets the maximum value from the Id trajectory generator
//! \param[in] handle  The estimator (EST) handle
//! \return    The maximum value, A
extern float32_t     EST_getMaxValue_Id_A(EST_Handle handle);
extern float32_t cla_EST_getMaxValue_Id_A(EST_Handle handle);


//! \brief     Gets the maximum value from the Iq trajectory generator
//! \param[in] handle  The estimator (EST) handle
//! \return    The maximum value, A
extern float32_t     EST_getMaxValue_Iq_A(EST_Handle handle);
extern float32_t cla_EST_getMaxValue_Iq_A(EST_Handle handle);


//! \brief     Gets the maximum value from the speed trajectory generator
//! \param[in] handle  The estimator (EST) handle
//! \return    The maximum value, Hz
extern float32_t     EST_getMaxValue_spd_Hz(EST_Handle handle);
extern float32_t cla_EST_getMaxValue_spd_Hz(EST_Handle handle);


//! \brief     Gets the minimum value from the Id trajectory generator
//! \param[in] handle  The estimator (EST) handle
//! \return    The minimum value, A
extern float32_t     EST_getMinValue_Id_A(EST_Handle handle);
extern float32_t cla_EST_getMinValue_Id_A(EST_Handle handle);


//! \brief     Gets the minimum value from the Iq trajectory generator
//! \param[in] handle  The estimator (EST) handle
//! \return    The minimum value, A
extern float32_t     EST_getMinValue_Iq_A(EST_Handle handle);
extern float32_t cla_EST_getMinValue_Iq_A(EST_Handle handle);


//! \brief     Gets the minimum value from the speed trajectory generator
//! \param[in] handle  The estimator (EST) handle
//! \return    The minimum value, Hz
extern float32_t     EST_getMinValue_spd_Hz(EST_Handle handle);
extern float32_t cla_EST_getMinValue_spd_Hz(EST_Handle handle);


//! \brief     Gets the number of Interrupt Service Routine (ISR) clock ticks per estimator clock tick
//! \param[in] handle  The estimator (EST) handle
//! \return    The number of Interrupt Service Routine (ISR) clock ticks per estimator clock tick
extern int16_t     EST_getNumIsrTicksPerEstTick(EST_Handle handle);
extern int16_t cla_EST_getNumIsrTicksPerEstTick(EST_Handle handle);


//! \brief     Gets the number of Interrupt Service Routine (ISR) clock ticks per estimator trajectory clock tick
//! \param[in] handle  The estimator (EST) handle
//! \return    The number of Interrupt Service Routine (ISR) clock ticks per estimator trajectory clock tick
extern int16_t     EST_getNumIsrTicksPerTrajTick(EST_Handle handle);
extern int16_t cla_EST_getNumIsrTicksPerTrajTick(EST_Handle handle);


//! \brief     Gets the inverse of the DC bus voltage in 1/Volt (1/V).
//! \param[in] handle  The estimator (EST) handle
//! \return    The inverse of the DC bus voltage, 1/V
extern float32_t     EST_getOneOverDcBus_invV(EST_Handle handle);
extern float32_t cla_EST_getOneOverDcBus_invV(EST_Handle handle);


//! \brief     Gets the power warp gain from the trajectory generator
//! \param[in] handle  The estimator (EST) handle
//! \return    The power warp gain, unitless
extern float32_t     EST_getPwGain(EST_Handle handle);
extern float32_t cla_EST_getPwGain(EST_Handle handle);


//! \brief     Gets the Rhf value from the estimator
//! \param[in] handle  The estimator (EST) handle
//! \return    The Rhf value, Ohm
extern float32_t     EST_getRhf_Ohm(EST_Handle handle);
extern float32_t cla_EST_getRhf_Ohm(EST_Handle handle);


//! \brief     Gets the R/L value from the estimator
//! \param[in] handle  The estimator (EST) handle
//! \return    The R/L value, rad/sec
extern float32_t     EST_getRoverL_rps(EST_Handle handle);
extern float32_t cla_EST_getRoverL_rps(EST_Handle handle);


//! \brief     Gets the coarse delta rotor resistance value from the estimator
//! \param[in] handle  The estimator (EST) handle
//! \return    The coarse delta rotor resistance value, Ohm
extern float32_t     EST_getRr_coarseDelta_Ohm(EST_Handle handle);
extern float32_t cla_EST_getRr_coarseDelta_Ohm(EST_Handle handle);


//! \brief     Gets the direct rotor resistance value in Ohm (\f$\Omega\f$).
//! \param[in] handle  The estimator (EST) handle
//! \return    The direct rotor resistance value, Ohm
extern float32_t     EST_getRr_d_Ohm(EST_Handle handle);
extern float32_t cla_EST_getRr_d_Ohm(EST_Handle handle);


//! \brief     Gets the direct/quadrature rotor resistance values from the estimator in Ohms (\f$\Omega\f$).
//! \details   Both direct and quadrature rotor resistances can be read from the estimator by using
//!            this function call and passing a pointer to a structure where these two values will be
//!            stored.
//! \param[in] handle       The estimator (EST) handle
//! \param[out] pRr_dq_Ohm  The pointer to the vector of direct/quadrature rotor resistance values, Ohm
extern void     EST_getRr_dq_Ohm(EST_Handle handle,MATH_Vec2 *pRr_dq_Ohm);
extern void cla_EST_getRr_dq_Ohm(EST_Handle handle,MATH_Vec2 *pRr_dq_Ohm);


//! \brief     Gets the fine delta rotor resistance value from the estimator
//! \param[in] handle  The estimator (EST) handle
//! \return    The fine delta rotor resistance value, Ohm
extern float32_t     EST_getRr_fineDelta_Ohm(EST_Handle handle);
extern float32_t cla_EST_getRr_fineDelta_Ohm(EST_Handle handle);


//! \brief     Gets the rotor resistance value in Ohms (\f$\Omega\f$).
//! \param[in] handle  The estimator (EST) handle
//! \return    The rotor resistance value, Ohm
extern float32_t     EST_getRr_Ohm(EST_Handle handle);
extern float32_t cla_EST_getRr_Ohm(EST_Handle handle);


//! \brief     Gets the quadrature rotor resistance value in Ohms (\f$\Omega\f$).
//! \param[in] handle  The estimator (EST) handle
//! \return    The quadrature rotor resistance value, Ohm
extern float32_t     EST_getRr_q_Ohm(EST_Handle handle);
extern float32_t cla_EST_getRr_q_Ohm(EST_Handle handle);


//! \brief     Gets the alpha stator resistance value in Ohms (\f$\Omega\f$).
//! \param[in] handle  The estimator (EST) handle
//! \return    The alpha stator resistance value, Ohm
extern float32_t     EST_getRs_a_Ohm(EST_Handle handle);
extern float32_t cla_EST_getRs_a_Ohm(EST_Handle handle);


//! \brief     Gets the alpha/beta stator resistance values from the estimator in Ohms (\f$\Omega\f$).
//! \details   Both alpha and beta stator resistances can be read from the estimator by using
//!            this function call and passing a pointer to a structure where these two values will be
//!            stored.
//! \param[in] handle       The estimator (EST) handle
//! \param[out] pRs_ab_Ohm  The pointer to the vector of alpha/beta stator resistance values, Ohm
extern void     EST_getRs_ab_Ohm(EST_Handle handle,MATH_Vec2 *pRs_ab_Ohm);
extern void cla_EST_getRs_ab_Ohm(EST_Handle handle,MATH_Vec2 *pRs_ab_Ohm);


//! \brief     Gets the beta stator resistance value in Ohms (\f$\Omega\f$).
//! \param[in] handle  The estimator (EST) handle
//! \return    The beta stator resistance value, Ohm
extern float32_t     EST_getRs_b_Ohm(EST_Handle handle);
extern float32_t cla_EST_getRs_b_Ohm(EST_Handle handle);


//! \brief     Gets the coarse delta stator resistance value from the estimator
//! \param[in] handle  The estimator (EST) handle
//! \return    The coarse delta stator resistance value, Ohm
extern float32_t     EST_getRs_coarseDelta_Ohm(EST_Handle handle);
extern float32_t cla_EST_getRs_coarseDelta_Ohm(EST_Handle handle);


//! \brief     Gets the direct stator resistance value in Ohms (\f$\Omega\f$).
//! \param[in] handle  The estimator (EST) handle
//! \return    The direct stator resistance value, Ohm
extern float32_t     EST_getRs_d_Ohm(EST_Handle handle);
extern float32_t cla_EST_getRs_d_Ohm(EST_Handle handle);


//! \brief     Gets the direct/quadrature stator resistance values from the estimator in Ohms (\f$\Omega\f$).
//! \details   Both direct and quadrature stator resistances can be read from the estimator by using
//!            this function call and passing a pointer to a structure where these two values will be
//!            stored.
//! \param[in] handle       The estimator (EST) handle
//! \param[out] pRs_dq_Ohm  The pointer to the vector of direct/quadrature stator resistance values, Ohm
extern void     EST_getRs_dq_Ohm(EST_Handle handle,MATH_Vec2 *pRs_dq_Ohm);
extern void cla_EST_getRs_dq_Ohm(EST_Handle handle,MATH_Vec2 *pRs_dq_Ohm);


//! \brief     Gets the fine delta stator resistance value from the estimator
//! \param[in] handle  The estimator (EST) handle
//! \return    The delta stator resistance value, Ohm
extern float32_t     EST_getRs_fineDelta_Ohm(EST_Handle handle);
extern float32_t cla_EST_getRs_fineDelta_Ohm(EST_Handle handle);


//! \brief     Gets the quadrature stator resistance value in Ohms (\f$\Omega\f$).
//! \param[in] handle  The estimator (EST) handle
//! \return    The quadrature stator resistance value, Ohm
extern float32_t     EST_getRs_q_Ohm(EST_Handle handle);
extern float32_t cla_EST_getRs_q_Ohm(EST_Handle handle);


//! \brief     Gets the stator resistance value in Ohms (\f$\Omega\f$).
//! \param[in] handle  The estimator (EST) handle
//! \return    The stator resistance value, Ohm
extern float32_t     EST_getRs_Ohm(EST_Handle handle);
extern float32_t cla_EST_getRs_Ohm(EST_Handle handle);


//! \brief     Gets the online stator resistance value in Ohm (\f$\Omega\f$).
//! \param[in] handle  The estimator (EST) handle
//! \return    The online stator resistance value, Ohm
extern float32_t     EST_getRsOnLine_Ohm(EST_Handle handle);
extern float32_t cla_EST_getRsOnLine_Ohm(EST_Handle handle);


//! \brief     Gets the angle value from the estimator
//! \param[in] handle  The estimator (EST) handle
//! \return    The angle value, rad
extern float32_t     EST_getRsOnLineAngle_rad(EST_Handle handle);
extern float32_t cla_EST_getRsOnLineAngle_rad(EST_Handle handle);


//! \brief     Gets the delta angle value from the estimator
//! \param[in] handle  The estimator (EST) handle
//! \return    The delta angle value, rad
extern float32_t     EST_getRsOnLineAngleDelta_rad(EST_Handle handle);
extern float32_t cla_EST_getRsOnLineAngleDelta_rad(EST_Handle handle);


//! \brief     Gets the Id magnitude value used for online stator resistance estimation in Ampere (A).
//! \param[in] handle  The estimator (EST) handle
//! \return    The Id magnitude value, A
extern float32_t     EST_getRsOnLineId_mag_A(EST_Handle handle);
extern float32_t cla_EST_getRsOnLineId_mag_A(EST_Handle handle);


//! \brief     Gets the Id value used for online stator resistance estimation in Ampere (A).
//! \param[in] handle  The estimator (EST) handle
//! \return    The Id value, A
extern float32_t     EST_getRsOnLineId_A(EST_Handle handle);
extern float32_t cla_EST_getRsOnLineId_A(EST_Handle handle);


//! \brief     Gets the online stator resistance filter parameters
//! \param[in] handle        The estimator (EST) handle
//! \param[in] filterType    The filter type
//! \param[in] filterNumber  The filter number
//! \param[in] b0            The pointer for the numerator coefficient value for z^0
//! \param[in] b1            The pointer for the numerator coefficient value for z^(-1)
//! \param[in] a1            The pointer for the denominator coefficient value for z^(-1)
//! \param[in] x1            The pointer for the input value at time sample n=-1
//! \param[in] y1            The pointer for the output value at time sample n=-1
extern void     EST_getRsOnLineLpFilterParams(EST_Handle handle,const EST_RsOnLineFilterType_e filterType,
                                              const uint16_t filterNumber,
                                              float32_t *b0,float32_t *b1,float32_t *a1,
                                              float32_t *x1,float32_t *y1);
extern void cla_EST_getRsOnLineLpFilterParams(EST_Handle handle,const EST_RsOnLineFilterType_e filterType,
                                              const uint16_t filterNumber,
                                              float32_t *b0,float32_t *b1,float32_t *a1,
                                              float32_t *x1,float32_t *y1);


//! \brief     Gets the sign of the direction value in 16 bit signed integer (int16_t).
//! \param[in] handle  The estimator (EST) handle
//! \return    The sign of the direction value (-1 for negative, 1 for positive)
extern int16_t     EST_getSignOfDirection(EST_Handle handle);
extern int16_t cla_EST_getSignOfDirection(EST_Handle handle);


//! \brief     Gets the reference speed value from the estimator
//! \param[in] handle  The estimator (EST) handle
//! \return    The reference speed value, Hz
extern float32_t     EST_getSpeed_ref_Hz(EST_Handle handle);
extern float32_t cla_EST_getSpeed_ref_Hz(EST_Handle handle);


//! \brief     Gets the speed value in revolutions per second (rpm).
//! \param[in] handle  The estimator (EST) handle
//! \return    The speed value, rpm
extern float32_t     EST_getSpeed_rpm(EST_Handle handle);
extern float32_t cla_EST_getSpeed_rpm(EST_Handle handle);


//! \brief     Gets the state of the estimator
//! \param[in] handle  The estimator (EST) handle
//! \return    The estimator state
extern EST_State_e     EST_getState(EST_Handle handle);
extern EST_State_e cla_EST_getState(EST_Handle handle);


//! \brief     Gets the target value from the Id trajectory generator
//! \param[in] handle  The estimator (EST) handle
//! \return    The target value, A
extern float32_t     EST_getTargetValue_Id_A(EST_Handle handle);
extern float32_t cla_EST_getTargetValue_Id_A(EST_Handle handle);


//! \brief     Gets the target value from the Iq trajectory generator
//! \param[in] handle  The estimator (EST) handle
//! \return    The target value, A
extern float32_t     EST_getTargetValue_Iq_A(EST_Handle handle);
extern float32_t cla_EST_getTargetValue_Iq_A(EST_Handle handle);


//! \brief     Gets the target value from the speed trajectory generator
//! \param[in] handle  The estimator (EST) handle
//! \return    The target value, Hz
extern float32_t     EST_getTargetValue_spd_Hz(EST_Handle handle);
extern float32_t cla_EST_getTargetValue_spd_Hz(EST_Handle handle);


//! \brief     Gets the torque scale factor for the torque equation
//! \param[in] handle   The estimator (EST) handle
//! \return    The torque scale factor
extern float32_t     EST_getTorque_sf(EST_Handle handle);
extern float32_t cla_EST_getTorque_sf(EST_Handle handle);


//! \brief     Gets the trajectory ISR count from the estimator
//! \param[in] handle  The estimator (EST) handle
//! \return    The trajectory count value
extern int16_t     EST_getTrajCount_isr(EST_Handle handle);
extern int16_t cla_EST_getTrajCount_isr(EST_Handle handle);


//! \brief     Gets the trajectory generator state
//! \param[in] handle  The estimator (EST) handle
//! \return    The state
extern EST_Traj_State_e     EST_getTrajState(EST_Handle handle);
extern EST_Traj_State_e cla_EST_getTrajState(EST_Handle handle);


//! \brief     Gets the Vab voltage vector in Volts (V)
//! \param[in] handle  The estimator (EST) handle
//! \param[in] pVab_V  The pointer to memory for the Vab vector, V
extern void     EST_getVab_V(EST_Handle handle,MATH_Vec2 *pVab_V);
extern void cla_EST_getVab_V(EST_Handle handle,MATH_Vec2 *pVab_V);


//! \brief     Gets the low pass filtered Vdq voltage vector in Volts (V)
//! \param[in] handle     The estimator (EST) handle
//! \param[in] pVdq_lp_V  The pointer to memory for the low pass filtered Vdq vector, V
extern void     EST_getVdq_lp_V(EST_Handle handle,MATH_Vec2 *pVdq_lp_V);
extern void cla_EST_getVdq_lp_V(EST_Handle handle,MATH_Vec2 *pVdq_lp_V);


//! \brief     Gets the Vdq voltage vector in Volts (V)
//! \param[in] handle  The estimator (EST) handle
//! \param[in] pVdq_V  The pointer to memory for the Vdq vector, V
extern void     EST_getVdq_V(EST_Handle handle,MATH_Vec2 *pVdq_V);
extern void cla_EST_getVdq_V(EST_Handle handle,MATH_Vec2 *pVdq_V);


//! \brief     Increments the ISR counter in the estimator
//! \param[in] handle  The estimator (EST) handle
extern void     EST_incrCounter_isr(EST_Handle handle);
extern void cla_EST_incrCounter_isr(EST_Handle handle);


//! \brief     Increments the ISR counter in the trajectory generator
//! \param[in] handle  The estimator (EST) handle
extern void     EST_incrTrajCounter_isr(EST_Handle handle);
extern void cla_EST_incrTrajCounter_isr(EST_Handle handle);


//! \brief     Initializes the estimator
//! \param[in] estNumber  The estimator number
//! \return    The estimator (EST) handle
extern EST_Handle     EST_initEst(const uint16_t estNumber);
extern EST_Handle cla_EST_initEst(const uint16_t estNumber);


//! \brief     Determines if the estimator (EST) is enabled
//! \param[in] handle  The estimator (EST) handle
extern bool     EST_isEnabled(EST_Handle handle);
extern bool cla_EST_isEnabled(EST_Handle handle);


//! \brief     Determines if there is an error in the estimator
//! \param[in] handle  The estimator (EST) handle
//! \return    A boolean value denoting if there is an error (true) or not (false)
extern bool     EST_isError(EST_Handle handle);
extern bool cla_EST_isError(EST_Handle handle);


//! \brief     Determines if the estimator is idle
//! \param[in] handle  The estimator (EST) handle
//! \return    A boolean value denoting if the estimator is idle (true) or not (false)
extern bool     EST_isIdle(EST_Handle handle);
extern bool cla_EST_isIdle(EST_Handle handle);


//! \brief     Determines if the estimator is waiting for the rotor to be locked
//! \param[in] handle  The estimator (EST) handle
//! \return    A boolean value denoting if the estimator is waiting for the rotor to be locked (true) or not (false)
extern bool     EST_isLockRotor(EST_Handle handle);
extern bool cla_EST_isLockRotor(EST_Handle handle);


//! \brief     Determines if the motor has been identified
//! \param[in] handle  The estimator (EST) handle
//! \return    A boolean value denoting if the motor is identified (true) or not (false)
extern bool     EST_isMotorIdentified(EST_Handle handle);
extern bool cla_EST_isMotorIdentified(EST_Handle handle);


//! \brief     Determines if the estimator is not ready for online control
//! \param[in] handle  The estimator (EST) handle
//! \return    A boolean value denoting if the estimator is not ready for online control (true) or not (false)
extern bool     EST_isNotOnLine(EST_Handle handle);
extern bool cla_EST_isNotOnLine(EST_Handle handle);


//! \brief     Determines if the estimator is ready for online control
//! \param[in] handle  The estimator (EST) handle
//! \return    A boolean value denoting if the estimator is ready for online control (true) or not (false)
extern bool     EST_isOnLine(EST_Handle handle);
extern bool cla_EST_isOnLine(EST_Handle handle);


//! \brief     Determines if the trajectory generator is enabled
//! \param[in] handle  The estimator (EST) handle
extern bool     EST_isTrajEnabled(EST_Handle handle);
extern bool cla_EST_isTrajEnabled(EST_Handle handle);


//! \brief     Determines if there is an error in the trajectory generator
//! \param[in] handle  The estimator (EST) handle
//! \return    A boolean value denoting if there is an error (true) or not (false)
extern bool     EST_isTrajError(EST_Handle handle);
extern bool cla_EST_isTrajError(EST_Handle handle);


//! \brief     Resets the isr counter
//! \param[in] handle  The estimator (EST) handle
extern void     EST_resetCounter_isr(EST_Handle handle);
extern void cla_EST_resetCounter_isr(EST_Handle handle);


//! \brief     Resets the state counter
//! \param[in] handle  The estimator (EST) handle
extern void     EST_resetCounter_state(EST_Handle handle);
extern void cla_EST_resetCounter_state(EST_Handle handle);


//! \brief     Resets the trajectory ISR counter
//! \param[in] handle  The estimator (EST) handle
extern void     EST_resetTrajCounter_isr(EST_Handle handle);
extern void cla_EST_resetTrajCounter_isr(EST_Handle handle);


//! \brief     Runs the estimator
//! \param[in] handle       The estimator (EST) handle
//! \param[in] pInputData   The pointer to the input data
//! \param[in] pOutputData  The pointer to the output data
extern void     EST_run(EST_Handle handle,
                        const EST_InputData_t *pInputData,
                        EST_OutputData_t *pOutputData);
extern void cla_EST_run(EST_Handle handle,
                        const EST_InputData_t *pInputData,
                        EST_OutputData_t *pOutputData);


//! \brief     Runs PowerWarp
//! \param[in] handle         The estimator (EST) handle
//! \param[in] Id_int_A   The intermediate value along the Id trajectory in Amperes
//! \param[in] Iq_A       The measured Iq value in Amperes
//! \return    The target value for the Id trajectory in Amperes
extern float32_t     EST_runPowerWarp(EST_Handle handle,const float32_t Id_int_A,const float32_t Iq_A);
extern float32_t cla_EST_runPowerWarp(EST_Handle handle,const float32_t Id_int_A,const float32_t Iq_A);


//! \brief     Runs the trajectory generator
//! \param[in] handle  The estimator (EST) handle
extern void     EST_runTraj(EST_Handle handle);
extern void cla_EST_runTraj(EST_Handle handle);


//! \brief     Sets the mechanical acceleration in the estimator
//! \param[in] handle      The estimator (EST) handle
//! \param[in] accel_rps2  The mechanical acceleration, rad/sec^2
extern void     EST_setAccel_rps2(EST_Handle handle,const float32_t accel_rps2);
extern void cla_EST_setAccel_rps2(EST_Handle handle,const float32_t accel_rps2);


//! \brief     Sets the angle value at t = n in both the estimator and trajectory in radians (rad).
//! \details   This function overwrites the estimated angle with a user's provided angle.
//!            The set value should be between -pi and pi
//!            The following example shows how to overwrite the estimated angle:
//! \code
//! float32_t Overwrite_Flux_Angle_rad = MATH_PI * 0.5;
//! EST_setAngle_rad(handle,Overwrite_Flux_Angle_rad);
//! \endcode
//! \details   This function is not recommended for general use, since this will automatically generate
//!            an axis misalignment between the rotor flux axis and the control signals driving the motor.
//!            The use of this function is recommended for advanced users interested in doing open loop
//!            startup algorithms that need to bypass the estimator.
//! \param[in] handle     The estimator (EST) handle
//! \param[in] angle_rad  The angle value at t = n, rad
extern void     EST_setAngle_rad(EST_Handle handle,const float32_t angle_rad);
extern void cla_EST_setAngle_rad(EST_Handle handle,const float32_t angle_rad);


//! \brief     Sets the angle value at t = n in the estimator in radians (rad).
//! \param[in] handle     The estimator (EST) handle
//! \param[in] angle_rad  The angle value at t = n, rad
extern void     EST_setAngle_est_rad(EST_Handle handle,const float32_t angle_rad);
extern void cla_EST_setAngle_est_rad(EST_Handle handle,const float32_t angle_rad);


//! \brief     Sets the angle value at t = n+1 in the estimator in radians (rad).
//! \details   This function overwrites the estimated angle with a user's provided angle.
//!            The set value should be between -2*pi and 2*pi
//!            The following example shows how to overwrite the estimated angle:
//! \code
//! float32_t Overwrite_Flux_Angle_np1_rad = MATH_TWO_PI * 0.5;
//! EST_setAngle_np1_rad(handle,Overwrite_Flux_Angle_np1_rad);
//! \endcode
//! \details   This function is not recommended for general use, since this will automatically generate
//!            an axis misalignment between the rotor flux axis and the control signals driving the motor.
//!            The use of this function is recommended for advanced users interested in doing open loop
//!            startup algorithms that need to bypass the estimator.
//! \param[in] handle         The estimator (EST) handle
//! \param[in] angle_np1_rad  The angle value at t = n+1, rad
extern void     EST_setAngle_np1_rad(EST_Handle handle,const float32_t angle_np1_rad);
extern void cla_EST_setAngle_np1_rad(EST_Handle handle,const float32_t angle_np1_rad);


//! \brief     Sets the angle value at t = n in the trajectory in radians (rad).
//! \param[in] handle     The estimator (EST) handle
//! \param[in] angle_rad  The angle value at t = n, rad
extern void     EST_setAngle_traj_rad(EST_Handle handle,const float32_t angle_rad);
extern void cla_EST_setAngle_traj_rad(EST_Handle handle,const float32_t angle_rad);


//! \brief     Sets the estimated angle delta value
//! \param[in] handle     The estimator (EST) handle
//! \param[in] delta_rad  The angle delta value, rad
extern void     EST_setAngleDelta_est_rad(EST_Handle handle,const float32_t delta_rad);
extern void cla_EST_setAngleDelta_est_rad(EST_Handle handle,const float32_t delta_rad);


//! \brief     Sets the trajectory angle delta value
//! \param[in] handle     The estimator (EST) handle
//! \param[in] delta_rad  The angle delta value, rad
extern void     EST_setAngleDelta_traj_rad(EST_Handle handle,const float32_t delta_rad);
extern void cla_EST_setAngleDelta_traj_rad(EST_Handle handle,const float32_t delta_rad);


//! \brief     Sets the ISR count in the estimator
//! \param[in] handle  The estimator (EST) handle
//! \param[in] value   The ISR count value
extern void     EST_setCount_isr(EST_Handle handle,const int16_t value);
extern void cla_EST_setCount_isr(EST_Handle handle,const int16_t value);


//! \brief     Sets the state count in the estimator
//! \param[in] handle  The estimator (EST) handle
//! \param[in] value   The state count value
extern void     EST_setCount_state(EST_Handle handle,const int32_t value);
extern void cla_EST_setCount_state(EST_Handle handle,const int32_t value);


//! \brief     Sets the ISR count value in the estimator trajectory
//! \param[in] handle  The estimator (EST) handle
//! \param[in] count   The desired count
extern void     EST_setCount_traj(EST_Handle handle,const int16_t value);
extern void cla_EST_setCount_traj(EST_Handle handle,const int16_t value);


//! \brief     Sets the DC bus voltage in the estimator in Volt (V).
//! \param[in] handle   The estimator (EST) handle
//! \param[in] dcBus_V  The DC bus voltage, V
extern void     EST_setDcBus_V(EST_Handle handle,const float32_t dcBus_V);
extern void cla_EST_setDcBus_V(EST_Handle handle,const float32_t dcBus_V);


//! \brief     Sets the electrical frequency in the estimator
//! \param[in] handle  The estimator (EST) handle
//! \param[in] fe_Hz   The electrical frequency value, Hz
extern void     EST_setFe_Hz(EST_Handle handle,const float32_t fe_Hz);
extern void cla_EST_setFe_Hz(EST_Handle handle,const float32_t fe_Hz);


//! \brief     Sets the absolute maximum electrical frequency in the estimator
//! \param[in] handle          The estimator (EST) handle
//! \param[in] fe_abs_max_rps  The absolute maximum electrical frequency value, rad/sec
extern void     EST_setFe_abs_max_rps(EST_Handle handle,const float32_t fe_abs_max_rps);
extern void cla_EST_setFe_abs_max_rps(EST_Handle handle,const float32_t fe_abs_max_rps);


//! \brief     Sets the electrical frequency in the estimator
//! \param[in] handle  The estimator (EST) handle
//! \param[in] fe_rps   The electrical frequency value, rad/sec
extern void     EST_setFe_rps(EST_Handle handle,const float32_t fe_rps);
extern void cla_EST_setFe_rps(EST_Handle handle,const float32_t fe_rps);


//! \brief     Sets the bypass lock rotor flag value in the estimator
//! \param[in] handle  The estimator (EST) handle
//! \param[in] state   The desired state
extern void     EST_setFlag_bypassLockRotor(EST_Handle handle,const bool state);
extern void cla_EST_setFlag_bypassLockRotor(EST_Handle handle,const bool state);


//! \brief     Sets the enable estimator flag value in the estimator
//! \param[in] handle  The estimator (EST) handle
//! \param[in] state   The desired state
extern void     EST_setFlag_enable(EST_Handle handle,const bool state);
extern void cla_EST_setFlag_enable(EST_Handle handle,const bool state);


//! \brief     Sets the enable force angle flag in the estimator
//! \param[in] handle  The estimator (EST) handle
//! \param[in] state   The desired flag state, on (1) or off (0)
//!            <b>true</b> Enable forced angle.
//!            <b>false</b> Disable forced angle.
extern void     EST_setFlag_enableForceAngle(EST_Handle handle,const bool state);
extern void cla_EST_setFlag_enableForceAngle(EST_Handle handle,const bool state);


//! \brief     Sets the enable PowerWarp flag in the estimator
//! \param[in] handle  The estimator (EST) handle
//! \param[in] state   The desired flag state, on (1) or off (0)
extern void     EST_setFlag_enablePowerWarp(EST_Handle handle,const bool state);
extern void cla_EST_setFlag_enablePowerWarp(EST_Handle handle,const bool state);


//! \brief     Sets the enable Rs online flag in the estimator
//! \param[in] handle  The estimator (EST) handle
//! \param[in] state   The desired flag state, on (1) or off (0)
//!            <b>true</b> Enable the Rs online recalibration algorithm. The estimator will run a set of
//!                    functions related to rs online which recalculates the stator resistance while the
//!                    motor is rotating. This algorithm is useful when motor heats up, and hence stator
//!                    resistance increases.
//!            <b>false</b> Disable the Rs online recalibration algorithm. No updates to Rs will be made
//!                    even if the motor heats up. Low speed performace, and startup performance with
//!                    full torque might be affected if stator resistance changes due to motor heating up.
//!                    The stator resistance will be fixed, and equal to the value returned by EST_getRs_Ohm().
extern void     EST_setFlag_enableRsOnLine(EST_Handle handle,const bool state);
extern void cla_EST_setFlag_enableRsOnLine(EST_Handle handle,const bool state);


//! \brief     Sets the enable stator resistance (Rs) re-calculation flag in the estimator
//! \param[in] handle  The estimator (EST) handle
//! \param[in] state   The desired flag state, on (1) or off (0)
//!            <b>true</b> Enable Rs recalibration. The estimator will inject a DC current to the D-axis
//!                    to recalibrate the stator resistance before the motor rotates. It is required that
//!                    the motor is at standstill to perform Rs recalibration. If online recalibration
//!                    of the stator resistance is needed, refer to EST_getFlag_enableRsOnLine() and
//!                    EST_setFlag_enableRsOnLine() functions.
//!            <b>false</b> Disable Rs recalibration. The estimator will start the motor with the resistance
//!                    value that was used before the motor was stopped, or what is returned by function:
//!                    EST_getRs_Ohm().
extern void     EST_setFlag_enableRsRecalc(EST_Handle handle,const bool state);
extern void cla_EST_setFlag_enableRsRecalc(EST_Handle handle,const bool state);


//! \brief     Sets the estimation complete flag in the estimator
//! \param[in] handle  The estimator (EST) handle
//! \param[in] state   The desired flag state, true (1) or false (0)
extern void     EST_setFlag_estComplete(EST_Handle handle,const bool state);
extern void cla_EST_setFlag_estComplete(EST_Handle handle,const bool state);


//! \brief     Sets the motor identified flag state in the estimator
//! \param[in] handle  The estimator (EST) handle
//! \param[in] state   The desired state
extern void     EST_setFlag_motorIdentified(EST_Handle handle,const bool state);
extern void cla_EST_setFlag_motorIdentified(EST_Handle handle,const bool state);


//! \brief     Sets the update stator resistance (Rs) flag in the estimator
//! \details   When the online resistance estimator is enabled, the update flag allows the online resistance
//!            to be copied to the resistance used by the estimator model. If the update flag is not set to true,
//!            the online resistance estimation will not be used by the estimator model, and if the resistance
//!            changes too much due to temperature increase, the model may not work as expected.
//! \code
//! EST_setFlag_updateRs(handle, true);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \param[in] state   The desired flag state
//!            <b>true</b> The stator resistance estimated by the Rs OnLine module will be copied to the'
//!                    the stator resistance used by the module, so of the motor's temperature changes,
//!                    the estimated angle will be calculated based on the most up to date stator
//!                    resistance
//!            <b>false</b> The stator resistance estimated by the Rs OnLine module may or may not be updated
//!                    depending on the enable flag, but will not be used in the motor's model used to generate
//!                    the estimated speed and angle.
extern void     EST_setFlag_updateRs(EST_Handle handle,const bool state);
extern void cla_EST_setFlag_updateRs(EST_Handle handle,const bool state);


//! \brief     Sets the asynchronous/mechanical frequency in the estimator
//! \param[in] handle  The estimator (EST) handle
//! \param[in] fm_Hz   The mechanical frequency value, Hz
extern void     EST_setFm_Hz(EST_Handle handle,const float32_t fm_Hz);
extern void cla_EST_setFm_Hz(EST_Handle handle,const float32_t fm_Hz);


//! \brief     Sets the low pass filtered asynchronous/mechanical frequency in the estimator
//! \param[in] handle    The estimator (EST) handle
//! \param[in] fm_lp_Hz  The low pass filtered mechanical frequency value, Hz
extern void     EST_setFm_lp_Hz(EST_Handle handle,const float32_t fm_lp_Hz);
extern void cla_EST_setFm_lp_Hz(EST_Handle handle,const float32_t fm_lp_Hz);


//! \brief     Sets the low pass filtered asynchronous/mechanical frequency in the estimator
//! \param[in] handle     The estimator (EST) handle
//! \param[in] fm_lp_rps  The low pass filtered mechanical frequency value, rad/sec
extern void     EST_setFm_lp_rps(EST_Handle handle,const float32_t fm_lp_rps);
extern void cla_EST_setFm_lp_rps(EST_Handle handle,const float32_t fm_lp_rps);


//! \brief     Sets the asynchronous/mechanical frequency in the estimator
//! \param[in] handle  The estimator (EST) handle
//! \param[in] fm_rps  The mechanical frequency value, rad/sec
extern void     EST_setFm_rps(EST_Handle handle,const float32_t fm_rps);
extern void cla_EST_setFm_rps(EST_Handle handle,const float32_t fm_rps);


//! \brief     Sets the slip frequency in the estimator
//! \param[in] handle    The estimator (EST) handle
//! \param[in] fslip_Hz  The slip frequency value, Hz
extern void     EST_setFslip_Hz(EST_Handle handle,const float32_t fslip_Hz);
extern void cla_EST_setFslip_Hz(EST_Handle handle,const float32_t fslip_Hz);


//! \brief     Sets the slip frequency in the estimator
//! \param[in] handle     The estimator (EST) handle
//! \param[in] fslip_rps  The slip frequency value, rad/sec
extern void     EST_setFslip_rps(EST_Handle handle,const float32_t fslip_rps);
extern void cla_EST_setFslip_rps(EST_Handle handle,const float32_t fslip_rps);


//! \brief     Sets the force angle delta value in the estimator in radians (rad).
//! \param[in] handle          The estimator (EST) handle
//! \param[in] angleDelta_rad  The force angle delta value, rad
extern void     EST_setForceAngleDelta_rad(EST_Handle handle,const float32_t angleDelta_rad);
extern void cla_EST_setForceAngleDelta_rad(EST_Handle handle,const float32_t angleDelta_rad);


//! \brief     Sets the value used to set the low pass filter pole location in the frequency estimator
//!            in radians (rad).
//! \param[in] handle    The estimator (EST) handle
//! \param[in] beta_rad  The value used to set the filter pole location, rad
extern void     EST_setFreqBeta_lp(EST_Handle handle,const float32_t beta_rad);
extern void cla_EST_setFreqBeta_lp(EST_Handle handle,const float32_t beta_rad);


//! \brief     Sets the direct current (Id) reference value in the estimator in Ampere (A).
//! \param[in] handle    The estimator (EST) handle
//! \param[in] Id_ref_A  The Id reference value, A
extern void     EST_setId_ref_A(EST_Handle handle,const float32_t Id_ref_A);
extern void cla_EST_setId_ref_A(EST_Handle handle,const float32_t Id_ref_A);


//! \brief     Sets the estimator to idle
//! \param[in] handle  The estimator (EST) handle
extern void     EST_setIdle(EST_Handle handle);
extern void cla_EST_setIdle(EST_Handle handle);


//! \brief     Sets the direct/quadrature current (Idq) reference value in the estimator in Ampere (A).
//! \param[in] handle      The estimator (EST) handle
//! \param[in] pIdq_ref_A  The pointer to the Idq reference values, A
extern void     EST_setIdq_ref_A(EST_Handle handle,const MATH_Vec2 *pIdq_ref_A);
extern void cla_EST_setIdq_ref_A(EST_Handle handle,const MATH_Vec2 *pIdq_ref_A);


//! \brief     Sets the Id rated current value in the estimator in Ampere (A).
//! \param[in] handle     The estimator (EST) handle
//! \param[in] IdRated_A  The Id rated current value, A
extern void     EST_setIdRated_A(EST_Handle handle,const float32_t IdRated_A);
extern void cla_EST_setIdRated_A(EST_Handle handle,const float32_t IdRated_A);


//! \brief     Sets the Id rated delta current value in the estimator in Ampere (A)
//! \param[in] handle           The estimator (EST) handle
//! \param[in] IdRated_delta_A  The delta Id rated current value, A
extern void     EST_setIdRated_delta_A(EST_Handle handle,const float32_t IdRated_delta_A);
extern void cla_EST_setIdRated_delta_A(EST_Handle handle,const float32_t IdRated_delta_A);


//! \brief     Sets the intermediate value in the Id trajectory generator
//! \param[in] handle      The estimator (EST) handle
//! \param[in] intValue_A  The intermediate value, A
extern void     EST_setIntValue_Id_A(EST_Handle handle,const float32_t intValue_A);
extern void cla_EST_setIntValue_Id_A(EST_Handle handle,const float32_t intValue_A);


//! \brief     Sets the intermediate value in the Iq trajectory generator
//! \param[in] handle      The estimator (EST) handle
//! \param[in] intValue_A  The intermediate value, A
extern void     EST_setIntValue_Iq_A(EST_Handle handle,const float32_t intValue_A);
extern void cla_EST_setIntValue_Iq_A(EST_Handle handle,const float32_t intValue_A);


//! \brief     Sets the intermediate value in the speed trajectory generator
//! \param[in] handle       The estimator (EST) handle
//! \param[in] intValue_Hz  The intermediate value, Hz
extern void     EST_setIntValue_spd_Hz(EST_Handle handle,const float32_t intValue_Hz);
extern void cla_EST_setIntValue_spd_Hz(EST_Handle handle,const float32_t intValue_Hz);


//! \brief     Sets the quadrature current (Iq) reference value in the estimator in Ampere (A).
//! \param[in] handle    The estimator (EST) handle
//! \param[in] Iq_ref_A  The Iq reference value, A
extern void     EST_setIq_ref_A(EST_Handle handle,const float32_t Iq_ref_A);
extern void cla_EST_setIq_ref_A(EST_Handle handle,const float32_t Iq_ref_A);


//! \brief     Sets the Lhf value in the estimator
//! \param[in] handle  The estimator (EST) handle
//! \param[in] Lhf_H   The Lhf inductance value, H
extern void     EST_setLhf_H(EST_Handle handle,const float32_t Lhf_H);
extern void cla_EST_setLhf_H(EST_Handle handle,const float32_t Lhf_H);


//! \brief     Sets the delta stator inductance value during coarse estimation
//! \param[in] handle      The estimator (EST) handle
//! \param[in] Ls_delta_H  The delta stator inductance value for coarse estimation, Henry
extern void     EST_setLs_coarseDelta_H(EST_Handle handle,const float32_t Ls_delta_H);
extern void cla_EST_setLs_coarseDelta_H(EST_Handle handle,const float32_t Ls_delta_H);


//! \brief     Sets the direct stator inductance value in the estimator in Henry (H).
//! \param[in] handle  The estimator (EST) handle
//! \param[in] Ls_d_H  The direct stator inductance value, Henry
extern void     EST_setLs_d_H(EST_Handle handle,const float32_t Ls_d_H);
extern void cla_EST_setLs_d_H(EST_Handle handle,const float32_t Ls_d_H);


//! \brief     Sets the direct/quadrature stator inductance vector values in the estimator in Henry (H).
//! \param[in] handle    The estimator (EST) handle
//! \param[in] pLs_dq_H  The pointer to the direct/quadrature stator inductance vector values, H
extern void     EST_setLs_dq_H(EST_Handle handle,const MATH_Vec2 *pLs_dq_H);
extern void cla_EST_setLs_dq_H(EST_Handle handle,const MATH_Vec2 *pLs_dq_H);


//! \brief     Sets the delta stator inductance value during fine estimation
//! \param[in] handle      The estimator (EST) handle
//! \param[in] Ls_delta_H  The delta stator inductance value for fine estimation, H
extern void     EST_setLs_fineDelta_H(EST_Handle handle,const float32_t Ls_delta_H);
extern void cla_EST_setLs_fineDelta_H(EST_Handle handle,const float32_t Ls_delta_H);


//! \brief     Sets the stator inductance value in the estimator in Henry (H).
//! \param[in] handle  The estimator (EST) handle
//! \param[in] Ls_H    The stator inductance value, H
extern void     EST_setLs_H(EST_Handle handle,const float32_t Ls_H);
extern void cla_EST_setLs_H(EST_Handle handle,const float32_t Ls_H);


//! \brief     Sets the quadrature stator inductance value in the estimator in Henry (H).
//! \param[in] handle  The estimator (EST) handle
//! \param[in] Ls_q_H  The quadrature stator inductance value, H
extern void     EST_setLs_q_H(EST_Handle handle,const float32_t Ls_q_H);
extern void cla_EST_setLs_q_H(EST_Handle handle,const float32_t Ls_q_H);


//! \brief     Sets the number of Interrupt Service Routine (ISR) clock ticks per estimator clock tick
//! \param[in] handle                 The estimator (EST) handle
//! \return    numIsrTicksPerEstTick  The number of Interrupt Service Routine (ISR) clock ticks per estimator clock tick
extern void     EST_setNumIsrTicksPerEstTick(EST_Handle handle,
                                             const int16_t numIsrTicksPerEstTick);
extern void cla_EST_setNumIsrTicksPerEstTick(EST_Handle handle,
                                             const int16_t numIsrTicksPerEstTick);


//! \brief     Sets the number of Interrupt Service Routine (ISR) clock ticks per estimator trajectory clock tick
//! \param[in] handle                  The estimator (EST) handle
//! \return    numIsrTicksPerTrajTick  The number of Interrupt Service Routine (ISR) clock ticks per estimator trajectory clock tick
extern void     EST_setNumIsrTicksPerTrajTick(EST_Handle handle,
                                              const int16_t numIsrTicksPerTrajTick);
extern void cla_EST_setNumIsrTicksPerTrajTick(EST_Handle handle,
                                              const int16_t numIsrTicksPerTrajTick);


//! \brief     Sets the maximum current delta value in the estimator in Ampere (A).
//! \param[in] handle             The estimator (EST) handle
//! \param[in] maxCurrentDelta_A  The maximum current delta value, A
extern void     EST_setMaxCurrentDelta_A(EST_Handle handle,
                                         const float32_t maxCurrentDelta_A);
extern void cla_EST_setMaxCurrentDelta_A(EST_Handle handle,
                                         const float32_t maxCurrentDelta_A);


//! \brief     Sets the maximum power warp current delta value used in the estimator
//!            in Ampere (A).
//! \param[in] handle             The estimator (EST) handle
//! \param[in] maxCurrentDelta_A  The maximum current delta value, A
extern void     EST_setMaxCurrentDelta_pw_A(EST_Handle handle,
                                            const float32_t maxCurrentDelta_A);
extern void cla_EST_setMaxCurrentDelta_pw_A(EST_Handle handle,
                                            const float32_t maxCurrentDelta_A);


//! \brief     Sets the maximum delta value in the Id trajectory generator
//! \param[in] handle      The estimator (EST) handle
//! \param[in] maxDelta_A  The maximum delta value, A
extern void     EST_setMaxDelta_Id_A(EST_Handle handle,const float32_t maxDelta_A);
extern void cla_EST_setMaxDelta_Id_A(EST_Handle handle,const float32_t maxDelta_A);


//! \brief     Sets the maximum delta value in the Iq trajectory generator
//! \param[in] handle      The estimator (EST) handle
//! \param[in] maxDelta_A  The maximum delta value, A
extern void     EST_setMaxDelta_Iq_A(EST_Handle handle,const float32_t maxDelta_A);
extern void cla_EST_setMaxDelta_Iq_A(EST_Handle handle,const float32_t maxDelta_A);


//! \brief     Sets the maximum delta value in the speed trajectory generator
//! \param[in] handle       The estimator (EST) handle
//! \param[in] maxDelta_Hz  The maximum delta value, Hz
extern void     EST_setMaxDelta_spd_Hz(EST_Handle handle,const float32_t maxDelta_Hz);
extern void cla_EST_setMaxDelta_spd_Hz(EST_Handle handle,const float32_t maxDelta_Hz);


//! \brief     Sets the maximum speed delta value during estimation
//! \param[in] handle            The estimator (EST) handle
//! \param[in] maxSpeedDelta_Hz  The maximum acceleration value, Hz
extern void     EST_setMaxSpeedDelta_Hz(EST_Handle handle,const float32_t maxSpeedDelta_Hz);
extern void cla_EST_setMaxSpeedDelta_Hz(EST_Handle handle,const float32_t maxSpeedDelta_Hz);


//! \brief     Sets the maximum value in the Id trajectory generator
//! \param[in] handle      The estimator (EST) handle
//! \param[in] maxValue_A  The maximum value, A
extern void     EST_setMaxValue_Id_A(EST_Handle handle,const float32_t maxValue_A);
extern void cla_EST_setMaxValue_Id_A(EST_Handle handle,const float32_t maxValue_A);


//! \brief     Sets the maximum value in the Iq trajectory generator
//! \param[in] handle      The estimator (EST) handle
//! \param[in] maxValue_A  The maximum value, A
extern void     EST_setMaxValue_Iq_A(EST_Handle handle,const float32_t maxValue_A);
extern void cla_EST_setMaxValue_Iq_A(EST_Handle handle,const float32_t maxValue_A);


//! \brief     Sets the maximum value in the speed trajectory generator
//! \param[in] handle      The estimator (EST) handle
//! \param[in] maxValue_Hz  The maximum value, Hz
extern void     EST_setMaxValue_spd_Hz(EST_Handle handle,const float32_t maxValue_Hz);
extern void cla_EST_setMaxValue_spd_Hz(EST_Handle handle,const float32_t maxValue_Hz);


//! \brief     Sets the minimum value in the Id trajectory generator
//! \param[in] handle      The estimator (EST) handle
//! \param[in] minValue_A  The minimum value, A
extern void     EST_setMinValue_Id_A(EST_Handle handle,const float32_t minValue_A);
extern void cla_EST_setMinValue_Id_A(EST_Handle handle,const float32_t minValue_A);


//! \brief     Sets the minimum value in the Iq trajectory generator
//! \param[in] handle      The estimator (EST) handle
//! \param[in] minValue_A  The minimum value, A
extern void     EST_setMinValue_Iq_A(EST_Handle handle,const float32_t minValue_A);
extern void cla_EST_setMinValue_Iq_A(EST_Handle handle,const float32_t minValue_A);


//! \brief     Sets the minimum value in the speed trajectory generator
//! \param[in] handle       The estimator (EST) handle
//! \param[in] minValue_Hz  The minimum value, Hz
extern void     EST_setMinValue_spd_Hz(EST_Handle handle,const float32_t minValue_Hz);
extern void cla_EST_setMinValue_spd_Hz(EST_Handle handle,const float32_t minValue_Hz);


//! \brief     Sets the estimator parameters using the user parameters structreu
//! \param[in] handle       The estimator (EST) handle
//! \param[in] pUserParams  A pointer to the user parameters structure
extern void     EST_setParams(EST_Handle handle, USER_Params *pUserParams);
extern void cla_EST_setParams(EST_Handle handle, USER_Params *pUserParams);


//! \brief     Sets the power warp gain in the trajectory generator
//! \param[in] handle  The estimator (EST) handle
//! \param[in] pwGain  The power warp gain, unitless
extern void     EST_setPwGain(EST_Handle handle,const float32_t pwGain);
extern void cla_EST_setPwGain(EST_Handle handle,const float32_t pwGain);


//! \brief     Sets the Rhf value in the estimator
//! \param[in] handle   The estimator (EST) handle
//! \param[in] Rhf_Ohm  The Rhf resistance value, Ohm
extern void     EST_setRhf_Ohm(EST_Handle handle,const float32_t Rhf_Ohm);
extern void cla_EST_setRhf_Ohm(EST_Handle handle,const float32_t Rhf_Ohm);


//! \brief     Sets the R/L value in the estimator
//! \param[in] handle      The estimator (EST) handle
//! \param[in] RoverL_rps  The R/L value, rad/sec
extern void     EST_setRoverL_rps(EST_Handle handle,const float32_t RoverL_rps);
extern void cla_EST_setRoverL_rps(EST_Handle handle,const float32_t RoverL_rps);


//! \brief     Sets the coarse delta rotor resistance value in the estimator
//! \param[in] handle              The estimator (EST) handle
//! \param[in] Rr_coarseDelta_Ohm  The coarse delta rotor resistance value, Ohm
extern void     EST_setRr_coarseDelta_Ohm(EST_Handle handle,const float32_t Rr_coarseDelta_Ohm);
extern void cla_EST_setRr_coarseDelta_Ohm(EST_Handle handle,const float32_t Rr_coarseDelta_Ohm);


//! \brief     Sets the direct rotor resistance value used in the estimator in Ohm (\f$\Omega\f$).
//! \param[in] handle    The estimator (EST) handle
//! \param[in] Rr_d_Ohm  The direct rotor resistance value, Ohm
extern void     EST_setRr_d_Ohm(EST_Handle handle,const float32_t Rr_d_Ohm);
extern void cla_EST_setRr_d_Ohm(EST_Handle handle,const float32_t Rr_d_Ohm);


//! \brief     Sets the direct/quadrature rotor resistance values used in the estimator in Ohm (\f$\Omega\f$).
//! \param[in] handle      The estimator (EST) handle
//! \param[in] pRr_dq_Ohm  The pointer to the vector of direct/quadrature rotor resistance values, Ohm
extern void     EST_setRr_dq_Ohm(EST_Handle handle,const MATH_Vec2 *pRr_dq_Ohm);
extern void cla_EST_setRr_dq_Ohm(EST_Handle handle,const MATH_Vec2 *pRr_dq_Ohm);


//! \brief     Sets the fine delta rotor resistance value in the estimator
//! \param[in] handle            The estimator (EST) handle
//! \param[in] Rr_fineDelta_Ohm  The fine delta rotor resistance value, Ohm
extern void     EST_setRr_fineDelta_Ohm(EST_Handle handle,const float32_t Rr_fineDelta_Ohm);
extern void cla_EST_setRr_fineDelta_Ohm(EST_Handle handle,const float32_t Rr_fineDelta_Ohm);


//! \brief     Sets the maximum rotor resistance maximum value for saturation
//! \param[in] handle      The estimator (EST) handle
//! \param[in] Rr_max_Ohm  The rotor maximum resistance value, Ohm
extern void     EST_setRr_max_Ohm(EST_Handle handle,const float32_t Rr_max_Ohm);
extern void cla_EST_setRr_max_Ohm(EST_Handle handle,const float32_t Rr_max_Ohm);


//! \brief     Sets the minimum rotor resistance value for saturation
//! \param[in] handle      The estimator (EST) handle
//! \param[in] Rr_min_Ohm  The minimum rotor resistance value, Ohm
extern void     EST_setRr_min_Ohm(EST_Handle handle,const float32_t Rr_min_Ohm);
extern void cla_EST_setRr_min_Ohm(EST_Handle handle,const float32_t Rr_min_Ohm);


//! \brief     Sets the rotor resistance value used in the estimator in Ohm (\f$\Omega\f$).
//! \param[in] handle  The estimator (EST) handle
//! \param[in] Rr_Ohm  The rotor resistance value, Ohm
extern void     EST_setRr_Ohm(EST_Handle handle,const float32_t Rr_Ohm);
extern void cla_EST_setRr_Ohm(EST_Handle handle,const float32_t Rr_Ohm);


//! \brief     Sets the quadrature rotor resistance value used in the estimator in Ohm (\f$\Omega\f$).
//! \param[in] handle    The estimator (EST) handle
//! \param[in] Rr_q_Ohm  The quadrature rotor resistance value, Ohm
extern void     EST_setRr_q_Ohm(EST_Handle handle,const float32_t Rr_q_Ohm);
extern void cla_EST_setRr_q_Ohm(EST_Handle handle,const float32_t Rr_q_Ohm);


//! \brief     Sets the alpha stator resistance value used in the estimator in Ohm (\f$\Omega\f$).
//! \param[in] handle    The estimator (EST) handle
//! \param[in] Rs_a_Ohm  The alpha stator resistance value, Ohm
extern void     EST_setRs_a_Ohm(EST_Handle handle,const float32_t Rs_a_Ohm);
extern void cla_EST_setRs_a_Ohm(EST_Handle handle,const float32_t Rs_a_Ohm);


//! \brief     Sets the alpha/beta stator resistance values used in the estimator in Ohm (\f$\Omega\f$).
//! \param[in] handle      The estimator (EST) handle
//! \param[in] pRs_ab_Ohm  The pointer to the vector of direct/quadrature stator resistance values, Ohm
extern void     EST_setRs_ab_Ohm(EST_Handle handle,const MATH_Vec2 *pRs_ab_Ohm);
extern void cla_EST_setRs_ab_Ohm(EST_Handle handle,const MATH_Vec2 *pRs_ab_Ohm);


//! \brief     Sets the beta stator resistance value used in the estimator in Ohm (\f$\Omega\f$).
//! \param[in] handle    The estimator (EST) handle
//! \param[in] Rs_b_Ohm  The beta stator resistance value, Ohm
extern void     EST_setRs_b_Ohm(EST_Handle handle,const float32_t Rs_b_Ohm);
extern void cla_EST_setRs_b_Ohm(EST_Handle handle,const float32_t Rs_b_Ohm);


//! \brief     Sets the coarse delta stator resistance value
//! \param[in] handle              The estimator (EST) handle
//! \param[in] Rs_coarseDelta_Ohm  The coarse delta stator resistance value, Ohm
extern void     EST_setRs_coarseDelta_Ohm(EST_Handle handle,const float32_t Rs_coarseDelta_Ohm);
extern void cla_EST_setRs_coarseDelta_Ohm(EST_Handle handle,const float32_t Rs_coarseDelta_Ohm);


//! \brief     Sets the direct stator resistance value used in the estimator in Ohm (\f$\Omega\f$).
//! \param[in] handle    The estimator (EST) handle
//! \param[in] Rs_d_Ohm  The direct stator resistance value, Ohm
extern void     EST_setRs_d_Ohm(EST_Handle handle,const float32_t Rs_d_Ohm);
extern void cla_EST_setRs_d_Ohm(EST_Handle handle,const float32_t Rs_d_Ohm);


//! \brief     Sets the direct/quadrature stator resistance values used in the estimator in Ohm (\f$\Omega\f$).
//! \param[in] handle      The estimator (EST) handle
//! \param[in] pRs_dq_Ohm  The pointer to the vector of direct/quadrature stator resistance values, Ohm
extern void     EST_setRs_dq_Ohm(EST_Handle handle,const MATH_Vec2 *pRs_dq_Ohm);
extern void cla_EST_setRs_dq_Ohm(EST_Handle handle,const MATH_Vec2 *pRs_dq_Ohm);


//! \brief     Sets the fine delta stator resistance value
//! \param[in] handle            The estimator (EST) handle
//! \param[in] Rs_fineDelta_Ohm  The fine delta stator resistance value, Ohm
extern void     EST_setRs_fineDelta_Ohm(EST_Handle handle,const float32_t Rs_fineDelta_Ohm);
extern void cla_EST_setRs_fineDelta_Ohm(EST_Handle handle,const float32_t Rs_fineDelta_Ohm);


//! \brief     Sets the stator resistance value used in the estimator in Ohm (\f$\Omega\f$).
//! \param[in] handle  The estimator (EST) handle
//! \param[in] Rs_Ohm  The stator resistance value, Ohm
extern void     EST_setRs_Ohm(EST_Handle handle,const float32_t Rs_Ohm);
extern void cla_EST_setRs_Ohm(EST_Handle handle,const float32_t Rs_Ohm);


//! \brief     Sets the quadrature stator resistance value used in the estimator in Ohm (\f$\Omega\f$).
//! \param[in] handle    The estimator (EST) handle
//! \param[in] Rs_q_Ohm  The quadrature stator resistance value, Ohm
extern void     EST_setRs_q_Ohm(EST_Handle handle,const float32_t Rs_q_Ohm);
extern void cla_EST_setRs_q_Ohm(EST_Handle handle,const float32_t Rs_q_Ohm);


//! \brief     Sets the beta of the filters used for online stator resistance estimation in radians (rad).
//! \param[in] handle    The estimator (EST) handle
//! \param[in] beta_rad  The beta value, rad
extern void     EST_setRsOnLine_beta_rad(EST_Handle handle,const float32_t beta_rad);
extern void cla_EST_setRsOnLine_beta_rad(EST_Handle handle,const float32_t beta_rad);


//! \brief     Sets the stator resistance value in the online stator resistance estimator in Ohm (\f$\Omega\f$).
//! \param[in] handle  The estimator (EST) handle
//! \param[in] Rs_Ohm  The stator resistance value, Ohm
extern void     EST_setRsOnLine_Ohm(EST_Handle handle,const float32_t Rs_Ohm);
extern void cla_EST_setRsOnLine_Ohm(EST_Handle handle,const float32_t Rs_Ohm);


//! \brief     Sets the angle value in the estimator
//! \param[in] handle     The estimator (EST) handle
//! \param[in] angle_rad  The angle value, rad
extern void     EST_setRsOnLineAngle_rad(EST_Handle handle,const float32_t angle_rad);
extern void cla_EST_setRsOnLineAngle_rad(EST_Handle handle,const float32_t angle_rad);


//! \brief     Sets the rotating angle delta in the online stator resistance estimator in radians
//! \param[in] handle          The estimator (EST) handle
//! \param[in] angleDelta_rad  The angle delta value, radians
extern void     EST_setRsOnLineAngleDelta_rad(EST_Handle handle,const float32_t angleDelta_rad);
extern void cla_EST_setRsOnLineAngleDelta_rad(EST_Handle handle,const float32_t angleDelta_rad);


//! \brief     Sets the Id value in the online stator resistance estimator in Ampere (A).
//! \param[in] handle  The estimator (EST) handle
//! \param[in] Id_A    The Id value, A
extern void     EST_setRsOnLineId_A(EST_Handle handle,const float32_t Id_A);
extern void cla_EST_setRsOnLineId_A(EST_Handle handle,const float32_t Id_A);


//! \brief     Sets the Id magnitude value used for online stator resistance estimation in Ampere (A).
//! \param[in] handle    The estimator (EST) handle
//! \param[in] Id_mag_A  The Id magnitude value, A
extern void     EST_setRsOnLineId_mag_A(EST_Handle handle,const float32_t Id_mag_A);
extern void cla_EST_setRsOnLineId_mag_A(EST_Handle handle,const float32_t Id_mag_A);


//! \brief     Sets the online stator resistance filter parameters
//! \param[in] handle        The estimator (EST) handle
//! \param[in] filterType    The filter type
//! \param[in] filterNumber  The filter number
//! \param[in] b0            The numerator coefficient value for z^0
//! \param[in] b1            The numerator coefficient value for z^(-1)
//! \param[in] a1            The denominator coefficient value for z^(-1)
//! \param[in] x1            The input value at time sample n=-1
//! \param[in] y1            The output value at time sample n=-1
extern void     EST_setRsOnLineLpFilterParams(EST_Handle handle,const EST_RsOnLineFilterType_e filterType,
                                              const uint16_t filterNumber,
                                              const float32_t b0,const float32_t b1,const float32_t a1,
                                              const float32_t x1,const float32_t y1);
extern void cla_EST_setRsOnLineLpFilterParams(EST_Handle handle,const EST_RsOnLineFilterType_e filterType,
                                              const uint16_t filterNumber,
                                              const float32_t b0,const float32_t b1,const float32_t a1,
                                              const float32_t x1,const float32_t y1);


//! \brief     Sets the reference speed in the estimator
//! \param[in] handle        The estimator (EST) handle
//! \param[in] speed_ref_Hz  The reference speed, Hz
extern void     EST_setSpeed_ref_Hz(EST_Handle handle,const float32_t speed_ref_Hz);
extern void cla_EST_setSpeed_ref_Hz(EST_Handle handle,const float32_t speed_ref_Hz);


//! \brief     Sets the target value in the Id trajectory generator
//! \param[in] handle         The estimator (EST) handle
//! \param[in] targetValue_A  The target value, A
extern void     EST_setTargetValue_Id_A(EST_Handle handle,const float32_t targetValue_A);
extern void cla_EST_setTargetValue_Id_A(EST_Handle handle,const float32_t targetValue_A);


//! \brief     Sets the target value in the Iq trajectory generator
//! \param[in] handle         The estimator (EST) handle
//! \param[in] targetValue_A  The target value, A
extern void     EST_setTargetValue_Iq_A(EST_Handle handle,const float32_t targetValue_A);
extern void cla_EST_setTargetValue_Iq_A(EST_Handle handle,const float32_t targetValue_A);


//! \brief     Sets the target value in the speed trajectory generator
//! \param[in] handle          The estimator (EST) handle
//! \param[in] targetValue_Hz  The target value, Hz
extern void     EST_setTargetValue_spd_Hz(EST_Handle handle,const float32_t targetValue_Hz);
extern void cla_EST_setTargetValue_spd_Hz(EST_Handle handle,const float32_t targetValue_Hz);


//! \brief     Sets up the trajectory generator
//! \param[in] handle                The trajectory generator (EST_Traj) handle
//! \param[in] ctrlHandle            The controller (CTRL) handle
//! \param[in] targetValue_spd_Hz    The target speed value during run time, Hz
//! \param[in] targetValue_Id_A      The target Id current value during run time, A
extern void     EST_setupTraj(EST_Handle handle,
                              CTRL_Handle ctrlHandle,
                              const float32_t targetValue_spd_Hz,
                              const float32_t targetValue_Id_A);
extern void cla_EST_setupTraj(EST_Handle handle,
                              CTRL_Handle ctrlHandle,
                              const float32_t targetValue_spd_Hz,
                              const float32_t targetValue_Id_A);


//! \brief     Updates the Id reference value used for online stator resistance estimation in Ampere (A).
//! \param[in] handle     The estimator (EST) handle
//! \param[in] pId_ref_A  The pointer to the Id reference value, A
extern void     EST_updateId_ref_A(EST_Handle handle,float32_t *pId_ref_A);
extern void cla_EST_updateId_ref_A(EST_Handle handle,float32_t *pId_ref_A);


//! \brief      Updates the estimator state
//! \param[in]  handle       The estimator (EST) handle
//! \param[in]  Id_target_A  The target Id current during each estimator state, A
//! \return     A boolean value denoting if the state has changed (true) or not (false)
extern bool     EST_updateState(EST_Handle handle,const float32_t Id_target_A);
extern bool cla_EST_updateState(EST_Handle handle,const float32_t Id_target_A);


//! \brief      Updates the trajectory generator state
//! \param[in]  handle  The estimator (EST) handle
//! \return     A boolean value denoting if the state has changed (true) or not (false)
extern bool     EST_updateTrajState(EST_Handle handle);
extern bool cla_EST_updateTrajState(EST_Handle handle);


//! \brief     Determines if a zero Iq current reference should be used in the controller
//! \param[in] handle  The estimator (EST) handle
//! \return    A boolean value denoting if a zero Iq current reference should be used (true) or not (false)
extern bool     EST_useZeroIq_ref(EST_Handle handle);
extern bool cla_EST_useZeroIq_ref(EST_Handle handle);


#ifdef __cplusplus
}
#endif // extern "C"

//! @} // ingroup
#endif // end of EST_H definition

