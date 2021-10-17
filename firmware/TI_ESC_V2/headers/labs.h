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

#ifndef LABS_H
#define LABS_H

//**************************************************************************
//! \file   solutions/common/sensorless_foc/include/labs.h
//! \brief  header file to be included in all labs
//!
//**************************************************************************

//**************************************************************************
//! \defgroup LABS LABS
//
//@{
//
//**************************************************************************

//
// modules
//
#ifdef __TMS320C28XX_CLA__
#include "libraries/math/include/CLAmath.h"
#else
#include "libraries/math/include/math.h"
#include <math.h>
#endif

#include "userParams.h"

#include "ctrl.h"
#include "clarke.h"
#include "est.h"
#include "filter_fo.h"
#include "ipark.h"
#include "park.h"
#include "pi.h"
#include "svgen.h"
#include "svgen_current.h"
#include "traj.h"
#include "vsf.h"
#include "fwc.h"
#include "mtpa.h"

#ifndef CLA
#include "vs_freq.h"
#endif

//
// Only for lab03&lab04--position open loop control
//
#include "angle_gen.h"
#include "datalog.h"
#include "cpu_time.h"

//
// solutions
//
#if !defined(__TMS320C28XX_CLA__)
#include "user.h"
#include "hal.h"
#endif

#include "drv8320.h"


#define LED_BLINK_FREQ_Hz           (1.0)     // 1Hz

#define FLYINGSTART_DELAY_TIME      300       // pwm cycles

#define OVER_VOLTAGE_BITS           0x0001    // DC Bus Over Voltage Fault
#define UNDER_VOLTAGE_BITS          0x0002    // DC Bus Under Voltage Fault
#define MOTOR_OVER_TEMPER_BITS      0x0004    // Motor over temperature Fault
#define MODULE_OVER_TEMPER_BITS     0x0008    // Module over temperature Fault
#define MODULE_OVER_CURRENT_BITS    0x0010    // Hardware Over Current Fault
#define OVER_RMS_CURRENT_BITS       0x0020    // Motor Phase Over Current Fault
#define OVER_PEAK_CURRENT_BITS      0x0040    // Software Over Current Fault
#define MULTI_OVER_CURRENT_BITS     0x0080    // Multiple times over current
#define MOTOR_LOST_PHASE_BITS       0x0100    // Motor Lost Phase
#define CURRENT_UNBALANCE_BITS      0x0200    // Motor Phase Current Unbalance
#define MOTOR_STALL_BITS            0x0400    // Motor Stall
#define MOTOR_OVER_SPEED_BITS       0x0800    // Motor Over Speed
#define STARTUP_FAILE_BITS          0x1000    // Startup failed
#define MOTOR_OVER_LOAD_BITS        0x2000    // Over Load Error
#define CONTROLLER_ERROR_BITS       0x4000    // FAST Controller error
#define RESERVE_15_BITS             0x8000    // Reserved for further use

#define MASK_ALL_FAULT_BITS         0x0000
#define ENABLE_ALL_FAULT_BITS       0xFFFF

//
// Block all fault protection except over current and over voltage faults
//
#define FAULT_MASK_OC_OV            OVER_VOLTAGE_BITS                          \
                                  + MODULE_OVER_CURRENT_BITS                   \
                                  + MASK_ALL_FAULT_BITS

//
// Block all fault protection except current, voltage and temperature faults
//
#define FAULT_MASK_OC_FU_OT         OVER_VOLTAGE_BITS                          \
                                  + UNDER_VOLTAGE_BITS                         \
                                  + MOTOR_OVER_TEMPER_BITS                     \
                                  + MODULE_OVER_TEMPER_BITS                    \
                                  + MODULE_OVER_CURRENT_BITS                   \
                                  + CONTROLLER_ERROR_BITS                      \
                                  + MASK_ALL_FAULT_BITS

//
// Block some fault protection
//
#define FAULT_MASK_OC_FV_OL         OVER_VOLTAGE_BITS                          \
                                  + UNDER_VOLTAGE_BITS                         \
                                  + MOTOR_OVER_TEMPER_BITS                     \
                                  + MODULE_OVER_TEMPER_BITS                    \
                                  + MODULE_OVER_CURRENT_BITS                   \
                                  + OVER_RMS_CURRENT_BITS                      \
                                  + OVER_PEAK_CURRENT_BITS                     \
                                  + MULTI_OVER_CURRENT_BITS                    \
                                  + CURRENT_UNBALANCE_BITS                     \
                                  + MOTOR_OVER_LOAD_BITS                       \
                                  + CONTROLLER_ERROR_BITS                      \
                                  + MASK_ALL_FAULT_BITS

//
// Enable all fault protection
//
#define FAULT_MASK_ALL_FLTS         OVER_VOLTAGE_BITS                          \
                                  + UNDER_VOLTAGE_BITS                         \
                                  + MOTOR_OVER_TEMPER_BITS                     \
                                  + MODULE_OVER_TEMPER_BITS                    \
                                  + MODULE_OVER_CURRENT_BITS                   \
                                  + OVER_RMS_CURRENT_BITS                      \
                                  + OVER_PEAK_CURRENT_BITS                     \
                                  + MULTI_OVER_CURRENT_BITS                    \
                                  + MOTOR_LOST_PHASE_BITS                      \
                                  + CURRENT_UNBALANCE_BITS                     \
                                  + MOTOR_STALL_BITS                           \
                                  + MOTOR_OVER_SPEED_BITS                      \
                                  + STARTUP_FAILE_BITS                         \
                                  + MOTOR_OVER_LOAD_BITS                       \
                                  + CONTROLLER_ERROR_BITS                      \
                                  + MASK_ALL_FAULT_BITS

//
//! \brief typedefs for the fault
//
typedef struct _FAULT_BITS_
{             // bits  description
    uint16_t overVoltage:1;         // 0  DC Bus Over Voltage Fault
    uint16_t underVoltage:1;        // 1  DC Bus Under Voltage Fault
    uint16_t motorOverTemp:1;       // 2  Motor over temperature Fault
    uint16_t moduleOverTemp:1;      // 3  Power module over temperature Fault

    uint16_t moduleOverCurrent:1;   // 4  Hardware Over Current Fault Flag
    uint16_t overRmsCurrent:1;      // 5  Motor Phase Over Current Fault
    uint16_t overPeakCurrent:1;     // 6  Software Over Current Fault Flag
    uint16_t multiOverCurrent:1;    // 7  Multiple times over current

    uint16_t motorLostPhase:1;      // 8  Motor Lost Phase
    uint16_t currentUnbalance:1;    // 9  Motor Phase Current Unbalance
    uint16_t motorStall:1;          // 10 Motor Stall
    uint16_t overSpeed:1;           // 11 Motor Over Speed

    uint16_t startupFailed:1;       // 12 Startup failed
    uint16_t overLoad:1;            // 13 Over Load Error
    uint16_t controllerError:1;     // 14 FAST Controller error
    uint16_t reserve15:1;           // 15 Reserved
} FAULT_BITS;

typedef union _FAULT_REG_t {
    uint16_t            all;
    FAULT_BITS          bit;
}FAULT_REG_t;


//
//! \brief Enumeration for the kit boards
//
typedef enum
{
    BOARD_BSXL8320RS_REVA = 0,          //!< the board is BOOSTXL_8320S
    BOARD_DRV8301_REVD    = 1,             //!< the board is DRV8301_RevD
    BOARD_HVMTRPFC_REV1P1 = 2,          //!< the board is HVMTRPFC_Rev1p1
    BOARD_DRV8312_REVD    = 3,             //!< the board is DRV8312_RevD
    BOARD_IDDK_REV2P1     = 4,               //!< the board is IDDK2.1
    BOARD_T200_CONTROLLER = 5
} Board_Kit_e;

//
//! \brief Initialization values of global variables
//
#define MOTOR_VARS_INIT {                                                      \
    false, /* flagEnableSys */                                                 \
    false, /* flagEnableRunAndIdentify */                                      \
    false, /* flagRunIdentAndOnLine */                                         \
    false, /* flagMotorIdentified */                                           \
    false, /* flagSetupController */                                           \
    true,  /* flagEnableForceAngle */                                          \
                                                                               \
    false, /* flagEnableRsRecalc */                                            \
    false, /* flagEnableRsOnLine */                                            \
    true,  /* flagEnableUserParams */                                          \
    true,  /* flagEnableOffsetcalc */                                          \
    false, /* flagEnablePowerWarp */                                           \
    false, /* flagBypassLockRotor */                                           \
                                                                               \
    true,  /* flagEnableSpeedCtrl */                                           \
    true,  /* flagEnableCurrentCtrl */                                         \
    MOTORCTRL_MODE_SPEED,  /* motorCtrlMode */                                 \
                                                                               \
    false, /* flagStateMotorRunning */                                         \
    false, /* flagEnableFlyingStart */                                         \
    false, /* flagStateFlyingStart */                                          \
    FLYINGSTART_MODE_HALT,  /* flyingStartMode */                              \
    0,     /* flyingStartTimeCnt */                                            \
    150,   /* flyingStartTimeDelay */                                          \
    0.0,   /* speedEst_Hz */                                                   \
                                                                               \
    false, /* flagEnableFWC */                                                 \
    false, /* bool flagEnableMTPA; */                                          \
    false, /* bool flagUpdateMTPAParams; */                                    \
                                                                               \
    CTRL_STATE_IDLE, /* ctrlState */                                           \
    EST_STATE_IDLE,  /* estState */                                            \
    EST_TRAJ_STATE_IDLE,    /* trajState */                                    \
    BOARD_BSXL8320RS_REVA,  /* boardKit */                                     \
                                                                               \
    10.0,  /* speedRef_Hz */                                                   \
    0.300, /* speedRef_krpm */                                                 \
    0.0,   /* speedTraj_Hz */                                                  \
    0.0,   /* speed_Hz */                                                      \
    0.0,   /* speed_krpm */                                                    \
    10.0,  /* accelerationMax_Hzps */                                          \
                                                                               \
    USER_MAX_VS_MAG_PU,               /* overModulation */                     \
    (0.1 * USER_MOTOR_MAX_CURRENT_A), /* RsOnLineCurrent_A */                  \
    0.0,  /* magneticCurrent_A */                                              \
    0.0,  /* Rr_Ohm */                                                         \
    0.0,  /* Rs_Ohm */                                                         \
    0.0,  /* RsOnLine_Ohm */                                                   \
    0.0,  /* Lsd_H */                                                          \
    0.0,  /* Lsq_H */                                                          \
    0.0,  /* RoverL_rps */                                                     \
    0.0,  /* flux_VpHz */                                                      \
    0.0,  /* flux_Wb */                                                        \
    0.0,  /* torque_Nm */                                                      \
                                                                               \
    0.1,  /* Kp_spd */                                                         \
    0.01, /* Ki_spd */                                                         \
                                                                               \
    0.2,  /* Kp_Id */                                                          \
    0.01, /* Ki_Id */                                                          \
                                                                               \
    0.2,  /* Kp_Iq */                                                          \
    0.01, /* Ki_Iq */                                                          \
                                                                               \
    USER_FWC_KP, /* Kp_fwc */                                                  \
    USER_FWC_KI, /* Ki_fwc */                                                  \
    USER_FWC_MAX_ANGLE_RAD, /* angleMax_fwc_rad */                             \
                                                                               \
    (0.8 * USER_MAX_VS_MAG_PU), /* VsRef_pu */                                 \
    (0.8 * USER_MAX_VS_MAG_PU  * USER_NOMINAL_DC_BUS_VOLTAGE_V), /* VsRef_V */ \
    0.0,  /* Vs_V */                                                           \
    0.0,  /* VdcBus_V */                                                       \
                                                                               \
    0.0,  /* IdRated_A */                                                      \
    0.0,  /* IdTarget_A */                                                     \
    0.0,  /* Is_A */                                                           \
    0.0,  /* IsRef_A */                                                        \
    0.0,  /* anglePhase_rad */                                                 \
                                                                               \
    {IA_OFFSET_A, IB_OFFSET_A, IC_OFFSET_A}, /* offsets_I_A */                 \
    {VA_OFFSET_V, VB_OFFSET_V, VC_OFFSET_V}, /* offsets_V_V */                 \
    (1.0 / VBUS_OFFSET_V), /* offset_invVbus_invV */                           \
                                                                               \
    1.0,  /* Vbus_sf */                                                        \
    0.0,  /* power_sf */                                                       \
    0.0,  /* power_W */                                                        \
                                                                               \
    0.0,  /* IsFilter_A */                                                     \
    0.0,  /* VsFilter_V */                                                     \
    0.0,  /* VbusFilter_V */                                                   \
                                                                               \
    0.0,  /* IqRefAbs_A */                                                     \
    0.0,  /* IqFdbAbs_A */                                                     \
                                                                               \
    0.0,  /* speedFdbAbs_Hz */                                                 \
    0.0,  /* speedRefAbs_Hz */                                                 \
    0.0,  /* speedRefDelta_Hz */                                               \
    0.0,  /* speedFdbDelta_Hz */                                               \
    0.0,  /* speedRefDeltaSet_Hz */                                            \
                                                                               \
    0.0,  /* float32_t mtpaConstant */                                         \
    0.0,  /* float32_t LsOnline_d_H */                                         \
    0.0,  /* float32_t LsOnline_q_H */                                         \
    0.0,  /* float32_t fluxOnline_VpHz */                                      \
                                                                               \
    0,  /* estISRCount */                                                      \
    0,  /* pwmISRCount */                                                      \
    0,  /* mainLoopCount */                                                    \
    0,  /* timerCnt_1ms */                                                     \
                                                                               \
    2048 + 1024 + 512 + 256, /* dacValH */                                     \
    2048 - 1024 - 512 - 256, /* dacValL */                                     \
    2048,  /* dacaVal */                                                       \
    2048,  /* dacbVal */                                                       \
                                                                               \
    0,  /* faultNow */                                                         \
    0,  /* faultUse */                                                         \
    0,  /* faultOld */                                                         \
    0   /* faultMask */                                                        \
}

//
//!  \brief typedefs for motorVars
//
typedef struct _MOTOR_Vars_t_
{
    bool flagEnableSys;
    bool flagEnableRunAndIdentify;
    bool flagRunIdentAndOnLine;
    bool flagMotorIdentified;
    bool flagSetupController;
    bool flagEnableForceAngle;

    bool flagEnableRsRecalc;
    bool flagEnableRsOnLine;
    bool flagEnableUserParams;
    bool flagEnableOffsetCalc;
    bool flagEnablePowerWarp;
    bool flagBypassLockRotor;

    bool flagEnableSpeedCtrl;
    bool flagEnableCurrentCtrl;
    MotorCtrl_Mode_e motorCtrlMode;

    bool flagStateMotorRunning;
    bool flagEnableFlyingStart;
    bool flagStateFlyingStart;
    FlyingStart_Mode_e flyingStartMode;
    uint16_t flyingStartTimeCnt;
    uint16_t flyingStartTimeDelay;
    float32_t speedEst_Hz;

    bool flagEnableFWC;
    bool flagEnableMTPA;
    bool flagUpdateMTPAParams;

    CTRL_State_e ctrlState;
    EST_State_e estState;
    EST_Traj_State_e trajState;
    Board_Kit_e boardKit;

    float32_t speedRef_Hz;
    float32_t speedRef_krpm;
    float32_t speedTraj_Hz;
    float32_t speed_Hz;
    float32_t speed_krpm;
    float32_t accelerationMax_Hzps;

    float32_t overModulation;
    float32_t RsOnLineCurrent_A;
    float32_t magneticCurrent_A;
    float32_t Rr_Ohm;
    float32_t Rs_Ohm;
    float32_t RsOnLine_Ohm;
    float32_t Ls_d_H;
    float32_t Ls_q_H;
    float32_t RoverL_rps;
    float32_t flux_VpHz;
    float32_t flux_Wb;
    float32_t torque_Nm;

    float32_t Kp_spd;
    float32_t Ki_spd;

    float32_t Kp_Id;
    float32_t Ki_Id;

    float32_t Kp_Iq;
    float32_t Ki_Iq;

    float32_t Kp_fwc;
    float32_t Ki_fwc;
    float32_t angleMax_fwc_rad;

    float32_t VsRef_pu;
    float32_t VsRef_V;
    float32_t Vs_V;
    float32_t VdcBus_V;

    float32_t IdRated_A;
    float32_t Id_target_A;
    float32_t Is_A;
    float32_t IsRef_A;
    float32_t angleCurrent_rad;

    MATH_Vec3 offsets_I_A;
    MATH_Vec3 offsets_V_V;
    float32_t offset_invVbus_invV;

    float32_t Vbus_sf;
    float32_t power_sf;
    float32_t power_W;

    float32_t IsFilter_A;
    float32_t VsFilter_V;
    float32_t VbusFilter_V;

    float32_t IqRefAbs_A;
    float32_t IqFdbAbs_A;

    float32_t speedFdbAbs_Hz;
    float32_t speedRefAbs_Hz;
    float32_t speedRefDelta_Hz;
    float32_t speedFdbDelta_Hz;
    float32_t speedRefDeltaSet_Hz;

    float32_t mtpaKconst;
    float32_t LsOnline_d_H;
    float32_t LsOnline_q_H;
    float32_t fluxOnline_Wb;

    uint32_t estISRCount;
    uint32_t pwmISRCount;
    uint32_t mainLoopCount;
    uint32_t timerCnt_1ms;

    uint16_t  dacValH;
    uint16_t  dacValL;
    uint16_t  dacaVal;
    uint16_t  dacbVal;

    FAULT_REG_t faultNow;
    FAULT_REG_t faultUse;
    FAULT_REG_t faultOld;
    FAULT_REG_t faultMask;
}MOTOR_Vars_t;


extern volatile MOTOR_Vars_t motorVars;
extern USER_Params   userParams;

extern PI_Handle     piHandle_Id;
extern PI_Handle     piHandle_Iq;
extern PI_Handle     piHandle_spd;

extern TRAJ_Handle   trajHandle_spd;

extern HAL_ADCData_t adcData;
extern HAL_PWMData_t pwmData;

extern EST_Handle    estHandle;

extern MATH_Vec2 Idq_in_A;
extern MATH_Vec2 Vab_out_V;
extern MATH_Vec2 Vdq_out_V;

//
// the function prototypes
//

//
//! \brief The main interrupt service (ISR) routine
//
extern __interrupt void mainISR(void);

//
//! \brief runs offset calculation using filters
//
extern void runOffsetsCalculation(void);

// CAN Function Prototypes ****************************

//
//! \brief Function called to calculate and send estimated RPM over CAN
//
extern void sendRPM(void);

//
//! \brief Function called to calculate and send estimated Voltage over CAN
//
extern void sendVoltage(void);

//
//! \brief Function called to calculate and send estimated Torque over CAN
//
extern void sendTorque(void);

//
//! \brief Function called to send board state over CAN
//
extern void sendState(void);

//
//! \brief Function called to send fault bits over CAN
//
extern void sendFault(void);

//******************************************************

//
//! \brief     Sets the number of current sensors
//! \param[in] handle             The Clarke (CLARKE) handle
//! \param[in] numCurrentSensors  The number of current sensors
//
static inline void
setupClarke_I(CLARKE_Handle handle, const uint_least8_t numCurrentSensors)
{
    float32_t alpha_sf, beta_sf;

    //
    // initialize the Clarke transform module for current
    //
    if(3 == numCurrentSensors)
    {
        alpha_sf = MATH_ONE_OVER_THREE;
        beta_sf = MATH_ONE_OVER_SQRT_THREE;
    }
    else if(2 == numCurrentSensors)
    {
        alpha_sf = 1.0;
        beta_sf = MATH_ONE_OVER_SQRT_THREE;
    }
    else
    {
        alpha_sf = 0.0;
        beta_sf = 0.0;
    }

    //
    // set the parameters
    //
    CLARKE_setScaleFactors(handle, alpha_sf, beta_sf);
    CLARKE_setNumSensors(handle, numCurrentSensors);

    return;
} // end of setupClarke_I() function

//
//! \brief     Sets the number of voltage sensors
//! \param[in] handle             The Clarke (CLARKE) handle
//! \param[in] numVoltageSensors  The number of voltage sensors
//
static inline void
setupClarke_V(CLARKE_Handle handle, const uint_least8_t numVoltageSensors)
{
    float32_t alpha_sf, beta_sf;

    //
    // initialize the Clarke transform module for voltage
    //
    if(3 == numVoltageSensors)
    {
        alpha_sf = MATH_ONE_OVER_THREE;
        beta_sf = MATH_ONE_OVER_SQRT_THREE;
    }
    else
    {
        alpha_sf = 0.0;
        beta_sf = 0.0;
    }

    //
    // set the parameters
    //
    CLARKE_setScaleFactors(handle, alpha_sf, beta_sf);
    CLARKE_setNumSensors(handle, numVoltageSensors);

    return;
} // end of setupClarke_V() function

//
//! \brief  Update the controllers
//
static inline void updateControllers(void)
{
    if(    (motorVars.estState == EST_STATE_ONLINE)
        && (motorVars.flagMotorIdentified == true))
    {
        //
        // update the Id controller
        //
        PI_setGains(piHandle_Id, motorVars.Kp_Id, motorVars.Ki_Id);

        //
        // update the Iq controller
        //
        PI_setGains(piHandle_Iq, motorVars.Kp_Iq, motorVars.Ki_Iq);

        //
        // update the speed controller
        //
        PI_setGains(piHandle_spd, motorVars.Kp_spd, motorVars.Ki_spd);
    }

    return;
}

//
//! \brief  Get the controllers Parameters
//
static inline void getControllers(void)
{
    //
    // Get the Id controller parameters
    //
    motorVars.Kp_Id = PI_getKp(piHandle_Id);
    motorVars.Ki_Id = PI_getKi(piHandle_Id);

    //
    // Get the Iq controller parameters
    //
    motorVars.Kp_Iq = PI_getKp(piHandle_Iq);
    motorVars.Ki_Iq = PI_getKi(piHandle_Iq);

    //
    // Get the speed controller parameters
    //
    motorVars.Kp_spd = PI_getKp(piHandle_spd);
    motorVars.Ki_spd = PI_getKi(piHandle_spd);

    return;
}

#ifdef _VSF_EN_
extern VSF_Obj    vsf;
extern VSF_Handle vsfHandle;

//
//! \brief  Computes the current controllers
//
static inline void computeCurrentControllers(void)
{
    float32_t RoverL_Kp_sf = userParams.RoverL_Kp_sf;
    float32_t dcBus_nominal_V = userParams.dcBus_nominal_V;
    float32_t maxCurrent_A = userParams.maxCurrent_A;
    float32_t RoverL_min_rps = userParams.RoverL_min_rps;
    float32_t currentCtrlPeriod_sec =
            ((float32_t)1.0)/((float32_t)vsf.pwmFreqNow_Hz);

    float32_t Kp = RoverL_Kp_sf * dcBus_nominal_V / maxCurrent_A;
    float32_t Ki = RoverL_min_rps * currentCtrlPeriod_sec;

    //
    // set the Id controller
    //
    PI_setGains(piHandle_Id, Kp, Ki);

    //
    // set the Iq controller
    //
    PI_setGains(piHandle_Iq, Kp, Ki);

    //
    // Get the Id controller parameters
    //
    motorVars.Kp_Id = PI_getKp(piHandle_Id);
    motorVars.Ki_Id = PI_getKi(piHandle_Id);

    //
    // Get the Iq controller parameters
    //
    motorVars.Kp_Iq = PI_getKp(piHandle_Iq);
    motorVars.Ki_Iq = PI_getKi(piHandle_Iq);

    return;
} // end of computeCurrentControllers() function


//
//! \brief  Computes the speed controller
//
static inline void computeSpeedControllers(void)
{

#if(!defined(USER_MOTOR_INERTIA_Kgm2))
    //
    // set the speed controller
    //
    PI_setGains(piHandle_spd, motorVars.Kp_spd, motorVars.Ki_spd);
#else
    float32_t BWc_rps = userParams.BWc_rps;

    float32_t speedCtrlPeriod_sec =
            ((float32_t)1.0)/((float32_t)vsf.pwmFreqNow_Hz);

    float32_t BWdelta = userParams.BWdelta;
    float32_t Kctrl_Wb_p_kgm2 = (float32_t)3.0 *
                              userParams.motor_numPolePairs *
                              userParams.motor_ratedFlux_Wb /
                              (float32_t) (2.0 * USER_MOTOR_INERTIA_Kgm2);

    float32_t Kp_spd =
            MATH_TWO_PI * BWc_rps / (BWdelta * Kctrl_Wb_p_kgm2);

    float32_t Ki_spd =
            BWc_rps * speedCtrlPeriod_sec / (BWdelta * BWdelta);

    //
    // set the speed controller
    //
    PI_setGains(piHandle_spd, Kp_spd, Ki_spd);
#endif

    //
    // Get the speed controller parameters
    //
    motorVars.Kp_spd = PI_getKp(piHandle_spd);
    motorVars.Ki_spd = PI_getKi(piHandle_spd);

    return;
} // end of computeSpeedController() function
#endif  // _VSF_EN_

//
//! \brief  Sets up the current controllers
//
static inline void setupCurrentControllers(void)
{
    float32_t RoverL_Kp_sf = userParams.RoverL_Kp_sf;
    float32_t dcBus_nominal_V = userParams.dcBus_nominal_V;
    float32_t maxCurrent_A = userParams.maxCurrent_A;
    float32_t RoverL_min_rps = userParams.RoverL_min_rps;
    float32_t currentCtrlPeriod_sec =
                (float32_t)userParams.numCtrlTicksPerCurrentTick /
                    userParams.ctrlFreq_Hz;
    float32_t outMax_V = userParams.Vd_sf * userParams.maxVsMag_V;

    float32_t Kp = RoverL_Kp_sf * dcBus_nominal_V / maxCurrent_A;
    float32_t Ki = RoverL_min_rps * currentCtrlPeriod_sec;

    //
    // set the Id controller
    //
    PI_setGains(piHandle_Id, Kp, Ki);
    PI_setUi(piHandle_Id, 0.0);
    PI_setRefValue(piHandle_Id, 0.0);
    PI_setFbackValue(piHandle_Id, 0.0);
    PI_setFfwdValue(piHandle_Id, 0.0);
    PI_setMinMax(piHandle_Id, -outMax_V, outMax_V);

    //
    // set the Iq controller
    //
    PI_setGains(piHandle_Iq, Kp, Ki);
    PI_setUi(piHandle_Iq, 0.0);
    PI_setRefValue(piHandle_Iq, 0.0);
    PI_setFbackValue(piHandle_Iq, 0.0);
    PI_setFfwdValue(piHandle_Iq, 0.0);
    PI_setMinMax(piHandle_Iq, -outMax_V, outMax_V);

    return;
} // end of setupCurrentControllers() function


//
//! \brief  Sets up the controllers
//
static inline void setupControllers(void)
{
    float32_t Ls_d_H = userParams.motor_Ls_d_H;
    float32_t Ls_q_H = userParams.motor_Ls_q_H;
    float32_t Rs_d_Ohm = userParams.motor_Rs_d_Ohm;
    float32_t Rs_q_Ohm = userParams.motor_Rs_q_Ohm;
    float32_t RdoverLd_rps = Rs_d_Ohm / Ls_d_H;
    float32_t RqoverLq_rps = Rs_q_Ohm / Ls_q_H;
    float32_t BWc_rps = userParams.BWc_rps;
    float32_t currentCtrlPeriod_sec =
                (float32_t)userParams.numCtrlTicksPerCurrentTick /
                userParams.ctrlFreq_Hz;
    float32_t outMax_V = userParams.Vd_sf * userParams.maxVsMag_V;

    float32_t Kp_Id = Ls_d_H * BWc_rps;
    float32_t Ki_Id = RdoverLd_rps * currentCtrlPeriod_sec;

    float32_t Kp_Iq = Ls_q_H * BWc_rps;
    float32_t Ki_Iq = RqoverLq_rps * currentCtrlPeriod_sec;

    //
    // set the Id controller
    //
    PI_setGains(piHandle_Id, Kp_Id, Ki_Id);
    PI_setUi(piHandle_Id, 0.0);
    PI_setRefValue(piHandle_Id, 0.0);
    PI_setFbackValue(piHandle_Id, 0.0);
    PI_setFfwdValue(piHandle_Id, 0.0);
    PI_setMinMax(piHandle_Id, -outMax_V, outMax_V);

    //
    // set the Iq controller
    //
    PI_setGains(piHandle_Iq, Kp_Iq, Ki_Iq);
    PI_setUi(piHandle_Iq, 0.0);
    PI_setRefValue(piHandle_Iq, 0.0);
    PI_setFbackValue(piHandle_Iq, 0.0);
    PI_setFfwdValue(piHandle_Iq, 0.0);
    PI_setMinMax(piHandle_Iq, 0.0, 0.0);

    //
    // set the speed controller
    //
    PI_setGains(piHandle_spd, motorVars.Kp_spd, motorVars.Ki_spd);

    //
    // set the speed controller
    //
    PI_setUi(piHandle_spd, 0.0);
    PI_setRefValue(piHandle_spd, 0.0);
    PI_setFbackValue(piHandle_spd, 0.0);
    PI_setFfwdValue(piHandle_spd, 0.0);
    PI_setMinMax(piHandle_spd,
                 (-userParams.maxCurrent_A),
                 userParams.maxCurrent_A);

    //
    // set the Id, Iq and speed controller parameters to motorVars
    //
    getControllers();

    return;
} // end of setupControllers() function

//
//! \brief      Updates the global motor variables
//! \param[in]  estHandle   The estimator (EST) handle
//
static inline void updateGlobalVariables(EST_Handle estHandle)
{
    //
    // get the states
    //
#ifdef __TMS320C28XX_CLA__
    motorVars.estState = cla_EST_getState(estHandle);
    motorVars.trajState = cla_EST_getTrajState(estHandle);
#else
    motorVars.estState = EST_getState(estHandle);
    motorVars.trajState = EST_getTrajState(estHandle);
#endif

    //
    // get the rotor resistance
    //
#ifdef __TMS320C28XX_CLA__
    motorVars.Rr_Ohm = cla_EST_getRr_Ohm(estHandle);
#else
    motorVars.Rr_Ohm = EST_getRr_Ohm(estHandle);
#endif

    //
    // get the stator resistance
    //
#ifdef __TMS320C28XX_CLA__
    motorVars.Rs_Ohm = cla_EST_getRs_Ohm(estHandle);
#else
    motorVars.Rs_Ohm = EST_getRs_Ohm(estHandle);
#endif

    //
    // get the stator inductance in the direct coordinate direction
    //
#ifdef __TMS320C28XX_CLA__
    motorVars.Ls_d_H = cla_EST_getLs_d_H(estHandle);
#else
    motorVars.Ls_d_H = EST_getLs_d_H(estHandle);
#endif

    //
    // get the stator inductance in the quadrature coordinate direction
    //
#ifdef __TMS320C28XX_CLA__
    motorVars.Ls_q_H = cla_EST_getLs_q_H(estHandle);
#else
    motorVars.Ls_q_H = EST_getLs_q_H(estHandle);
#endif

    //
    // get the flux, V/Hz
    //
#ifdef __TMS320C28XX_CLA__
    motorVars.flux_VpHz = cla_EST_getFlux_Wb(estHandle) * MATH_TWO_PI;
#else
    motorVars.flux_VpHz = EST_getFlux_Wb(estHandle) * MATH_TWO_PI;
#endif

    //
    // get the flux, Wb
    //
#ifdef __TMS320C28XX_CLA__
    motorVars.flux_Wb = cla_EST_getFlux_Wb(estHandle);
#else
    motorVars.flux_Wb = EST_getFlux_Wb(estHandle);
#endif

    //
    // get the rated magnetizing current value
    //
#ifdef __TMS320C28XX_CLA__
    motorVars.magneticCurrent_A = cla_EST_getIdRated_A(estHandle);
#else
    motorVars.magneticCurrent_A = EST_getIdRated_A(estHandle);
#endif

    //
    // get R/L
    //
#ifdef __TMS320C28XX_CLA__
    motorVars.RoverL_rps = cla_EST_getRoverL_rps(estHandle);
#else
    motorVars.RoverL_rps = EST_getRoverL_rps(estHandle);
#endif

    //
    // get the low pass filtered speed estimate
    //
#ifdef __TMS320C28XX_CLA__
    motorVars.speed_Hz = cla_EST_getFm_lp_Hz(estHandle);
#else
    motorVars.speed_Hz = EST_getFm_lp_Hz(estHandle);
#endif

    motorVars.speed_krpm = motorVars.speed_Hz * (60.0 / 1000.0 /
                                                 USER_MOTOR_NUM_POLE_PAIRS);

    //
    // get the torque estimate
    //
#ifdef __TMS320C28XX_CLA__
    motorVars.torque_Nm = cla_EST_computeTorque_Nm(estHandle);
#else
    motorVars.torque_Nm = EST_computeTorque_Nm(estHandle);
#endif

    //
    // get the stator resistance estimate from RsOnLine
    //
#ifdef __TMS320C28XX_CLA__
    motorVars.RsOnLine_Ohm = cla_EST_getRsOnLine_Ohm(estHandle);
#else
    motorVars.RsOnLine_Ohm = EST_getRsOnLine_Ohm(estHandle);
#endif

    motorVars.VdcBus_V = adcData.dcBus_V;

    //
    // compute the vector voltage
    //
    motorVars.Vs_V = sqrt((Vdq_out_V.value[0] * Vdq_out_V.value[0]) +
                          (Vdq_out_V.value[1] * Vdq_out_V.value[1]));

    return;
} // end of updateGlobalVariables() function


//
//! \brief      Flying start run control
//! \param[in]  estHandle   The estimator (EST) handle
//
static inline void runFlyingStart(EST_Handle estHandle)
{
    if(motorVars.flagEnableFlyingStart == true)
    {
        motorVars.speedEst_Hz = (motorVars.speedEst_Hz * 0.8) +
                                (motorVars.speed_Hz * 0.20);

        if(motorVars.flagStateFlyingStart == true)
        {
            motorVars.flyingStartTimeCnt++;

            if(motorVars.flyingStartTimeCnt >= motorVars.flyingStartTimeDelay)
            {
                motorVars.flagStateFlyingStart = false;

                TRAJ_setIntValue(trajHandle_spd, motorVars.speedEst_Hz);
            }
        }
    }

    return;
} // end of runFlyingStart() function


//
//! \brief     Runs Rs online
//! \param[in]  estHandle   The estimator (EST) handle
//
extern void runRsOnLine(EST_Handle estHandle);


//
//! \brief      Control motor running
//! \param[in]  estHandle   The estimator (EST) handle
//
extern void runMotorCtrl(EST_Handle estHandle);


//
//@} //defgroup
//
#endif // end of LABS_H definition
