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

#ifndef LABS_DM_H
#define LABS_DM_H

//**************************************************************************
//! \file   solutions/common/sensorless_foc/include/labs.h
//! \brief  header file to be included in all labs
//!
//**************************************************************************

// **************************************************************************

//! \defgroup LABS LABS
//@{

// **************************************************************************
// modules
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
#include "fwc.h"
#include "mtpa.h"

#include "angle_gen.h"
#include "datalog.h"
#include "cpu_time.h"


// solutions
#if !defined(__TMS320C28XX_CLA__)

#include "user_m1.h"
#include "user_m2.h"
#include "hal_dm.h"

#endif

#ifdef DRV8301_SPI
#include "drv8301.h"
#endif

#ifdef DRV8320_SPI
#include "drv8320.h"
#endif

#define LED_BLINK_FREQ_Hz           (1.0)     // 1Hz

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
    BOARD_DRV8301_REVD = 1,             //!< the board is DRV8301_RevD
    BOARD_HVMTRPFC_REV1P1 = 2,          //!< the board is HVMTRPFC_Rev1p1
    BOARD_DRV8312_REVD = 3,             //!< the board is DRV8312_RevD
    BOARD_IDDK_REV2P1 = 4               //!< the board is IDDK2.1
} Board_Kit_e;

//! \brief Initialization values of global variables
//!

#define MOTOR1_VARS_INIT { \
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
    USER_M1_MAX_VS_MAG_PU,            /* overModulation */                     \
    (0.1 * USER_M1_MOTOR_MAX_CURRENT_A), /* RsOnLineCurrent_A */               \
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
    0.5,  /* Kp_Id */                                                          \
    0.01, /* Ki_Id */                                                          \
                                                                               \
    0.5,  /* Kp_Iq */                                                          \
    0.01, /* Ki_Iq */                                                          \
                                                                               \
    USER_M1_FWC_KP, /* Kp_fwc */                                               \
    USER_M1_FWC_KI, /* Ki_fwc */                                               \
    USER_M1_FWC_MIN_ANGLE_RAD, /* angleMax_fwc_rad */                          \
                                                                               \
    (0.8 * USER_M1_MAX_VS_MAG_PU), /* VsRef_pu */                              \
    (0.8 * USER_M1_MAX_VS_MAG_PU * USER_M1_NOMINAL_DC_BUS_VOLTAGE_V), /* VsRef_V */ \
    0.0,  /* Vs_V */                                                           \
    0.0,  /* VdcBus_V */                                                       \
                                                                               \
    0.0,  /* IdRated_A */                                                      \
    0.0,  /* IdTarget_A */                                                     \
    0.0,  /* Is_A */                                                           \
    0.0,  /* IsRef_A */                                                        \
    0.0,  /* anglePhase_rad */                                                 \
                                                                               \
    {USER_M1_IA_OFFSET_A, USER_M1_IB_OFFSET_A, USER_M1_IC_OFFSET_A}, /* offsets_I_A */ \
    {USER_M1_VA_OFFSET_V, USER_M1_VB_OFFSET_V, USER_M1_VC_OFFSET_V}, /* offsets_V_V */ \
    (1.0 / USER_M1_VBUS_OFFSET_V), /* offset_invVbus_invV */                   \
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
    2048 + 1024 + 512, /* dacValH */                                           \
    2048 - 1024 - 512, /* dacValL */                                           \
    2048,  /* dacaVal */                                                       \
    2048,  /* dacbVal */                                                       \
                                                                               \
    0,  /* faultNow */                                                         \
    0,  /* faultUse */                                                         \
    0   /* faultMask */                                                        \
}


#define MOTOR2_VARS_INIT { \
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
    USER_M2_MAX_VS_MAG_PU,            /* overModulation */                     \
    (0.1 * USER_M2_MOTOR_MAX_CURRENT_A), /* RsOnLineCurrent_A */               \
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
    0.5,  /* Kp_Id */                                                          \
    0.01, /* Ki_Id */                                                          \
                                                                               \
    0.5,  /* Kp_Iq */                                                          \
    0.01, /* Ki_Iq */                                                          \
                                                                               \
    USER_M2_FWC_KP, /* Kp_fwc */                                               \
    USER_M2_FWC_KI, /* Ki_fwc */                                               \
    USER_M2_FWC_MIN_ANGLE_RAD, /* angleMax_fwc_rad */                          \
                                                                               \
    (0.8 * USER_M2_MAX_VS_MAG_PU), /* VsRef_pu */                              \
    (0.8 * USER_M2_MAX_VS_MAG_PU * USER_M2_NOMINAL_DC_BUS_VOLTAGE_V), /* VsRef_V */ \
    0.0,  /* Vs_V */                                                           \
    0.0,  /* VdcBus_V */                                                       \
                                                                               \
    0.0,  /* IdRated_A */                                                      \
    0.0,  /* IdTarget_A */                                                     \
    0.0,  /* Is_A */                                                           \
    0.0,  /* IsRef_A */                                                        \
    0.0,  /* anglePhase_rad */                                                 \
                                                                               \
    {USER_M2_IA_OFFSET_A, USER_M2_IB_OFFSET_A, USER_M2_IC_OFFSET_A}, /* offsets_I_A */ \
    {USER_M2_VA_OFFSET_V, USER_M2_VB_OFFSET_V, USER_M2_VC_OFFSET_V}, /* offsets_V_V */ \
    (1.0 / USER_M2_VBUS_OFFSET_V), /* offset_invVbus_invV */                   \
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
    2048 + 1024 + 512, /* dacValH */                                           \
    2048 - 1024 - 512, /* dacValL */                                           \
    2048,  /* dacaVal */                                                       \
    2048,  /* dacbVal */                                                       \
                                                                               \
    0,  /* faultNow */                                                         \
    0,  /* faultUse */                                                         \
    0   /* faultMask */                                                        \
}

// typedefs
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
    FAULT_REG_t faultMask;
}MOTOR_Vars_t;

//------------------------------------------------------------------------
#define SYSTEM_VARS_INIT  { \
        false, \
        false, \
        false, \
        20.0, \
        10.0 \
}

typedef struct _SYSTEM_Vars_t_
{
    bool flagEnableSystem;
    bool flagEnableSynControl;
    bool flagEnableRun;

    float32_t speedSet_Hz;
    float32_t accelerationMaxSet_Hzps;
}SYSTEM_Vars_t;

extern volatile SYSTEM_Vars_t systemVars;
extern volatile MOTOR_Vars_t motorVars[2];
extern USER_Params  userParams[2];

extern PI_Handle    piHandle_Id[2];
extern PI_Handle    piHandle_Iq[2];
extern PI_Handle    piHandle_spd[2];

extern HAL_ADCData_t adcData[2];
extern HAL_PWMData_t pwmData[2];

extern EST_Handle    estHandle[2];            //!< the handle for the estimator


// **************************************************************************
// the function prototypes

//! \brief The main interrupt service (ISR) routine
extern __interrupt void mainISR(void);

//! \brief runs offset calculation using filters
extern void runOffsetsCalculation(const uint16_t motorNum);

//! \brief     Sets the number of current sensors
//! \param[in] handle             The Clarke (CLARKE) handle
//! \param[in] numCurrentSensors  The number of current sensors
static inline void
setupClarke_I(CLARKE_Handle handle,const uint16_t numCurrentSensors)
{
    float32_t alpha_sf, beta_sf;

    // initialize the Clarke transform module for current
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

    // set the parameters
    CLARKE_setScaleFactors(handle, alpha_sf, beta_sf);
    CLARKE_setNumSensors(handle, numCurrentSensors);

    return;
} // end of setupClarke_I() function

//! \brief     Sets the number of voltage sensors
//! \param[in] handle             The Clarke (CLARKE) handle
//! \param[in] numVoltageSensors  The number of voltage sensors
static inline void
setupClarke_V(CLARKE_Handle handle,const uint16_t numVoltageSensors)
{
    float32_t alpha_sf,beta_sf;

    // initialize the Clarke transform module for voltage
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

    // set the parameters
    CLARKE_setScaleFactors(handle, alpha_sf, beta_sf);
    CLARKE_setNumSensors(handle, numVoltageSensors);

    return;
} // end of setupClarke_V() function


//! \brief  Update the controllers
static inline void updateControllers(const HAL_MotorNum_e motorNum)
{
    if((motorVars[motorNum].estState == EST_STATE_ONLINE)
            && (motorVars[motorNum].flagMotorIdentified == true))
    {
        // update the Id controller
        PI_setGains(piHandle_Id[motorNum],
                    motorVars[motorNum].Kp_Id, motorVars[motorNum].Ki_Id);

        // update the Iq controller
        PI_setGains(piHandle_Iq[motorNum],
                    motorVars[motorNum].Kp_Iq, motorVars[motorNum].Ki_Iq);

        // update the speed controller
        PI_setGains(piHandle_spd[motorNum],
                    motorVars[motorNum].Kp_spd, motorVars[motorNum].Ki_spd);
    }
}


//! \brief  Get the controllers Parameters
static inline void getControllers(const HAL_MotorNum_e motorNum)
{
    // Get the Id controller parameters
    motorVars[motorNum].Kp_Id = PI_getKp(piHandle_Id[motorNum]);
    motorVars[motorNum].Ki_Id = PI_getKi(piHandle_Id[motorNum]);

    // Get the Iq controller parameters
    motorVars[motorNum].Kp_Iq = PI_getKp(piHandle_Iq[motorNum]);
    motorVars[motorNum].Ki_Iq = PI_getKi(piHandle_Iq[motorNum]);

    // Get the speed controller parameters
    motorVars[motorNum].Kp_spd = PI_getKp(piHandle_spd[motorNum]);
    motorVars[motorNum].Ki_spd = PI_getKi(piHandle_spd[motorNum]);
}


//! \brief  Sets up the current controllers
static inline void setupCurrentControllers(const HAL_MotorNum_e motorNum)
{
    float32_t RoverL_Kp_sf = userParams[motorNum].RoverL_Kp_sf;
    float32_t dcBus_nominal_V = userParams[motorNum].dcBus_nominal_V;
    float32_t maxCurrent_A = userParams[motorNum].maxCurrent_A;
    float32_t RoverL_min_rps = userParams[motorNum].RoverL_min_rps;
    float32_t currentCtrlPeriod_sec =
                (float32_t)userParams[motorNum].numCtrlTicksPerCurrentTick /
                    userParams[motorNum].ctrlFreq_Hz;

    float32_t outMax_V = userParams[motorNum].Vd_sf *
                         userParams[motorNum].maxVsMag_V;

    float32_t Kp = RoverL_Kp_sf * dcBus_nominal_V / maxCurrent_A;
    float32_t Ki = RoverL_min_rps * currentCtrlPeriod_sec;

    // set the Id controller
    PI_setGains(piHandle_Id[motorNum], Kp, Ki);
    PI_setUi(piHandle_Id[motorNum], 0.0);
    PI_setRefValue(piHandle_Id[motorNum], 0.0);
    PI_setFbackValue(piHandle_Id[motorNum], 0.0);
    PI_setFfwdValue(piHandle_Id[motorNum], 0.0);
    PI_setMinMax(piHandle_Id[motorNum], -outMax_V, outMax_V);

    // set the Iq controller
    PI_setGains(piHandle_Iq[motorNum], Kp, Ki);
    PI_setUi(piHandle_Iq[motorNum], 0.0);
    PI_setRefValue(piHandle_Iq[motorNum], 0.0);
    PI_setFbackValue(piHandle_Iq[motorNum], 0.0);
    PI_setFfwdValue(piHandle_Iq[motorNum], 0.0);
    PI_setMinMax(piHandle_Iq[motorNum], -outMax_V, outMax_V);

    return;
} // end of setupCurrentControllers() function


//! \brief  Sets up the controllers
static inline void setupControllers(const HAL_MotorNum_e motorNum)
{
    float32_t Ls_d_H = userParams[motorNum].motor_Ls_d_H;
    float32_t Ls_q_H = userParams[motorNum].motor_Ls_q_H;
    float32_t Rs_d_Ohm = userParams[motorNum].motor_Rs_d_Ohm;
    float32_t Rs_q_Ohm = userParams[motorNum].motor_Rs_q_Ohm;
    float32_t RdoverLd_rps = Rs_d_Ohm / Ls_d_H;
    float32_t RqoverLq_rps = Rs_q_Ohm / Ls_q_H;
    float32_t BWc_rps = userParams[motorNum].BWc_rps;
    float32_t currentCtrlPeriod_sec =
                (float32_t)userParams[motorNum].numCtrlTicksPerCurrentTick /
                userParams[motorNum].ctrlFreq_Hz;

    float32_t outMax_V = userParams[motorNum].Vd_sf *
            userParams[motorNum].maxVsMag_V;

    float32_t Kp_Id = Ls_d_H * BWc_rps;
    float32_t Ki_Id = RdoverLd_rps * currentCtrlPeriod_sec;

    float32_t Kp_Iq = Ls_q_H * BWc_rps;
    float32_t Ki_Iq = RqoverLq_rps * currentCtrlPeriod_sec;

    // set the Id controller
    PI_setGains(piHandle_Id[motorNum], Kp_Id, Ki_Id);
    PI_setUi(piHandle_Id[motorNum], 0.0);
    PI_setRefValue(piHandle_Id[motorNum], 0.0);
    PI_setFbackValue(piHandle_Id[motorNum], 0.0);
    PI_setFfwdValue(piHandle_Id[motorNum], 0.0);
    PI_setMinMax(piHandle_Id[motorNum], -outMax_V, outMax_V);

    // set the Iq controller
    PI_setGains(piHandle_Iq[motorNum], Kp_Iq, Ki_Iq);
    PI_setUi(piHandle_Iq[motorNum], 0.0);
    PI_setRefValue(piHandle_Iq[motorNum], 0.0);
    PI_setFbackValue(piHandle_Iq[motorNum], 0.0);
    PI_setFfwdValue(piHandle_Iq[motorNum], 0.0);
    PI_setMinMax(piHandle_Iq[motorNum], 0.0, 0.0);

#if (!defined(USER_MOTOR_INERTIA_Kgm2))
    // set the speed controller
    PI_setGains(piHandle_spd[motorNum], motorVars[motorNum].Kp_spd,
                motorVars[motorNum].Ki_spd);
#else
    float32_t speedCtrlPeriod_sec =
        (float32_t)userParams[motorNum].numCtrlTicksPerSpeedTick /
        userParams[motorNum].ctrlFreq_Hz;

    float32_t BWdelta = userParams[motorNum].BWdelta;

    float32_t Kctrl_Wb_p_kgm2 = userParams[motorNum].Kctrl_Wb_p_kgm2;

    float32_t Kp_spd = MATH_TWO_PI * BWc_rps / (BWdelta * Kctrl_Wb_p_kgm2);
    float32_t Ki_spd = BWc_rps * speedCtrlPeriod_sec / (BWdelta * BWdelta);

    // set the speed controller
    PI_setGains(piHandle_spd[motorNum], Kp_spd, Ki_spd);
#endif

    // set the speed controller
    PI_setUi(piHandle_spd[motorNum], 0.0);
    PI_setRefValue(piHandle_spd[motorNum], 0.0);
    PI_setFbackValue(piHandle_spd[motorNum], 0.0);
    PI_setFfwdValue(piHandle_spd[motorNum], 0.0);
    PI_setMinMax(piHandle_spd[motorNum],
                 -userParams[motorNum].maxCurrent_A,
                 userParams[motorNum].maxCurrent_A);

    // set the Id, Iq and speed controller parameters to motorVars
    getControllers(motorNum);

    return;
} // end of setupControllers() function


//! \brief      Updates the global motor variables
//! \param[in]  estHandle   The estimator (EST) handle
static inline void
updateGlobalVariables(EST_Handle estHandle, const HAL_MotorNum_e motorNum)
{
    // get the states
#ifdef __TMS320C28XX_CLA__
    motorVars[motorNum].estState = cla_EST_getState(estHandle);
    motorVars[motorNum].trajState = cla_EST_getTrajState(estHandle);
#else
    motorVars[motorNum].estState = EST_getState(estHandle);
    motorVars[motorNum].trajState = EST_getTrajState(estHandle);
#endif

    // get the rotor resistance
#ifdef __TMS320C28XX_CLA__
    motorVars[motorNum].Rr_Ohm = cla_EST_getRr_Ohm(estHandle);
#else
    motorVars[motorNum].Rr_Ohm = EST_getRr_Ohm(estHandle);
#endif

    // get the stator resistance
#ifdef __TMS320C28XX_CLA__
    motorVars[motorNum].Rs_Ohm = cla_EST_getRs_Ohm(estHandle);
#else
    motorVars[motorNum].Rs_Ohm = EST_getRs_Ohm(estHandle);
#endif

    // get the stator inductance in the direct coordinate direction
#ifdef __TMS320C28XX_CLA__
    motorVars[motorNum].Ls_d_H = cla_EST_getLs_d_H(estHandle);
#else
    motorVars[motorNum].Ls_d_H = EST_getLs_d_H(estHandle);
#endif

    // get the stator inductance in the quadrature coordinate direction
#ifdef __TMS320C28XX_CLA__
    motorVars[motorNum].Ls_q_H = cla_EST_getLs_q_H(estHandle);
#else
    motorVars[motorNum].Ls_q_H = EST_getLs_q_H(estHandle);
#endif

    // get the flux, V/Hz
#ifdef __TMS320C28XX_CLA__
    motorVars[motorNum].flux_VpHz = cla_EST_getFlux_Wb(estHandle) * MATH_TWO_PI;
#else
    motorVars[motorNum].flux_VpHz = EST_getFlux_Wb(estHandle) * MATH_TWO_PI;
#endif

    // get the rated magnetizing current value
#ifdef __TMS320C28XX_CLA__
    motorVars[motorNum].magneticCurrent_A = cla_EST_getIdRated_A(estHandle);
#else
    motorVars[motorNum].magneticCurrent_A = EST_getIdRated_A(estHandle);
#endif

    // get R/L
#ifdef __TMS320C28XX_CLA__
    motorVars[motorNum].RoverL_rps = cla_EST_getRoverL_rps(estHandle);
#else
    motorVars[motorNum].RoverL_rps = EST_getRoverL_rps(estHandle);
#endif

    // get the low pass filtered speed estimate
#ifdef __TMS320C28XX_CLA__
    motorVars[motorNum].speed_Hz = cla_EST_getFm_lp_Hz(estHandle);
#else
    motorVars[motorNum].speed_Hz = EST_getFm_lp_Hz(estHandle);
#endif

    motorVars[motorNum].speed_krpm = motorVars[motorNum].speed_Hz *
            (60.0 / 1000.0 / userParams[motorNum].motor_numPolePairs);

    // get the torque estimate
#ifdef __TMS320C28XX_CLA__
    motorVars[motorNum].torque_Nm = cla_EST_computeTorque_Nm(estHandle);
#else
    motorVars[motorNum].torque_Nm = EST_computeTorque_Nm(estHandle);
#endif

    // get the stator resistance estimate from RsOnLine
#ifdef __TMS320C28XX_CLA__
    motorVars[motorNum].RsOnLine_Ohm = cla_EST_getRsOnLine_Ohm(estHandle);
#else
    motorVars[motorNum].RsOnLine_Ohm = EST_getRsOnLine_Ohm(estHandle);
#endif

    motorVars[motorNum].VdcBus_V = adcData[motorNum].dcBus_V;

    return;
} // end of updateGlobalVariables() function

//@} //defgroup
#endif // end of LABS_DM_H definition
