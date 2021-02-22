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

//| Modified from lab8 from TI's InstaSPIN FOC MotorControl SDK
//| Other modifications are found throughout project files
//|
//| For The TMS320F2800041C and DRV8320
//| And use with the custom motor controller
//| for the T200 UUV thruster
//|
//| Mark Belbin -- Feb 2021

// **************************************************************************

#include "labs.h"

#pragma CODE_SECTION(mainISR, ".TI.ramfunc");

//
// the globals
//

#define CAN_TELEM_FREQ_Hz  (10.0)     // 10 Hz
#define RPM_RESET_FREQ_Hz  (0.2)     // 0.2 Hz, 5 seconds

uint16_t rxMsgData[3]; // Placeholder for recieved data, maximum of three bytes
uint16_t rpmMsgData[3];
uint16_t voltageData[2];
uint16_t torqueData[2];
uint16_t faults[2];
uint16_t boardState[1];

bool RPMset = false;

//***********************

HAL_ADCData_t adcData = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0.0};

HAL_PWMData_t pwmData = {{0.0, 0.0, 0.0}};

uint16_t counterLED = 0;  //!< Counter used to divide down the ISR rate for
                           //!< visually blinking an LED

uint16_t counterCAN = 0; // Counter used to determine when to send CAN telemetry
uint32_t counterRPM = 0; // Counter to check if RPM signals are not being recieved

uint16_t counterSpeed = 0;
uint16_t counterTrajSpeed = 0;
uint16_t counterTrajId = 0;

uint32_t offsetCalcCount = 0;     //!< Counter used to count the wait time
                                  //!< for offset calibration, unit: ISR cycles

uint32_t offsetCalcWaitTime = 50000;  //!< Wait time setting for current/voltage
                                      //!< offset calibration, unit: ISR cycles

EST_InputData_t estInputData = {0, {0.0, 0.0}, {0.0, 0.0}, 0.0, 0.0};

EST_OutputData_t estOutputData = {0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  {0.0, 0.0}, {0.0, 0.0}, 0, 0.0};
float32_t angleDelta_rad;   //!< the rotor angle compensation value
float32_t angleEst_rad;     //!< the rotor angle from FAST estimator
float32_t angleGen_rad;     //!< the rotor angle from rampgen module
float32_t angleFoc_rad;     //!< the rotor angle for FOC modules

MATH_Vec2 Idq_ref_A;        //!< the reference current on d&q rotation axis
MATH_Vec2 Idq_offset_A;     //!< the offsetting current on d&q rotation axis
MATH_Vec2 Iab_in_A;         //!< the alpha&beta axis current are converter from
                            //!< 3-phase sampling input current of motor
MATH_Vec2 Idq_in_A;         //!< the d&q axis current are converter from
                            //!< 3-phase sampling input current of motor
MATH_Vec2 Vab_out_V;        //!< the output control voltage on alpha&beta axis
MATH_Vec2 Vdq_out_V;        //!< the output control voltage on d&q axis

USER_Params userParams;    //!< the user parameters for motor control
                            //!< and hardware board configuration
#pragma DATA_SECTION(userParams, "ctrl_data");

volatile MOTOR_Vars_t motorVars = MOTOR_VARS_INIT;
#pragma DATA_SECTION(motorVars, "ctrl_data");

CTRL_Handle   ctrlHandle;       //!< the handle for the controller
CTRL_Obj      ctrl;             //!< the controller object

CLARKE_Handle clarkeHandle_I;   //!< the handle for the current Clarke transform
CLARKE_Obj    clarke_I;         //!< the current Clarke transform object

CLARKE_Handle clarkeHandle_V;   //!< the handle for the voltage Clarke transform
CLARKE_Obj    clarke_V;         //!< the voltage Clarke transform object

EST_Handle    estHandle;        //!< the handle for the estimator

HAL_Handle    halHandle;     //!< the handle for the hardware abstraction layer
HAL_Obj       hal;           //!< the hardware abstraction layer object

IPARK_Handle  iparkHandle;      //!< the handle for the inverse Park transform
IPARK_Obj     ipark;            //!< the inverse Park transform object

PARK_Handle   parkHandle;       //!< the handle for the Park object
PARK_Obj      park;             //!< the Park transform object

PI_Handle     piHandle_Id;      //!< the handle for the Id PI controller
PI_Obj        pi_Id;            //!< the Id PI controller object

PI_Handle     piHandle_Iq;      //!< the handle for the Iq PI controller
PI_Obj        pi_Iq;            //!< the Iq PI controller object

PI_Handle     piHandle_fwc;     //!< the handle for the Iq PI controller
PI_Obj        pi_fwc;           //!< the Iq PI controller object

PI_Handle     piHandle_spd;     //!< the handle for the speed PI controller
PI_Obj        pi_spd;           //!< the speed PI controller object

SVGEN_Handle  svgenHandle;      //!< the handle for the space vector generator
SVGEN_Obj     svgen;            //!< the space vector generator object

SVGENCURRENT_Obj svgencurrent;              //!< the handle for the space vector
                                            //!< generator current
SVGENCURRENT_Handle svgencurrentHandle;     //!< the space vector generator
                                            //!< current object

TRAJ_Handle   trajHandle_spd; //!< the handle for the speed reference trajectory
TRAJ_Obj      traj_spd;       //!< the speed reference trajectory object

TRAJ_Handle   trajHandle_Id;    //!< the handle for the id reference trajectory
TRAJ_Obj      traj_Id;          //!< the id reference trajectory object

TRAJ_Handle   trajHandle_fwc;   //!< the handle for the id reference trajectory
TRAJ_Obj      traj_fwc;         //!< the id reference trajectory object

//!< the handles for the current offset calculation
FILTER_FO_Handle  filterHandle_I[USER_NUM_CURRENT_SENSORS];
//!< the current offset calculation
FILTER_FO_Obj     filter_I[USER_NUM_CURRENT_SENSORS];

//
// the handles for the voltage offset calculation
//
FILTER_FO_Handle  filterHandle_V[USER_NUM_VOLTAGE_SENSORS];

//
// the voltage offset calculation
//
FILTER_FO_Obj     filter_V[USER_NUM_VOLTAGE_SENSORS];

SVGENCURRENT_MeasureShunt_e measurableShuntThisCycle = SVGENCURRENT_ALL_PHASE_MEASURABLE;
SVGENCURRENT_IgnoreShunt_e ignoreShuntNextCycle = SVGENCURRENT_USE_ALL;
SVGENCURRENT_VmidShunt_e midVolShunt = SVGENCURRENT_VMID_A;

//
//  set the offset, default value of 1 microsecond
//
int16_t cmpOffset = (int16_t)(1.0 * USER_SYSTEM_FREQ_MHz);

MATH_Vec3 adcDataPrev = {0.0, 0.0, 0.0};
MATH_Vec3 pwmDataPrev = {0.0, 0.0, 0.0};

#ifdef DRV8320_SPI
//
// Watch window interface to the 8320 SPI
//
DRV8320_SPIVars_t drvSPI8320Vars;
#pragma DATA_SECTION(drvSPI8320Vars, "ctrl_data");
#endif

//
// the functions
//

void main(void)
{
    uint16_t estNumber = 0;
    bool flagEstStateChanged = false;

    // Global board state
    boardState[0] = 0x00;

    // Define board used
    motorVars.boardKit = BOARD_T200_CONTROLLER;

    //************************************

    //
    // initialize the user parameters
    //
    USER_setParams(&userParams);

    userParams.flag_bypassMotorId = true;

    motorVars.flagEnableOffsetCalc = true;

    motorVars.Kp_spd = 0.2; // Set Kp and Ki values for speed controller.
    motorVars.Ki_spd = 0.001;

    //
    // initialize the user parameters
    //
    USER_setParams_priv(&userParams);

    //
    // initialize the driver
    //
    halHandle = HAL_init(&hal, sizeof(hal));

    //
    // set the driver parameters
    //
    HAL_setParams(halHandle);

    //
    // initialize the Clarke modules
    //
    clarkeHandle_I = CLARKE_init(&clarke_I, sizeof(clarke_I));
    clarkeHandle_V = CLARKE_init(&clarke_V, sizeof(clarke_V));

    //
    // set the Clarke parameters
    //
    setupClarke_I(clarkeHandle_I, userParams.numCurrentSensors);
    setupClarke_V(clarkeHandle_V, userParams.numVoltageSensors);

    //
    // initialize the estimator
    //
    estHandle = EST_initEst(estNumber);

    //
    // set the default estimator parameters
    //
    EST_setParams(estHandle, &userParams);
    EST_setFlag_enableForceAngle(estHandle, motorVars.flagEnableForceAngle);
    EST_setFlag_enableRsRecalc(estHandle, motorVars.flagEnableRsRecalc);

    //
    // initialize the inverse Park module
    //
    iparkHandle = IPARK_init(&ipark, sizeof(ipark));

    //
    // initialize the Park module
    //
    parkHandle = PARK_init(&park, sizeof(park));

    //
    // initialize the PI controllers
    //
    piHandle_Id  = PI_init(&pi_Id, sizeof(pi_Id));
    piHandle_Iq  = PI_init(&pi_Iq, sizeof(pi_Iq));
    piHandle_fwc = PI_init(&pi_fwc, sizeof(pi_fwc));
    piHandle_spd = PI_init(&pi_spd, sizeof(pi_spd));

    //
    // setup the controllers, speed, d/q-axis current pid regulator
    //
    setupControllers();

    //
    // initialize the space vector generator module
    //
    svgenHandle = SVGEN_init(&svgen, sizeof(svgen));

    //
    // Initialize and setup the 100% SVM generator
    //
    svgencurrentHandle = SVGENCURRENT_init(&svgencurrent, sizeof(svgencurrent));

    //
    // setup svgencurrent for over modulation function
    //
    {
        float32_t minWidth_usec = 2.0;  // set minimum pwm width to 2.0us

        uint16_t minWidth_counts =
                (uint16_t)(minWidth_usec * USER_SYSTEM_FREQ_MHz);

        //
        // maximum output voltage duty: 0.5
        // convert the minimum pwm width to duty by divide Tpwm
        // Tpwm = 1/(USER_PWM_FREQ_kHz * 1000) = * (USER_PWM_FREQ_kHz * 0.001)
        // The minimum pwm width is the same in two zero vectors for svpwm
        float32_t dutyLimit =
                0.5 - (2.0 * minWidth_usec * USER_PWM_FREQ_kHz * 0.001);

        SVGENCURRENT_setMinWidth(svgencurrentHandle, minWidth_counts);
        SVGENCURRENT_setIgnoreShunt(svgencurrentHandle, SVGENCURRENT_USE_ALL);
        SVGENCURRENT_setMode(svgencurrentHandle, SVGENCURRENT_ALL_PHASE_MEASURABLE);
        SVGENCURRENT_setVlimit(svgencurrentHandle, dutyLimit);
    }

    //
    // initialize the speed reference trajectory
    //
    trajHandle_spd = TRAJ_init(&traj_spd, sizeof(traj_spd));

    //
    // configure the speed reference trajectory (Hz)
    //
    TRAJ_setTargetValue(trajHandle_spd, 0.0);
    TRAJ_setIntValue(trajHandle_spd, 0.0);
    TRAJ_setMinValue(trajHandle_spd, -USER_MOTOR_FREQ_MAX_HZ);
    TRAJ_setMaxValue(trajHandle_spd, USER_MOTOR_FREQ_MAX_HZ);
    TRAJ_setMaxDelta(trajHandle_spd, (USER_MAX_ACCEL_Hzps / USER_ISR_FREQ_Hz));

    //
    // initialize the Id reference trajectory
    //
    trajHandle_Id = TRAJ_init(&traj_Id, sizeof(traj_Id));

    //
    // configure the Id reference trajectory
    //
    TRAJ_setTargetValue(trajHandle_Id, 0.0);
    TRAJ_setIntValue(trajHandle_Id, 0.0);
    TRAJ_setMinValue(trajHandle_Id, -USER_MOTOR_MAX_CURRENT_A);
    TRAJ_setMaxValue(trajHandle_Id,  USER_MOTOR_MAX_CURRENT_A);
    TRAJ_setMaxDelta(trajHandle_Id,
                     (USER_MOTOR_RES_EST_CURRENT_A / USER_ISR_FREQ_Hz));

    //
    // initialize the fwc reference trajectory
    //
    trajHandle_fwc = TRAJ_init(&traj_fwc, sizeof(traj_fwc));

    //
    // configure the fwc reference trajectory
    //
    TRAJ_setTargetValue(trajHandle_fwc, 0.0);
    TRAJ_setIntValue(trajHandle_fwc, 0.0);
    TRAJ_setMinValue(trajHandle_fwc, -USER_MOTOR_MAX_CURRENT_A);
    TRAJ_setMaxValue(trajHandle_fwc,  USER_MOTOR_MAX_CURRENT_A);
    TRAJ_setMaxDelta(trajHandle_fwc,
                     (USER_MOTOR_RES_EST_CURRENT_A / USER_ISR_FREQ_Hz));

    //
    // initialize and configure offsets using first-order filter
    //
    {
        //
        // Sets the first-order filter denominator coefficients
        // a1, the filter coefficient value for z^(-1)
        // b0, the filter coefficient value for z^0
        // b1, the filter coefficient value for z^(-1)
        //
        uint16_t cnt = 0;
        float32_t b0 = userParams.offsetPole_rps / userParams.ctrlFreq_Hz;
        float32_t a1 = (b0 - 1.0);
        float32_t b1 = 0.0;

        //
        // For Current offset calibration filter
        //
        for(cnt = 0; cnt < USER_NUM_CURRENT_SENSORS; cnt++)
        {
            filterHandle_I[cnt] = FILTER_FO_init(&filter_I[cnt],
                                                   sizeof(filter_I[cnt]));

            FILTER_FO_setDenCoeffs(filterHandle_I[cnt], a1);
            FILTER_FO_setNumCoeffs(filterHandle_I[cnt], b0, b1);

            FILTER_FO_setInitialConditions(filterHandle_I[cnt],
                                        motorVars.offsets_I_A.value[cnt],
                                        motorVars.offsets_I_A.value[cnt]);
        }

        //
        // For Voltage offset calibration filter
        //
        for(cnt = 0; cnt < USER_NUM_VOLTAGE_SENSORS; cnt++)
        {
            filterHandle_V[cnt] = FILTER_FO_init(&filter_V[cnt],
                                                   sizeof(filter_V[cnt]));

            FILTER_FO_setDenCoeffs(filterHandle_V[cnt], a1);
            FILTER_FO_setNumCoeffs(filterHandle_V[cnt], b0, b1);

            FILTER_FO_setInitialConditions(filterHandle_V[cnt],
                                        motorVars.offsets_V_V.value[cnt],
                                        motorVars.offsets_V_V.value[cnt]);
        }

        motorVars.flagEnableOffsetCalc = true;
        offsetCalcCount = 0;
    }

    motorVars.offset_invVbus_invV = 0.5;   // the scale factor of half of dc bus

    motorVars.faultMask.all = FAULT_MASK_ALL_FLTS;

    //
    // setup faults
    //
    //HAL_setupFaults(halHandle); // NOT SETTING FAULTS DUE TO CMPSS tripping incorrectly at 1.5A

    //
    // setup OVM PWM
    //
    HAL_setOvmParams(halHandle, &pwmData);

    //
    // turn on the DRV8320 if present
    //
    HAL_enableDRV(halHandle);

    //
    // initialize the DRV8320 interface
    //
    HAL_setupDRVSPI(halHandle, &drvSPI8320Vars);

    drvSPI8320Vars.Ctrl_Reg_05.VDS_LVL = DRV8320_VDS_LEVEL_1P300_V;
    drvSPI8320Vars.Ctrl_Reg_05.DEAD_TIME = DRV8320_DEADTIME_100_NS;
    drvSPI8320Vars.writeCmd = 1;

    //
    // Set some global variables
    //
    motorVars.pwmISRCount = 0;          // Clear the counter
    motorVars.speedRef_Hz = 0.0;      // Set reference frequency to 0 Hz
    motorVars.flagEnableForceAngle = 0; // Disable ForceAngle
    motorVars.accelerationMax_Hzps = 500; // Set initial accel to 500 (Max is 1000)

    //
    // disable the PWM
    //
    HAL_disablePWM(halHandle);

    //
    // initialize the interrupt vector table
    //
    HAL_initIntVectorTable(halHandle);

    //
    // enable the ADC interrupts
    //
    HAL_enableADCInts(halHandle);

    //
    // disable global interrupts
    //
    HAL_enableGlobalInts(halHandle);

    //
    // enable debug interrupts
    //
    HAL_enableDebugInt(halHandle);

    //
    // Waiting for enable system flag to be set
    //
    while(motorVars.flagEnableSys == false)
    {

    }

    //
    // loop while the enable system flag is true
    //
    while(motorVars.flagEnableSys == true)
    {
        //
        // 1ms time base
        //
        if(HAL_getTimerStatus(halHandle, HAL_CPU_TIMER1))
        {
            motorVars.timerCnt_1ms++;

            HAL_clearTimerFlag(halHandle, HAL_CPU_TIMER1);
        }

        motorVars.mainLoopCount++;

        //
        // set the reference value for internal DACA and DACB
        //
        HAL_setDACValue(halHandle, 0, motorVars.dacaVal);
        HAL_setDACValue(halHandle, 1, motorVars.dacbVal);

        //
        // set internal DAC value for on-chip comparator for current protection
        //
        {
            uint16_t  cmpssCnt;

            for(cmpssCnt = 0; cmpssCnt < HAL_NUM_CMPSS_CURRENT; cmpssCnt++)
            {
                HAL_setCMPSSDACValueHigh(halHandle,
                                         cmpssCnt, motorVars.dacValH);

                HAL_setCMPSSDACValueLow(halHandle,
                                        cmpssCnt, motorVars.dacValL);
            }
        }

        //
        // enable or disable force angle
        //
        EST_setFlag_enableForceAngle(estHandle,
                                     motorVars.flagEnableForceAngle);


        if(HAL_getPwmEnableStatus(halHandle) == true)
        {
            if(HAL_getTripFaults(halHandle) != 0)
            {
                motorVars.faultNow.bit.moduleOverCurrent = 1;
            }
        }

        motorVars.faultUse.all =
                motorVars.faultNow.all & motorVars.faultMask.all;

        //
        // Had some faults to stop the motor
        //
        if(motorVars.faultUse.all != 0)
        {
            motorVars.flagRunIdentAndOnLine = false;
            motorVars.flagEnableRunAndIdentify = false;
        }

        if((motorVars.flagRunIdentAndOnLine == true) &&
           (motorVars.flagEnableOffsetCalc == false))
        {
            if(HAL_getPwmEnableStatus(halHandle) == false)
            {
                //
                // enable the estimator
                //
                EST_enable(estHandle);

                //
                // enable the trajectory generator
                //
                EST_enableTraj(estHandle);

                //
                // enable the PWM
                //
                HAL_enablePWM(halHandle);
            }

            //
            // set the reference to the trajectory of speed
            //
            TRAJ_setTargetValue(trajHandle_spd, motorVars.speedRef_Hz);

            //
            // set the acceleration to the trajectory of speed
            //
            TRAJ_setMaxDelta(trajHandle_spd,
                           (motorVars.accelerationMax_Hzps / USER_ISR_FREQ_Hz));
        }
        else if(motorVars.flagEnableOffsetCalc == false)
        {
            //
            // disable the estimator
            //
            EST_disable(estHandle);

            //
            // disable the trajectory generator
            //
            EST_disableTraj(estHandle);

            //
            // disable the PWM
            //
            HAL_disablePWM(halHandle);

            //
            // clear integral outputs of the controllers
            //
            PI_setUi(piHandle_Id, 0.0);
            PI_setUi(piHandle_Iq, 0.0);
            PI_setUi(piHandle_fwc, 0.0);
            PI_setUi(piHandle_spd, 0.0);

            //
            // clear current references
            //
            Idq_ref_A.value[0] = 0.0;
            Idq_ref_A.value[1] = 0.0;

            //
            // clear current offsets
            //
            Idq_offset_A.value[0] = 0.0;
            Idq_offset_A.value[1] = 0.0;

            //
            // get the magnetic current for ACIM
            //
            motorVars.IdRated_A = EST_getIdRated_A(estHandle);

            //
            // clear current and angle for FWC and MTPA
            //
            motorVars.VsRef_pu = USER_MAX_VS_MAG_PU;
            motorVars.IsRef_A = 0.0;
            motorVars.angleCurrent_rad = 0.0;

            //
            // clear the trajectory of speed
            //
            TRAJ_setTargetValue(trajHandle_spd, 0.0);
            TRAJ_setIntValue(trajHandle_spd, 0.0);
        }

        //
        // check the trajectory generator
        //
        if(EST_isTrajError(estHandle) == true)
        {
            //
            // disable the PWM
            //
            HAL_disablePWM(halHandle);

            //
            // set the enable system flag to false
            //
            motorVars.flagEnableSys = false;
        }
        else
        {
            //
            // update the trajectory generator state
            //
            EST_updateTrajState(estHandle);
        }

        //
        // check the estimator
        //
        if(EST_isError(estHandle) == true)
        {
            //
            // disable the PWM
            //
            HAL_disablePWM(halHandle);

            //
            // set the enable system flag to false
            //
            motorVars.flagEnableSys = false;
        }
        else        // No estimator error
        {
            motorVars.Id_target_A = EST_getIntValue_Id_A(estHandle);

            flagEstStateChanged = EST_updateState(estHandle, 0.0);

            if(flagEstStateChanged == true)
            {
                //
                // configure the trajectory generator
                //
                EST_configureTraj(estHandle);
            }
        }

        if(EST_isMotorIdentified(estHandle) == true)
        {
            if(motorVars.flagSetupController == true)
            {
                //
                // update the controller
                // set custom current and speed controllers gains
                //
                updateControllers();
            }
            else
            {
                motorVars.flagMotorIdentified = true;
                motorVars.flagSetupController = true;

                setupControllers();
            }
        }
        //
        // update the global variables
        //
        updateGlobalVariables(estHandle);

        //
        // DRV8320 Read/Write
        //
        HAL_writeDRVData(halHandle, &drvSPI8320Vars);

        HAL_readDRVData(halHandle, &drvSPI8320Vars);

    } // end of while() loop

    //
    // disable the PWM
    //
    HAL_disablePWM(halHandle);

} // end of main() function

__interrupt void mainISR(void)
{

    motorVars.pwmISRCount++;

    //
    // toggle status LED
    //
    counterLED++;

    if(counterLED > (uint32_t)(USER_ISR_FREQ_Hz / LED_BLINK_FREQ_Hz))
    {
        HAL_toggleLED(halHandle, HAL_GPIO_TESTLED);
        counterLED = 0;
    }


    //
    // Send Telemetry over CAN
    //
    counterCAN++;
    counterRPM++;


    if(counterCAN > (uint32_t)(USER_ISR_FREQ_Hz / CAN_TELEM_FREQ_Hz))
    {
        if (boardState[0] == 0x01 || boardState[0] == 0x02) {

            // Send measuredRPM
            sendRPM();

            // Send measuredVoltage
            sendVoltage();

            // Send measuredTorque
            sendTorque();

            // Send faultStatus
            sendFault();
        }

        // Send boardState
        sendState();

        // Reset CAN Telemetry Counter
        counterCAN = 0;
    }

    // Check board state
    if (boardState[0] == 0x02 && motorVars.flagRunIdentAndOnLine == 0) {
        boardState[0] = 0x01; // If there's a fault, and motor becomes disabled, change back to state 0x01, motor off.
    }

    //
    // Read Incoming CAN Commands if Available
    //

    // Arm Command
    if (CAN_readMessage(CANB_BASE, arm_id, rxMsgData)) {
        if (boardState[0] == 0x00 && rxMsgData[0] == 0x01) { // Only do something in IDLE state
            boardState[0] = 0x01; // Change to ARM state
            motorVars.flagEnableSys = 1; // Enable control system
        }
    }

    // Abort Command
    if (CAN_readMessage(CANB_BASE, abort_id, rxMsgData)) {
        if (rxMsgData[0] == 0x01) {
            motorVars.flagEnableSys = 0; // Disable control system
            motorVars.flagRunIdentAndOnLine = 0;
            boardState[0] = 0x03; // Change to ABORT state
        }
    }

    // Motor On/Off Command
    if (CAN_readMessage(CANB_BASE, motor_onoff_id, rxMsgData)) {
        if (boardState[0] == 0x01 && rxMsgData[0] == 0x01) {
            boardState[0] = 0x02; // Change to MOTOR ENABLED state
            motorVars.faultNow.bit.moduleOverCurrent = 0; // Clear any false overcurrent fault on motor enable.
            motorVars.speedRef_Hz = 0.0; // Set initial speed to 0.
            motorVars.flagRunIdentAndOnLine = 1; // Enable motor
        }

        else if (boardState[0] == 0x02 && rxMsgData[0] == 0x00) {
            boardState[0] = 0x01; // Change to ARM state
            motorVars.flagRunIdentAndOnLine = 0; // Disable motor
        }
    }

    // Set RPM Command
    if (CAN_readMessage(CANB_BASE, setRPM_id, rxMsgData) && boardState[0] == 0x02) { // Check if board is in MotorEnabled State
        if (rxMsgData[0] == 0x00) { // Clockwise
            motorVars.speedRef_Hz = (float32_t)((rxMsgData[1] << 8) | rxMsgData[2]) * USER_MOTOR_NUM_POLE_PAIRS / 60.0;

        }
        else if (rxMsgData[0] == 0x01){ // CounterClockwise (negative)
            motorVars.speedRef_Hz = (float32_t)((rxMsgData[1] << 8) | rxMsgData[2]) * -1.0 * USER_MOTOR_NUM_POLE_PAIRS / 60.0;
        }
        if (motorVars.speedRef_Hz > 400.0) {motorVars.speedRef_Hz = 400.0;} // Clamp at +3500 RPM
        if (motorVars.speedRef_Hz < -400.0) {motorVars.speedRef_Hz = -400.0;} // Clamp at -3500 RPM

        RPMset = true; // An RPM command has been recieved
    }

    // Set Acceleration Command
    if (CAN_readMessage(CANB_BASE, setAccel_id, rxMsgData)) {
        motorVars.accelerationMax_Hzps = (float32_t)(rxMsgData[0] << 8 | rxMsgData[1]);
        if (motorVars.accelerationMax_Hzps > 1500.0) {motorVars.accelerationMax_Hzps = 1500.0;}
    }

    // Check for no RPM signal
    if (!RPMset && counterRPM > (uint32_t)(USER_ISR_FREQ_Hz / RPM_RESET_FREQ_Hz)) {
        motorVars.speedRef_Hz = 0.0;
        counterRPM = 0;
    }

    if (RPMset && counterRPM > (uint32_t)(USER_ISR_FREQ_Hz / RPM_RESET_FREQ_Hz)) {
        RPMset = false; // Reset RPM detected flag
        counterRPM = 0;
    }

    //
    // acknowledge the ADC interrupt
    //
    HAL_ackADCInt(halHandle, ADC_INT_NUMBER2);

    //
    // read the ADC data with offsets
    //
    HAL_readADCDataWithOffsets(halHandle, &adcData);

    //
    // calculate Vbus scale factor to scale offsets with Vbus
    //
    motorVars.Vbus_sf = adcData.dcBus_V * motorVars.offset_invVbus_invV;

    //
    // remove offsets
    //
    adcData.I_A.value[0] -= motorVars.offsets_I_A.value[0];
    adcData.I_A.value[1] -= motorVars.offsets_I_A.value[1];
    adcData.I_A.value[2] -= motorVars.offsets_I_A.value[2];
    adcData.V_V.value[0] -= motorVars.offsets_V_V.value[0] * motorVars.Vbus_sf;
    adcData.V_V.value[1] -= motorVars.offsets_V_V.value[1] * motorVars.Vbus_sf;
    adcData.V_V.value[2] -= motorVars.offsets_V_V.value[2] * motorVars.Vbus_sf;

    //
    // Over Modulation Supporting
    //
    {
        measurableShuntThisCycle = SVGENCURRENT_getMode(svgencurrentHandle);

        //
        // run the current reconstruction algorithm
        //
        SVGENCURRENT_RunRegenCurrent(svgencurrentHandle,
                                     &adcData.I_A, &adcDataPrev);
    }

    if(motorVars.flagEnableOffsetCalc == false)
    {
        //
        // Verify close speed loop sensorless by FAST Estimator
        // Dual current close loop
        //
        float32_t outMax_V;
        MATH_Vec2 phasor;

        //
        // run Clarke transform on current
        //
        CLARKE_run(clarkeHandle_I, &adcData.I_A, &(estInputData.Iab_A));

        //
        // run Clarke transform on voltage
        //
        CLARKE_run(clarkeHandle_V, &adcData.V_V, &(estInputData.Vab_V));

        counterTrajSpeed++;

        if(counterTrajSpeed >= userParams.numIsrTicksPerTrajTick)
        {
            //
            // clear counter
            //
            counterTrajSpeed = 0;

            //
            // run a trajectory for speed reference,
            // so the reference changes with a ramp instead of a step
            //
            TRAJ_run(trajHandle_spd);

            motorVars.speedTraj_Hz = TRAJ_getIntValue(trajHandle_spd);
        }

        //
        // store the input data into a buffer
        //
        estInputData.dcBus_V = adcData.dcBus_V;
        estInputData.speed_ref_Hz = motorVars.speedTraj_Hz;
        estInputData.speed_int_Hz = motorVars.speedTraj_Hz;

        //
        // run the estimator
        //
        EST_run(estHandle, &estInputData, &estOutputData);

        //
        // get Idq, reutilizing a Park transform used inside the estimator.
        // This is optional, user's Park works as well
        //
        EST_getIdq_A(estHandle, (MATH_Vec2 *)(&(Idq_in_A)));

        //
        // run the speed controller
        //
        counterSpeed++;

        if(counterSpeed >= userParams.numCtrlTicksPerSpeedTick)
        {
            counterSpeed = 0;

            PI_run_series(piHandle_spd,
                          estInputData.speed_ref_Hz,
                          estOutputData.fm_lp_rps * MATH_ONE_OVER_TWO_PI,
                          0.0,
                          (float32_t *)(&(Idq_ref_A.value[1])));
        }

        //
        // get the reference Id
        //
        Idq_ref_A.value[0] = EST_getIdRated_A(estHandle);

        //
        // update Id reference for Rs OnLine
        //
        EST_updateId_ref_A(estHandle,
                           (float32_t *)&(Idq_ref_A.value[0]));

        //
        // Maximum voltage output
        //
        userParams.maxVsMag_V = userParams.maxVsMag_pu * adcData.dcBus_V;
        PI_setMinMax(piHandle_Id,
                     (-userParams.maxVsMag_V), userParams.maxVsMag_V);

        //
        // run the Id controller
        //
        PI_run_series(piHandle_Id,
                      Idq_ref_A.value[0] + Idq_offset_A.value[0],
                      Idq_in_A.value[0],
                      0.0,
                      &(Vdq_out_V.value[0]));

        //
        // calculate Iq controller limits, and run Iq controller using fast RTS
        // function, callable assembly
        //
        outMax_V = sqrt((userParams.maxVsMag_V * userParams.maxVsMag_V) -
                        (Vdq_out_V.value[0] * Vdq_out_V.value[0]));

        PI_setMinMax(piHandle_Iq, -outMax_V, outMax_V);
        PI_run_series(piHandle_Iq,
                      Idq_ref_A.value[1] + Idq_offset_A.value[1],
                      Idq_in_A.value[1],
                      0.0,
                      &(Vdq_out_V.value[1]));

        //
        // store the Idq reference values used by the controller
        //
        EST_setIdq_ref_A(estHandle, &Idq_ref_A);

        //
        // compute angle with delay compensation
        //
        angleDelta_rad = userParams.angleDelayed_sf_sec *
                         estOutputData.fm_lp_rps;

        angleEst_rad = MATH_incrAngle(estOutputData.angle_rad,
                                            angleDelta_rad);

        angleFoc_rad = angleEst_rad;

        //
        // compute the sin/cos phasor using fast RTS function, callable assembly
        //
        phasor.value[0] = cos(angleFoc_rad);
        phasor.value[1] = sin(angleFoc_rad);


        //
        // set the phasor in the inverse Park transform
        //
        IPARK_setPhasor(iparkHandle, &phasor);

        //
        // run the inverse Park module
        //
        IPARK_run(iparkHandle, &Vdq_out_V, &Vab_out_V);

        //
        // setup the space vector generator (SVGEN) module
        //
        SVGEN_setup(svgenHandle, estOutputData.oneOverDcBus_invV);

        //
        // run the space vector generator (SVGEN) module
        //
        SVGEN_run(svgenHandle, &Vab_out_V, &(pwmData.Vabc_pu));
    }
    else if(motorVars.flagEnableOffsetCalc == true)
    {
        runOffsetsCalculation();
    }

    if(HAL_getPwmEnableStatus(halHandle) == false)
    {
        //
        // clear PWM data
        //
        pwmData.Vabc_pu.value[0] = 0.0;
        pwmData.Vabc_pu.value[1] = 0.0;
        pwmData.Vabc_pu.value[2] = 0.0;
    }
    else
    {
        //
        // run the PWM compensation and current ignore algorithm
        //
        SVGENCURRENT_compPWMData(svgencurrentHandle,
                                 &(pwmData.Vabc_pu), &pwmDataPrev);
    }

    //
    // write the PWM compare values
    //
    HAL_writePWMData(halHandle, &pwmData);

    {
        ignoreShuntNextCycle = SVGENCURRENT_getIgnoreShunt(svgencurrentHandle);
        midVolShunt = SVGENCURRENT_getVmid(svgencurrentHandle);

        //
        // Set trigger point in the middle of the low side pulse
        //
        HAL_setTrigger(halHandle, &pwmData, ignoreShuntNextCycle, midVolShunt);
    }

    motorVars.estISRCount++;

    return;
} // end of mainISR() function

void runOffsetsCalculation(void)
{
    uint16_t cnt;
    float32_t Vin;
    float32_t invVdcbus;

    if(motorVars.flagEnableSys == true)
    {
        //
        // enable the PWM
        //
        HAL_enablePWM(halHandle);

        //
        // set to the half of inverse dc bus voltage
        //
        invVdcbus = 2.0 / adcData.dcBus_V;

        //
        // set the 3-phase output PWMs to 50% duty cycle
        //
        pwmData.Vabc_pu.value[0] = 0.0;
        pwmData.Vabc_pu.value[1] = 0.0;
        pwmData.Vabc_pu.value[2] = 0.0;

        for(cnt = 0; cnt < USER_NUM_CURRENT_SENSORS; cnt++)
        {
            //
            // reset current offsets used
            //
            motorVars.offsets_I_A.value[cnt] = 0.0;

            //
            // run current offset estimation
            //
            FILTER_FO_run(filterHandle_I[cnt],
                          adcData.I_A.value[cnt]);
        }

        for(cnt = 0; cnt < USER_NUM_VOLTAGE_SENSORS; cnt++)
        {
            //
            // reset voltage offsets used
            //
            motorVars.offsets_V_V.value[cnt] = 0.0;

            Vin = adcData.V_V.value[cnt] * invVdcbus;

            //
            // run voltage offset estimation
            //
            FILTER_FO_run(filterHandle_V[cnt], Vin);
        }

        offsetCalcCount++;

        if(offsetCalcCount >= offsetCalcWaitTime)
        {
            for(cnt = 0; cnt < USER_NUM_CURRENT_SENSORS; cnt++)
            {
                //
                // get calculated current offsets from filter
                //
                motorVars.offsets_I_A.value[cnt] =
                        FILTER_FO_get_y1(filterHandle_I[cnt]);

                //
                // clear current filters
                //
                FILTER_FO_setInitialConditions(filterHandle_I[cnt],
                                        motorVars.offsets_I_A.value[cnt],
                                        motorVars.offsets_I_A.value[cnt]);
            }

            for(cnt = 0; cnt < USER_NUM_VOLTAGE_SENSORS; cnt++)
            {
                //
                // get calculated voltage offsets from filter
                //
                motorVars.offsets_V_V.value[cnt] =
                        FILTER_FO_get_y1(filterHandle_V[cnt]);

                //
                // clear voltage filters
                //
                FILTER_FO_setInitialConditions(filterHandle_V[cnt],
                                        motorVars.offsets_V_V.value[cnt],
                                        motorVars.offsets_V_V.value[cnt]);
            }

            offsetCalcCount = 0;
            motorVars.flagEnableOffsetCalc = false;

            //
            // disable the PWM
            //
            HAL_disablePWM(halHandle);
        }
    }

    return;
} // end of runOffsetsCalculation() function


// CAN FUNCTIONS *********************************

void sendRPM(void) {

    if (motorVars.speed_krpm < 0) { // CounterClockwise
        rpmMsgData[0] = 0x01; // Direction Byte
        rpmMsgData[1] = (uint16_t)(motorVars.speed_krpm*-1000) >> 8; //High Byte
        rpmMsgData[2] = (uint16_t)(motorVars.speed_krpm*-1000) & 0x00FF; //Low Byte
    }
    else { // Clockwise
        rpmMsgData[0] = 0x00; // Direction Byte
        rpmMsgData[1] = (uint16_t)(motorVars.speed_krpm*1000) >> 8; //High Byte
        rpmMsgData[2] = (uint16_t)(motorVars.speed_krpm*1000) & 0x00FF; //Low Byte
    }

    CAN_sendMessage(CANB_BASE, measuredRPM_id, 3, rpmMsgData); //Send message

    return;
}

void sendVoltage(void) {
    uint16_t volt_int = (uint16_t)(motorVars.VdcBus_V);
    float32_t volt_decimal = motorVars.VdcBus_V - (float32_t)volt_int;
    uint16_t volt_sign = 0x0000;
    uint16_t volt_binary_float = 0x0000;


    if (motorVars.VdcBus_V < 0.0) {
        volt_sign = 0x8000; //1000 0000 0000 0000;
    }

    int i;
    for (i=0; i<10; i++) {
        volt_decimal *= 2.0;
        if (volt_decimal > 1.0) {
            volt_binary_float |= 0x0001; // Add in 1
            volt_decimal -= 1.0;
        }
        else {
            volt_binary_float |= 0x0000; // Add in 0
        }
        volt_binary_float = volt_binary_float << 1; // Shift decimal value
    }

    volt_binary_float = volt_sign | (volt_int << 10) | volt_binary_float; // Sign + 5 bit integer + 10 bit decimal = 16 bit custom float

    voltageData[0] = volt_binary_float >> 8; // High Byte
    voltageData[1] = volt_binary_float & 0x00FF; // Low Byte

    CAN_sendMessage(CANB_BASE, measuredVoltage_id, 2, voltageData); // Send message

    return;
}

void sendTorque(void) {
    uint16_t torque_int = (uint16_t)(motorVars.torque_Nm);
    float32_t torque_decimal = motorVars.torque_Nm - (float32_t)torque_int;
    uint16_t torque_sign = 0x0000;
    uint16_t torque_binary_float = 0x0000;


    if (motorVars.torque_Nm < 0.0) {
        torque_sign = 0x8000; //1000 0000 0000 0000;
    }

    int i;
    for (i=0; i<10; i++) {
        torque_decimal *= 2.0;
        if (torque_decimal > 1.0) {
            torque_binary_float |= 0x0001; // Add in 1
            torque_decimal -= 1;
        }
        else {
            torque_binary_float |= 0x0000; // Add in 0
        }
        torque_binary_float = torque_binary_float << 1; // Shift decimal value
    }

    torque_binary_float = torque_sign | (torque_int << 10) | torque_binary_float; // Sign + 5 bit integer + 10 bit decimal = 16 bit custom float

    torqueData[0] = torque_binary_float >> 8; // High Byte
    torqueData[1] = torque_binary_float & 0x00FF; // Low Byte

    CAN_sendMessage(CANB_BASE, measuredTorque_id, 2, torqueData); // Send message

    return;
}

void sendState(void) {

    CAN_sendMessage(CANB_BASE, boardState_id, 1, boardState); //Send message

    return;
}

void sendFault(void) {

    uint16_t faultbits =  (motorVars.faultNow.bit.overVoltage)
                        | (motorVars.faultNow.bit.underVoltage << 1)
                        | (motorVars.faultNow.bit.motorOverTemp << 2)
                        | (motorVars.faultNow.bit.moduleOverTemp << 3)
                        | (motorVars.faultNow.bit.moduleOverCurrent << 4)
                        | (motorVars.faultNow.bit.overRmsCurrent << 5)
                        | (motorVars.faultNow.bit.overPeakCurrent << 6)
                        | (motorVars.faultNow.bit.multiOverCurrent << 7)
                        | (motorVars.faultNow.bit.motorLostPhase << 8)
                        | (motorVars.faultNow.bit.currentUnbalance << 9)
                        | (motorVars.faultNow.bit.motorStall << 10)
                        | (motorVars.faultNow.bit.overSpeed << 11)
                        | (motorVars.faultNow.bit.startupFailed << 12)
                        | (motorVars.faultNow.bit.overLoad << 13)
                        | (motorVars.faultNow.bit.controllerError << 14)
                        | (motorVars.faultNow.bit.reserve15 << 15);

    faults[0] = faultbits >> 8; //High Byte
    faults[1] = faultbits & 0x00FF; //Low Byte

    CAN_sendMessage(CANB_BASE, faultStatus_id, 2, faults); //Send message
    return;
}




//************************************************



//
// end of file
//
