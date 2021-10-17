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

#ifndef DRV8320_H
#define DRV8320_H

//! \file   solutions/boostxl_drv8320rs/f28004x/drivers/include/drv8320.h
//! \brief  Contains public interface to various functions related
//!         to the DRV8320 object
//!

// **************************************************************************
// the includes
#include <math.h>

// drivers
#include "spi.h"
#include "gpio.h"

// **************************************************************************
// modules

// **************************************************************************
// solutions

//!
//! \defgroup drv8320

//!
//! \ingroup drv8320
//@{

#ifdef __cplusplus
extern "C" {
#endif

// **************************************************************************
// the defines

//! \brief Defines the address mask
//!
#define DRV8320_ADDR_MASK                   (0x7800)

//! \brief Defines the data mask
//!
#define DRV8320_DATA_MASK                   (0x07FF)

//! \brief Defines the R/W mask
//!
#define DRV8320_RW_MASK                     (0x8000)

//
// STATUS00
//
//! \brief Defines the R/W mask
//!
#define DRV8320_FAULT_TYPE_MASK             (0x07FF)

#define DRV8320_STATUS00_VDS_LC_BITS        (1 << 0)
#define DRV8320_STATUS00_VDS_HC_BITS        (1 << 1)
#define DRV8320_STATUS00_VDS_LB_BITS        (1 << 2)
#define DRV8320_STATUS00_VDS_HB_BITS        (1 << 3)
#define DRV8320_STATUS00_VDS_LA_BITS        (1 << 4)
#define DRV8320_STATUS00_VDS_HA_BITS        (1 << 5)

//! \brief Defines the location of the OTSD (Over temperature shutdown) bits
//! in the Status 1 register
#define DRV8320_STATUS00_OTSD_BITS          (1 << 6)
#define DRV8320_STATUS00_UVLO_BITS          (1 << 7)
#define DRV8320_STATUS00_GDF_BITS           (1 << 8)
#define DRV8320_STATUS00_VDS_OCP_BITS       (1 << 9)
#define DRV8320_STATUS00_FAULT_BITS         (1 << 10)

//
// STATUS01
//
//! \brief Defines the location of the VGS_LC bits in the Status 2 register
//!
#define DRV8320_STATUS01_VGS_LC_BITS        (1 << 0)

//! \brief Defines the location of the VGS_HC bits in the Status 2 register
//!
#define DRV8320_STATUS01_VGS_HC_BITS        (1 << 1)

//! \brief Defines the location of the VGS_LB bits in the Status 2 register
//!
#define DRV8320_STATUS01_VGS_LB_BITS        (1 << 2)

//! \brief Defines the location of the VGS_HB bits in the Status 2 register
//!
#define DRV8320_STATUS01_VGS_HB_BITS        (1 << 3)

//! \brief Defines the location of the VGS_LA bits in the Status 2 register
//!
#define DRV8320_STATUS01_VGS_LA_BITS        (1 << 4)

//! \brief Defines the location of the VGS_HA bits in the Status 2 register
//!
#define DRV8320_STATUS01_VGS_HA_BITS        (1 << 5)

//! \brief Defines the location of the CPUV (charge pump undervoltage) bits in
//! the Status 2 register
#define DRV8320_STATUS01_CPUV_BITS          (1 << 6)

//! \brief Defines the location of the OTW bits in the Status 2 register
//!
#define DRV8320_STATUS01_OTW_BITS           (1 << 7)

//! \brief Defines the location of the SC_OC bits in the Status 2 register
//!
#define DRV8320_STATUS01_SC_OC_BITS         (1 << 8)

//! \brief Defines the location of the SB_OC bits in the Status 2 register
//!
#define DRV8320_STATUS01_SB_OC_BITS         (1 << 9)

//! \brief Defines the location of the SA_OC bits in the Status 2 register
//!
#define DRV8320_STATUS01_SA_OC_BITS         (1 << 10)

//
// CTRL02
//
//! \brief Defines the location of the CLR_FLT bits in the Control 2 register
//!
#define DRV8320_CTRL02_CLR_FLT_BITS         (1 << 0)

//! \brief Defines the location of the BRAKE bits in the Control 2 register
//!
#define DRV8320_CTRL02_BRAKE_BITS           (1 << 1)

//! \brief Defines the location of the COAST bits in the Control 2 register
//!
#define DRV8320_CTRL02_COAST_BITS           (1 << 2)

//! \brief Defines the location of the 1PWM_DIR bits in the Control 2 register
//!
#define DRV8320_CTRL02_PWM1_DIR_BITS        (1 << 3)

//! \brief Defines the location of the 1PWM_COM bits in the Control 2 register
//!
#define DRV8320_CTRL02_PWM1_COM_BITS        (1 << 4)

//! \brief Defines the location of the PWM_MODE bits in the Control 2 register
//!
#define DRV8320_CTRL02_PWM_MODE_BITS        (3 << 5)

//! \brief Defines the location of the OTW_REP bits in the Control 2 register
//!
#define DRV8320_CTRL02_OTW_REP_BITS         (1 << 7)

//! \brief Defines the location of the DIS_GDF bits in the Control 2 register
//!
#define DRV8320_CTRL02_DIS_GDF_BITS         (1 << 8)

//! \brief Defines the location of the DIS_CPUV bits in the Control 2 register
//!
#define DRV8320_CTRL02_DIS_CPUV_BITS        (1 << 9)

//! \brief Defines the location of the RESERVED1 bits in the Control 2 register
//!
#define DRV8320_CTRL02_RESERVED1_BITS       (1 << 10)

//
// CTRL03
//
//! \brief Defines the location of the IDRIVEN_HS bits in the Control 3
//! register
#define DRV8320_CTRL03_IDRIVEN_HS_BITS      (15 << 0)

//! \brief Defines the location of the IDRIVEP_HS bits in the Control 3
//! register
#define DRV8320_CTRL03_IDRIVEP_HS_BITS      (15 << 4)

//! \brief Defines the location of the LOCK bits in the Control 3 register
//!
#define DRV8320_CTRL03_LOCK_BITS            (7 << 8)

//
// CTRL04
//
//! \brief Defines the location of the IDRIVEN_LS bits in the Control 4
//! register
#define DRV8320_CTRL04_IDRIVEN_LS_BITS      (15 << 0)

//! \brief Defines the location of the IDRIVEP_LS bits in the Control 4
//! register
#define DRV8320_CTRL04_IDRIVEP_LS_BITS      (15 << 4)

//! \brief Defines the location of the TDRIVE bits in the Control 4 register
//!
#define DRV8320_CTRL04_TDRIVE_BITS          (3 << 8)

//! \brief Defines the location of the CBC bits in the Control 4 register
//!
#define DRV8320_CTRL04_CBC_BITS             (1 << 10)

//
// CTRL05
//
//! \brief Defines the location of the VDS_LVL bits in the Control 5 register
//!
#define DRV8320_CTRL05_VDS_LVL_BITS         (15 << 0)

//! \brief Defines the location of the OCP_DEG bits in the Control 5 register
//!
#define DRV8320_CTRL05_OCP_DEG_BITS         (3 << 4)

//! \brief Defines the location of the OCP_MODE bits in the Control 5 register
//!
#define DRV8320_CTRL05_OCP_MODE_BITS        (3 << 6)

//! \brief Defines the location of the DEAD_TIME bits in the Control 5 register
//!
#define DRV8320_CTRL05_DEAD_TIME_BITS       (3 << 8)

//! \brief Defines the location of the TRETRY bits in the Control 5 register
//!
#define DRV8320_CTRL05_TRETRY_BITS          (1 << 10)

//
// CTRL06
//
//! \brief Defines the location of the SEN_LVL bits in the Control 6 register
//!
#define DRV8320_CTRL06_SEN_LVL_BITS         (3 << 0)

//! \brief Defines the location of the CSA_CAL_C bits in the Control 6 register
//!
#define DRV8320_CTRL06_CSA_CAL_C_BITS       (1 << 2)

//! \brief Defines the location of the CSA_CAL_B bits in the Control 6 register
//!
#define DRV8320_CTRL06_CSA_CAL_B_BITS       (1 << 3)

//! \brief Defines the location of the CSA_CAL_A bits in the Control 6 register
//!
#define DRV8320_CTRL06_CSA_CAL_A_BITS       (1 << 4)

//! \brief Defines the location of the DIS_SEN bits in the Control 6 register
//!
#define DRV8320_CTRL06_DIS_SEN_BITS         (1 << 5)

//! \brief Defines the location of the CSA_GAIN bits in the Control 6 register
//!
#define DRV8320_CTRL06_CSA_GAIN_BITS        (3 << 6)

//! \brief Defines the location of the LS_REF bits in the Control 6 register
//!
#define DRV8320_CTRL06_LS_REF_BITS          (1 << 8)

//! \brief Defines the location of the VREF_DIV bits in the Control 6 register
//!
#define DRV8320_CTRL06_VREF_DIV_BITS        (1 << 9)

//! \brief Defines the location of the CSA_FET bits in the Control 6 register
//!
#define DRV8320_CTRL06_CSA_FET_BITS         (1 << 10)

// **************************************************************************
// the typedefs

//! \brief Enumeration for the R/W modes
//!
typedef enum
{
    DRV8320_CTRLMODE_READ     = (1 << 15), //!< Read Mode
    DRV8320_CTRLMODE_WRITE    = (0 << 15)  //!< Write Mode
} DRV8320_CtrlMode_e;

//! \brief Enumeration for the Status 0 register, faults
//!
typedef enum
{
    DRV8320_VDS_LC      = (1 << 0),    //!< VDS overcurrent fault on C low-side MOSFET
    DRV8320_VDS_HC      = (1 << 1),    //!< VDS overcurrent fault on C high-side MOSFET
    DRV8320_VDS_LB      = (1 << 2),    //!< VDS overcurrent fault on B low-side MOSFET
    DRV8320_VDS_HB      = (1 << 3),    //!< VDS overcurrent fault on B high-side MOSFET
    DRV8320_VDS_LA      = (1 << 4),    //!< VDS overcurrent fault on A low-side MOSFET
    DRV8320_VDS_HA      = (1 << 5),    //!< VDS overcurrent fault on A high-side MOSFET
    DRV8320_OTSD        = (1 << 6),    //!< Overtemperature shutdown
    DRV8320_UVLO        = (1 << 7),    //!< Undervoltage lockout fault condition
    DRV8320_GDF         = (1 << 8),    //!< Gate driver fault condition
    DRV8320_VDS_OCP     = (1 << 9),    //!< VDS monitor overcurrent fault condition
    DRV8320_FAULT       = (1 << 10)    //!< FAULT type, 0-Warning, 1-Latched
} DRV8320_STATUS00_WarningWatchdog_e;

//! \brief Enumeration for the Status 1 register, OV/VDS faults
//!
typedef enum
{
    DRV8320_VGS_LC      = (1 << 0),    //!< VGS gate drive fault on C low-side MOSFET
    DRV8320_VGS_HC      = (1 << 1),    //!< VGS gate drive fault on C high-side MOSFET
    DRV8320_VGS_LB      = (1 << 2),    //!< VGS gate drive fault on B low-side MOSFET
    DRV8320_VGS_HB      = (1 << 3),    //!< VGS gate drive fault on B high-side MOSFET
    DRV8320_VGS_LA      = (1 << 4),    //!< VGS gate drive fault on A low-side MOSFET
    DRV8320_VGS_HA      = (1 << 5),    //!< VGS gate drive fault on A high-side MOSFET
    DRV8320_CPUV        = (1 << 6),    //!< charge pump undervoltage fault
    DRV8320_OTW         = (1 << 7),    //!< overtemperature warning
    DRV8320_SC_OC       = (1 << 8),    //!< overcurrent on phase C
    DRV8320_SB_OC       = (1 << 9),    //!< overcurrent on phase B
    DRV8320_SA_OC       = (1 << 10)    //!< overcurrent on phase A
} DRV8320_STATUS01_OvVdsFaults_e;

//! \brief Enumeration for the driver PWM mode
//!
typedef enum
{
    DRV8320_PWMMODE_6 = (0 << 5),     //!< PWM_MODE = 6 inputs
    DRV8320_PWMMODE_3 = (1 << 5),     //!< PWM_MODE = 3 inputs
    DRV8320_PWMMODE_1 = (2 << 5)      //!< PWM_MODE = 1 input
} DRV8320_CTRL02_PWMMode_e;

//! \brief Enumeration for the high side gate drive peak source current;
//! gate currents not consistent with DS
//!
typedef enum
{
    DRV8320_ISOUR_HS_0P010_A = (0 << 4),  //!< IDRIVEP_HS = 0.010A
    DRV8320_ISOUR_HS_0P030_A = (1 << 4),  //!< IDRIVEP_HS = 0.030A
    DRV8320_ISOUR_HS_0P060_A = (2 << 4),  //!< IDRIVEP_HS = 0.060A
    DRV8320_ISOUR_HS_0P080_A = (3 << 4),  //!< IDRIVEP_HS = 0.080A
    DRV8320_ISOUR_HS_0P120_A = (4 << 4),  //!< IDRIVEP_HS = 0.120A
    DRV8320_ISOUR_HS_0P140_A = (5 << 4),  //!< IDRIVEP_HS = 0.140A
    DRV8320_ISOUR_HS_0P170_A = (6 << 4),  //!< IDRIVEP_HS = 0.170A
    DRV8320_ISOUR_HS_0P190_A = (7 << 4),  //!< IDRIVEP_HS = 0.190A
    DRV8320_ISOUR_HS_0P260_A = (8 << 4),  //!< IDRIVEP_HS = 0.260A
    DRV8320_ISOUR_HS_0P330_A = (9 << 4),  //!< IDRIVEP_HS = 0.330A
    DRV8320_ISOUR_HS_0P370_A = (10 << 4), //!< IDRIVEP_HS = 0.370A
    DRV8320_ISOUR_HS_0P440_A = (11 << 4), //!< IDRIVEP_HS = 0.440A
    DRV8320_ISOUR_HS_0P570_A = (12 << 4), //!< IDRIVEP_HS = 0.570A
    DRV8320_ISOUR_HS_0P680_A = (13 << 4), //!< IDRIVEP_HS = 0.680A
    DRV8320_ISOUR_HS_0P820_A = (14 << 4), //!< IDRIVEP_HS = 0.820A
    DRV8320_ISOUR_HS_1P000_A = (15 << 4)  //!< IDRIVEP_HS = 1.000A
} DRV8320_CTRL03_PeakSourCurHS_e;

//! \brief Enumeration for the high side gate drive peak sink current;
//! gate currents not consistent with DS
//!
typedef enum
{
    DRV8320_ISINK_HS_0P020_A = (0 << 0),  //!< IDRIVEN_HS = 0.020A
    DRV8320_ISINK_HS_0P060_A = (1 << 0),  //!< IDRIVEN_HS = 0.060A
    DRV8320_ISINK_HS_0P120_A = (2 << 0),  //!< IDRIVEN_HS = 0.120A
    DRV8320_ISINK_HS_0P160_A = (3 << 0),  //!< IDRIVEN_HS = 0.160A
    DRV8320_ISINK_HS_0P240_A = (4 << 0),  //!< IDRIVEN_HS = 0.240A
    DRV8320_ISINK_HS_0P280_A = (5 << 0),  //!< IDRIVEN_HS = 0.280A
    DRV8320_ISINK_HS_0P340_A = (6 << 0),  //!< IDRIVEN_HS = 0.340A
    DRV8320_ISINK_HS_0P380_A = (7 << 0),  //!< IDRIVEN_HS = 0.380A
    DRV8320_ISINK_HS_0P520_A = (8 << 0),  //!< IDRIVEN_HS = 0.520A
    DRV8320_ISINK_HS_0P660_A = (9 << 0),  //!< IDRIVEN_HS = 0.660A
    DRV8320_ISINK_HS_0P740_A = (10 << 0), //!< IDRIVEN_HS = 0.740A
    DRV8320_ISINK_HS_0P880_A = (11 << 0), //!< IDRIVEN_HS = 0.880A
    DRV8320_ISINK_HS_1P140_A = (12 << 0), //!< IDRIVEN_HS = 1.140A
    DRV8320_ISINK_HS_1P360_A = (13 << 0), //!< IDRIVEN_HS = 1.360A
    DRV8320_ISINK_HS_1P640_A = (14 << 0), //!< IDRIVEN_HS = 1.640A
    DRV8320_ISINK_HS_2P000_A = (15 << 0)  //!< IDRIVEN_HS = 2.000A
} DRV8320_CTRL03_PeakSinkCurHS_e;

//! \brief Enumeration for the high side and low side gate drive peak source
//! time; adapt timings to DRV8320
//!
typedef enum
{
    DRV8320_LOCK_LOCK     = (6 << 8),     //!< Lock settings
    DRV8320_LOCK_UNLOCK   = (3 << 8)      //!< Unlock settings
} DRV8320_CTRL03_Lock_e;

//! \brief Enumeration for the high side and low side gate drive peak source
//! time; adapt timings to DRV8320
//!
typedef enum
{
    DRV8320_TSOUR_500_NS  = (0 << 8),     //!< TDRIVE = 500ns
    DRV8320_TSOUR_1000_NS = (1 << 8),     //!< TDRIVE = 1000ns
    DRV8320_TSOUR_2000_NS = (2 << 8),     //!< TDRIVE = 2000ns
    DRV8320_TSOUR_4000_NS = (3 << 8)      //!< TDRIVE = 4000ns
} DRV8320_CTRL04_PeakTime_e;

//! \brief Enumeration for the low side gate drive peak source current;
//!  adapt current ratings
//!
typedef enum
{
    DRV8320_ISOUR_LS_0P010_A = (0 << 4),  //!< IDRIVEP_LS = 0.010A
    DRV8320_ISOUR_LS_0P030_A = (1 << 4),  //!< IDRIVEP_LS = 0.030A
    DRV8320_ISOUR_LS_0P060_A = (2 << 4),  //!< IDRIVEP_LS = 0.060A
    DRV8320_ISOUR_LS_0P080_A = (3 << 4),  //!< IDRIVEP_LS = 0.080A
    DRV8320_ISOUR_LS_0P120_A = (4 << 4),  //!< IDRIVEP_LS = 0.120A
    DRV8320_ISOUR_LS_0P140_A = (5 << 4),  //!< IDRIVEP_LS = 0.140A
    DRV8320_ISOUR_LS_0P170_A = (6 << 4),  //!< IDRIVEP_LS = 0.170A
    DRV8320_ISOUR_LS_0P190_A = (7 << 4),  //!< IDRIVEP_LS = 0.190A
    DRV8320_ISOUR_LS_0P260_A = (8 << 4),  //!< IDRIVEP_LS = 0.260A
    DRV8320_ISOUR_LS_0P330_A = (9 << 4),  //!< IDRIVEP_LS = 0.330A
    DRV8320_ISOUR_LS_0P370_A = (10 << 4), //!< IDRIVEP_LS = 0.370A
    DRV8320_ISOUR_LS_0P440_A = (11 << 4), //!< IDRIVEP_LS = 0.440A
    DRV8320_ISOUR_LS_0P570_A = (12 << 4), //!< IDRIVEP_LS = 0.570A
    DRV8320_ISOUR_LS_0P680_A = (13 << 4), //!< IDRIVEP_LS = 0.680A
    DRV8320_ISOUR_LS_0P820_A = (14 << 4), //!< IDRIVEP_LS = 0.820A
    DRV8320_ISOUR_LS_1P000_A = (15 << 4)  //!< IDRIVEP_LS = 1.000A
} DRV8320_CTRL04_PeakSourCurLS_e;

//! \brief Enumeration for the low side gate drive peak sink current;
//!  adapt current ratings
//!
typedef enum
{
    DRV8320_ISINK_LS_0P020_A = (0 << 0),  //!< IDRIVEN_LS = 0.020A
    DRV8320_ISINK_LS_0P060_A = (1 << 0),  //!< IDRIVEN_LS = 0.060A
    DRV8320_ISINK_LS_0P120_A = (2 << 0),  //!< IDRIVEN_LS = 0.120A
    DRV8320_ISINK_LS_0P160_A = (3 << 0),  //!< IDRIVEN_LS = 0.160A
    DRV8320_ISINK_LS_0P240_A = (4 << 0),  //!< IDRIVEN_LS = 0.240A
    DRV8320_ISINK_LS_0P280_A = (5 << 0),  //!< IDRIVEN_LS = 0.280A
    DRV8320_ISINK_LS_0P340_A = (6 << 0),  //!< IDRIVEN_LS = 0.340A
    DRV8320_ISINK_LS_0P380_A = (7 << 0),  //!< IDRIVEN_LS = 0.380A
    DRV8320_ISINK_LS_0P520_A = (8 << 0),  //!< IDRIVEN_LS = 0.520A
    DRV8320_ISINK_LS_0P660_A = (9 << 0),  //!< IDRIVEN_LS = 0.660A
    DRV8320_ISINK_LS_0P740_A = (10 << 0), //!< IDRIVEN_LS = 0.740A
    DRV8320_ISINK_LS_0P880_A = (11 << 0), //!< IDRIVEN_LS = 0.880A
    DRV8320_ISINK_LS_1P140_A = (12 << 0), //!< IDRIVEN_LS = 1.140A
    DRV8320_ISINK_LS_1P360_A = (13 << 0), //!< IDRIVEN_LS = 1.360A
    DRV8320_ISINK_LS_1P640_A = (14 << 0), //!< IDRIVEN_LS = 1.640A
    DRV8320_ISINK_LS_2P000_A = (15 << 0)  //!< IDRIVEN_LS = 2.000A
} DRV8320_CTRL04_PeakSinkCurLS_e;

//! \brief Enumeration for the VDS comparator threshold
//!
typedef enum
{
    DRV8320_VDS_LEVEL_0P060_V = (0 << 0),    //!< VDS_LEVEL = 0.060V
    DRV8320_VDS_LEVEL_0P130_V = (1 << 0),    //!< VDS_LEVEL = 0.130V
    DRV8320_VDS_LEVEL_0P200_V = (2 << 0),    //!< VDS_LEVEL = 0.200V
    DRV8320_VDS_LEVEL_0P260_V = (3 << 0),    //!< VDS_LEVEL = 0.260V
    DRV8320_VDS_LEVEL_0P310_V = (4 << 0),    //!< VDS_LEVEL = 0.310V
    DRV8320_VDS_LEVEL_0P450_V = (5 << 0),    //!< VDS_LEVEL = 0.450V
    DRV8320_VDS_LEVEL_0P530_V = (6 << 0),    //!< VDS_LEVEL = 0.530V
    DRV8320_VDS_LEVEL_0P600_V = (7 << 0),    //!< VDS_LEVEL = 0.600V
    DRV8320_VDS_LEVEL_0P680_V = (8 << 0),    //!< VDS_LEVEL = 0.680V
    DRV8320_VDS_LEVEL_0P750_V = (9 << 0),    //!< VDS_LEVEL = 0.750V
    DRV8320_VDS_LEVEL_0P940_V = (10 << 0),   //!< VDS_LEVEL = 0.940V
    DRV8320_VDS_LEVEL_1P130_V = (11 << 0),   //!< VDS_LEVEL = 1.130V
    DRV8320_VDS_LEVEL_1P300_V = (12 << 0),   //!< VDS_LEVEL = 1.300V
    DRV8320_VDS_LEVEL_1P500_V = (13 << 0),   //!< VDS_LEVEL = 1.500V
    DRV8320_VDS_LEVEL_1P700_V = (14 << 0),   //!< VDS_LEVEL = 1.700V
    DRV8320_VDS_LEVEL_1P880_V = (15 << 0)    //!< VDS_LEVEL = 1.880V
} DRV8320_CTRL05_VDSLVL_e;

//! \brief Enumeration for the OCP/VDS sense deglitch time;
//!  adapt deglitch time comments
//!
typedef enum
{
    DRV8320_VDSDEG_2_US = (0 << 4),       //!< OCP_DEG = 2us
    DRV8320_VDSDEG_4_US = (1 << 4),       //!< OCP_DEG = 4us
    DRV8320_VDSDEG_6_US = (2 << 4),       //!< OCP_DEG = 6us
    DRV8320_VDSDEG_8_US = (3 << 4)        //!< OCP_DEG = 8us
} DRV8320_CTRL05_OcpDeg_e;

//! \brief Enumeration for the OCP report mode
//!
typedef enum
{
    DRV8320_LATCHED_SHUTDOWN = (0 << 6),  //!< OCP_MODE = Latched fault
    DRV8320_AUTOMATIC_RETRY  = (1 << 6),  //!< OCP_MODE = Automatic Retry
    DRV8320_REPORT_ONLY      = (2 << 6),  //!< OCP_MODE = Report only
    DRV8320_DISABLE_OCP      = (3 << 6)   //!< OCP_MODE = Disabled
} DRV8320_CTRL05_OcpMode_e;

//! \brief Enumeration for the driver dead time
//!
typedef enum
{
    DRV8320_DEADTIME_50_NS  = (0 << 8),   //!< DEAD_TIME = 50ns
    DRV8320_DEADTIME_100_NS = (1 << 8),   //!< DEAD_TIME = 100ns
    DRV8320_DEADTIME_200_NS = (2 << 8),   //!< DEAD_TIME = 200ns
    DRV8320_DEADTIME_400_NS = (3 << 8)    //!< DEAD_TIME = 400ns
} DRV8320_CTRL05_DeadTime_e;

//! \brief Enumeration for the register addresses
//!
typedef enum
{
    DRV8320_ADDRESS_STATUS_0  = (0 << 11),  //!< Status Register 0
    DRV8320_ADDRESS_STATUS_1  = (1 << 11),  //!< Status Register 1
    DRV8320_ADDRESS_CONTROL_2 = (2 << 11),  //!< Control Register 2
    DRV8320_ADDRESS_CONTROL_3 = (3 << 11),  //!< Control Register 3
    DRV8320_ADDRESS_CONTROL_4 = (4 << 11),  //!< Control Register 4
    DRV8320_ADDRESS_CONTROL_5 = (5 << 11)   //!< Control Register 5
} DRV8320_Address_e;

//! \brief Object for the DRV8320 STATUS00 register
//!
typedef struct _DRV8320_Stat00_t_
{
    bool                  VDS_LC;         // Bits 0
    bool                  VDS_HC;         // Bits 1
    bool                  VDS_LB;         // Bits 2
    bool                  VDS_HB;         // Bits 3
    bool                  VDS_LA;         // Bits 4
    bool                  VDS_HA;         // Bits 5
    bool                  OTSD;           // Bits 6
    bool                  UVLO;           // Bits 7
    bool                  GDF;            // Bits 8
    bool                  VDS_OCP;        // Bits 9
    bool                  FAULT;          // Bits 10
}DRV8320_Stat00_t;

//! \brief Object for the DRV8320 STATUS01 register
//!
typedef struct _DRV8320_Stat01_t_
{
    bool                  VGS_LC;         // Bits 0
    bool                  VGS_HC;         // Bits 1
    bool                  VGS_LB;         // Bits 2
    bool                  VGS_HB;         // Bits 3
    bool                  VGS_LA;         // Bits 4
    bool                  VGS_HA;         // Bits 5
    bool                  CPUV;           // Bits 6
    bool                  OTW;            // Bits 7
    bool                  SC_OC;          // Bits 8
    bool                  SB_OC;          // Bits 9
    bool                  SA_OC;          // Bits 10
}DRV8320_Stat01_t;

//! \brief Object for the DRV8320 CTRL02 register
//!
typedef struct _DRV8320_Ctrl02_t_
{
    bool                          CLR_FLT;        // Bits 0
    bool                          BRAKE;          // Bits 1
    bool                          COAST;          // Bits 2
    bool                          PWM1_DIR;       // Bits 3
    bool                          PWM1_COM;       // Bits 4
    DRV8320_CTRL02_PWMMode_e      PWM_MODE;       // Bits 6-5
    bool                          OTW_REP;        // Bits 7
    bool                          DIS_GDF;        // Bits 8
    bool                          DIS_CPUV;       // Bits 9
    bool                          CTRL02_RSV1;    // Bits 10
}DRV8320_Ctrl02_t;

//! \brief Object for the DRV8320 CTRL03 register
//!
typedef struct _DRV8320_Ctrl03_t_
{
    DRV8320_CTRL03_PeakSinkCurHS_e    IDRIVEN_HS;     // Bits 3-0
    DRV8320_CTRL03_PeakSourCurHS_e    IDRIVEP_HS;     // Bits 7-4
    DRV8320_CTRL03_Lock_e             LOCK;           // Bits 10-8
}DRV8320_Ctrl03_t;

//! \brief Object for the DRV8320 CTRL04 register
//!
typedef struct _DRV8320_Ctrl04_t_
{
    DRV8320_CTRL04_PeakSinkCurLS_e    IDRIVEN_LS;     // Bits 3-0
    DRV8320_CTRL04_PeakSourCurLS_e    IDRIVEP_LS;     // Bits 7-4
    DRV8320_CTRL04_PeakTime_e         TDRIVE;         // Bits 9-8
    bool                              CBC;            // Bits 10
}DRV8320_Ctrl04_t;

//! \brief Object for the DRV8320 CTRL05 register
//!
typedef struct _DRV8320_Ctrl05_t_
{
    DRV8320_CTRL05_VDSLVL_e           VDS_LVL;        // Bits 3-0
    DRV8320_CTRL05_OcpDeg_e           OCP_DEG;        // Bits 5-4
    DRV8320_CTRL05_OcpMode_e          OCP_MODE;       // Bits 7-5
    DRV8320_CTRL05_DeadTime_e         DEAD_TIME;      // Bits 9-8
    bool                              TRETRY;         // Bits 10
}DRV8320_Ctrl05_t;

//! \brief Object for the DRV8320 registers and commands
//!
typedef struct _DRV8320_SPIVars_t_
{
    DRV8320_Stat00_t          Stat_Reg_00;
    DRV8320_Stat01_t          Stat_Reg_01;

    DRV8320_Ctrl02_t          Ctrl_Reg_02;
    DRV8320_Ctrl03_t          Ctrl_Reg_03;
    DRV8320_Ctrl04_t          Ctrl_Reg_04;
    DRV8320_Ctrl05_t          Ctrl_Reg_05;
    bool                      writeCmd;
    bool                      readCmd;

    uint16_t                  manWriteAddr;
    uint16_t                  manReadAddr;
    uint16_t                  manWriteData;
    uint16_t                  manReadData;
    bool                      manWriteCmd;
    bool                      manReadCmd;
}DRV8320_SPIVars_t;

//! \brief Defines the DRV8320 object
//!
typedef struct _DRV8320_Obj_
{
    uint32_t  spiHandle;     //!< handle for the serial peripheral interface
    uint32_t  gpioNumber_CS; //!< GPIO connected to the DRV8320 CS pin
    uint32_t  gpioNumber_EN; //!< GPIO connected to the DRV8320 enable pin
    bool      rxTimeOut;     //!< timeout flag for the RX FIFO
    bool      enableTimeOut; //!< timeout flag for DRV8320 enable
} DRV8320_Obj;

//! \brief Defines the DRV8320 handle
//!
typedef struct _DRV8320_Obj_ *DRV8320_Handle;

//! \brief Defines the DRV8320 Word type
//!
typedef  uint16_t    DRV8320_Word_t;

// **************************************************************************
// the globals

// **************************************************************************
// the function prototypes

//! \brief     Initializes the DRV8320 object
//! \param[in] pMemory   A pointer to the memory for the DRV8320 object
//! \param[in] numBytes  The number of bytes allocated for the DRV8320
//!                      object, bytes
//! \return    The DRV8320 object handle
extern DRV8320_Handle DRV8320_init(void *pMemory);

//! \brief     Builds the control word
//! \param[in] ctrlMode  The control mode
//! \param[in] regName   The register name
//! \param[in] data      The data
//! \return    The control word
static inline DRV8320_Word_t DRV8320_buildCtrlWord(
                                            const DRV8320_CtrlMode_e ctrlMode,
                                            const DRV8320_Address_e regAddr,
                                            const uint16_t data)
{
    DRV8320_Word_t ctrlWord = ctrlMode | regAddr | (data & DRV8320_DATA_MASK);

    return(ctrlWord);
} // end of DRV8320_buildCtrlWord() function

//! \brief     Enables the DRV8320
//! \param[in] handle     The DRV8320 handle
extern void DRV8320_enable(DRV8320_Handle handle);

//! \brief     Sets the SPI handle in the DRV8320
//! \param[in] handle     The DRV8320 handle
//! \param[in] spiHandle  The SPI handle to use
void DRV8320_setSPIHandle(DRV8320_Handle handle,uint32_t spiHandle);

//! \brief     Sets the GPIO number in the DRV8320
//! \param[in] handle       The DRV8320 handle
//! \param[in] gpioHandle   The GPIO number to use
void DRV8320_setGPIOCSNumber(DRV8320_Handle handle,uint32_t gpioNumber);

//! \brief     Sets the GPIO number in the DRV8320
//! \param[in] handle       The DRV8320 handle
//! \param[in] gpioHandle   The GPIO number to use
void DRV8320_setGPIONumber(DRV8320_Handle handle,uint32_t gpioNumber);

//! \brief     Resets the enable timeout flag
//! \param[in] handle   The DRV8320 handle
static inline void DRV8320_resetEnableTimeout(DRV8320_Handle handle)
{
    DRV8320_Obj *obj = (DRV8320_Obj *)handle;

    obj->enableTimeOut = false;

    return;
} // end of DRV8320_resetEnableTimeout() function

//! \brief     Resets the RX fifo timeout flag
//! \param[in] handle   The DRV8320 handle
static inline void DRV8320_resetRxTimeout(DRV8320_Handle handle)
{
    DRV8320_Obj *obj = (DRV8320_Obj *)handle;

    obj->rxTimeOut = false;

    return;
} // end of DRV8320_resetRxTimeout() function

//! \brief     Initialize the interface to all 8320 SPI variables
//! \param[in] handle  The DRV8320 handle
extern void DRV8320_setupSPI(DRV8320_Handle handle,
                             DRV8320_SPIVars_t *drv8320SPIVars);

//! \brief     Reads data from the DRV8320 register
//! \param[in] handle   The DRV8320 handle
//! \param[in] regAddr  The register address
//! \return    The data value
extern uint16_t DRV8320_readSPI(DRV8320_Handle handle,
                                const DRV8320_Address_e regAddr);

//! \brief     Writes data to the DRV8320 register
//! \param[in] handle   The DRV8320 handle
//! \param[in] regAddr  The register name
//! \param[in] data     The data value
extern void DRV8320_writeSPI(DRV8320_Handle handle,
                             const DRV8320_Address_e regAddr,
                             const uint16_t data);

//! \brief     Write to the DRV8320 SPI registers
//! \param[in] handle  The DRV8320 handle
//! \param[in] drv8320SPIVars  The (DRV8320_SPIVars_t) structure that contains
//!                           all DRV8320 Status/Control register options
extern void DRV8320_writeData(DRV8320_Handle handle,
                              DRV8320_SPIVars_t *drv8320SPIVars);

//! \brief     Read from the DRV8320 SPI registers
//! \param[in] handle  The DRV8320 handle
//! \param[in] drv8320SPIVars  The (DRV8320_SPIVars_t) structure that contains
//!                           all DRV8320 Status/Control register options
extern void DRV8320_readData(DRV8320_Handle handle,
                             DRV8320_SPIVars_t *drv8320SPIVars);

#ifdef __cplusplus
}
#endif // extern "C"

//@}  // ingroup

#endif // end of DRV8320_H definition
