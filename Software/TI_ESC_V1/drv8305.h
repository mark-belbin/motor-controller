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
#ifndef _DRV8305_H_
#define _DRV8305_H_

//! \file   drivers/drvic/drv8305/src/32b/f28x/f2802x/drv8305.h
//! \brief  Contains public interface to various functions related
//!         to the DRV8305 object
//!
//! (C) Copyright 2015, Texas Instruments, Inc.


// **************************************************************************
// the includes

// drivers
#include "sw/drivers/spi/src/32b/f28x/f2802x/spi.h"
#include "sw/drivers/gpio/src/32b/f28x/f2802x/gpio.h"


// **************************************************************************
// modules


// **************************************************************************
// solutions


//!
//! \defgroup DRV8305

//!
//! \ingroup DRV8305
//@{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines

//! \brief Defines the address mask
//!
#define DRV8305_ADDR_MASK                   (0x7800)


//! \brief Defines the data mask
//!
#define DRV8305_DATA_MASK                   (0x07FF)


//! \brief Defines the R/W mask
//!
#define DRV8305_RW_MASK                     (0x8000)


//! \brief Defines the R/W mask
//!
#define DRV8305_FAULT_TYPE_MASK             (0x07FF)


//! \brief Defines the location of the OTW (Over temperature warning) bits in the Status 1 register
//!
#define DRV8305_STATUS01_OTW_BITS           (1 << 0)

//! \brief Defines the location of the TEMP_FLAG3 (Over temperature warning 3) bits in the Status 1 register
//!
#define DRV8305_STATUS01_TEMP_FLAG3_BITS    (1 << 1)

//! \brief Defines the location of the TEMP_FLAG2 (Over temperature warning 2) bits in the Status 1 register
//!
#define DRV8305_STATUS01_TEMP_FLAG2_BITS    (1 << 2)

//! \brief Defines the location of the TEMP_FLAG1 (Over temperature warning 1) bits in the Status 1 register
//!
#define DRV8305_STATUS01_TEMP_FLAG1_BITS    (1 << 3)

//! \brief Defines the location of the VCPH_UVFL (Charge pump high, under voltage) bits in the Status 1 register
//!
#define DRV8305_STATUS01_VCPH_UVFL_BITS     (1 << 4)

//! \brief Defines the location of the VDS_STATUS (Real time "OR" of 0x02[D5:D10]) bits in the Status 1 register
//!
#define DRV8305_STATUS01_VDS_STATUS_BITS    (1 << 5)

//! \brief Defines the location of the PVDD_OVFL (PVDD over voltage) bits in the Status 1 register
//!
#define DRV8305_STATUS01_PVDD_OVFL_BITS     (1 << 6)

//! \brief Defines the location of the PVDD_UVFL (PVDD under voltage) bits in the Status 1 register
//!
#define DRV8305_STATUS01_PVDD_UVFL_BITS     (1 << 7)

//! \brief Defines the location of the TEMP_FLAG4 (Over temperature warning 4) bits in the Status 1 register
//!
#define DRV8305_STATUS01_TEMP_FLAG4_BITS    (1 << 8)

//! \brief Defines the location of the RESERVED1 bits in the Status 1 register
//!
#define DRV8305_STATUS01_RESERVED1_BITS     (1 << 9)

//! \brief Defines the location of the FAULT (0-Warning, 1-Latched) bits in the Status 1 register
//!
#define DRV8305_STATUS01_FAULT_BITS         (1 << 10)


//! \brief Defines the location of the SNS_A_OCP (Sense amp A over current) bits in the Status 2 register
//!
#define DRV8305_STATUS02_SNS_A_OCP_BITS     (1 << 0)

//! \brief Defines the location of the SNS_B_OCP (Sense amp B over current) bits in the Status 2 register
//!
#define DRV8305_STATUS02_SNS_B_OCP_BITS     (1 << 1)

//! \brief Defines the location of the SNS_C_OCP (Sense amp C over current) bits in the Status 2 register
//!
#define DRV8305_STATUS02_SNS_C_OCP_BITS     (1 << 2)

//! \brief Defines the location of the RESERVED1 bits in the Status 2 register
//!
#define DRV8305_STATUS02_RESERVED1_BITS     (1 << 3)

//! \brief Defines the location of the RESERVED2 bits in the Status 2 register
//!
#define DRV8305_STATUS02_RESERVED2_BITS     (1 << 4)

//! \brief Defines the location of the FETLC_VDS (C low side, VDS Fault) bits in the Status 2 register
//!
#define DRV8305_STATUS02_FETLC_VDS_BITS     (1 << 5)

//! \brief Defines the location of the FETHC_VDS (C high side, VDS Fault) bits in the Status 2 register
//!
#define DRV8305_STATUS02_FETHC_VDS_BITS     (1 << 6)

//! \brief Defines the location of the FETLB_VDS (B low side, VDS Fault) bits in the Status 2 register
//!
#define DRV8305_STATUS02_FETLB_VDS_BITS     (1 << 7)

//! \brief Defines the location of the FETHB_VDS (B high side, VDS Fault) bits in the Status 2 register
//!
#define DRV8305_STATUS02_FETHB_VDS_BITS     (1 << 8)

//! \brief Defines the location of the FETLA_VDS (A low side, VDS Fault) bits in the Status 2 register
//!
#define DRV8305_STATUS02_FETLA_VDS_BITS     (1 << 9)

//! \brief Defines the location of the FETHA_VDS (A high side, VDS Fault) bits in the Status 2 register
//!
#define DRV8305_STATUS02_FETHA_VDS_BITS     (1 << 10)


//! \brief Defines the location of the VCPH_OVLO_ABS (Charge pump high, absolute over voltage) bits in the Status 3 register
//!
#define DRV8305_STATUS03_VCPH_OVLO_ABS_BITS (1 << 0)

//! \brief Defines the location of the VCPH_OVLO (Charge pump high, over voltage) bits in the Status 3 register
//!
#define DRV8305_STATUS03_VCPH_OVLO_BITS     (1 << 1)

//! \brief Defines the location of the VCPH_UVLO2 (Charge pump high, under voltage 2) bits in the Status 3 register
//!
#define DRV8305_STATUS03_VCPH_UVLO2_BITS    (1 << 2)

//! \brief Defines the location of the RESERVED1 bits in the Status 3 register
//!
#define DRV8305_STATUS03_RESERVED1_BITS     (1 << 3)

//! \brief Defines the location of the VCP_LSD_UVLO2 (Charge pump low, under voltage 2) bits in the Status 3 register
//!
#define DRV8305_STATUS03_VCP_LSD_UVLO2_BITS (1 << 4)

//! \brief Defines the location of the AVDD_UVLO (AVDD regulator under voltage) bits in the Status 3 register
//!
#define DRV8305_STATUS03_AVDD_UVLO_BITS     (1 << 5)

//! \brief Defines the location of the VREG_UV (VREG regulator under voltage) bits in the Status 3 register
//!
#define DRV8305_STATUS03_VREG_UV_BITS       (1 << 6)

//! \brief Defines the location of the RESERVED2 bits in the Status 3 register
//!
#define DRV8305_STATUS03_RESERVED2_BITS     (1 << 7)

//! \brief Defines the location of the OTS (Over temperature shut down) bits in the Status 3 register
//!
#define DRV8305_STATUS03_OTS_BITS           (1 << 8)

//! \brief Defines the location of the WD_FAULT (Watchdog fault) bits in the Status 3 register
//!
#define DRV8305_STATUS03_WD_FAULT_BITS      (1 << 9)

//! \brief Defines the location of the PVDD_UVLO2 (PVDD under voltage 2) bits in the Status 3 register
//!
#define DRV8305_STATUS03_PVDD_UVLO2_BITS    (1 << 10)


//! \brief Defines the location of the RESERVED1 bits in the Status 4 register
//!
#define DRV8305_STATUS04_RESERVED1_BITS     (1 << 0)

//! \brief Defines the location of the RESERVED2 bits in the Status 4 register
//!
#define DRV8305_STATUS04_RESERVED2_BITS     (1 << 1)

//! \brief Defines the location of the RESERVED3 bits in the Status 4 register
//!
#define DRV8305_STATUS04_RESERVED3_BITS     (1 << 2)

//! \brief Defines the location of the RESERVED4 bits in the Status 4 register
//!
#define DRV8305_STATUS04_RESERVED4_BITS     (1 << 3)

//! \brief Defines the location of the RESERVED5 bits in the Status 4 register
//!
#define DRV8305_STATUS04_RESERVED5_BITS     (1 << 4)

//! \brief Defines the location of the FETLC_VGS (C low side, VGS Fault) bits in the Status 4 register
//!
#define DRV8305_STATUS04_FETLC_VGS_BITS     (1 << 5)

//! \brief Defines the location of the FETHC_VGS (C high side, VGS Fault) bits in the Status 4 register
//!
#define DRV8305_STATUS04_FETHC_VGS_BITS     (1 << 6)

//! \brief Defines the location of the FETLB_VGS (B low side, VGS Fault) bits in the Status 4 register
//!
#define DRV8305_STATUS04_FETLB_VGS_BITS     (1 << 7)

//! \brief Defines the location of the FETHB_VGS (B high side, VGS Fault) bits in the Status 4 register
//!
#define DRV8305_STATUS04_FETHB_VGS_BITS     (1 << 8)

//! \brief Defines the location of the FETLA_VGS (A low side, VGS Fault) bits in the Status 4 register
//!
#define DRV8305_STATUS04_FETLA_VGS_BITS     (1 << 9)

//! \brief Defines the location of the FETHA_VGS (A high side, VGS Fault) bits in the Status 4 register
//!
#define DRV8305_STATUS04_FETHA_VGS_BITS     (1 << 10)


//! \brief Defines the location of the IDRIVEP_HS bits in the Control 5 register
//!
#define DRV8305_CTRL05_IDRIVEP_HS_BITS      (15 << 0)

//! \brief Defines the location of the IDRIVEN_HS bits in the Control 5 register
//!
#define DRV8305_CTRL05_IDRIVEN_HS_BITS      (15 << 4)

//! \brief Defines the location of the TDRIVEN bits in the Control 5 register
//!
#define DRV8305_CTRL05_TDRIVEN_BITS         (3 << 8)

//! \brief Defines the location of the RESERVED1 bits in the Control 5 register
//!
#define DRV8305_CTRL05_RESERVED1_BITS       (1 << 10)


//! \brief Defines the location of the IDRIVEP_LS bits in the Control 6 register
//!
#define DRV8305_CTRL06_IDRIVEP_LS_BITS      (15 << 0)

//! \brief Defines the location of the IDRIVEN_LS bits in the Control 6 register
//!
#define DRV8305_CTRL06_IDRIVEN_LS_BITS      (15 << 4)

//! \brief Defines the location of the TDRIVEP bits in the Control 6 register
//!
#define DRV8305_CTRL06_TDRIVEP_BITS         (3 << 8)

//! \brief Defines the location of the RESERVED1 bits in the Control 6 register
//!
#define DRV8305_CTRL06_RESERVED1_BITS       (1 << 10)


//! \brief Defines the location of the TVDS bits in the Control 7 register
//!
#define DRV8305_CTRL07_TVDS_BITS            (3 << 0)

//! \brief Defines the location of the TBLANK bits in the Control 7 register
//!
#define DRV8305_CTRL07_TBLANK_BITS          (3 << 2)

//! \brief Defines the location of the DEAD_TIME bits in the Control 7 register
//!
#define DRV8305_CTRL07_DEAD_TIME_BITS       (7 << 4)

//! \brief Defines the location of the PWM_MODE bits in the Control 7 register
//!
#define DRV8305_CTRL07_PWM_MODE_BITS        (3 << 7)

//! \brief Defines the location of the COMM_OPT bits in the Control 7 register
//!
#define DRV8305_CTRL07_COMM_OPT_BITS        (1 << 9)

//! \brief Defines the location of the RESERVED1 bits in the Control 7 register
//!
#define DRV8305_CTRL07_RESERVED1_BITS       (1 << 10)


//! \brief Defines the location of the RESERVED bits in the Control 8 register
//!
#define DRV8305_CTRL08_RESERVED_BITS        (2047 << 0)


//! \brief Defines the location of the SET_VCPH_UV bits in the Control 9 register
//!
#define DRV8305_CTRL09_SET_VCPH_UV_BITS     (1 << 0)

//! \brief Defines the location of the CLR_FLTS bits in the Control 9 register
//!
#define DRV8305_CTRL09_CLR_FLTS_BITS        (1 << 1)

//! \brief Defines the location of the SLEEP bits in the Control 9 register
//!
#define DRV8305_CTRL09_SLEEP_BITS           (1 << 2)

//! \brief Defines the location of the WD_EN bits in the Control 9 register
//!
#define DRV8305_CTRL09_WD_EN_BITS           (1 << 3)

//! \brief Defines the location of the DIS_SNS_OCP bits in the Control 9 register
//!
#define DRV8305_CTRL09_DIS_SNS_OCP_BITS     (1 << 4)

//! \brief Defines the location of the WD_DLY bits in the Control 9 register
//!
#define DRV8305_CTRL09_WD_DLY_BITS          (3 << 5)

//! \brief Defines the location of the EN_SNS_CLAMP bits in the Control 9 register
//!
#define DRV8305_CTRL09_EN_SNS_CLAMP_BITS    (1 << 7)

//! \brief Defines the location of the DIS_GDRV_FAULT bits in the Control 9 register
//!
#define DRV8305_CTRL09_DIS_GDRV_FAULT_BITS  (1 << 8)

//! \brief Defines the location of the DISABLE bits in the Control 9 register
//!
#define DRV8305_CTRL09_DISABLE_BITS         (1 << 9)

//! \brief Defines the location of the FLIP_OTS bits in the Control 9 register
//!
#define DRV8305_CTRL09_FLIP_OTS_BITS        (1 << 10)


//! \brief Defines the location of the GAIN_CS1 bits in the Control A register
//!
#define DRV8305_CTRL0A_GAIN_CS1_BITS        (3 << 0)

//! \brief Defines the location of the GAIN_CS2 bits in the Control A register
//!
#define DRV8305_CTRL0A_GAIN_CS2_BITS        (3 << 2)

//! \brief Defines the location of the GAIN_CS3 bits in the Control A register
//!
#define DRV8305_CTRL0A_GAIN_CS3_BITS        (3 << 4)

//! \brief Defines the location of the CS_BLANK bits in the Control A register
//!
#define DRV8305_CTRL0A_CS_BLANK_BITS        (3 << 6)

//! \brief Defines the location of the DC_CAL_CH1 bits in the Control A register
//!
#define DRV8305_CTRL0A_DC_CAL_CH1_BITS      (1 << 8)

//! \brief Defines the location of the DC_CAL_CH2 bits in the Control A register
//!
#define DRV8305_CTRL0A_DC_CAL_CH2_BITS      (1 << 9)

//! \brief Defines the location of the DC_CAL_CH3 bits in the Control A register
//!
#define DRV8305_CTRL0A_DC_CAL_CH3_BITS      (1 << 10)


//! \brief Defines the location of the VREG_UV_LEVEL bits in the Control B register
//!
#define DRV8305_CTRL0B_VREG_UV_LEVEL_BITS   (3 << 0)

//! \brief Defines the location of the DIS_PWRGD bits in the Control B register
//!
#define DRV8305_CTRL0B_DIS_PWRGD_BITS       (1 << 2)

//! \brief Defines the location of the SLP_DLY bits in the Control B register
//!
#define DRV8305_CTRL0B_SLP_DLY_BITS         (3 << 3)

//! \brief Defines the location of the RESERVED1 bits in the Control B register
//!
#define DRV8305_CTRL0B_RESERVED1_BITS       (1 << 5)

//! \brief Defines the location of the RESERVED2 bits in the Control B register
//!
#define DRV8305_CTRL0B_RESERVED2_BITS       (1 << 6)

//! \brief Defines the location of the RESERVED3 bits in the Control B register
//!
#define DRV8305_CTRL0B_RESERVED3_BITS       (1 << 7)

//! \brief Defines the location of the VREF_SCALING bits in the Control B register
//!
#define DRV8305_CTRL0B_VREF_SCALING_BITS    (3 << 8)

//! \brief Defines the location of the RESERVED4 bits in the Control B register
//!
#define DRV8305_CTRL0B_RESERVED4_BITS       (1 << 10)


//! \brief Defines the location of the VDS_MODE bits in the Control B register
//!
#define DRV8305_CTRL0C_VDS_MODE_BITS        (7 << 0)

//! \brief Defines the location of the VDS_LEVEL bits in the Control B register
//!
#define DRV8305_CTRL0C_VDS_LEVEL_BITS       (31 << 3)

//! \brief Defines the location of the RESERVED1 bits in the Control B register
//!
#define DRV8305_CTRL0C_RESERVED1_BITS       (1 << 8)

//! \brief Defines the location of the RESERVED2 bits in the Control B register
//!
#define DRV8305_CTRL0C_RESERVED2_BITS       (1 << 9)

//! \brief Defines the location of the RESERVED3 bits in the Control B register
//!
#define DRV8305_CTRL0C_RESERVED3_BITS       (1 << 10)


// **************************************************************************
// the typedefs

//! \brief Enumeration for the R/W modes
//!
typedef enum
{
  CtrlMode_Read = 1 << 15,   //!< Read Mode
  CtrlMode_Write = 0 << 15   //!< Write Mode
} DRV8305_CtrlMode_e;


//! \brief Enumeration for the Status 1 register, warning and watchdog reset
//!
typedef enum
{
  OTW         = (1 << 0),    //!< Over temperature warning
  TEMP_FLAG3  = (1 << 1),    //!< Junction temperature flag setting 3
  TEMP_FLAG2  = (1 << 2),    //!< Junction temperature flag setting 2
  TEMP_FLAG1  = (1 << 3),    //!< Junction temperature flag setting 1
  VCPH_UVFL   = (1 << 4),    //!< VCPH under voltage protection warning
  VDS_STATUS  = (1 << 5),    //!< Real time "OR" of 0x02[D5:10]
  PVDD_OVFL   = (1 << 6),    //!< PVDD over voltage protection warning
  PVDD_UVFL   = (1 << 7),    //!< PVDD under voltage protection warning
  TEMP_FLAG4  = (1 << 8),    //!< Junction temperature flag setting 4 (no warning)
  STAT01_RSV1 = (1 << 9),    //!< Reserved
  FAULT       = (1 << 10)    //!< FAULT type, 0-Warning, 1-Latched
} DRV8305_STATUS01_WarningWatchdog_e;


//! \brief Enumeration for the Status 2 register, OV/VDS faults
//!
typedef enum
{
  SNS_A_OCP   = (1 << 0),     //!< Sense amp A over current
  SNS_B_OCP   = (1 << 1),     //!< Sense amp B over current
  SNS_C_OCP   = (1 << 2),     //!< Sense amp C over current
  STAT02_RSV2 = (1 << 3),     //!< Reserved
  STAT02_RSV1 = (1 << 4),     //!< Reserved
  FETLC_VDS   = (1 << 5),     //!< C low side, VDS fault
  FETHC_VDS   = (1 << 6),     //!< C high side, VDS fault
  FETLB_VDS   = (1 << 7),     //!< B low side, VDS fault
  FETHB_VDS   = (1 << 8),     //!< B high side, VDS fault
  FETLA_VDS   = (1 << 9),     //!< A low side, VDS fault
  FETHA_VDS   = (1 << 10)     //!< A high side, VDS fault
} DRV8305_STATUS02_OvVdsFaults_e;


//! \brief Enumeration for the Status 3 register, IC faults
//!
typedef enum
{
  VCPH_OVLO_ABS = (1 << 0), //!< VCPH over voltage protection fault (relative to GND)
  VCPH_OVLO     = (1 << 1), //!< VCPH over voltage protection warning
  VCPH_UVLO2    = (1 << 2), //!< VCPH under voltage protection fault
  STAT03_RSV2   = (1 << 3), //!< Reserved
  VCP_LSD_UVLO2 = (1 << 4), //!< VCP_LSD under voltage protection fault
  AVDD_UVLO     = (1 << 5), //!< AVDD under voltage protection fault
  VREG_UV       = (1 << 6), //!< VREG under voltage protection fault
  STAT03_RSV1   = (1 << 7), //!< Reserved
  OTS           = (1 << 8), //!< Over temperature shut down fault
  WD_FAULT      = (1 << 9), //!< Watchdog fault
  PVDD_UVLO2    = (1 << 10) //!< PVDD under voltage protection fault
} DRV8305_STATUS03_ICFaults_e;


//! \brief Enumeration for the Status 4 register, Vgs faults
//!
typedef enum
{
  STAT04_RSV5 = (1 << 0),     //!< Reserved
  STAT04_RSV4 = (1 << 1),     //!< Reserved
  STAT04_RSV3 = (1 << 2),     //!< Reserved
  STAT04_RSV2 = (1 << 3),     //!< Reserved
  STAT04_RSV1 = (1 << 4),     //!< Reserved
  FETLC_VGS   = (1 << 5),     //!< C low side, VGS fault
  FETHC_VGS   = (1 << 6),     //!< C high side, VGS fault
  FETLB_VGS   = (1 << 7),     //!< B low side, VGS fault
  FETHB_VGS   = (1 << 8),     //!< B high side, VGS fault
  FETLA_VGS   = (1 << 9),     //!< A low side, VGS fault
  FETHA_VGS   = (1 << 10)     //!< A high side, VGS fault
} DRV8305_STATUS04_VgsFaults_e;


//! \brief Enumeration for the high side gate drive peak source current
//!
typedef enum 
{
  ISour_HS_0p010_A = (0 << 0),  //!< IDRIVEP_HS = 0.010A
  ISour_HS_0p020_A = (1 << 0),  //!< IDRIVEP_HS = 0.020A
  ISour_HS_0p030_A = (2 << 0),  //!< IDRIVEP_HS = 0.030A
  ISour_HS_0p040_A = (3 << 0),  //!< IDRIVEP_HS = 0.040A
  ISour_HS_0p050_A = (4 << 0),  //!< IDRIVEP_HS = 0.050A
  ISour_HS_0p060_A = (5 << 0),  //!< IDRIVEP_HS = 0.060A
  ISour_HS_0p070_A = (6 << 0),  //!< IDRIVEP_HS = 0.070A
  ISour_HS_0p125_A = (7 << 0),  //!< IDRIVEP_HS = 0.125A
  ISour_HS_0p250_A = (8 << 0),  //!< IDRIVEP_HS = 0.250A
  ISour_HS_0p500_A = (9 << 0),  //!< IDRIVEP_HS = 0.500A
  ISour_HS_0p750_A = (10 << 0), //!< IDRIVEP_HS = 0.750A
  ISour_HS_1p000_A = (11 << 0)  //!< IDRIVEP_HS = 1.000A
} DRV8305_CTRL05_PeakSourCurHS_e;


//! \brief Enumeration for the high side gate drive peak sink current
//!
typedef enum 
{
  ISink_HS_0p020_A = (0 << 4),  //!< IDRIVEN_HS = 0.020A
  ISink_HS_0p030_A = (1 << 4),  //!< IDRIVEN_HS = 0.030A
  ISink_HS_0p040_A = (2 << 4),  //!< IDRIVEN_HS = 0.040A
  ISink_HS_0p050_A = (3 << 4),  //!< IDRIVEN_HS = 0.050A
  ISink_HS_0p060_A = (4 << 4),  //!< IDRIVEN_HS = 0.060A
  ISink_HS_0p070_A = (5 << 4),  //!< IDRIVEN_HS = 0.070A
  ISink_HS_0p125_A = (6 << 4),  //!< IDRIVEN_HS = 0.125A
  ISink_HS_0p250_A = (7 << 4),  //!< IDRIVEN_HS = 0.250A
  ISink_HS_0p500_A = (8 << 4),  //!< IDRIVEN_HS = 0.500A
  ISink_HS_0p750_A = (9 << 4),  //!< IDRIVEN_HS = 0.750A
  ISink_HS_1p000_A = (10 << 4), //!< IDRIVEN_HS = 1.000A
  ISink_HS_1p250_A = (11 << 4)  //!< IDRIVEN_HS = 1.250A
} DRV8305_CTRL05_PeakSinkCurHS_e;


//! \brief Enumeration for the high side and low side gate drive peak source time
//!
typedef enum 
{
  TSour_250_ns  = (0 << 8),     //!< TDRIVEN = 250ns
  TSour_500_ns  = (1 << 8),     //!< TDRIVEN = 500ns
  TSour_1000_ns = (2 << 8),     //!< TDRIVEN = 1000ns
  TSour_2000_ns = (3 << 8)      //!< TDRIVEN = 2000ns
} DRV8305_CTRL05_PeakSourTime_e;


//! \brief Enumeration for the low side gate drive peak source current
//!
typedef enum 
{
  ISour_LS_0p010_A = (0 << 0),  //!< IDRIVEP_LS = 0.010A
  ISour_LS_0p020_A = (1 << 0),  //!< IDRIVEP_LS = 0.020A
  ISour_LS_0p030_A = (2 << 0),  //!< IDRIVEP_LS = 0.030A
  ISour_LS_0p040_A = (3 << 0),  //!< IDRIVEP_LS = 0.040A
  ISour_LS_0p050_A = (4 << 0),  //!< IDRIVEP_LS = 0.050A
  ISour_LS_0p060_A = (5 << 0),  //!< IDRIVEP_LS = 0.060A
  ISour_LS_0p070_A = (6 << 0),  //!< IDRIVEP_LS = 0.070A
  ISour_LS_0p125_A = (7 << 0),  //!< IDRIVEP_LS = 0.125A
  ISour_LS_0p250_A = (8 << 0),  //!< IDRIVEP_LS = 0.250A
  ISour_LS_0p500_A = (9 << 0),  //!< IDRIVEP_LS = 0.500A
  ISour_LS_0p750_A = (10 << 0), //!< IDRIVEP_LS = 0.750A
  ISour_LS_1p000_A = (11 << 0)  //!< IDRIVEP_LS = 1.000A
} DRV8305_CTRL06_PeakSourCurLS_e;


//! \brief Enumeration for the low side gate drive peak sink current
//!
typedef enum 
{
  ISink_LS_0p020_A = (0 << 4),  //!< IDRIVEN_LS = 0.020A
  ISink_LS_0p030_A = (1 << 4),  //!< IDRIVEN_LS = 0.030A
  ISink_LS_0p040_A = (2 << 4),  //!< IDRIVEN_LS = 0.040A
  ISink_LS_0p050_A = (3 << 4),  //!< IDRIVEN_LS = 0.050A
  ISink_LS_0p060_A = (4 << 4),  //!< IDRIVEN_LS = 0.060A
  ISink_LS_0p070_A = (5 << 4),  //!< IDRIVEN_LS = 0.070A
  ISink_LS_0p125_A = (6 << 4),  //!< IDRIVEN_LS = 0.125A
  ISink_LS_0p250_A = (7 << 4),  //!< IDRIVEN_LS = 0.250A
  ISink_LS_0p500_A = (8 << 4),  //!< IDRIVEN_LS = 0.500A
  ISink_LS_0p750_A = (9 << 4),  //!< IDRIVEN_LS = 0.750A
  ISink_LS_1p000_A = (10 << 4), //!< IDRIVEN_LS = 1.000A
  ISink_LS_1p250_A = (11 << 4)  //!< IDRIVEN_LS = 1.250A
} DRV8305_CTRL06_PeakSinkCurLS_e;


//! \brief Enumeration for the high side and low side gate drive peak sink time
//!
typedef enum 
{
  TSink_250_ns  = (0 << 8),     //!< TDRIVEP = 250ns
  TSink_500_ns  = (1 << 8),     //!< TDRIVEP = 500ns
  TSink_1000_ns = (2 << 8),     //!< TDRIVEP = 1000ns
  TSink_2000_ns = (3 << 8)      //!< TDRIVEP = 2000ns
} DRV8305_CTRL06_PeakSinkTime_e;


//! \brief Enumeration for the VDS sense deglitch time
//!
typedef enum 
{
  VDSDeg_0_us = (0 << 0),       //!< TVDS = 0us
  VDSDeg_2_us = (1 << 0),       //!< TVDS = 2us
  VDSDeg_4_us = (2 << 0),       //!< TVDS = 4us
  VDSDeg_8_us = (3 << 0)        //!< TVDS = 8us
} DRV8305_CTRL07_VDSDeglitch_e;


//! \brief Enumeration for the VDS sense blanking time
//!
typedef enum 
{
  VDSBlnk_0_us = (0 << 2),      //!< TBLANK = 0us
  VDSBlnk_2_us = (1 << 2),      //!< TBLANK = 2us
  VDSBlnk_4_us = (2 << 2),      //!< TBLANK = 4us
  VDSBlnk_8_us = (3 << 2)       //!< TBLANK = 8us
} DRV8305_CTRL07_VDSBlanking_e;


//! \brief Enumeration for the driver dead time
//!
typedef enum 
{
  DeadTime_40_ns = (0 << 4),    //!< DEAD_TIME = 40ns
  DeadTime_60_ns = (1 << 4),    //!< DEAD_TIME = 60ns
  DeadTime_100_ns = (2 << 4),   //!< DEAD_TIME = 100ns
  DeadTime_500_ns = (3 << 4),   //!< DEAD_TIME = 500ns
  DeadTime_1000_ns = (4 << 4),  //!< DEAD_TIME = 1000ns
  DeadTime_2000_ns = (5 << 4),  //!< DEAD_TIME = 2000ns
  DeadTime_4000_ns = (6 << 4),  //!< DEAD_TIME = 4000ns
  DeadTime_6000_ns = (7 << 4)   //!< DEAD_TIME = 6000ns
} DRV8305_CTRL07_DeadTime_e;


//! \brief Enumeration for the driver PWM mode
//!
typedef enum 
{
  PwmMode_6 = (0 << 7),     //!< PWM_MODE = 6 inputs
  PwmMode_3 = (1 << 7),     //!< PWM_MODE = 3 inputs
  PwmMode_1 = (2 << 7)      //!< PWM_MODE = 1 input
} DRV8305_CTRL07_PwmMode_e;


//! \brief Enumeration for the driver commutation mode in 1 input
//!
typedef enum 
{
  Diod = (0 << 9),  //!< COMM_OPT = Diode free wheeling
  Actv = (1 << 9)   //!< COMM_OPT = Active free wheeling
} DRV8305_CTRL07_CommOption_e;


//! \brief Enumeration for the IC's watchdog timer
//!
typedef enum 
{
  WD_Dly_10_ms = (0 << 5),  //!< WD_DLY = 10ms
  WD_Dly_20_ms = (1 << 5),  //!< WD_DLY = 20ms
  WD_Dly_50_ms = (2 << 5),  //!< WD_DLY = 50ms
  WD_Dly_100_ms = (3 << 5)  //!< WD_DLY = 100ms
} DRV8305_CTRL09_WatchDelay_e;


//! \brief Enumeration for the gain of shunt amplifier 1
//!
typedef enum 
{
  Gain1_10VpV = (0 << 0),   //!< GAIN_CS1 = 10V/V
  Gain1_20VpV = (1 << 0),   //!< GAIN_CS1 = 20V/V
  Gain1_40VpV = (2 << 0),   //!< GAIN_CS1 = 40V/V
  Gain1_80VpV = (3 << 0)    //!< GAIN_CS1 = 80V/V
} DRV8305_CTRL0A_CSGain1_e;


//! \brief Enumeration for the gain of shunt amplifier 2
//!
typedef enum 
{
  Gain2_10VpV = (0 << 2),   //!< GAIN_CS2 = 10V/V
  Gain2_20VpV = (1 << 2),   //!< GAIN_CS2 = 20V/V
  Gain2_40VpV = (2 << 2),   //!< GAIN_CS2 = 40V/V
  Gain2_80VpV = (3 << 2)    //!< GAIN_CS2 = 80V/V
} DRV8305_CTRL0A_CSGain2_e;


//! \brief Enumeration for the gain of shunt amplifier 3
//!
typedef enum 
{
  Gain3_10VpV = (0 << 4),   //!< GAIN_CS3 = 10V/V
  Gain3_20VpV = (1 << 4),   //!< GAIN_CS3 = 20V/V
  Gain3_40VpV = (2 << 4),   //!< GAIN_CS3 = 40V/V
  Gain3_80VpV = (3 << 4)    //!< GAIN_CS3 = 80V/V
} DRV8305_CTRL0A_CSGain3_e;


//! \brief Enumeration for the blanking time of the shunt amplifiers
//!
typedef enum 
{
  Blank_0p00_us = (0 << 6),   //!< CS_BLANK = 0.0us
  Blank_0p50_us = (1 << 6),   //!< CS_BLANK = 0.5us
  Blank_2p50_us = (2 << 6),   //!< CS_BLANK = 2.5us
  Blank_10p0_us = (3 << 6)    //!< CS_BLANK = 10.0us
} DRV8305_CTRL0A_CSBlank_e;


//! \brief Enumeration for the DC calibration modes
//!
typedef enum 
{
  DcCalMode_Ch1_Load   = (0 << 8),  //!< Shunt amplifier 1 connected to load via input pins
  DcCalMode_Ch1_NoLoad = (1 << 8),  //!< Shunt amplifier 1 disconnected from load and input pins are shorted
  DcCalMode_Ch2_Load   = (0 << 9),  //!< Shunt amplifier 2 connected to load via input pins
  DcCalMode_Ch2_NoLoad = (1 << 9),  //!< Shunt amplifier 2 disconnected from load and input pins are shorted
  DcCalMode_Ch3_Load   = (0 << 10), //!< Shunt amplifier 3 connected to load via input pins
  DcCalMode_Ch3_NoLoad = (1 << 10)  //!< Shunt amplifier 3 disconnected from load and input pins are shorted
} DRV8305_CTRL0A_DcCalMode_e;


//! \brief Enumeration for the VREG under voltage level
//!
typedef enum 
{
  UvLevel_10 = (0 << 0),    //!< VREG_UV_LEVEL = VSET-10%
  UvLevel_20 = (1 << 0),    //!< VREG_UV_LEVEL = VSET-20%
  UvLevel_30 = (2 << 0)     //!< VREG_UV_LEVEL = VSET-30%
} DRV8305_CTRL0B_VregUvLevel_e;


//! \brief Enumeration for the delay to power down VREG after SLEEP
//!
typedef enum 
{
  SleepDly_0_us  = (0 << 3),    //!< SLP_DLY = 0us
  SleepDly_10_us = (1 << 3),    //!< SLP_DLY = 10us
  SleepDly_50_us = (2 << 3),    //!< SLP_DLY = 50us
  SleepDly_1_ms  = (3 << 3)     //!< SLP_DLY = 1ms
} DRV8305_CTRL0B_SleepDelay_e;


//! \brief Enumeration for the VREF scaling factor
//!
typedef enum 
{
  Scaling_k_1 = (0 << 8),   //!< VREF_SCALING = 1
  Scaling_k_2 = (1 << 8),   //!< VREF_SCALING = 2
  Scaling_k_4 = (2 << 8)    //!< VREF_SCALING = 4
} DRV8305_CTRL0B_VrefScaling_e;


//! \brief Enumeration for the VDS report mode
//!
typedef enum 
{
  Latched_Shutdown = (0 << 0),  //!< VDS_MODE = Latched shut down
  Report_Only  = (1 << 0),      //!< VDS_MODE = Report only
  Disable_VDS = (2 << 0)        //!< VDS_MODE = Disabled
} DRV8305_CTRL0C_VDSMode_e;


//! \brief Enumeration for the VDS comparator threshold
//!
typedef enum 
{
  VDS_Level_0p060_V = (0 << 3),    //!< VDS_LEVEL = 0.060V
  VDS_Level_0p068_V = (1 << 3),    //!< VDS_LEVEL = 0.068V
  VDS_Level_0p076_V = (2 << 3),    //!< VDS_LEVEL = 0.076V
  VDS_Level_0p086_V = (3 << 3),    //!< VDS_LEVEL = 0.086V
  VDS_Level_0p097_V = (4 << 3),    //!< VDS_LEVEL = 0.097V
  VDS_Level_0p109_V = (5 << 3),    //!< VDS_LEVEL = 0.109V
  VDS_Level_0p123_V = (6 << 3),    //!< VDS_LEVEL = 0.123V
  VDS_Level_0p138_V = (7 << 3),    //!< VDS_LEVEL = 0.138V
  VDS_Level_0p155_V = (8 << 3),    //!< VDS_LEVEL = 0.155V
  VDS_Level_0p175_V = (9 << 3),    //!< VDS_LEVEL = 0.175V
  VDS_Level_0p197_V = (10 << 3),   //!< VDS_LEVEL = 0.197V
  VDS_Level_0p222_V = (11 << 3),   //!< VDS_LEVEL = 0.222V
  VDS_Level_0p250_V = (12 << 3),   //!< VDS_LEVEL = 0.250V
  VDS_Level_0p282_V = (13 << 3),   //!< VDS_LEVEL = 0.282V
  VDS_Level_0p317_V = (14 << 3),   //!< VDS_LEVEL = 0.317V
  VDS_Level_0p358_V = (15 << 3),   //!< VDS_LEVEL = 0.358V
  VDS_Level_0p403_V = (16 << 3),   //!< VDS_LEVEL = 0.403V
  VDS_Level_0p454_V = (17 << 3),   //!< VDS_LEVEL = 0.454V
  VDS_Level_0p511_V = (18 << 3),   //!< VDS_LEVEL = 0.511V
  VDS_Level_0p576_V = (19 << 3),   //!< VDS_LEVEL = 0.576V
  VDS_Level_0p648_V = (20 << 3),   //!< VDS_LEVEL = 0.648V
  VDS_Level_0p730_V = (21 << 3),   //!< VDS_LEVEL = 0.730V
  VDS_Level_0p822_V = (22 << 3),   //!< VDS_LEVEL = 0.822V
  VDS_Level_0p926_V = (23 << 3),   //!< VDS_LEVEL = 0.926V
  VDS_Level_1p043_V = (24 << 3),   //!< VDS_LEVEL = 1.043V
  VDS_Level_1p175_V = (25 << 3),   //!< VDS_LEVEL = 1.175V
  VDS_Level_1p324_V = (26 << 3),   //!< VDS_LEVEL = 1.324V
  VDS_Level_1p491_V = (27 << 3),   //!< VDS_LEVEL = 1.491V
  VDS_Level_1p679_V = (28 << 3),   //!< VDS_LEVEL = 1.679V
  VDS_Level_1p892_V = (29 << 3),   //!< VDS_LEVEL = 1.892V
  VDS_Level_2p131_V = (30 << 3)    //!< VDS_LEVEL = 2.131V
} DRV8305_CTRL0C_VDSLevel_e;


//! \brief Enumeration for the register addresses
//!
typedef enum 
{
  Address_Status_1  = 1 << 11,   //!< Status Register 1
  Address_Status_2  = 2 << 11,   //!< Status Register 2
  Address_Status_3  = 3 << 11,   //!< Status Register 3
  Address_Status_4  = 4 << 11,   //!< Status Register 4
  Address_Control_5 = 5 << 11,   //!< Control Register 5
  Address_Control_6 = 6 << 11,   //!< Control Register 6
  Address_Control_7 = 7 << 11,   //!< Control Register 7
  Address_Control_8 = 8 << 11,   //!< Control Register 8
  Address_Control_9 = 9 << 11,   //!< Control Register 9
  Address_Control_A = 10 << 11,  //!< Control Register A
  Address_Control_B = 11 << 11,  //!< Control Register B
  Address_Control_C = 12 << 11   //!< Control Register C
} DRV8305_Address_e;


//! \brief Enumeration for the shunt amplifier number
//!
typedef enum 
{
  ShuntAmpNumber_1 = 1,      //!< Shunt amplifier number 1
  ShuntAmpNumber_2 = 2,      //!< Shunt amplifier number 2
  ShuntAmpNumber_3 = 3       //!< Shunt amplifier number 3
} DRV8305_ShuntAmpNumber_e;


//! \brief Object for the DRV8305 STATUS01 register
//!
typedef struct _DRV_SPI_8305_Stat01_t_
{
  bool                  OTW;            // Bits 0
  bool                  TEMP_FLAG3;     // Bits 1
  bool                  TEMP_FLAG2;     // Bits 2
  bool                  TEMP_FLAG1;     // Bits 3
  bool                  VCPH_UVFL;      // Bits 4
  bool                  VDS_STATUS;     // Bits 5
  bool                  PVDD_OVFL;      // Bits 6
  bool                  PVDD_UVFL;      // Bits 7
  bool                  TEMP_FLAG4;     // Bits 8
  bool                  STAT01_RSV1;    // Bits 9
  bool                  FAULT;          // Bits 10
}DRV_SPI_8305_Stat01_t_;


//! \brief Object for the DRV8305 STATUS02 register
//!
typedef struct _DRV_SPI_8305_Stat02_t_
{
  bool                  SNS_A_OCP;      // Bits 0
  bool                  SNS_B_OCP;      // Bits 1
  bool                  SNS_C_OCP;      // Bits 2
  bool                  STAT02_RSV2;    // Bits 3
  bool                  STAT02_RSV1;    // Bits 4
  bool                  FETLC_VDS;      // Bits 5
  bool                  FETHC_VDS;      // Bits 6
  bool                  FETLB_VDS;      // Bits 7
  bool                  FETHB_VDS;      // Bits 8
  bool                  FETLA_VDS;      // Bits 9
  bool                  FETHA_VDS;      // Bits 10
}DRV_SPI_8305_Stat02_t_;


//! \brief Object for the DRV8305 STATUS03 register
//!
typedef struct _DRV_SPI_8305_Stat03_t_
{
  bool                  VCPH_OVLO_ABS;  // Bits 0
  bool                  VCPH_OVLO;      // Bits 1
  bool                  VCPH_UVLO2;     // Bits 2
  bool                  STAT03_RSV2;    // Bits 3
  bool                  VCP_LSD_UVLO2;  // Bits 4
  bool                  AVDD_UVLO;      // Bits 5
  bool                  VREG_UV;        // Bits 6
  bool                  STAT03_RSV1;    // Bits 7
  bool                  OTS;            // Bits 8
  bool                  WD_FAULT;       // Bits 9
  bool                  PVDD_UVLO2;     // Bits 10
}DRV_SPI_8305_Stat03_t_;


//! \brief Object for the DRV8305 STATUS04 register
//!
typedef struct _DRV_SPI_8305_Stat04_t_
{
  bool                  STAT04_RSV5;    // Bits 0
  bool                  STAT04_RSV4;    // Bits 1
  bool                  STAT04_RSV3;    // Bits 2
  bool                  STAT04_RSV2;    // Bits 3
  bool                  STAT04_RSV1;    // Bits 4
  bool                  FETLC_VGS;      // Bits 5
  bool                  FETHC_VGS;      // Bits 6
  bool                  FETLB_VGS;      // Bits 7
  bool                  FETHB_VGS;      // Bits 8
  bool                  FETLA_VGS;      // Bits 9
  bool                  FETHA_VGS;      // Bits 10
}DRV_SPI_8305_Stat04_t_;


//! \brief Object for the DRV8305 CTRL05 register
//!
typedef struct _DRV_SPI_8305_Ctrl05_t_
{
  DRV8305_CTRL05_PeakSourCurHS_e    IDRIVEP_HS;     // Bits 3-0
  DRV8305_CTRL05_PeakSinkCurHS_e    IDRIVEN_HS;     // Bits 7-4
  DRV8305_CTRL05_PeakSourTime_e     TDRIVEN;        // Bits 9-8
  bool                              CTRL05_RSV1;    // Bits 10
}DRV_SPI_8305_Ctrl05_t_;


//! \brief Object for the DRV8305 CTRL06 register
//!
typedef struct _DRV_SPI_8305_Ctrl06_t_
{
  DRV8305_CTRL06_PeakSourCurLS_e    IDRIVEP_LS;     // Bits 3-0
  DRV8305_CTRL06_PeakSinkCurLS_e    IDRIVEN_LS;     // Bits 7-4
  DRV8305_CTRL06_PeakSinkTime_e     TDRIVEP;        // Bits 9-8
  bool                              CTRL06_RSV1;    // Bits 10
}DRV_SPI_8305_Ctrl06_t_;


//! \brief Object for the DRV8305 CTRL07 register
//!
typedef struct _DRV_SPI_8305_Ctrl07_t_
{
  DRV8305_CTRL07_VDSDeglitch_e      TVDS;           // Bits 1-0
  DRV8305_CTRL07_VDSBlanking_e      TBLANK;         // Bits 3-2
  DRV8305_CTRL07_DeadTime_e         DEAD_TIME;      // Bits 6-4
  DRV8305_CTRL07_PwmMode_e          PWM_MODE;       // Bits 8-7
  DRV8305_CTRL07_CommOption_e       COMM_OPT;       // Bits 9
  bool                              CTRL07_RSV1;    // Bits 10
}DRV_SPI_8305_Ctrl07_t_;


//! \brief Object for the DRV8305 CTRL08 register
//!
typedef struct _DRV_SPI_8305_Ctrl08_t_
{
  uint16_t                          CTRL08_RSV1;    // Bits 10-0
}DRV_SPI_8305_Ctrl08_t_;


//! \brief Object for the DRV8305 CTRL09 register
//!
typedef struct _DRV_SPI_8305_Ctrl09_t_
{
  bool                          SET_VCPH_UV;    // Bits 0
  bool                          CLR_FLTS;       // Bits 1
  bool                          SLEEP;          // Bits 2
  bool                          WD_EN;          // Bits 3
  bool                          DIS_SNS_OCP;    // Bits 4
  DRV8305_CTRL09_WatchDelay_e   WD_DLY;         // Bits 6-5
  bool                          EN_SNS_CLAMP;   // Bits 7
  bool                          DIS_GDRV_FAULT; // Bits 8
  bool                          DISABLE;        // Bits 9
  bool                          FLIP_OTS;       // Bits 10
}DRV_SPI_8305_Ctrl09_t_;


//! \brief Object for the DRV8305 CTRL0A register
//!
typedef struct _DRV_SPI_8305_Ctrl0A_t_
{
  DRV8305_CTRL0A_CSGain1_e      GAIN_CS1;       // Bits 1-0
  DRV8305_CTRL0A_CSGain2_e      GAIN_CS2;       // Bits 3-2
  DRV8305_CTRL0A_CSGain3_e      GAIN_CS3;       // Bits 5-4
  DRV8305_CTRL0A_CSBlank_e      CS_BLANK;       // Bits 7-6
  bool                          DC_CAL_CH1;     // Bits 8
  bool                          DC_CAL_CH2;     // Bits 9
  bool                          DC_CAL_CH3;     // Bits 10
}DRV_SPI_8305_Ctrl0A_t_;


//! \brief Object for the DRV8305 CTRL0B register
//!
typedef struct _DRV_SPI_8305_Ctrl0B_t_
{
  DRV8305_CTRL0B_VregUvLevel_e  VREG_UV_LEVEL;  // Bits 1-0
  bool                          DIS_PWRGD;      // Bits 2
  DRV8305_CTRL0B_SleepDelay_e   SLP_DLY;        // Bits 4-3
  bool                          CTRL0B_RSV1;    // Bits 5
  bool                          CTRL0B_RSV2;    // Bits 6
  bool                          CTRL0B_RSV3;    // Bits 7
  DRV8305_CTRL0B_VrefScaling_e  VREF_SCALING;   // Bits 9-8
  bool                          CTRL0B_RSV4;    // Bits 10
}DRV_SPI_8305_Ctrl0B_t_;


//! \brief Object for the DRV8305 CTRL0C register
//!
typedef struct _DRV_SPI_8305_Ctrl0C_t_
{
  DRV8305_CTRL0C_VDSMode_e      VDS_MODE;       // Bits 2-0
  DRV8305_CTRL0C_VDSLevel_e     VDS_LEVEL;      // Bits 7-3
  bool                          CTRL0C_RSV1;    // Bits 8
  bool                          CTRL0C_RSV2;    // Bits 9
  bool                          CTRL0C_RSV3;    // Bits 10
}DRV_SPI_8305_Ctrl0C_t_;


//! \brief Object for the DRV8305 registers and commands
//!
typedef struct _DRV_SPI_8305_Vars_t_
{
  DRV_SPI_8305_Stat01_t_    Stat_Reg_01;
  DRV_SPI_8305_Stat02_t_    Stat_Reg_02;
  DRV_SPI_8305_Stat03_t_    Stat_Reg_03;
  DRV_SPI_8305_Stat04_t_    Stat_Reg_04;

  DRV_SPI_8305_Ctrl05_t_    Ctrl_Reg_05;
  DRV_SPI_8305_Ctrl06_t_    Ctrl_Reg_06;
  DRV_SPI_8305_Ctrl07_t_    Ctrl_Reg_07;
  DRV_SPI_8305_Ctrl08_t_    Ctrl_Reg_08;
  DRV_SPI_8305_Ctrl09_t_    Ctrl_Reg_09;
  DRV_SPI_8305_Ctrl0A_t_    Ctrl_Reg_0A;
  DRV_SPI_8305_Ctrl0B_t_    Ctrl_Reg_0B;
  DRV_SPI_8305_Ctrl0C_t_    Ctrl_Reg_0C;
  bool                      WriteCmd;
  bool                      ReadCmd;

  uint16_t                  ManWriteAddr;
  uint16_t                  ManReadAddr;
  uint16_t                  ManWriteData;
  uint16_t                  ManReadData;
  bool                      ManWriteCmd;
  bool                      ManReadCmd;
}DRV_SPI_8305_Vars_t;


//! \brief Defines the DRV8305 object
//!
typedef struct _DRV8305_Obj_
{
  SPI_Handle       spiHandle;                  //!< the handle for the serial peripheral interface
  GPIO_Handle      gpioHandle;                 //!< the gpio handle that is connected to the drv8305 enable pin
  GPIO_Number_e    gpioNumber;                 //!< the gpio number that is connected to the drv8305 enable pin
  bool             RxTimeOut;                  //!< the timeout flag for the RX fifo
  bool             enableTimeOut;              //!< the timeout flag for drv8305 enable
} DRV8305_Obj;


//! \brief Defines the DRV8305 handle
//!
typedef struct _DRV8305_Obj_ *DRV8305_Handle;


//! \brief Defines the DRV8305 Word type
//!
typedef  uint16_t    DRV8305_Word_t;


// **************************************************************************
// the globals



// **************************************************************************
// the function prototypes

//! \brief     Initializes the DRV8305 object
//! \param[in] pMemory   A pointer to the memory for the DRV8305 object
//! \param[in] numBytes  The number of bytes allocated for the DRV8305 object, bytes
//! \return    The DRV8305 object handle
extern DRV8305_Handle DRV8305_init(void *pMemory,const size_t numBytes);


//! \brief     Builds the control word
//! \param[in] ctrlMode  The control mode
//! \param[in] regName   The register name
//! \param[in] data      The data
//! \return    The control word
static inline DRV8305_Word_t DRV8305_buildCtrlWord(const DRV8305_CtrlMode_e ctrlMode,
                                             const DRV8305_Address_e regAddr,
                                             const uint16_t data)
{
  DRV8305_Word_t ctrlWord = ctrlMode | regAddr | (data & DRV8305_DATA_MASK);

  return(ctrlWord);
} // end of DRV8305_buildCtrlWord() function


//! \brief     Enables the DRV8305
//! \param[in] handle     The DRV8305 handle
extern void DRV8305_enable(DRV8305_Handle handle);


//! \brief     Gets the high side gate drive peak source current level
//! \param[in] handle     The DRV8305 handle
//! \return    The peak source current level, A
extern DRV8305_CTRL05_PeakSourCurHS_e DRV8305_getPeakSourCurHS(DRV8305_Handle handle);


//! \brief     Gets the high side gate drive peak sink current level
//! \param[in] handle     The DRV8305 handle
//! \return    The peak sink current level, A
extern DRV8305_CTRL05_PeakSinkCurHS_e DRV8305_getPeakSinkCurHS(DRV8305_Handle handle);


//! \brief     Gets the high side and low side gate drive peak source time
//! \param[in] handle     The DRV8305 handle
//! \return    The peak source time, ns
extern DRV8305_CTRL05_PeakSourTime_e DRV8305_getPeakSourTime(DRV8305_Handle handle);


//! \brief     Gets the low side gate drive peak source current level
//! \param[in] handle     The DRV8305 handle
//! \return    The peak source current level, A
extern DRV8305_CTRL06_PeakSourCurLS_e DRV8305_getPeakSourCurLS(DRV8305_Handle handle);


//! \brief     Gets the low side side gate drive peak sink current level
//! \param[in] handle     The DRV8305 handle
//! \return    The peak sink current level, A
extern DRV8305_CTRL06_PeakSinkCurLS_e DRV8305_getPeakSinkCurLS(DRV8305_Handle handle);


//! \brief     Gets the high side and low side gate drive peak sink time
//! \param[in] handle     The DRV8305 handle
//! \return    The peak sink time, ns
extern DRV8305_CTRL06_PeakSinkTime_e DRV8305_getPeakSinkTime(DRV8305_Handle handle);


//! \brief     Gets the VDS sense deglitch time
//! \param[in] handle     The DRV8305 handle
//! \return    The deglitch time time, us
extern DRV8305_CTRL07_VDSDeglitch_e DRV8305_getVDSDeglitch(DRV8305_Handle handle);


//! \brief     Gets the VDS sense blanking time
//! \param[in] handle     The DRV8305 handle
//! \return    The blanking time time, us
extern DRV8305_CTRL07_VDSBlanking_e DRV8305_getVDSBlanking(DRV8305_Handle handle);


//! \brief     Gets the driver dead time
//! \param[in] handle     The DRV8305 handle
//! \return    The dead time, ns
extern DRV8305_CTRL07_DeadTime_e DRV8305_getDeadTime(DRV8305_Handle handle);


//! \brief     Gets the driver PWM mode
//! \param[in] handle     The DRV8305 handle
//! \return    The PWM mode
extern DRV8305_CTRL07_PwmMode_e DRV8305_getPwmMode(DRV8305_Handle handle);


//! \brief     Gets the driver commutation mode for 1PWM mode
//! \param[in] handle     The DRV8305 handle
//! \return    The commutation mode
extern DRV8305_CTRL07_CommOption_e DRV8305_getCommOption(DRV8305_Handle handle);


//! \brief     Gets the watchdog timer setting
//! \param[in] handle     The DRV8305 handle
//! \return    The watchdog timer setting, ms
extern DRV8305_CTRL09_WatchDelay_e DRV8305_getWatchDelay(DRV8305_Handle handle);


//! \brief     Gets the gain of shunt amplifier 1
//! \param[in] handle     The DRV8305 handle
//! \return    The amplifier gain, V/V
extern DRV8305_CTRL0A_CSGain1_e DRV8305_getCSGain1(DRV8305_Handle handle);


//! \brief     Gets the gain of shunt amplifier 2
//! \param[in] handle     The DRV8305 handle
//! \return    The amplifier gain, V/V
extern DRV8305_CTRL0A_CSGain2_e DRV8305_getCSGain2(DRV8305_Handle handle);


//! \brief     Gets the gain of shunt amplifier 3
//! \param[in] handle     The DRV8305 handle
//! \return    The amplifier gain, V/V
extern DRV8305_CTRL0A_CSGain3_e DRV8305_getCSGain3(DRV8305_Handle handle);


//! \brief     Gets the blanking time of the shunt amplifiers
//! \param[in] handle     The DRV8305 handle
//! \return    The amplifier blanking time, us
extern DRV8305_CTRL0A_CSBlank_e DRV8305_getCSBlank(DRV8305_Handle handle);


//! \brief     Gets the DC calibration mode
//! \param[in] handle     The DRV8305 handle
//! \param[in] ampNumber  The shunt amplifier number
//! \return    The DC calibration mode
extern DRV8305_CTRL0A_DcCalMode_e DRV8305_getDcCalMode(DRV8305_Handle handle,
                                                const DRV8305_ShuntAmpNumber_e ampNumber);

//! \brief     Gets the under voltage level of VREF
//! \param[in] handle     The DRV8305 handle
//! \return    The UV level, %
extern DRV8305_CTRL0B_VregUvLevel_e DRV8305_getVregUvLevel(DRV8305_Handle handle);


//! \brief     Gets the delay time to power down VREF after SLEEP
//! \param[in] handle     The DRV8305 handle
//! \return    The delay time, us
extern DRV8305_CTRL0B_SleepDelay_e DRV8305_getSleepDelay(DRV8305_Handle handle);


//! \brief     Gets the VREF scaling factor
//! \param[in] handle     The DRV8305 handle
//! \return    The VREF scaling factor
extern DRV8305_CTRL0B_VrefScaling_e DRV8305_getVrefScaling(DRV8305_Handle handle);


//! \brief     Gets the VDS protection mode
//! \param[in] handle     The DRV8305 handle
//! \return    The VDS protection mode
extern DRV8305_CTRL0C_VDSMode_e DRV8305_getVDSMode(DRV8305_Handle handle);


//! \brief     Gets the VDS OC level
//! \param[in] handle     The DRV8305 handle
//! \return    The VDS OC level, V
extern DRV8305_CTRL0C_VDSLevel_e DRV8305_getVDSLevel(DRV8305_Handle handle);


//! \brief     Sets the SPI handle in the DRV8305
//! \param[in] handle     The DRV8305 handle
//! \param[in] spiHandle  The SPI handle to use
void DRV8305_setSpiHandle(DRV8305_Handle handle,SPI_Handle spiHandle);


//! \brief     Sets the GPIO handle in the DRV8305
//! \param[in] handle       The DRV8305 handle
//! \param[in] gpioHandle   The GPIO handle to use
void DRV8305_setGpioHandle(DRV8305_Handle handle,GPIO_Handle gpioHandle);


//! \brief     Sets the GPIO number in the DRV8305
//! \param[in] handle       The DRV8305 handle
//! \param[in] gpioHandle   The GPIO number to use
void DRV8305_setGpioNumber(DRV8305_Handle handle,GPIO_Number_e gpioNumber);


//! \brief     Sets the high side gate drive peak source current level
//! \param[in] handle   The DRV8305 handle
//! \param[in] level    The peak source current level, A
extern void DRV8305_setPeakSourCurHS(DRV8305_Handle handle,const DRV8305_CTRL05_PeakSourCurHS_e level);


//! \brief     Sets the high side gate drive peak sink current level
//! \param[in] handle   The DRV8305 handle
//! \param[in] level    The peak sink current level, A
extern void DRV8305_setPeakSinkCurHS(DRV8305_Handle handle,const DRV8305_CTRL05_PeakSinkCurHS_e level);


//! \brief     Sets the high side and low side gate drive peak source time
//! \param[in] handle   The DRV8305 handle
//! \param[in] time     The peak source time, ns
extern void DRV8305_setPeakSourTime(DRV8305_Handle handle,const DRV8305_CTRL05_PeakSourTime_e time);


//! \brief     Sets the the low side gate drive peak source current level
//! \param[in] handle   The DRV8305 handle
//! \param[in] level    The peak source current level, A
extern void DRV8305_setPeakSourCurLS(DRV8305_Handle handle,const DRV8305_CTRL06_PeakSourCurLS_e level);


//! \brief     Sets the low side side gate drive peak sink current level
//! \param[in] handle   The DRV8305 handle
//! \param[in] level    The peak sink current level, A
extern void DRV8305_setPeakSinkCurLS(DRV8305_Handle handle,const DRV8305_CTRL06_PeakSinkCurLS_e level);


//! \brief     Sets the high side and low side gate drive peak sink time
//! \param[in] handle   The DRV8305 handle
//! \param[in] time     The peak sink time, ns
extern void DRV8305_setPeakSinkTime(DRV8305_Handle handle,const DRV8305_CTRL06_PeakSinkTime_e time);


//! \brief     Sets the VDS sense deglitch time
//! \param[in] handle   The DRV8305 handle
//! \param[in] time     The deglitch time time, us
extern void DRV8305_setVDSDeglitch(DRV8305_Handle handle,const DRV8305_CTRL07_VDSDeglitch_e time);


//! \brief     Sets the VDS sense blanking time
//! \param[in] handle   The DRV8305 handle
//! \param[in] time     The blanking time time, us
extern void DRV8305_setVDSBlanking(DRV8305_Handle handle,const DRV8305_CTRL07_VDSBlanking_e time);


//! \brief     Sets the driver dead time
//! \param[in] handle   The DRV8305 handle
//! \param[in] time     The dead time, ns
extern void DRV8305_setDeadTime(DRV8305_Handle handle,const DRV8305_CTRL07_DeadTime_e time);


//! \brief     Sets the driver PWM mode
//! \param[in] handle   The DRV8305 handle
//! \param[in] mode     The PWM mode
extern void DRV8305_setPwmMode(DRV8305_Handle handle,const DRV8305_CTRL07_PwmMode_e mode);


//! \brief     Sets the driver commutation mode for 1PWM mode
//! \param[in] handle   The DRV8305 handle
//! \param[in] mode     The commutation mode
extern void DRV8305_setCommOption(DRV8305_Handle handle,const DRV8305_CTRL07_CommOption_e mode);


//! \brief     Sets the watchdog timer setting
//! \param[in] handle   The DRV8305 handle
//! \param[in] time     The watchdog timer setting, ms
extern void DRV8305_setWatchDelay(DRV8305_Handle handle,const DRV8305_CTRL09_WatchDelay_e time);


//! \brief     Sets the gain of shunt amplifier 1
//! \param[in] handle   The DRV8305 handle
//! \param[in] gain     The amplifier gain, V/V
extern void DRV8305_setCSGain1(DRV8305_Handle handle,const DRV8305_CTRL0A_CSGain1_e gain);


//! \brief     Sets the gain of shunt amplifier 2
//! \param[in] handle   The DRV8305 handle
//! \param[in] gain     The amplifier gain, V/V
extern void DRV8305_setCSGain2(DRV8305_Handle handle,const DRV8305_CTRL0A_CSGain2_e gain);


//! \brief     Sets the gain of shunt amplifier 3
//! \param[in] handle   The DRV8305 handle
//! \param[in] gain     The amplifier gain, V/V
extern void DRV8305_setCSGain3(DRV8305_Handle handle,const DRV8305_CTRL0A_CSGain3_e gain);


//! \brief     Sets the blanking time of the shunt amplifiers
//! \param[in] handle   The DRV8305 handle
//! \param[in] time     The amplifier blanking time, us
extern void DRV8305_setCSBlank(DRV8305_Handle handle,const DRV8305_CTRL0A_CSBlank_e time);


//! \brief     Sets the DC calibration mode
//! \param[in] handle     The DRV8305 handle
//! \param[in] ampNumber  The shunt amplifier number
//! \param[in] mode       The DC calibration mode
extern void DRV8305_setDcCalMode(DRV8305_Handle handle,
                                 const DRV8305_ShuntAmpNumber_e ampNumber,
                                 const DRV8305_CTRL0A_DcCalMode_e mode);


//! \brief     Sets the under voltage level of VREF
//! \param[in] handle   The DRV8305 handle
//! \param[in] level    The UV level, %
extern void DRV8305_setVregUvLevel(DRV8305_Handle handle,const DRV8305_CTRL0B_VregUvLevel_e level);


//! \brief     Sets the delay time to power down VREF after SLEEP
//! \param[in] handle   The DRV8305 handle
//! \param[in] time     The delay time, us
extern void DRV8305_setSleepDelay(DRV8305_Handle handle,const DRV8305_CTRL0B_SleepDelay_e time);


//! \brief     Sets the VREF scaling factor
//! \param[in] handle   The DRV8305 handle
//! \param[in] mode     The VREF scaling factor
extern void DRV8305_setVrefScaling(DRV8305_Handle handle,const DRV8305_CTRL0B_VrefScaling_e mode);


//! \brief     Sets the VDS protection mode
//! \param[in] handle   The DRV8305 handle
//! \param[in] mode     The VDS protection mode
extern void DRV8305_setVDSMode(DRV8305_Handle handle,const DRV8305_CTRL0C_VDSMode_e mode);


//! \brief     Sets the VDS OC level
//! \param[in] handle   The DRV8305 handle
//! \param[in] level    The VDS OC level, V
extern void DRV8305_setVDSLevel(DRV8305_Handle handle,const DRV8305_CTRL0C_VDSLevel_e level);


//! \brief     Determines if DRV8305 fault has occurred
//! \param[in] handle     The DRV8305 handle
//! \return    A boolean value denoting if a fault has occurred (true) or not (false)
extern bool DRV8305_isFault(DRV8305_Handle handle);


//! \brief     Resets the DRV8305
//! \param[in] handle   The DRV8305 handle
extern void DRV8305_reset(DRV8305_Handle handle);


//! \brief     Resets the enable timeout flag
//! \param[in] handle   The DRV8305 handle
static inline void DRV8305_resetEnableTimeout(DRV8305_Handle handle)
{
  DRV8305_Obj *obj = (DRV8305_Obj *)handle;

  obj->enableTimeOut = false;

  return;
} // end of DRV8305_resetEnableTimeout() function


//! \brief     Resets the RX fifo timeout flag
//! \param[in] handle   The DRV8305 handle
static inline void DRV8305_resetRxTimeout(DRV8305_Handle handle)
{
  DRV8305_Obj *obj = (DRV8305_Obj *)handle;

  obj->RxTimeOut = false;

  return;
} // end of DRV8305_resetRxTimeout() function


//! \brief     Initialize the interface to all 8305 SPI variables
//! \param[in] handle  The DRV8305 handle
extern void DRV8305_setupSpi(DRV8305_Handle handle, DRV_SPI_8305_Vars_t *Spi_8305_Vars);


//! \brief     Reads data from the DRV8305 register
//! \param[in] handle   The DRV8305 handle
//! \param[in] regAddr  The register address
//! \return    The data value
extern uint16_t DRV8305_readSpi(DRV8305_Handle handle,const DRV8305_Address_e regAddr);


//! \brief     Writes data to the DRV8305 register
//! \param[in] handle   The DRV8305 handle
//! \param[in] regAddr  The register name
//! \param[in] data     The data value
extern void DRV8305_writeSpi(DRV8305_Handle handle,const DRV8305_Address_e regAddr,const uint16_t data);


//! \brief     Write to the DRV8305 SPI registers
//! \param[in] handle  The DRV8305 handle
//! \param[in] Spi_8305_Vars  The (DRV_SPI_8305_Vars_t) structure that contains all DRV8305 Status/Control register options
extern void DRV8305_writeData(DRV8305_Handle handle, DRV_SPI_8305_Vars_t *Spi_8305_Vars);

//! \brief     Read from the DRV8305 SPI registers
//! \param[in] handle  The DRV8305 handle
//! \param[in] Spi_8305_Vars  The (DRV_SPI_8305_Vars_t) structure that contains all DRV8305 Status/Control register options
extern void DRV8305_readData(DRV8305_Handle handle, DRV_SPI_8305_Vars_t *Spi_8305_Vars);


#ifdef __cplusplus
}
#endif // extern "C"

//@}  // ingroup

#endif // end of _DRV8305_H_ definition





