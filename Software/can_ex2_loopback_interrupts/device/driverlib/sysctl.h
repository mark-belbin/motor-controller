//###########################################################################
//
// FILE:   sysctl.h
//
// TITLE:  C28x system control driver.
//
//###########################################################################
// $TI Release: F28004x Support Library v1.10.00.00 $
// $Release Date: Tue May 26 17:06:03 IST 2020 $
// $Copyright:
// Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
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
//###########################################################################

#ifndef SYSCTL_H
#define SYSCTL_H

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! \addtogroup sysctl_api SysCtl
//! @{
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_nmi.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "cpu.h"
#include "debug.h"
#include "interrupt.h"


//*****************************************************************************
//
// Defines for system control functions. Not intended for use by application
// code.
//
//*****************************************************************************
//
// Shifted pattern for WDCR register's WDCHK field.
//
#define SYSCTL_WD_CHKBITS       0x0028U

//
// Keys for WDKEY field. The first enables resets and the second resets.
//
#define SYSCTL_WD_ENRSTKEY      0x0055U
#define SYSCTL_WD_RSTKEY        0x00AAU

//
// Values to help decode peripheral parameter
//
#define SYSCTL_PERIPH_REG_M     0x001FU
#define SYSCTL_PERIPH_REG_S     0x0000U
#define SYSCTL_PERIPH_BIT_M     0x1F00U
#define SYSCTL_PERIPH_BIT_S     0x0008U

//
//Keys for the System control registers write protection
//
#define SYSCTL_REG_KEY       0xA5A50000U
#define SYSCTL_PLL_KEY       0XCAFE0000U

//
//Values to help access shifting of bits
//
#define SYSCTL_TYPE_LOCK_S              0xFU


//
// LPM defines for LPMCR.LPM
//
#define SYSCTL_LPM_IDLE         0x0000U
#define SYSCTL_LPM_HALT         0x0002U

//
// Default internal oscillator frequency, 10 MHz
//
#define SYSCTL_DEFAULT_OSC_FREQ     10000000U

//
// Mask for SYNCSELECT.SYNCIN
//
#define SYSCTL_SYNCSELECT_SYNCIN_M  SYSCTL_SYNCSELECT_EPWM4SYNCIN_M

//
// Boot ROM Booting and Reset Status
//
#define SYSCTL_BOOT_ROM_STATUS    0x0002U
#define SYSCTL_BOOT_ROM_POR       0x2000U
#define SYSCTL_BOOT_ROM_XRS       0x1000U

//
// Device_cal function which is available in OTP memory
// This function is called in SysCtl_resetPeripheral after resetting
// analog peripherals
//
#define Device_cal ((void (*)(void))((uintptr_t)0x070282))

//*****************************************************************************
//
// The following are values that can be passed to the SysCtl_setClock() API as
// the config parameter.
//
//*****************************************************************************
//
// System clock divider (SYSDIV)
//
#define SYSCTL_SYSDIV_M     0x00001F80U // Mask for SYSDIV value in config
#define SYSCTL_SYSDIV_S     7U          // Shift for SYSDIV value in config

//! Macro to format system clock divider value. x must be 1 or even values up
//! to 126.
#define SYSCTL_SYSDIV(x)    ((((x) / 2U) << SYSCTL_SYSDIV_S) & SYSCTL_SYSDIV_M)

//
// Integer multiplier (IMULT)
//
#define SYSCTL_IMULT_M      0x0000007FU // Mask for IMULT value in config
#define SYSCTL_IMULT_S      0U          // Shift for IMULT value in config
//! Macro to format integer multiplier value. x is a number from 1 to 127.
//!
#define SYSCTL_IMULT(x)     (((x) << SYSCTL_IMULT_S) & SYSCTL_IMULT_M)

#ifndef DOXYGEN_PDF_IGNORE
//
// Fractional multiplier (FMULT)
//
#define SYSCTL_FMULT_M      0x00006000U // Mask for FMULT value in config
#define SYSCTL_FMULT_S      13U         // Shift for FMULT value in config
#define SYSCTL_FMULT_NONE   0x00000000U //!< No fractional multiplier
#define SYSCTL_FMULT_0      0x00000000U //!< No fractional multiplier
#define SYSCTL_FMULT_1_4    0x00002000U //!< Fractional multiplier of 0.25
#define SYSCTL_FMULT_1_2    0x00004000U //!< Fractional multiplier of 0.50
#define SYSCTL_FMULT_3_4    0x00006000U //!< Fractional multiplier of 0.75

//
// Oscillator source
//
// Also used with the SysCtl_selectOscSource(), SysCtl_turnOnOsc(),
// and SysCtl_turnOffOsc() functions as the oscSource parameter.
//
#define SYSCTL_OSCSRC_M         0x00030000U // Mask for OSCSRC value in config
#define SYSCTL_OSCSRC_S         16U         // Shift for OSCSRC value in config
//! Internal oscillator INTOSC2
#define SYSCTL_OSCSRC_OSC2      0x00000000U
//! External oscillator (XTAL) in crystal mode
#define SYSCTL_OSCSRC_XTAL      0x00010000U
//! External oscillator (XTAL) in single-ended mode
#define SYSCTL_OSCSRC_XTAL_SE   0x00030000U
//! Internal oscillator INTOSC1
#define SYSCTL_OSCSRC_OSC1      0x00020000U

//
// Enable/disable PLL
//
#define SYSCTL_PLL_ENABLE       0x80000000U //!< Enable PLL
#define SYSCTL_PLL_DISABLE      0x00000000U //!< Disable PLL

//
// Number of PLL retries for SW work around
//
#define SYSCTL_PLL_RETRIES              100U    // Number of PLL retries
#define SYSCTL_PLLLOCK_TIMEOUT          2000U   // PLL Lock timeout
#define SYSCTL_DCC_COUNTER0_WINDOW      100U    // DCC Counter0 Window
//*****************************************************************************
//
// Values that can be passed to SysCtl_clearNMIStatus(),
// SysCtl_forceNMIFlags(), SysCtl_isNMIFlagSet(), and
// SysCtl_isNMIShadowFlagSet() as the nmiFlags parameter and returned by
// SysCtl_getNMIFlagStatus() and SysCtl_getNMIShadowFlagStatus().
//
//*****************************************************************************
#define SYSCTL_NMI_NMIINT             0x1U //!<  NMI Interrupt Flag
#define SYSCTL_NMI_CLOCKFAIL          0x2U //!<  Clock Fail Interrupt Flag
#define SYSCTL_NMI_RAMUNCERR          0x4U //!<  RAM Uncorrectable Error NMI
#define SYSCTL_NMI_FLUNCERR           0x8U //!<  Flash Uncorrectable Error NMI
#define SYSCTL_NMI_PIEVECTERR         0x40U //!<  PIE Vector Fetch Error Flag
#define SYSCTL_NMI_CLBNMI             0x100U //!<  Configurable Logic Block NMI
#define SYSCTL_NMI_SWERR              0x2000U //!<  SW Error Force NMI Flag

//*****************************************************************************
//
// The following are values that can be passed to the SysCtl_clearResetCause()
// API as rstCauses or returned by the SysCtl_getResetCause() API.
//
//*****************************************************************************
#define SYSCTL_CAUSE_POR                 0x00000001U //!< Power-on reset
#define SYSCTL_CAUSE_XRS                 0x00000002U //!< External reset pin
#define SYSCTL_CAUSE_WDRS                0x00000004U //!< Watchdog reset
#define SYSCTL_CAUSE_NMIWDRS             0x00000008U //!< NMI watchdog reset
#define SYSCTL_CAUSE_SCCRESET            0x00000100U //!< SCCRESETn by DCSM
//*****************************************************************************
//
// The following values define the adcsocSrc parameter for
// SysCtl_enableExtADCSOCSource() and SysCtl_disableExtADCSOCSource().
//
//*****************************************************************************
#define SYSCTL_ADCSOC_SRC_PWM1SOCA   0x1U //!<ePWM1 SOCA for ADCSOCAO
#define SYSCTL_ADCSOC_SRC_PWM2SOCA   0x2U //!<ePWM2 SOCA for ADCSOCAO
#define SYSCTL_ADCSOC_SRC_PWM3SOCA   0x4U //!<ePWM3 SOCA for ADCSOCAO
#define SYSCTL_ADCSOC_SRC_PWM4SOCA   0x8U //!<ePWM4 SOCA for ADCSOCAO
#define SYSCTL_ADCSOC_SRC_PWM5SOCA   0x10U //!<ePWM5 SOCA for ADCSOCAO
#define SYSCTL_ADCSOC_SRC_PWM6SOCA   0x20U //!<ePWM6 SOCA for ADCSOCAO
#define SYSCTL_ADCSOC_SRC_PWM7SOCA   0x40U //!<ePWM7 SOCA for ADCSOCAO
#define SYSCTL_ADCSOC_SRC_PWM8SOCA   0x80U //!<ePWM8 SOCA for ADCSOCAO
#define SYSCTL_ADCSOC_SRC_PWM1SOCB   0x10000U //!<ePWM1 SOCB for ADCSOCBO
#define SYSCTL_ADCSOC_SRC_PWM2SOCB   0x20000U //!<ePWM2 SOCB for ADCSOCBO
#define SYSCTL_ADCSOC_SRC_PWM3SOCB   0x40000U //!<ePWM3 SOCB for ADCSOCBO
#define SYSCTL_ADCSOC_SRC_PWM4SOCB   0x80000U //!<ePWM4 SOCB for ADCSOCBO
#define SYSCTL_ADCSOC_SRC_PWM5SOCB   0x100000U //!<ePWM5 SOCB for ADCSOCBO
#define SYSCTL_ADCSOC_SRC_PWM6SOCB   0x200000U //!<ePWM6 SOCB for ADCSOCBO
#define SYSCTL_ADCSOC_SRC_PWM7SOCB   0x400000U //!<ePWM7 SOCB for ADCSOCBO
#define SYSCTL_ADCSOC_SRC_PWM8SOCB   0x800000U //!<ePWM8 SOCB for ADCSOCBO
#endif

//*****************************************************************************
//
//! The following are values that can be passed to SysCtl_enablePeripheral()
//! and SysCtl_disablePeripheral() as the \e peripheral parameter.
//
//*****************************************************************************
typedef enum
{
    SYSCTL_PERIPH_CLK_CLA1 = 0x0000, //!< CLA1 clock
    SYSCTL_PERIPH_CLK_DMA = 0x0200, //!< DMA clock
    SYSCTL_PERIPH_CLK_TIMER0 = 0x0300, //!< CPUTIMER0 clock
    SYSCTL_PERIPH_CLK_TIMER1 = 0x0400, //!< CPUTIMER1 clock
    SYSCTL_PERIPH_CLK_TIMER2 = 0x0500, //!< CPUTIMER2 clock
    SYSCTL_PERIPH_CLK_HRPWM = 0x1000, //!< HRPWM clock
    SYSCTL_PERIPH_CLK_TBCLKSYNC = 0x1200, //!< TBCLKSYNC clock
    SYSCTL_PERIPH_CLK_EPWM1 = 0x0002, //!< EPWM1 clock
    SYSCTL_PERIPH_CLK_EPWM2 = 0x0102, //!< EPWM2 clock
    SYSCTL_PERIPH_CLK_EPWM3 = 0x0202, //!< EPWM3 clock
    SYSCTL_PERIPH_CLK_EPWM4 = 0x0302, //!< EPWM4 clock
    SYSCTL_PERIPH_CLK_EPWM5 = 0x0402, //!< EPWM5 clock
    SYSCTL_PERIPH_CLK_EPWM6 = 0x0502, //!< EPWM6 clock
    SYSCTL_PERIPH_CLK_EPWM7 = 0x0602, //!< EPWM7 clock
    SYSCTL_PERIPH_CLK_EPWM8 = 0x0702, //!< EPWM8 clock
    SYSCTL_PERIPH_CLK_ECAP1 = 0x0003, //!< ECAP1 clock
    SYSCTL_PERIPH_CLK_ECAP2 = 0x0103, //!< ECAP2 clock
    SYSCTL_PERIPH_CLK_ECAP3 = 0x0203, //!< ECAP3 clock
    SYSCTL_PERIPH_CLK_ECAP4 = 0x0303, //!< ECAP4 clock
    SYSCTL_PERIPH_CLK_ECAP5 = 0x0403, //!< ECAP5 clock
    SYSCTL_PERIPH_CLK_ECAP6 = 0x0503, //!< ECAP6 clock
    SYSCTL_PERIPH_CLK_ECAP7 = 0x0603, //!< ECAP7 clock
    SYSCTL_PERIPH_CLK_EQEP1 = 0x0004, //!< EQEP1 clock
    SYSCTL_PERIPH_CLK_EQEP2 = 0x0104, //!< EQEP2 clock
    SYSCTL_PERIPH_CLK_SD1 = 0x0006, //!< SD1 clock
    SYSCTL_PERIPH_CLK_SCIA = 0x0007, //!< SCI_A clock
    SYSCTL_PERIPH_CLK_SCIB = 0x0107, //!< SCI_B clock
    SYSCTL_PERIPH_CLK_SPIA = 0x0008, //!< SPI_A clock
    SYSCTL_PERIPH_CLK_SPIB = 0x0108, //!< SPI_B clock
    SYSCTL_PERIPH_CLK_I2CA = 0x0009, //!< I2C_A clock
    SYSCTL_PERIPH_CLK_CANA = 0x000A, //!< CAN_A clock
    SYSCTL_PERIPH_CLK_CANB = 0x010A, //!< CAN_B clock
    SYSCTL_PERIPH_CLK_ADCA = 0x000D, //!< ADC_A clock
    SYSCTL_PERIPH_CLK_ADCB = 0x010D, //!< ADC_B clock
    SYSCTL_PERIPH_CLK_ADCC = 0x020D, //!< ADC_C clock
    SYSCTL_PERIPH_CLK_CMPSS1 = 0x000E, //!< CMPSS1 clock
    SYSCTL_PERIPH_CLK_CMPSS2 = 0x010E, //!< CMPSS2 clock
    SYSCTL_PERIPH_CLK_CMPSS3 = 0x020E, //!< CMPSS3 clock
    SYSCTL_PERIPH_CLK_CMPSS4 = 0x030E, //!< CMPSS4 clock
    SYSCTL_PERIPH_CLK_CMPSS5 = 0x040E, //!< CMPSS5 clock
    SYSCTL_PERIPH_CLK_CMPSS6 = 0x050E, //!< CMPSS6 clock
    SYSCTL_PERIPH_CLK_CMPSS7 = 0x060E, //!< CMPSS7 clock
    SYSCTL_PERIPH_CLK_PGA1 = 0x000F, //!< PGA1 clock
    SYSCTL_PERIPH_CLK_PGA2 = 0x010F, //!< PGA2 clock
    SYSCTL_PERIPH_CLK_PGA3 = 0x020F, //!< PGA3 clock
    SYSCTL_PERIPH_CLK_PGA4 = 0x030F, //!< PGA4 clock
    SYSCTL_PERIPH_CLK_PGA5 = 0x040F, //!< PGA5 clock
    SYSCTL_PERIPH_CLK_PGA6 = 0x050F, //!< PGA6 clock
    SYSCTL_PERIPH_CLK_PGA7 = 0x060F, //!< PGA7 clock
    SYSCTL_PERIPH_CLK_DACA = 0x1010, //!< DAC_A clock
    SYSCTL_PERIPH_CLK_DACB = 0x1110, //!< DAC_B clock
    SYSCTL_PERIPH_CLK_CLB1 = 0x0011, //!< CLB1 clock
    SYSCTL_PERIPH_CLK_CLB2 = 0x0111, //!< CLB2 clock
    SYSCTL_PERIPH_CLK_CLB3 = 0x0211, //!< CLB3 clock
    SYSCTL_PERIPH_CLK_CLB4 = 0x0311, //!< CLB4 clock
    SYSCTL_PERIPH_CLK_FSITXA = 0x0012, //!< FSITX_A clock
    SYSCTL_PERIPH_CLK_FSIRXA = 0x0112, //!< FSIRX_A clock
    SYSCTL_PERIPH_CLK_LINA = 0x0013, //!< LIN_A clock
    SYSCTL_PERIPH_CLK_PMBUSA = 0x0014, //!< PMBUS_A clock
    SYSCTL_PERIPH_CLK_DCC0 = 0x0015  //!< DCC_0 clock
} SysCtl_PeripheralPCLOCKCR;

//*****************************************************************************
//
//! The following are values that can be passed to SysCtl_resetPeripheral() as
//! the \e peripheral parameter.
//
//*****************************************************************************
typedef enum
{
    SYSCTL_PERIPH_RES_CPU1CLA1 = 0x0000, //!< Reset CPU1_CLA1 clock
    SYSCTL_PERIPH_RES_EPWM1 = 0x0002, //!< Reset EPWM1 clock
    SYSCTL_PERIPH_RES_EPWM2 = 0x0102, //!< Reset EPWM2 clock
    SYSCTL_PERIPH_RES_EPWM3 = 0x0202, //!< Reset EPWM3 clock
    SYSCTL_PERIPH_RES_EPWM4 = 0x0302, //!< Reset EPWM4 clock
    SYSCTL_PERIPH_RES_EPWM5 = 0x0402, //!< Reset EPWM5 clock
    SYSCTL_PERIPH_RES_EPWM6 = 0x0502, //!< Reset EPWM6 clock
    SYSCTL_PERIPH_RES_EPWM7 = 0x0602, //!< Reset EPWM7 clock
    SYSCTL_PERIPH_RES_EPWM8 = 0x0702, //!< Reset EPWM8 clock
    SYSCTL_PERIPH_RES_ECAP1 = 0x0003, //!< Reset ECAP1 clock
    SYSCTL_PERIPH_RES_ECAP2 = 0x0103, //!< Reset ECAP2 clock
    SYSCTL_PERIPH_RES_ECAP3 = 0x0203, //!< Reset ECAP3 clock
    SYSCTL_PERIPH_RES_ECAP4 = 0x0303, //!< Reset ECAP4 clock
    SYSCTL_PERIPH_RES_ECAP5 = 0x0403, //!< Reset ECAP5 clock
    SYSCTL_PERIPH_RES_ECAP6 = 0x0503, //!< Reset ECAP6 clock
    SYSCTL_PERIPH_RES_ECAP7 = 0x0603, //!< Reset ECAP7 clock
    SYSCTL_PERIPH_RES_EQEP1 = 0x0004, //!< Reset EQEP1 clock
    SYSCTL_PERIPH_RES_EQEP2 = 0x0104, //!< Reset EQEP2 clock
    SYSCTL_PERIPH_RES_SD1 = 0x0006, //!< Reset SD1 clock
    SYSCTL_PERIPH_RES_SCIA = 0x0007, //!< Reset SCI_A clock
    SYSCTL_PERIPH_RES_SCIB = 0x0107, //!< Reset SCI_B clock
    SYSCTL_PERIPH_RES_SPIA = 0x0008, //!< Reset SPI_A clock
    SYSCTL_PERIPH_RES_SPIB = 0x0108, //!< Reset SPI_B clock
    SYSCTL_PERIPH_RES_I2CA = 0x0009, //!< Reset I2C_A clock
    SYSCTL_PERIPH_RES_CANA = 0x000A, //!< Reset CAN_A clock
    SYSCTL_PERIPH_RES_CANB = 0x010A, //!< Reset CAN_B clock
    SYSCTL_PERIPH_RES_ADCA = 0x000D, //!< Reset ADC_A clock
    SYSCTL_PERIPH_RES_ADCB = 0x010D, //!< Reset ADC_B clock
    SYSCTL_PERIPH_RES_ADCC = 0x020D, //!< Reset ADC_C clock
    SYSCTL_PERIPH_RES_CMPSS1 = 0x000E, //!< Reset CMPSS1 clock
    SYSCTL_PERIPH_RES_CMPSS2 = 0x010E, //!< Reset CMPSS2 clock
    SYSCTL_PERIPH_RES_CMPSS3 = 0x020E, //!< Reset CMPSS3 clock
    SYSCTL_PERIPH_RES_CMPSS4 = 0x030E, //!< Reset CMPSS4 clock
    SYSCTL_PERIPH_RES_CMPSS5 = 0x040E, //!< Reset CMPSS5 clock
    SYSCTL_PERIPH_RES_CMPSS6 = 0x050E, //!< Reset CMPSS6 clock
    SYSCTL_PERIPH_RES_CMPSS7 = 0x060E, //!< Reset CMPSS7 clock
    SYSCTL_PERIPH_RES_PGA1 = 0x000F, //!< Reset PGA1 clock
    SYSCTL_PERIPH_RES_PGA2 = 0x010F, //!< Reset PGA2 clock
    SYSCTL_PERIPH_RES_PGA3 = 0x020F, //!< Reset PGA3 clock
    SYSCTL_PERIPH_RES_PGA4 = 0x030F, //!< Reset PGA4 clock
    SYSCTL_PERIPH_RES_PGA5 = 0x040F, //!< Reset PGA5 clock
    SYSCTL_PERIPH_RES_PGA6 = 0x050F, //!< Reset PGA6 clock
    SYSCTL_PERIPH_RES_PGA7 = 0x060F, //!< Reset PGA7 clock
    SYSCTL_PERIPH_RES_DACA = 0x1010, //!< Reset DAC_A clock
    SYSCTL_PERIPH_RES_DACB = 0x1110, //!< Reset DAC_B clock
    SYSCTL_PERIPH_RES_CLB1 = 0x0011, //!< Reset CLB1 clock
    SYSCTL_PERIPH_RES_CLB2 = 0x0111, //!< Reset CLB2 clock
    SYSCTL_PERIPH_RES_CLB3 = 0x0211, //!< Reset CLB3 clock
    SYSCTL_PERIPH_RES_CLB4 = 0x0311, //!< Reset CLB4 clock
    SYSCTL_PERIPH_RES_LINA = 0x0013, //!< Reset LIN_A clock
    SYSCTL_PERIPH_RES_PMBUSA = 0x0014  //!< Reset PMBUS_A clock
} SysCtl_PeripheralSOFTPRES;

//*****************************************************************************
//
//! The following are values that can be passed to
//! SysCtl_setWatchdogPredivider() as the \e predivider parameter.
//
//*****************************************************************************
typedef enum
{
    SYSCTL_WD_PREDIV_2    = 0x800,  //!< PREDIVCLK = INTOSC1 / 2
    SYSCTL_WD_PREDIV_4    = 0x900,  //!< PREDIVCLK = INTOSC1 / 4
    SYSCTL_WD_PREDIV_8    = 0xA00,  //!< PREDIVCLK = INTOSC1 / 8
    SYSCTL_WD_PREDIV_16   = 0xB00,  //!< PREDIVCLK = INTOSC1 / 16
    SYSCTL_WD_PREDIV_32   = 0xC00,  //!< PREDIVCLK = INTOSC1 / 32
    SYSCTL_WD_PREDIV_64   = 0xD00,  //!< PREDIVCLK = INTOSC1 / 64
    SYSCTL_WD_PREDIV_128  = 0xE00,  //!< PREDIVCLK = INTOSC1 / 128
    SYSCTL_WD_PREDIV_256  = 0xF00,  //!< PREDIVCLK = INTOSC1 / 256
    SYSCTL_WD_PREDIV_512  = 0x000,  //!< PREDIVCLK = INTOSC1 / 512
    SYSCTL_WD_PREDIV_1024 = 0x100,  //!< PREDIVCLK = INTOSC1 / 1024
    SYSCTL_WD_PREDIV_2048 = 0x200,  //!< PREDIVCLK = INTOSC1 / 2048
    SYSCTL_WD_PREDIV_4096 = 0x300   //!< PREDIVCLK = INTOSC1 / 4096
} SysCtl_WDPredivider;

//*****************************************************************************
//
//! The following are values that can be passed to
//! SysCtl_setWatchdogPrescaler() as the \e prescaler parameter.
//
//*****************************************************************************
typedef enum
{
    SYSCTL_WD_PRESCALE_1  = 1,      //!< WDCLK = PREDIVCLK / 1
    SYSCTL_WD_PRESCALE_2  = 2,      //!< WDCLK = PREDIVCLK / 2
    SYSCTL_WD_PRESCALE_4  = 3,      //!< WDCLK = PREDIVCLK / 4
    SYSCTL_WD_PRESCALE_8  = 4,      //!< WDCLK = PREDIVCLK / 8
    SYSCTL_WD_PRESCALE_16 = 5,      //!< WDCLK = PREDIVCLK / 16
    SYSCTL_WD_PRESCALE_32 = 6,      //!< WDCLK = PREDIVCLK / 32
    SYSCTL_WD_PRESCALE_64 = 7       //!< WDCLK = PREDIVCLK / 64
} SysCtl_WDPrescaler;

//*****************************************************************************
//
//! The following are values that can be passed to
//! SysCtl_setWatchdogMode() as the \e prescaler parameter.
//
//*****************************************************************************
typedef enum
{
    //! Watchdog can generate a reset signal
    SYSCTL_WD_MODE_RESET,
    //! Watchdog can generate an interrupt signal; reset signal is disabled
    SYSCTL_WD_MODE_INTERRUPT
} SysCtl_WDMode;

//*****************************************************************************
//
//! The following are values that can be passed to SysCtl_setLowSpeedClock() as
//! the \e prescaler parameter.
//
//*****************************************************************************
typedef enum
{
    SYSCTL_LSPCLK_PRESCALE_1    = 0,    //!< LSPCLK = SYSCLK / 1
    SYSCTL_LSPCLK_PRESCALE_2    = 1,    //!< LSPCLK = SYSCLK / 2
    SYSCTL_LSPCLK_PRESCALE_4    = 2,    //!< LSPCLK = SYSCLK / 4 (default)
    SYSCTL_LSPCLK_PRESCALE_6    = 3,    //!< LSPCLK = SYSCLK / 6
    SYSCTL_LSPCLK_PRESCALE_8    = 4,    //!< LSPCLK = SYSCLK / 8
    SYSCTL_LSPCLK_PRESCALE_10   = 5,    //!< LSPCLK = SYSCLK / 10
    SYSCTL_LSPCLK_PRESCALE_12   = 6,    //!< LSPCLK = SYSCLK / 12
    SYSCTL_LSPCLK_PRESCALE_14   = 7     //!< LSPCLK = SYSCLK / 14
} SysCtl_LSPCLKPrescaler;

//*****************************************************************************
//
//! The following are values that can be passed to
//! SysCtl_setPeripheralAccessControl() and SysCtl_getPeripheralAccessControl()
//! as the \e peripheral parameter.
//
//*****************************************************************************
typedef enum
{
    SYSCTL_ACCESS_ADCA          = 0x0, //!< ADCA access
    SYSCTL_ACCESS_ADCB          = 0x2, //!< ADCB access
    SYSCTL_ACCESS_ADCC          = 0x4, //!< ADCC access
    SYSCTL_ACCESS_CMPSS1        = 0x10, //!< CMPSS1 access
    SYSCTL_ACCESS_CMPSS2        = 0x12, //!< CMPSS2 access
    SYSCTL_ACCESS_CMPSS3        = 0x14, //!< CMPSS3 access
    SYSCTL_ACCESS_CMPSS4        = 0x16, //!< CMPSS4 access
    SYSCTL_ACCESS_CMPSS5        = 0x18, //!< CMPSS5 access
    SYSCTL_ACCESS_CMPSS6        = 0x1A, //!< CMPSS6 access
    SYSCTL_ACCESS_CMPSS7        = 0x1C, //!< CMPSS7 access
    SYSCTL_ACCESS_DACA          = 0x28, //!< DACA access
    SYSCTL_ACCESS_DACB          = 0x2A, //!< DACB access
    SYSCTL_ACCESS_PGA1          = 0x38, //!< PGA1 access
    SYSCTL_ACCESS_PGA2          = 0x3A, //!< PGA2 access
    SYSCTL_ACCESS_PGA3          = 0x3C, //!< PGA3 access
    SYSCTL_ACCESS_PGA4          = 0x3E, //!< PGA4 access
    SYSCTL_ACCESS_PGA5          = 0x40, //!< PGA5 access
    SYSCTL_ACCESS_PGA6          = 0x42, //!< PGA6 access
    SYSCTL_ACCESS_PGA7          = 0x44, //!< PGA7 access
    SYSCTL_ACCESS_EPWM1         = 0x48, //!< EPWM1 access
    SYSCTL_ACCESS_EPWM2         = 0x4A, //!< EPWM2 access
    SYSCTL_ACCESS_EPWM3         = 0x4C, //!< EPWM3 access
    SYSCTL_ACCESS_EPWM4         = 0x4E, //!< EPWM4 access
    SYSCTL_ACCESS_EPWM5         = 0x50, //!< EPWM5 access
    SYSCTL_ACCESS_EPWM6         = 0x52, //!< EPWM6 access
    SYSCTL_ACCESS_EPWM7         = 0x54, //!< EPWM7 access
    SYSCTL_ACCESS_EPWM8         = 0x56, //!< EPWM8 access
    SYSCTL_ACCESS_EQEP1         = 0x70, //!< EQEP1 access
    SYSCTL_ACCESS_EQEP2         = 0x72, //!< EQEP2 access
    SYSCTL_ACCESS_ECAP1         = 0x80, //!< ECAP1 access
    SYSCTL_ACCESS_ECAP2         = 0x82, //!< ECAP2 access
    SYSCTL_ACCESS_ECAP3         = 0x84, //!< ECAP3 access
    SYSCTL_ACCESS_ECAP4         = 0x86, //!< ECAP4 access
    SYSCTL_ACCESS_ECAP5         = 0x88, //!< ECAP5 access
    SYSCTL_ACCESS_ECAP6         = 0x8A, //!< ECAP6 access
    SYSCTL_ACCESS_ECAP7         = 0x8C, //!< ECAP7 access
    SYSCTL_ACCESS_SDFM1         = 0xA8, //!< SDFM1 access
    SYSCTL_ACCESS_CLB1          = 0xB0, //!< CLB1 access
    SYSCTL_ACCESS_CLB2          = 0xB2, //!< CLB2 access
    SYSCTL_ACCESS_CLB3          = 0xB4, //!< CLB3 access
    SYSCTL_ACCESS_CLB4          = 0xB6, //!< CLB4 access
    SYSCTL_ACCESS_CLA1PROMCRC   = 0xC0, //!< CLA1PROMCRC access
    SYSCTL_ACCESS_SPIA          = 0x110, //!< SPIA access
    SYSCTL_ACCESS_SPIB          = 0x112, //!< SPIB access
    SYSCTL_ACCESS_PMBUSA       = 0x130, //!< PMBUS_A access
    SYSCTL_ACCESS_LINA         = 0x138, //!< LIN_A access
    SYSCTL_ACCESS_CANA         = 0x140, //!< DCANA access
    SYSCTL_ACCESS_CANB         = 0x142, //!< DCANB access
    SYSCTL_ACCESS_FSIATX        = 0x158, //!< FSIATX access
    SYSCTL_ACCESS_FSIARX        = 0x15A, //!< FSIARX access
    SYSCTL_ACCESS_HRPWMA       = 0x1AA  //!< HRPWM_A access
} SysCtl_AccessPeripheral;

//*****************************************************************************
//
//! The following are values that can be passed to
//! SysCtl_setPeripheralAccessControl() and SysCtl_getPeripheralAccessControl()
//! as the \e master parameter.
//
//*****************************************************************************
typedef enum
{
    SYSCTL_ACCESS_CPU1  = 0U,   //!< CPU access to the peripheral
    SYSCTL_ACCESS_CLA1  = 2U,   //!< CLA1 access to the peripheral
    SYSCTL_ACCESS_DMA1  = 4U    //!< DMA access to the peripheral
} SysCtl_AccessMaster;
//*****************************************************************************
//
//! The following are values that can be passed to
//! SysCtl_setPeripheralAccessControl() as the \e permission parameter.
//
//*****************************************************************************
typedef enum
{
    //! Full Access for both read and write.
    SYSCTL_ACCESS_FULL      = 3U,
    //! Protected RD access such that FIFOs. Clear on read, registers are not
    //! changed and no write access.
    SYSCTL_ACCESS_PROTECTED = 2U,
    //! No read or write access.
    SYSCTL_ACCESS_NONE      = 0U
} SysCtl_AccessPermission;

//*****************************************************************************
//
//! The following are values that can be passed to
//! SysCtl_selectClockOutSource() as the \e source parameter.
//
//*****************************************************************************
typedef enum
{
    SYSCTL_CLOCKOUT_PLLSYS     = 0U,   //!< PLL System Clock post SYSCLKDIV
    SYSCTL_CLOCKOUT_PLLRAW     = 1U,   //!< PLL Raw Clock
    SYSCTL_CLOCKOUT_SYSCLK     = 2U,   //!< CPU System Clock
    SYSCTL_CLOCKOUT_INTOSC1    = 5U,   //!< Internal Oscillator 1
    SYSCTL_CLOCKOUT_INTOSC2    = 6U,   //!< Internal Oscillator 2
    SYSCTL_CLOCKOUT_XTALOSC    = 7U,   //!< External Oscillator
} SysCtl_ClockOut;

//*****************************************************************************
//
//! The following are values that can be passed to
//! SysCtl_setExternalOscMode() as the \e mode parameter.
//
//*****************************************************************************
typedef enum
{
    SYSCTL_XTALMODE_CRYSTAL = 1U,   //!< XTAL Oscillator Crystal Mode
    SYSCTL_XTALMODE_SINGLE  = 2U    //!< XTAL Oscillator Single-Ended Mode
} SysCtl_ExternalOscMode;

//*****************************************************************************
//
//! The following values define the \e syncInput parameter for
//! SysCtl_setSyncInputConfig().
//
//*****************************************************************************
typedef enum
{
    SYSCTL_SYNC_IN_EPWM4 = 0, //!< Sync input to EPWM4
    SYSCTL_SYNC_IN_EPWM7 = 3, //!< Sync input to EPWM7
    SYSCTL_SYNC_IN_ECAP1 = 9, //!< Sync input to ECAP1
    SYSCTL_SYNC_IN_ECAP4 = 12, //!< Sync input to ECAP4
    SYSCTL_SYNC_IN_ECAP6 = 15, //!< Sync input to ECAP6
    SYSCTL_SYNC_IN_EPWM1 = 29  //!< Sync input to EPWM1
} SysCtl_SyncInput;

//*****************************************************************************
//
//! The following values define the \e syncSrc parameter for
//! SysCtl_setSyncInputConfig(). Note that some of these are only valid for
//! certain values of \e syncInput. See device technical reference manual for
//! info on time-base counter synchronization for details.
//
//*****************************************************************************
typedef enum
{
    //! EPWM1SYNCOUT
    SYSCTL_SYNC_IN_SRC_EPWM1SYNCOUT     = 0,
    //! EPWM4SYNCOUT
    SYSCTL_SYNC_IN_SRC_EPWM4SYNCOUT     = 1,
    //! EPWM7SYNCOUT
    SYSCTL_SYNC_IN_SRC_EPWM7SYNCOUT     = 2,
    //! ECAP1SYNCOUT
    SYSCTL_SYNC_IN_SRC_ECAP1SYNCOUT     = 4,
    //! EXTSYNCIN1--Valid for all values of syncInput
    SYSCTL_SYNC_IN_SRC_EXTSYNCIN1       = 5,
    //! EXTSYNCIN2--Valid for all values of syncInput
    SYSCTL_SYNC_IN_SRC_EXTSYNCIN2       = 6,
} SysCtl_SyncInputSource;

//*****************************************************************************
//
//! The following values define the \e syncSrc parameter for
//! SysCtl_setSyncOutputConfig().
//
//*****************************************************************************
typedef enum
{
    SYSCTL_SYNC_OUT_SRC_EPWM1SYNCOUT,   //!< EPWM1SYNCOUT --> EXTSYNCOUT
    SYSCTL_SYNC_OUT_SRC_EPWM4SYNCOUT,   //!< EPWM4SYNCOUT --> EXTSYNCOUT
    SYSCTL_SYNC_OUT_SRC_EPWM7SYNCOUT    //!< EPWM7SYNCOUT --> EXTSYNCOUT

} SysCtl_SyncOutputSource;

//*****************************************************************************
//
//! The following values define the \e parametric parameter for
//! SysCtl_getDeviceParametric().
//
//*****************************************************************************
typedef enum
{
    SYSCTL_DEVICE_QUAL,       //!< Device Qualification Status
    SYSCTL_DEVICE_PINCOUNT,   //!< Device Pin Count
    SYSCTL_DEVICE_INSTASPIN,  //!< Device InstaSPIN Feature Set
    SYSCTL_DEVICE_FLASH,      //!< Device Flash size (KB)
    SYSCTL_DEVICE_FAMILY,     //!< Device Family
    SYSCTL_DEVICE_PARTNO,     //!< Device Part Number
    SYSCTL_DEVICE_CLASSID     //!< Device Class ID
} SysCtl_DeviceParametric;

//*****************************************************************************
//
//! The following are values that can be passed to
//! SysCtl_setXClk() as \e divider parameter.
//
//*****************************************************************************
typedef enum
{
    SYSCTL_XCLKOUT_DIV_1  = 0,      //!<  XCLKOUT =  XCLKOUT / 1
    SYSCTL_XCLKOUT_DIV_2  = 1,      //!<  XCLKOUT =  XCLKOUT / 2
    SYSCTL_XCLKOUT_DIV_4  = 2,      //!<  XCLKOUT =  XCLKOUT / 4
    SYSCTL_XCLKOUT_DIV_8  = 3       //!<  XCLKOUT =  XCLKOUT / 8

}SysCtl_XClkDivider;

//*****************************************************************************
//
//! The following are values that can be passed to
//! SysCtl_setCputimer2Clk() as \e divider parameter.
//
//*****************************************************************************
typedef enum
{
    SYSCTL_TMR2CLKPRESCALE_1,   //!<  Cputimer2 clock =  Cputimer2 clock / 1
    SYSCTL_TMR2CLKPRESCALE_2,   //!<  Cputimer2 clock =  Cputimer2 clock / 2
    SYSCTL_TMR2CLKPRESCALE_4,   //!<  Cputimer2 clock =  Cputimer2 clock / 4
    SYSCTL_TMR2CLKPRESCALE_8,   //!<  Cputimer2 clock =  Cputimer2 clock / 8
    SYSCTL_TMR2CLKPRESCALE_16   //!<  Cputimer2 clock =  Cputimer2 clock / 16

}SysCtl_Cputimer2ClkDivider;

//*****************************************************************************
//
//! The following are values that can be passed to SysCtl_setCputimer2Clk()
//! as \e source parameter.
//
//*****************************************************************************
typedef enum
{
    SYSCTL_TMR2CLKSRCSEL_SYSCLK     = 0U,   //!< System Clock
    SYSCTL_TMR2CLKSRCSEL_INTOSC1    = 1U,   //!< Internal Oscillator 1
    SYSCTL_TMR2CLKSRCSEL_INTOSC2    = 2U,   //!< Internal Oscillator 2
    SYSCTL_TMR2CLKSRCSEL_XTAL       = 3U,   //!< Crystal oscillator
    SYSCTL_TMR2CLKSRCSEL_AUXPLLCLK  = 6U    //!< Aux PLL CLock

}SysCtl_Cputimer2ClkSource;


//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
//*****************************************************************************
//
//! Wrapper function for Device_cal function
//!
//! \param None
//!
//! This is a wrapper function for the Device_cal function available in the OTP
//! memory. The function saves and restores the core registers which are being
//! used by the Device_cal function
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_deviceCal(void)
{
    //
    // Save the core registers used by Device_cal function in the stack
    //
    asm(" PUSH ACC");
    asm(" PUSH DP");
    asm(" PUSH XAR0");
    asm(" PUSH XAR2");
    asm(" PUSH XAR3");
    asm(" PUSH XAR4");
    asm(" PUSH XAR5");

    //
    // Call the Device_cal function
    //
    Device_cal();

    //
    // Restore the core registers
    //
    asm(" POP XAR5");
    asm(" POP XAR4");
    asm(" POP XAR3");
    asm(" POP XAR2");
    asm(" POP XAR0");
    asm(" POP DP");
    asm(" POP ACC");
}

//*****************************************************************************
//
//! Resets a peripheral
//!
//! \param peripheral is the peripheral to reset.
//!
//! This function uses the SOFTPRESx registers to reset a specified peripheral.
//! Module registers will be returned to their reset states.
//!
//! \note This includes registers containing trim values.The peripheral
//! software reset needed by CPU2 can be communicated to CPU1 via
//! IPC for all shared peripherals.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_resetPeripheral(SysCtl_PeripheralSOFTPRES peripheral)
{
    uint16_t regIndex;
    uint16_t bitIndex;

    //
    // Decode the peripheral variable.
    //
    regIndex = (uint16_t)2U * ((uint16_t)peripheral &
                               (uint16_t)SYSCTL_PERIPH_REG_M);
    bitIndex = ((uint16_t)peripheral & SYSCTL_PERIPH_BIT_M) >>
               SYSCTL_PERIPH_BIT_S;

    EALLOW;

    //
    // Sets the appropriate reset bit and then clears it.
    //
    HWREG(DEVCFG_BASE + SYSCTL_O_SOFTPRES0 + regIndex) |=
        ((uint32_t)1U << bitIndex);
    HWREG(DEVCFG_BASE + SYSCTL_O_SOFTPRES0 + regIndex) &=
        ~((uint32_t)1U << bitIndex);

    //
    // Call Device_cal function
    //
    if(((peripheral & SYSCTL_PERIPH_REG_M) == 0xDU) ||      // ADCx
       ((peripheral & SYSCTL_PERIPH_REG_M) == 0xFU) ||      // PGAx
       ((peripheral & SYSCTL_PERIPH_REG_M) == 0x10U)        // DACx
       )
    {
        SysCtl_deviceCal();
    }

    EDIS;
}

//*****************************************************************************
//
//! Enables a peripheral.
//!
//! \param peripheral is the peripheral to enable.
//!
//! Peripherals are enabled with this function.  At power-up, all peripherals
//! are disabled; they must be enabled in order to operate or respond to
//! register reads/writes.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_enablePeripheral(SysCtl_PeripheralPCLOCKCR peripheral)
{
    uint16_t regIndex;
    uint16_t bitIndex;

    //
    // Decode the peripheral variable.
    //
    regIndex = (uint16_t)2U * ((uint16_t)peripheral &
                               (uint16_t)SYSCTL_PERIPH_REG_M);
    bitIndex = ((uint16_t)peripheral & SYSCTL_PERIPH_BIT_M) >>
               SYSCTL_PERIPH_BIT_S;

    EALLOW;

    //
    // Turn on the module clock.
    //
    HWREG(CPUSYS_BASE + SYSCTL_O_PCLKCR0 + regIndex) |=
        ((uint32_t)1U << bitIndex);
    EDIS;
}

//*****************************************************************************
//
//! Disables a peripheral.
//!
//! \param peripheral is the peripheral to disable.
//!
//! Peripherals are disabled with this function.  Once disabled, they will not
//! operate or respond to register reads/writes.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_disablePeripheral(SysCtl_PeripheralPCLOCKCR peripheral)
{
    uint16_t regIndex;
    uint16_t bitIndex;

    //
    // Decode the peripheral variable.
    //
    regIndex = (uint16_t)2U * ((uint16_t)peripheral &
                               (uint16_t)SYSCTL_PERIPH_REG_M);
    bitIndex = ((uint16_t)peripheral & SYSCTL_PERIPH_BIT_M) >>
               SYSCTL_PERIPH_BIT_S;

    EALLOW;

    //
    // Turn off the module clock.
    //
    HWREG(CPUSYS_BASE + SYSCTL_O_PCLKCR0 + regIndex) &=
        ~((uint32_t)1U << bitIndex);
    EDIS;
}

//*****************************************************************************
//
//! Resets the device.
//!
//! This function performs a watchdog reset of the device.
//!
//! \return This function does not return.
//
//*****************************************************************************
static inline void
SysCtl_resetDevice(void)
{
    //
    // Write an incorrect check value to the watchdog control register
    // This will cause a device reset
    //
    EALLOW;

    //
    // Enable the watchdog
    //
    HWREGH(WD_BASE + SYSCTL_O_WDCR) = SYSCTL_WD_CHKBITS;

    //
    // Write a bad check value
    //
    HWREGH(WD_BASE + SYSCTL_O_WDCR) = 0U;

    EDIS;

    //
    // The device should have reset, so this should never be reached.  Just in
    // case, loop forever.
    //
    while(1)
    {
    }
}

//*****************************************************************************
//
//! Gets the reason for a reset.
//!
//! This function will return the reason(s) for a reset.  Since the reset
//! reasons are sticky until either cleared by software or an external reset,
//! multiple reset reasons may be returned if multiple resets have occurred.
//! The reset reason will be a logical OR of
//! - \b SYSCTL_CAUSE_POR - Power-on reset
//! - \b SYSCTL_CAUSE_XRS - External reset pin
//! - \b SYSCTL_CAUSE_WDRS - Watchdog reset
//! - \b SYSCTL_CAUSE_NMIWDRS - NMI watchdog reset
//! - \b SYSCTL_CAUSE_SCCRESET - SCCRESETn reset from DCSM
//!
//! \note If you re-purpose the reserved boot ROM RAM, the POR and XRS reset
//! statuses won't be accurate.
//!
//! \return Returns the reason(s) for a reset.
//
//*****************************************************************************
static inline uint32_t
SysCtl_getResetCause(void)
{
    uint32_t resetCauses;

    //
    // Read CPU reset register
    //
    resetCauses = HWREG(CPUSYS_BASE + SYSCTL_O_RESC) &
                  ((uint32_t)SYSCTL_RESC_POR | (uint32_t)SYSCTL_RESC_XRSN |
                   (uint32_t)SYSCTL_RESC_WDRSN |
                   (uint32_t)SYSCTL_RESC_NMIWDRSN |
                   (uint32_t)SYSCTL_RESC_SCCRESETN
                   );

    //
    // Set POR and XRS Causes from boot ROM Status
    //
    if((HWREG(SYSCTL_BOOT_ROM_STATUS) & (uint32_t)SYSCTL_BOOT_ROM_POR) ==
       (uint32_t)SYSCTL_BOOT_ROM_POR)
    {
        resetCauses |= SYSCTL_RESC_POR;
    }
    if((HWREG(SYSCTL_BOOT_ROM_STATUS) & (uint32_t)SYSCTL_BOOT_ROM_XRS) ==
       (uint32_t)SYSCTL_BOOT_ROM_XRS)
    {
        resetCauses |= SYSCTL_RESC_XRSN;
    }

    //
    // Return the reset reasons.
    //
    return(resetCauses);
}

//*****************************************************************************
//
//! Clears reset reasons.
//!
//! \param rstCauses are the reset causes to be cleared; must be a logical
//! OR of \b SYSCTL_CAUSE_POR, \b SYSCTL_CAUSE_XRS, \b SYSCTL_CAUSE_WDRS,
//! \b SYSCTL_CAUSE_NMIWDRS,
//! and/or \b SYSCTL_CAUSE_SCCRESET.
//!
//! This function clears the specified sticky reset reasons.  Once cleared,
//! another reset for the same reason can be detected, and a reset for a
//! different reason can be distinguished (instead of having two reset causes
//! set).  If the reset reason is used by an application, all reset causes
//! should be cleared after they are retrieved with SysCtl_getResetCause().
//!
//! \note Some reset causes are cleared by the boot ROM.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_clearResetCause(uint32_t rstCauses)
{
    //
    // Clear the given reset reasons.
    //
    HWREG(CPUSYS_BASE + SYSCTL_O_RESCCLR) = rstCauses;
}

//*****************************************************************************
//
//! Sets the low speed peripheral clock rate prescaler.
//!
//! \param prescaler is the LSPCLK rate relative to SYSCLK
//!
//! This function configures the clock rate of the low speed peripherals. The
//! \e prescaler parameter is the value by which the SYSCLK rate is divided to
//! get the LSPCLK rate. For example, a \e prescaler of
//! \b SYSCTL_LSPCLK_PRESCALE_4 will result in a LSPCLK rate that is a quarter
//! of the SYSCLK rate.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_setLowSpeedClock(SysCtl_LSPCLKPrescaler prescaler)
{
    //
    // Write the divider selection to the appropriate register.
    //
    EALLOW;
    HWREG(CLKCFG_BASE + SYSCTL_O_LOSPCP) =
        (HWREG(CLKCFG_BASE + SYSCTL_O_LOSPCP) & ~SYSCTL_LOSPCP_LSPCLKDIV_M) |
        (uint32_t)prescaler;
    EDIS;
}

//*****************************************************************************
//
//! Selects a clock source to mux to an external GPIO pin (XCLKOUT).
//!
//! \param source is the internal clock source to be configured.
//!
//! This function configures the specified clock source to be muxed to an
//! external clock out (XCLKOUT) GPIO pin. The \e source parameter may take a
//! value of one of the following values:
//! - \b SYSCTL_CLOCKOUT_PLLSYS
//! - \b SYSCTL_CLOCKOUT_PLLRAW
//! - \b SYSCTL_CLOCKOUT_SYSCLK
//! - \b SYSCTL_CLOCKOUT_INTOSC1
//! - \b SYSCTL_CLOCKOUT_INTOSC2
//! - \b SYSCTL_CLOCKOUT_XTALOSC
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_selectClockOutSource(SysCtl_ClockOut source)
{
    EALLOW;

    //
    // Clear clock out source
    //
    HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL3) &=
        ~SYSCTL_CLKSRCCTL3_XCLKOUTSEL_M;

    //
    // Set clock out source
    //
    HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL3) |= (uint16_t)source;

    EDIS;
}

//*****************************************************************************
//
//! Set the external oscillator mode.
//!
//! \param mode is the external oscillator mode to be configured.
//!
//! This function sets the external oscillator mode specified by the \e mode
//! parameter which may take one of two values:
//! - \b SYSCTL_XTALMODE_CRYSTAL - Crystal Mode
//! - \b SYSCTL_XTALMODE_SINGLE  - Single-Ended Mode
//!
//! \note The external oscillator must be powered off before this configuration
//! can be performed.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_setExternalOscMode(SysCtl_ExternalOscMode mode)
{
    EALLOW;

    switch(mode)
    {
        case SYSCTL_XTALMODE_CRYSTAL:
            //
            // Set mode to Crystal
            //
            HWREG(CLKCFG_BASE + SYSCTL_O_XTALCR) &= ~SYSCTL_XTALCR_SE;
            break;

        case SYSCTL_XTALMODE_SINGLE:
            //
            // Set mode to Single-Ended
            //
            HWREG(CLKCFG_BASE + SYSCTL_O_XTALCR) |= SYSCTL_XTALCR_SE;
            break;

        default:
            //
            // Do nothing. Not a valid mode value.
            //
            break;
    }

    EDIS;
}

//*****************************************************************************
//
//! Gets the external oscillator counter value.
//!
//! This function returns the X1 clock counter value. When the return value
//! reaches 0x3FF, it freezes. Before switching from INTOSC2 to an external
//! oscillator (XTAL), an application should call this function to make sure
//! the counter is saturated.
//!
//! \return Returns the value of the 10-bit X1 clock counter.
//
//*****************************************************************************
static inline uint16_t
SysCtl_getExternalOscCounterValue(void)
{
    return(HWREGH(CLKCFG_BASE + SYSCTL_O_X1CNT) & SYSCTL_X1CNT_X1CNT_M);
}

//*****************************************************************************
//
//! Clears the external oscillator counter value.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_clearExternalOscCounterValue(void)
{
    HWREG(CLKCFG_BASE + SYSCTL_O_X1CNT) |= SYSCTL_X1CNT_CLR;
}

//*****************************************************************************
//
//! Turns on the specified oscillator sources.
//!
//! \param oscSource is the oscillator source to be configured.
//!
//! This function turns on the oscillator specified by the \e oscSource
//! parameter which may take a value of  \b SYSCTL_OSCSRC_XTAL
//! or \b SYSCTL_OSCSRC_OSC2.
//!
//! \note \b SYSCTL_OSCSRC_OSC1 is not a valid value for \e oscSource.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_turnOnOsc(uint32_t oscSource)
{
    ASSERT(
           (oscSource == SYSCTL_OSCSRC_OSC2) ||
           (oscSource == SYSCTL_OSCSRC_XTAL)
          );

    EALLOW;

    switch(oscSource)
    {
        case SYSCTL_OSCSRC_OSC2:
            //
            // Turn on INTOSC2
            //
            HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL1) &=
                ~SYSCTL_CLKSRCCTL1_INTOSC2OFF;
            break;

        case SYSCTL_OSCSRC_XTAL:
            //
            // Turn on XTALOSC
            //
            HWREGH(CLKCFG_BASE + SYSCTL_O_XTALCR) &= ~SYSCTL_XTALCR_OSCOFF;

            break;

        default:
            //
            // Do nothing. Not a valid oscSource value.
            //
            break;
    }

    EDIS;
}

//*****************************************************************************
//
//! Turns off the specified oscillator sources.
//!
//! \param oscSource is the oscillator source to be configured.
//!
//! This function turns off the oscillator specified by the \e oscSource
//! parameter which may take a value of  \b SYSCTL_OSCSRC_XTAL
//! or \b SYSCTL_OSCSRC_OSC2.
//!
//! \note \b SYSCTL_OSCSRC_OSC1 is not a valid value for \e oscSource.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_turnOffOsc(uint32_t oscSource)
{
    ASSERT(
           (oscSource == SYSCTL_OSCSRC_OSC2) |
           (oscSource == SYSCTL_OSCSRC_XTAL)
          );

    EALLOW;

    switch(oscSource)
    {
        case SYSCTL_OSCSRC_OSC2:
            //
            // Turn off INTOSC2
            //
            HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL1) |=
                SYSCTL_CLKSRCCTL1_INTOSC2OFF;
            break;

        case SYSCTL_OSCSRC_XTAL:
            //
            // Turn off XTALOSC
            //
            HWREGH(CLKCFG_BASE + SYSCTL_O_XTALCR) |= SYSCTL_XTALCR_OSCOFF;
            break;

        default:
            //
            // Do nothing. Not a valid oscSource value.
            //
            break;
    }

    EDIS;
}

//*****************************************************************************
//
//! Enters IDLE mode.
//!
//! This function puts the device into IDLE mode. The CPU clock is gated while
//! all peripheral clocks are left running. Any enabled interrupt will wake the
//! CPU up from IDLE mode.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_enterIdleMode(void)
{
    EALLOW;

    //
    // Configure the device to go into IDLE mode when IDLE is executed.
    //
    HWREG(CPUSYS_BASE + SYSCTL_O_LPMCR) =
                (HWREG(CPUSYS_BASE + SYSCTL_O_LPMCR) & ~SYSCTL_LPMCR_LPM_M) |
                SYSCTL_LPM_IDLE;

    EDIS;

#ifndef _DUAL_HEADERS
    IDLE;
#else
    IDLE_ASM;
#endif
}

//*****************************************************************************
//
//! Enters HALT mode.
//!
//! This function puts the device into HALT mode. This will gate almost all
//! systems and clocks and allows for the power-down of oscillators and analog
//! blocks. The watchdog may be left clocked to produce a reset. See
//! SysCtl_enableWatchdogInHalt() to enable this. GPIOs should be
//! configured to wake the CPU subsystem. See SysCtl_enableLPMWakeupPin().
//!
//!
//! The CPU will receive an interrupt (WAKEINT) on wakeup.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_enterHaltMode(void)
{
    EALLOW;

    //
    // Configure the device to go into HALT mode when IDLE is executed.
    //
    HWREG(CPUSYS_BASE + SYSCTL_O_LPMCR) =
                (HWREG(CPUSYS_BASE + SYSCTL_O_LPMCR) & ~SYSCTL_LPMCR_LPM_M) |
                SYSCTL_LPM_HALT;

    HWREGH(CLKCFG_BASE + SYSCTL_O_SYSPLLCTL1) &=
                ~(SYSCTL_SYSPLLCTL1_PLLCLKEN | SYSCTL_SYSPLLCTL1_PLLEN);

    EDIS;

#ifndef _DUAL_HEADERS
    IDLE;
#else
    IDLE_ASM;
#endif
}

//*****************************************************************************
//! Enables a pin to wake up the device from the following mode(s):
//!  - HALT
//!
//! \param pin is the identifying number of the pin.
//!
//! This function connects a pin to the LPM circuit, allowing an event on the
//! pin to wake up the device when when it is in following mode(s):
//!  - HALT
//!
//! The pin is specified by its numerical value. For example, GPIO34 is
//! specified by passing 34 as \e pin. Only GPIOs 0 through 63 are capable of
//! being connected to the LPM circuit.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_enableLPMWakeupPin(uint32_t pin)
{
    uint32_t pinMask;

    //
    // Check the arguments.
    //
    ASSERT(pin <= 63U);

    pinMask = (uint32_t)1U << (uint32_t)(pin % 32U);

    EALLOW;

    if(pin < 32U)
    {
        HWREG(CPUSYS_BASE + SYSCTL_O_GPIOLPMSEL0) |= pinMask;
    }
    else
    {
        HWREG(CPUSYS_BASE + SYSCTL_O_GPIOLPMSEL1) |= pinMask;
    }

    EDIS;
}

//*****************************************************************************
//! Disables a pin to wake up the device from the following mode(s):
//!  - HALT
//!
//! \param pin is the identifying number of the pin.
//!
//! This function disconnects a pin to the LPM circuit, disallowing an event on
//! the pin to wake up the device when when it is in following mode(s):
//!  - HALT
//!
//! The pin is specified by its numerical value. For example, GPIO34 is
//! specified by passing 34 as \e pin. Only GPIOs 0 through 63 are valid.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_disableLPMWakeupPin(uint32_t pin)
{
    uint32_t pinMask;

    //
    // Check the arguments.
    //
    ASSERT(pin <= 63U);

    pinMask = (uint32_t)1U << (uint32_t)(pin % 32U);

    EALLOW;

    if(pin < 32U)
    {
        HWREG(CPUSYS_BASE + SYSCTL_O_GPIOLPMSEL0) &= ~pinMask;
    }
    else
    {
        HWREG(CPUSYS_BASE + SYSCTL_O_GPIOLPMSEL1) &= ~pinMask;
    }

    EDIS;
}

//*****************************************************************************
//
//! Enable the watchdog to run while in HALT mode.
//!
//! This function configures the watchdog to continue to run while in HALT
//! mode. Additionally, INTOSC1 and INTOSC2 are not powered down when the
//! system enters HALT mode. By default the watchdog is gated when the system
//! enters HALT.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_enableWatchdogInHalt(void)
{
    EALLOW;

    //
    // Set the watchdog HALT mode ignore bit.
    //
    HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL1) |= SYSCTL_CLKSRCCTL1_WDHALTI;

    EDIS;
}

//*****************************************************************************
//
//! Disable the watchdog from running while in HALT mode.
//!
//! This function gates the watchdog when the system enters HALT mode. INTOSC1
//! and INTOSC2 will be powered down. This is the default behavior of the
//! device.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_disableWatchdogInHalt(void)
{
    EALLOW;

    //
    // Clear the watchdog HALT mode ignore bit.
    //
    HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL1) &= ~SYSCTL_CLKSRCCTL1_WDHALTI;

    EDIS;
}

//*****************************************************************************
//
//! Configures whether the watchdog generates a reset or an interrupt signal.
//!
//! \param mode is a flag to select the watchdog mode.
//!
//! This function configures the action taken when the watchdog counter reaches
//! its maximum value. When the \e mode parameter is
//! \b SYSCTL_WD_MODE_INTERRUPT, the watchdog is enabled to generate a watchdog
//! interrupt signal and disables the generation of a reset signal. This will
//! allow the watchdog module to wake up the device from IDLE
//!
//! When the \e mode parameter is \b SYSCTL_WD_MODE_RESET, the watchdog will
//! be put into reset mode and generation of a watchdog interrupt signal will
//! be disabled. This is how the watchdog is configured by default.
//!
//! \note Check the status of the watchdog interrupt using
//! SysCtl_isWatchdogInterruptActive() before calling this function. If the
//! interrupt is still active, switching from interrupt mode to reset mode will
//! immediately reset the device.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_setWatchdogMode(SysCtl_WDMode mode)
{
    EALLOW;

    //
    // Either set or clear the WDENINT bit to that will determine whether the
    // watchdog will generate a reset signal or an interrupt signal. Take care
    // not to write a 1 to WDOVERRIDE.
    //
    if(mode == SYSCTL_WD_MODE_INTERRUPT)
    {
        HWREGH(WD_BASE + SYSCTL_O_SCSR) =
                (HWREGH(WD_BASE + SYSCTL_O_SCSR) & ~SYSCTL_SCSR_WDOVERRIDE) |
                SYSCTL_SCSR_WDENINT;
    }
    else
    {
        HWREGH(WD_BASE + SYSCTL_O_SCSR) &= ~(SYSCTL_SCSR_WDENINT |
                                             SYSCTL_SCSR_WDOVERRIDE);
    }

    EDIS;
}

//*****************************************************************************
//
//! Gets the status of the watchdog interrupt signal.
//!
//! This function returns the status of the watchdog interrupt signal. If the
//! interrupt is active, this function will return \b true. If \b false, the
//! interrupt is NOT active.
//!
//! \note Make sure to call this function to ensure that the interrupt is not
//! active before making any changes to the configuration of the watchdog to
//! prevent any unexpected behavior. For instance, switching from interrupt
//! mode to reset mode while the interrupt is active will immediately reset the
//! device.
//!
//! \return \b true if the interrupt is active and \b false if it is not.
//
//*****************************************************************************
static inline bool
SysCtl_isWatchdogInterruptActive(void)
{
    //
    // If the status bit is cleared, the WDINTn signal is active.
    //
    return((HWREGH(WD_BASE + SYSCTL_O_SCSR) & SYSCTL_SCSR_WDINTS) == 0U);
}

//*****************************************************************************
//
//! Disables the watchdog.
//!
//! This function disables the watchdog timer. Note that the watchdog timer is
//! enabled on reset.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_disableWatchdog(void)
{
    EALLOW;

    //
    // Set the disable bit.
    //
    HWREGH(WD_BASE + SYSCTL_O_WDCR) |= SYSCTL_WD_CHKBITS | SYSCTL_WDCR_WDDIS;

    EDIS;
}

//*****************************************************************************
//
//! Enables the watchdog.
//!
//! This function enables the watchdog timer. Note that the watchdog timer is
//! enabled on reset.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_enableWatchdog(void)
{
    EALLOW;

    //
    // Clear the disable bit.
    //
    HWREGH(WD_BASE + SYSCTL_O_WDCR) = (HWREGH(WD_BASE + SYSCTL_O_WDCR) &
                                       ~SYSCTL_WDCR_WDDIS) | SYSCTL_WD_CHKBITS;

    EDIS;
}

//*****************************************************************************
//
//! Services the watchdog.
//!
//! This function resets the watchdog.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_serviceWatchdog(void)
{
    EALLOW;

    //
    // Enable the counter to be reset and then reset it.
    //
    HWREGH(WD_BASE + SYSCTL_O_WDKEY) = SYSCTL_WD_ENRSTKEY;
    HWREGH(WD_BASE + SYSCTL_O_WDKEY) = SYSCTL_WD_RSTKEY;

    EDIS;
}

//*****************************************************************************
//
//! Writes the first key to enter the watchdog reset.
//!
//! This function writes the first key to enter the watchdog reset.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_enableWatchdogReset(void)
{
    EALLOW;

    //
    // Enable the counter to be reset
    //
    HWREGH(WD_BASE + SYSCTL_O_WDKEY) = SYSCTL_WD_ENRSTKEY;

    EDIS;
}

//*****************************************************************************
//
//! Writes the second key to reset the watchdog.
//!
//! This function writes the second key to reset the watchdog.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_resetWatchdog(void)
{
    EALLOW;

    //
    // Reset the watchdog counter
    //
    HWREGH(WD_BASE + SYSCTL_O_WDKEY) = SYSCTL_WD_RSTKEY;
    EDIS;
}

//*****************************************************************************
//
//! Sets up watchdog clock (WDCLK) pre-divider.
//!
//! \param predivider is the value that configures the pre-divider.
//!
//! This function sets up the watchdog clock (WDCLK) pre-divider. There are two
//! dividers that scale INTOSC1 to WDCLK. The \e predivider parameter divides
//! INTOSC1 down to PREDIVCLK and the prescaler (set by the
//! SysCtl_setWatchdogPrescaler() function) divides PREDIVCLK down to WDCLK.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_setWatchdogPredivider(SysCtl_WDPredivider predivider)
{
    uint16_t regVal;

    regVal = (uint16_t)predivider | (uint16_t)SYSCTL_WD_CHKBITS;

    EALLOW;

    //
    // Write the predivider to the appropriate register.
    //
    HWREGH(WD_BASE + SYSCTL_O_WDCR) = (HWREGH(WD_BASE + SYSCTL_O_WDCR) &
                                       ~(SYSCTL_WDCR_WDPRECLKDIV_M)) | regVal;

    EDIS;
}

//*****************************************************************************
//
//! Sets up watchdog clock (WDCLK) prescaler.
//!
//! \param prescaler is the value that configures the watchdog clock relative
//! to the value from the pre-divider.
//!
//! This function sets up the watchdog clock (WDCLK) prescaler. There are two
//! dividers that scale INTOSC1 to WDCLK. The predivider (set with the
//! SysCtl_setWatchdogPredivider() function) divides INTOSC1 down to PREDIVCLK
//! and the \e prescaler parameter divides PREDIVCLK down to WDCLK.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_setWatchdogPrescaler(SysCtl_WDPrescaler prescaler)
{
    uint16_t regVal;

    regVal = (uint16_t)prescaler | (uint16_t)SYSCTL_WD_CHKBITS;

    EALLOW;

    //
    // Write the prescaler to the appropriate register.
    //
    HWREGH(WD_BASE + SYSCTL_O_WDCR) = (HWREGH(WD_BASE + SYSCTL_O_WDCR) &
                                       ~(SYSCTL_WDCR_WDPS_M)) | regVal;

    EDIS;
}

//*****************************************************************************
//
//! Gets the watchdog counter value.
//!
//! \return Returns the current value of the 8-bit watchdog counter. If this
//! count value overflows, a watchdog output pulse is generated.
//
//*****************************************************************************
static inline uint16_t
SysCtl_getWatchdogCounterValue(void)
{
    //
    // Read and return the value of the watchdog counter.
    //
    return(HWREGH(WD_BASE + SYSCTL_O_WDCNTR));
}

//*****************************************************************************
//
//! Gets the watchdog reset status.
//!
//! This function returns the watchdog reset status. If this function returns
//! \b true, that indicates that a watchdog reset generated the last reset
//! condition. Otherwise, it was an external device or power-up reset
//! condition.
//!
//! \return Returns \b true if the watchdog generated the last reset condition.
//
//*****************************************************************************
static inline bool
SysCtl_getWatchdogResetStatus(void)
{
    //
    // Read and return the status of the watchdog reset status flag.
    //
    return((HWREGH(CPUSYS_BASE + SYSCTL_O_RESC) & SYSCTL_RESC_WDRSN) != 0U);
}

//*****************************************************************************
//
//! Clears the watchdog reset status.
//!
//! This function clears the watchdog reset status. To check if it was set
//! first, see SysCtl_getWatchdogResetStatus().
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_clearWatchdogResetStatus(void)
{
    EALLOW;

    //
    // Read and return the status of the watchdog reset status flag.
    //
    HWREGH(CPUSYS_BASE + SYSCTL_O_RESCCLR) = SYSCTL_RESCCLR_WDRSN;

    EDIS;
}

//*****************************************************************************
//
//! Set the minimum threshold value for windowed watchdog
//!
//! \param value is the value to set the window threshold
//!
//! This function sets the minimum threshold value used to define the lower
//! limit of the windowed watchdog functionality.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_setWatchdogWindowValue(uint16_t value)
{
    EALLOW;

    //
    // Clear the windowed value
    //
    HWREGH(WD_BASE + SYSCTL_O_WDWCR) &= ~SYSCTL_WDWCR_MIN_M;

    //
    // Set the windowed value
    //
    HWREGH(WD_BASE + SYSCTL_O_WDWCR) |= (value & SYSCTL_WDWCR_MIN_M);

    EDIS;
}

//*****************************************************************************
//
//! Clears the watchdog override.
//!
//! This function clears the watchdog override and locks the watchdog timer
//! module to remain in its prior state which could be either enable /disable.
//! The watchdog timer will remain in this state until the next system reset.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_clearWatchdogOverride(void)
{
    EALLOW;

    HWREGH(WD_BASE + SYSCTL_O_SCSR) |= SYSCTL_SCSR_WDOVERRIDE;

    EDIS;
}

//*****************************************************************************
//
//! Enable the NMI Global interrupt bit
//!
//! \b Note: This bit should be set after the device security related
//! initialization is complete.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_enableNMIGlobalInterrupt(void)
{
    EALLOW;

    HWREGH(NMI_BASE + NMI_O_CFG) |= NMI_CFG_NMIE;

    EDIS;
}

//*****************************************************************************
//
//! Read NMI interrupts.
//!
//! Read the current state of NMI interrupt.
//!
//! \return \b true if NMI interrupt is triggered, \b false if not.
//
//*****************************************************************************
static inline bool
SysCtl_getNMIStatus(void)
{
    //
    // Read and return the current value of the NMI flag register, masking out
    // all but the NMI bit.
    //
    return((HWREGH(NMI_BASE + NMI_O_FLG) & NMI_FLG_NMIINT) != 0U);
}

//*****************************************************************************
//
//! Read NMI Flags.
//!
//! Read the current state of individual NMI interrupts
//!
//! \return Value of NMIFLG register. These defines are provided to decode
//! the value:
//! - \b SYSCTL_NMI_NMIINT             -  NMI Interrupt Flag
//! - \b SYSCTL_NMI_CLOCKFAIL          -  Clock Fail Interrupt Flag
//! - \b SYSCTL_NMI_RAMUNCERR          -  RAM Uncorrectable Error NMI
//! - \b SYSCTL_NMI_FLUNCERR           -  Flash Uncorrectable Error NMI
//! - \b SYSCTL_NMI_PIEVECTERR         -  PIE Vector Fetch Error Flag
//! - \b SYSCTL_NMI_CLBNMI             -  Configurable Logic Block NMI
//! - \b SYSCTL_NMI_SWERR              -  SW Error Force NMI Flag
//
//*****************************************************************************
static inline uint16_t
SysCtl_getNMIFlagStatus(void)
{
    //
    // Read and return the current value of the NMI flag register.
    //
    return(HWREGH(NMI_BASE + NMI_O_FLG));
}

//*****************************************************************************
//
//! Check if the individual NMI interrupts are set.
//!
//! \param nmiFlags Bit mask of the NMI interrupts that user wants to clear.
//! The bit format of this parameter is same as of the NMIFLG register. These
//! defines are provided:
//! - \b SYSCTL_NMI_NMIINT             -  NMI Interrupt Flag
//! - \b SYSCTL_NMI_CLOCKFAIL          -  Clock Fail Interrupt Flag
//! - \b SYSCTL_NMI_RAMUNCERR          -  RAM Uncorrectable Error NMI
//! - \b SYSCTL_NMI_FLUNCERR           -  Flash Uncorrectable Error NMI
//! - \b SYSCTL_NMI_PIEVECTERR         -  PIE Vector Fetch Error Flag
//! - \b SYSCTL_NMI_CLBNMI             -  Configurable Logic Block NMI
//! - \b SYSCTL_NMI_SWERR              -  SW Error Force NMI Flag
//!
//! Check if interrupt flags corresponding to the passed in bit mask are
//! asserted.
//!
//! \return \b true if any of the NMI asked for in the parameter bit mask
//! is set. \b false if none of the NMI requested in the parameter bit mask are
//! set.
//
//*****************************************************************************
static inline bool
SysCtl_isNMIFlagSet(uint32_t nmiFlags)
{
    //
    // Check the arguments.
    // Make sure if reserved bits are not set in nmiFlags.
    //
    ASSERT((nmiFlags & ~(
                         SYSCTL_NMI_NMIINT             |
                         SYSCTL_NMI_CLOCKFAIL          |
                         SYSCTL_NMI_RAMUNCERR          |
                         SYSCTL_NMI_FLUNCERR           |
                         SYSCTL_NMI_PIEVECTERR         |
                         SYSCTL_NMI_CLBNMI             |
                         SYSCTL_NMI_SWERR
                         )) == 0);

    //
    // Read the flag register and return true if any of them are set.
    //
    return((HWREGH(NMI_BASE + NMI_O_FLG) & nmiFlags) != 0U);
}

//*****************************************************************************
//
//! Function to clear individual NMI interrupts.
//!
//! \param nmiFlags Bit mask of the NMI interrupts that user wants to clear.
//! The bit format of this parameter is same as of the NMIFLG register. These
//! defines are provided:
//! - \b SYSCTL_NMI_NMIINT             -  NMI Interrupt Flag
//! - \b SYSCTL_NMI_CLOCKFAIL          -  Clock Fail Interrupt Flag
//! - \b SYSCTL_NMI_RAMUNCERR          -  RAM Uncorrectable Error NMI
//! - \b SYSCTL_NMI_FLUNCERR           -  Flash Uncorrectable Error NMI
//! - \b SYSCTL_NMI_PIEVECTERR         -  PIE Vector Fetch Error Flag
//! - \b SYSCTL_NMI_CLBNMI             -  Configurable Logic Block NMI
//! - \b SYSCTL_NMI_SWERR              -  SW Error Force NMI Flag
//!
//! Clear NMI interrupt flags that correspond with the passed in bit mask.
//!
//! \b Note: The NMI Interrupt flag is always cleared by default and
//! therefore doesn't have to be included in the bit mask.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_clearNMIStatus(uint32_t nmiFlags)
{
    //
    // Check the arguments.
    // Make sure if reserved bits are not set in nmiFlags.
    //
    ASSERT((nmiFlags & ~(
                         SYSCTL_NMI_NMIINT             |
                         SYSCTL_NMI_CLOCKFAIL          |
                         SYSCTL_NMI_RAMUNCERR          |
                         SYSCTL_NMI_FLUNCERR           |
                         SYSCTL_NMI_PIEVECTERR         |
                         SYSCTL_NMI_CLBNMI             |
                         SYSCTL_NMI_SWERR
                         )) == 0);

    EALLOW;

    //
    // Clear the individual flags as well as NMI Interrupt flag
    //
    HWREGH(NMI_BASE + NMI_O_FLGCLR) = nmiFlags;
    HWREGH(NMI_BASE + NMI_O_FLGCLR) = NMI_FLG_NMIINT;

    EDIS;
}

//*****************************************************************************
//
//! Clear all the NMI Flags that are currently set.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_clearAllNMIFlags(void)
{
    uint16_t nmiFlags;

    //
    // Read the flag status register and then write to the clear register,
    // clearing all the flags that were returned plus the NMI flag.
    //
    EALLOW;

    nmiFlags = SysCtl_getNMIFlagStatus();
    HWREGH(NMI_BASE + NMI_O_FLGCLR) = nmiFlags;
    HWREGH(NMI_BASE + NMI_O_FLGCLR) = NMI_FLG_NMIINT;

    EDIS;
}

//*****************************************************************************
//
//! Function to force individual NMI interrupt fail flags
//!
//! \param nmiFlags Bit mask of the NMI interrupts that user wants to clear.
//! The bit format of this parameter is same as of the NMIFLG register. These
//! defines are provided:
//! - \b SYSCTL_NMI_NMIINT             -  NMI Interrupt Flag
//! - \b SYSCTL_NMI_CLOCKFAIL          -  Clock Fail Interrupt Flag
//! - \b SYSCTL_NMI_RAMUNCERR          -  RAM Uncorrectable Error NMI
//! - \b SYSCTL_NMI_FLUNCERR           -  Flash Uncorrectable Error NMI
//! - \b SYSCTL_NMI_PIEVECTERR         -  PIE Vector Fetch Error Flag
//! - \b SYSCTL_NMI_CLBNMI             -  Configurable Logic Block NMI
//! - \b SYSCTL_NMI_SWERR              -  SW Error Force NMI Flag
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_forceNMIFlags(uint32_t nmiFlags)
{
    //
    // Check the arguments.
    // Make sure if reserved bits are not set in nmiFlags.
    //
    ASSERT((nmiFlags & ~(
                         SYSCTL_NMI_NMIINT             |
                         SYSCTL_NMI_CLOCKFAIL          |
                         SYSCTL_NMI_RAMUNCERR          |
                         SYSCTL_NMI_FLUNCERR           |
                         SYSCTL_NMI_PIEVECTERR         |
                         SYSCTL_NMI_CLBNMI             |
                         SYSCTL_NMI_SWERR
                        )) == 0);

    EALLOW;

    //
    // Set the Flags for the individual interrupts in the NMI flag
    // force register
    //
    HWREGH(NMI_BASE + NMI_O_FLGFRC) |= nmiFlags;

    EDIS;
}

//*****************************************************************************
//
//! Gets the NMI watchdog counter value.
//!
//! \b Note: The counter is clocked at the SYSCLKOUT rate.
//!
//! \return Returns the NMI watchdog counter register's current value.
//
//*****************************************************************************
static inline uint16_t
SysCtl_getNMIWatchdogCounter(void)
{
    //
    // Read and return the NMI watchdog counter register's value.
    //
    return(HWREGH(NMI_BASE + NMI_O_WDCNT));
}

//*****************************************************************************
//
//! Sets the NMI watchdog period value.
//!
//! \param wdPeriod is the 16-bit value at which a reset is generated.
//!
//! This function writes to the NMI watchdog period register that holds the
//! value to which the NMI watchdog counter is compared. When the two registers
//! match, a reset is generated. By default, the period is 0xFFFF.
//!
//! \note If a value smaller than the current counter value is passed into the
//! \e wdPeriod parameter, a NMIRSn will be forced.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_setNMIWatchdogPeriod(uint16_t wdPeriod)
{
    EALLOW;

    //
    // Write to the period register.
    //
    HWREGH(NMI_BASE + NMI_O_WDPRD) = wdPeriod;

    EDIS;
}

//*****************************************************************************
//
//! Gets the NMI watchdog period value.
//!
//! \return Returns the NMI watchdog period register's current value.
//
//*****************************************************************************
static inline uint16_t
SysCtl_getNMIWatchdogPeriod(void)
{
    //
    // Read and return the NMI watchdog period register's value.
    //
    return(HWREGH(NMI_BASE + NMI_O_WDPRD));
}

//*****************************************************************************
//
//! Read NMI Shadow Flags.
//!
//! Read the current state of individual NMI interrupts
//!
//! \return Value of NMISHDFLG register. These defines are provided to decode
//! the value:
//! - \b SYSCTL_NMI_NMIINT             -  NMI Interrupt Flag
//! - \b SYSCTL_NMI_CLOCKFAIL          -  Clock Fail Interrupt Flag
//! - \b SYSCTL_NMI_RAMUNCERR          -  RAM Uncorrectable Error NMI
//! - \b SYSCTL_NMI_FLUNCERR           -  Flash Uncorrectable Error NMI
//! - \b SYSCTL_NMI_PIEVECTERR         -  PIE Vector Fetch Error Flag
//! - \b SYSCTL_NMI_CLBNMI             -  Configurable Logic Block NMI
//! - \b SYSCTL_NMI_SWERR              -  SW Error Force NMI Flag
//
//*****************************************************************************
static inline uint32_t
SysCtl_getNMIShadowFlagStatus(void)
{
    //
    // Read and return the current value of the NMI shadow flag register.
    //
    return(HWREGH(NMI_BASE + NMI_O_SHDFLG));
}

//*****************************************************************************
//
//! Check if the individual NMI shadow flags are set.
//!
//! \param nmiFlags Bit mask of the NMI interrupts that user wants  to clear.
//! The bit format of this parameter is same as of the NMIFLG register. These
//! defines are provided:
//! - \b SYSCTL_NMI_NMIINT             -  NMI Interrupt Flag
//! - \b SYSCTL_NMI_CLOCKFAIL          -  Clock Fail Interrupt Flag
//! - \b SYSCTL_NMI_RAMUNCERR          -  RAM Uncorrectable Error NMI
//! - \b SYSCTL_NMI_FLUNCERR           -  Flash Uncorrectable Error NMI
//! - \b SYSCTL_NMI_PIEVECTERR         -  PIE Vector Fetch Error Flag
//! - \b SYSCTL_NMI_CLBNMI             -  Configurable Logic Block NMI
//! - \b SYSCTL_NMI_SWERR              -  SW Error Force NMI Flag
//!
//! Check if interrupt flags corresponding to the passed in bit mask are
//! asserted.
//!
//! \return \b true if any of the NMI asked for in the parameter bit mask
//! is set. \b false if none of the NMI requested in the parameter bit mask are
//! set.
//
//*****************************************************************************
static inline bool
SysCtl_isNMIShadowFlagSet(uint32_t nmiFlags)
{
    //
    // Check the arguments.
    // Make sure if reserved bits are not set in nmiFlags.
    //
    ASSERT((nmiFlags & ~(
                         SYSCTL_NMI_NMIINT             |
                         SYSCTL_NMI_CLOCKFAIL          |
                         SYSCTL_NMI_RAMUNCERR          |
                         SYSCTL_NMI_FLUNCERR           |
                         SYSCTL_NMI_PIEVECTERR         |
                         SYSCTL_NMI_CLBNMI             |
                         SYSCTL_NMI_SWERR
                        )) == 0);

    //
    // Read the flag register and return true if any of them are set.
    //
    return((HWREGH(NMI_BASE + NMI_O_SHDFLG) & nmiFlags) != 0U);
}

//*****************************************************************************
//
//! Enable the missing clock detection (MCD) Logic
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_enableMCD(void)
{
    EALLOW;

    HWREGH(CLKCFG_BASE + SYSCTL_O_MCDCR) &= ~(SYSCTL_MCDCR_MCLKOFF);

    EDIS;
}

//*****************************************************************************
//
//! Disable the missing clock detection (MCD) Logic
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_disableMCD(void)
{
    EALLOW;

    HWREGH(CLKCFG_BASE + SYSCTL_O_MCDCR) |= SYSCTL_MCDCR_MCLKOFF;

    EDIS;
}

//*****************************************************************************
//
//! Get the missing clock detection Failure Status
//!
//! \note A failure means the oscillator clock is missing
//!
//! \return Returns \b true if a failure is detected or \b false if a
//! failure isn't detected
//
//*****************************************************************************
static inline bool
SysCtl_isMCDClockFailureDetected(void)
{
    //
    //  Check the status bit to determine failure
    //
    return((HWREGH(CLKCFG_BASE + SYSCTL_O_MCDCR) & SYSCTL_MCDCR_MCLKSTS) != 0U);
}

//*****************************************************************************
//
//! Reset the missing clock detection logic after clock failure
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_resetMCD(void)
{
    EALLOW;

    HWREGH(CLKCFG_BASE + SYSCTL_O_MCDCR) |= SYSCTL_MCDCR_MCLKCLR;

    EDIS;
}

//*****************************************************************************
//
//! Re-connect missing clock detection clock source to stop simulating clock
//! failure
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_connectMCDClockSource(void)
{
    EALLOW;

    HWREGH(CLKCFG_BASE + SYSCTL_O_MCDCR) &= ~(SYSCTL_MCDCR_OSCOFF);

    EDIS;
}

//*****************************************************************************
//
//! Disconnect missing clock detection clock source to simulate clock failure.
//! This is for testing the MCD functionality.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_disconnectMCDClockSource(void)
{
    EALLOW;

    HWREGH(CLKCFG_BASE + SYSCTL_O_MCDCR) |= SYSCTL_MCDCR_OSCOFF;

    EDIS;
}

//*****************************************************************************
//
//! Lock the Access Control Registers
//!
//! This function locks the access control registers and puts them in a
//! read-only state.
//!
//! \note Only a reset can unlock the access control registers.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_lockAccessControlRegs(void)
{
    EALLOW;

    HWREGH(PERIPHAC_BASE + SYSCTL_O_PERIPH_AC_LOCK) |=
        SYSCTL_PERIPH_AC_LOCK_LOCK_AC_WR;

    EDIS;
}

//*****************************************************************************
//
//! Set the peripheral access control permissions
//!
//! \param peripheral is the selected peripheral
//! \param master is the selected master (CPU1, CLA1, or DMA1)
//! \param permission is the selected access permissions
//!
//! This function sets the specified peripheral access control permissions for
//! the the specified master (CPU1, CLA1, or DMA1)
//!
//! The \e peripheral parameter can have one enumerated value in the format of
//! \b SYSCTL_ACCESS_X where X is the name of the peripheral instance to be
//! configured such as \b SYSCTL_ACCESS_ADCA.
//!
//! The \e master parameter can have one the following enumerated values:
//! - \b SYSCTL_ACCESS_CPU1 - CPU1 Master
//! - \b SYSCTL_ACCESS_CLA1 - CLA1 Master
//! - \b SYSCTL_ACCESS_DMA1 - DMA1 Master
//!
//! The \e permission parameter can have one the following enumerated values:
//! - \b SYSCTL_ACCESS_FULL      - Full Access for both read and write
//! - \b SYSCTL_ACCESS_PROTECTED - Protected read access such that FIFOs, clear
//!                                on read registers are not changed, and no
//!                                write access
//! - \b SYSCTL_ACCESS_NONE      - No read or write access
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_setPeripheralAccessControl(SysCtl_AccessPeripheral peripheral,
                                  SysCtl_AccessMaster master,
                                  SysCtl_AccessPermission permission)
{
    //
    // Set master permissions for specified peripheral. Each master has
    // two bits dedicated to its permission setting.
    //
    EALLOW;

    HWREGH(PERIPHAC_BASE + (uint16_t)peripheral) =
        (HWREGH(PERIPHAC_BASE + (uint16_t)peripheral) &
         ~(0x3U << (uint16_t)master)) |
        ((uint32_t)permission << (uint16_t)master);

    EDIS;
}

//*****************************************************************************
//
//! Get the peripheral access control permissions
//!
//! \param peripheral is the selected peripheral
//! \param master is the selected master (CPU1, CLA1, or DMA1)
//!
//! This function gets the specified peripheral access control permissions for
//! the the specified master (CPU1, CLA1, or DMA1)
//!
//! The \e peripheral parameter can have one enumerated value in the format of
//! \b SYSCTL_ACCESS_X where X is the name of the peripheral instance to be
//! configured such as \b SYSCTL_ACCESS_ADCA.
//!
//! The \e master parameter can have one the following enumerated values:
//! - \b SYSCTL_ACCESS_CPU1 - CPU1 Master
//! - \b SYSCTL_ACCESS_CLA1 - CLA1 Master
//! - \b SYSCTL_ACCESS_DMA1 - DMA1 Master
//!
//! \return Returns one of the following enumerated permission values:
//! - \b SYSCTL_ACCESS_FULL      - Full Access for both read and write
//! - \b SYSCTL_ACCESS_PROTECTED - Protected read access such that FIFOs, clear
//!                                on read registers are not changed, and no
//!                                write access
//! - \b SYSCTL_ACCESS_NONE      - No read or write access
//
//*****************************************************************************
static inline uint32_t
SysCtl_getPeripheralAccessControl(SysCtl_AccessPeripheral peripheral,
                                  SysCtl_AccessMaster master)
{
    //
    // Read master permissions for specified peripheral. Each master has
    // two bits dedicated to its permission setting.
    //
    return((HWREGH(PERIPHAC_BASE + (uint16_t)peripheral) >> (uint16_t)master) &
           0x3U);
}

//*****************************************************************************
//
//! Configures the sync input source for the ePWM and eCAP signals.
//!
//! \param syncInput is the sync input being configured
//! \param syncSrc is sync input source selection.
//!
//! This function configures the sync input source for the ePWM and eCAP
//! modules. The \e syncInput parameter is the sync input being configured. It
//! should be passed a value of \b SYSCTL_SYNC_IN_XXXX, where XXXX is the ePWM
//! or eCAP instance the sync signal is entering.
//!
//! The \e syncSrc parameter is the sync signal selected as the source of the
//! sync input. It should be passed a value of \b SYSCTL_SYNC_IN_SRC_XXXX,
//! XXXX is a sync signal coming from an ePWM, eCAP or external sync output.
//! where For example, a \e syncInput value of \b SYSCTL_SYNC_IN_ECAP1 and a
//! \e syncSrc value of \b SYSCTL_SYNC_IN_SRC_EPWM1SYNCOUT will make the
//! EPWM1SYNCOUT signal drive eCAP1's SYNCIN signal.
//!
//! Note that some \e syncSrc values are only valid for certain values of
//! \e syncInput. See device technical reference manual for details on
//! time-base counter synchronization.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_setSyncInputConfig(SysCtl_SyncInput syncInput,
                          SysCtl_SyncInputSource syncSrc)
{
    uint32_t clearMask;

    //
    // Write the input sync source selection to the appropriate register.
    //
    EALLOW;
    if(syncInput == SYSCTL_SYNC_IN_EPWM1)
    {
        //
        // Note that EXTSYNCIN1 is the only valid option for ePWM1
        //
        HWREG(SYNCSOC_BASE + SYSCTL_O_SYNCSELECT) &=
            ~SYSCTL_SYNCSELECT_EPWM1SYNCIN_M;
    }
    else
    {
        clearMask = (uint32_t)SYSCTL_SYNCSELECT_SYNCIN_M <<
                    (uint32_t)syncInput;

        HWREG(SYNCSOC_BASE + SYSCTL_O_SYNCSELECT) =
            (HWREG(SYNCSOC_BASE + SYSCTL_O_SYNCSELECT) & ~clearMask) |
            ((uint32_t)syncSrc << (uint32_t)syncInput);
    }

    EDIS;
}
//*****************************************************************************
//
//! Configures the sync output source.
//!
//! \param syncSrc is sync output source selection.
//!
//! This function configures the sync output source from the ePWM modules. The
//! \e syncSrc parameter is a value \b SYSCTL_SYNC_OUT_SRC_XXXX, where XXXX is
//! a sync signal coming from an ePWM such as SYSCTL_SYNC_OUT_SRC_EPWM1SYNCOUT
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_setSyncOutputConfig(SysCtl_SyncOutputSource syncSrc)
{
    //
    // Write the sync output source selection to the appropriate register.
    //
    EALLOW;
    HWREG(SYNCSOC_BASE + SYSCTL_O_SYNCSELECT) =
        (HWREG(SYNCSOC_BASE + SYSCTL_O_SYNCSELECT) &
         ~((uint32_t)SYSCTL_SYNCSELECT_SYNCOUT_M)) |
        ((uint32_t)syncSrc << SYSCTL_SYNCSELECT_SYNCOUT_S);
    EDIS;

}
//*****************************************************************************
//
//! Enables ePWM SOC signals to drive an external (off-chip) ADCSOC signal.
//!
//! \param adcsocSrc is a bit field of the selected signals to be enabled
//!
//! This function configures which ePWM SOC signals are enabled as a source for
//! either ADCSOCAO or ADCSOCBO. The \e adcsocSrc parameter takes a logical OR
//! of \b SYSCTL_ADCSOC_SRC_PWMxSOCA/B values that correspond to different
//! signals.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_enableExtADCSOCSource(uint32_t adcsocSrc)
{
    //
    // Set the bits that correspond to signal to be enabled.
    //
    EALLOW;
    HWREG(SYNCSOC_BASE + SYSCTL_O_ADCSOCOUTSELECT) |= adcsocSrc;
    EDIS;
}

//*****************************************************************************
//
//! Disables ePWM SOC signals from driving an external ADCSOC signal.
//!
//! \param adcsocSrc is a bit field of the selected signals to be disabled
//!
//! This function configures which ePWM SOC signals are disabled as a source
//! for either ADCSOCAO or ADCSOCBO. The \e adcsocSrc parameter takes a logical
//! OR of \b SYSCTL_ADCSOC_SRC_PWMxSOCA/B values that correspond to different
//! signals.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_disableExtADCSOCSource(uint32_t adcsocSrc)
{
    //
    // Clear the bits that correspond to signal to be disabled.
    //
    EALLOW;
    HWREG(SYNCSOC_BASE + SYSCTL_O_ADCSOCOUTSELECT) &= ~adcsocSrc;
    EDIS;
}

//*****************************************************************************
//
//! Locks the SOC Select of the Trig X-BAR.
//!
//! This function locks the external ADC SOC select of the Trig X-BAR.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_lockExtADCSOCSelect(void)
{
    //
    // Lock the ADCSOCOUTSELECT bit of the SYNCSOCLOCK register.
    //
    EALLOW;
    HWREG(SYNCSOC_BASE + SYSCTL_O_SYNCSOCLOCK) =
        SYSCTL_SYNCSOCLOCK_ADCSOCOUTSELECT;
    EDIS;
}

//*****************************************************************************
//
//! Locks the Sync Select of the Trig X-BAR.
//!
//! This function locks Sync Input and Output Select of the Trig X-BAR.
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_lockSyncSelect(void)
{
    //
    // Lock the SYNCSELECT register.
    //
    EALLOW;
    HWREG(SYNCSOC_BASE + SYSCTL_O_SYNCSOCLOCK) = SYSCTL_SYNCSOCLOCK_SYNCSELECT;
    EDIS;
}
//*****************************************************************************
//
//! Get the Device Silicon Revision ID
//!
//! This function returns the silicon revision ID for the device.
//!
//! \note This API is applicable only for the CPU1 subsystem.
//!
//! \return Returns the silicon revision ID value.
//
//*****************************************************************************
static inline uint32_t
SysCtl_getDeviceRevision(void)
{
    //
    // Returns the device silicon revision ID
    //
    return(HWREG(DEVCFG_BASE + SYSCTL_O_REVID));
}


//*****************************************************************************
//
//! Gets the error status of the Efuse
//!
//! The function provides both the Efuse Autoload & the Efuse Self Test
//! Error Status.
//!
//! \note This API is applicable only for the CPU1 subsystem.
//!
//! \return Fuse Error status.
//
//*****************************************************************************
static inline uint16_t
SysCtl_getEfuseError(void)
{
    return(HWREGH(DEVCFG_BASE + SYSCTL_O_FUSEERR));
}

//*****************************************************************************
//
//! Sets up XCLK divider.
//!
//! \param divider is the value that configures the divider.
//!
//! This function sets up the XCLK divider. There is only one
//! divider that scales INTOSC1 to XCLK.
//!
//! The \e divider parameter can have one enumerated value from
//! SysCtl_XClkDivider
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_setXClk(SysCtl_XClkDivider divider)
{
    //
    // Clears the divider then configures it.
    //
    EALLOW;
    HWREGH(CLKCFG_BASE + SYSCTL_O_XCLKOUTDIVSEL) =
                        (HWREGH(CLKCFG_BASE + SYSCTL_O_XCLKOUTDIVSEL) &
                         ~(SYSCTL_XCLKOUTDIVSEL_XCLKOUTDIV_M)) |
                        (uint16_t)divider;
    EDIS;
}

//*****************************************************************************
//
//! Sets up PLLSYSCLK divider.
//!
//! \param divider is the value that configures the divider.
//!
//! This function sets up the PLLSYSCLK divider. There is only one
//! divider that scales PLLSYSCLK to generate the system clock.
//!
//! The \e divider parameter can have one value from the set below:
//!     0x0 = /1
//!     0x1 = /2
//!     0x2 = /4 (default on reset)
//!     0x3 = /6
//!     0x4 = /8
//!    ......
//!     0x3F =/126
//!
//! \return None.
//!
//! \note Please make sure to check if the PLL is locked and valid using the
//! SysCtl_isPLLValid() before setting the divider.
//
//*****************************************************************************
static inline void
SysCtl_setPLLSysClk(uint16_t divider)
{
    //
    // Clears the divider then configures it.
    //
    EALLOW;
    HWREGH(CLKCFG_BASE + SYSCTL_O_SYSCLKDIVSEL) =
                    (HWREGH(CLKCFG_BASE + SYSCTL_O_SYSCLKDIVSEL) &
                     ~(SYSCTL_SYSCLKDIVSEL_PLLSYSCLKDIV_M)) | divider;
    EDIS;
}

//*****************************************************************************
//
//! Sets up CPU Timer 2  CLK source & divider.
//!
//! \param divider is the value that configures the divider.
//! \param source is the source for the clock divider
//!
//! This function sets up the CPU Timer 2 CLK divider based on the source that
//! is selected. There is only one divider that scales the "source" to
//! CPU Timer 2 CLK.
//!
//! The \e divider parameter can have one enumerated value from
//! SysCtl_Cputimer2ClkDivider
//! The \e source parameter can have one enumerated value from
//! SysCtl_Cputimer2ClkSource
//!
//! \return None.
//
//*****************************************************************************
static inline void
SysCtl_setCputimer2Clk(SysCtl_Cputimer2ClkDivider divider,
                               SysCtl_Cputimer2ClkSource source)
{
  //
    // Clears the divider & the source, then configures it.
    //
    EALLOW;
    HWREGH(CPUSYS_BASE + SYSCTL_O_TMR2CLKCTL) =
                    (HWREGH(CPUSYS_BASE + SYSCTL_O_TMR2CLKCTL) &
                     ~(SYSCTL_TMR2CLKCTL_TMR2CLKSRCSEL_M |
                       SYSCTL_TMR2CLKCTL_TMR2CLKPRESCALE_M));

    HWREGH(CPUSYS_BASE + SYSCTL_O_TMR2CLKCTL) |=
                    ((uint16_t)divider << SYSCTL_TMR2CLKCTL_TMR2CLKPRESCALE_S) |
                    ((uint16_t)source << SYSCTL_TMR2CLKCTL_TMR2CLKSRCSEL_S);
    EDIS;
}

//*****************************************************************************
//
//! Gets the PIE Vector Fetch Error Handler Routine Address.
//!
//! The function indicates the address of the PIE Vector Fetch Error
//! handler routine.
//!
//! \return Error Handler Address.
//!
//! \note Its the responsibility of user to initialize this register. If this
//! register is not initialized, a default error handler at address
//! 0x3fffbe will get executed.
//
//*****************************************************************************
static inline uint32_t
SysCtl_getPIEVErrAddr(void)
{
    return(HWREG(CPUSYS_BASE + SYSCTL_O_PIEVERRADDR));
}

//*****************************************************************************
//
//! Delays for a fixed number of cycles.
//!
//! \param count is the number of delay loop iterations to perform.
//!
//! This function generates a constant length delay using assembly code. The
//! loop takes 5 cycles per iteration plus 9 cycles of overhead.
//!
//! \note If count is equal to zero, the loop will underflow and run for a
//! very long time.
//!
//! \return None.
//
//*****************************************************************************
extern void
SysCtl_delay(uint32_t count);

//*****************************************************************************
//
//! Calculates the system clock frequency (SYSCLK).
//!
//! \param clockInHz is the frequency of the oscillator clock source (OSCCLK).
//!
//! This function determines the frequency of the system clock based on the
//! frequency of the oscillator clock source (from \e clockInHz) and the PLL
//! and clock divider configuration registers.
//!
//! \return Returns the system clock frequency. If a missing clock is detected,
//! the function will return the INTOSC1 frequency. This needs to be
//! corrected and cleared (see SysCtl_resetMCD()) before trying to call this
//! function again.
//
//*****************************************************************************
extern uint32_t
SysCtl_getClock(uint32_t clockInHz);

//*****************************************************************************
//
//! Configures the clocking of the device.
//!
//! \param config is the required configuration of the device clocking.
//!
//! This function configures the clocking of the device.  The input crystal
//! frequency, oscillator to be used, use of the PLL, and the system clock
//! divider are all configured with this function.
//!
//! The \e config parameter is the OR of several different values, many of
//! which are grouped into sets where only one can be chosen.
//!
//! - The system clock divider is chosen with the macro \b SYSCTL_SYSDIV(x)
//!   where x is either 1 or an even value up to 126.
//!
//! - The use of the PLL is chosen with either \b SYSCTL_PLL_ENABLE or
//!   \b SYSCTL_PLL_DISABLE.
//!
//! - The integer multiplier is chosen \b SYSCTL_IMULT(x) where x is a value
//!   from 1 to 127.
//!
//! - The fractional multiplier is chosen with either \b SYSCTL_FMULT_0,
//!   \b SYSCTL_FMULT_1_4, \b SYSCTL_FMULT_1_2, or \b SYSCTL_FMULT_3_4.
//!
//! - The oscillator source chosen with \b SYSCTL_OSCSRC_OSC2,
//!   \b SYSCTL_OSCSRC_XTAL, \b SYSCTL_OSCSRC_XTAL_SE or \b SYSCTL_OSCSRC_OSC1.
//!
//! This function uses the DCC to check that the PLLRAWCLK is running at the
//! expected rate. If you are using the DCC, you must back up its configuration
//! before calling this function and restore it afterward.
//! Locking PLL sequence is only done if the multipliers are updated.
//!
//! \note See your device errata for more details about locking the PLL.
//!
//! \return Returns \b false if a missing clock error is detected. This needs
//! to be cleared (see SysCtl_resetMCD()) before trying to call this function
//! again. Also, returns \b false if the PLLRAWCLK is not running and its
//! expected rate after \b SYSCTL_PLL_RETRIES retries. Otherwise, returns
//! \b true.
//
//*****************************************************************************
extern bool
SysCtl_setClock(uint32_t config);

//*****************************************************************************
//
//! Validates PLL Raw Clock Frequency (PLLRAWCLK)
//!
//! \param oscSource is the Clock Source for the PLL that is also used for DCC
//! \param pllMult has the PLL Multiplier Register configuration which include
//!  integer and fractional multiplier used to configure the DCC Counter1 clock
//!
//!  This function uses DCC module to validate the PLL clock frequency.
//!  It uses oscSource as a reference clock for DCC, and PLL is used as clock
//!  under test. As long as the Counter0 (running of oscSource) & Counter1
//!  (running of PLL) expire at the same time, DCC will not generate an Error.
//!  This function gives 100 attempts for PLL to lock and make sure frequency
//!  is as expected.
//!
//!  \note This function does not validate if PLL output frequency (PLLRAWCLK)
//!  is within the operating range as per the datasheet.
//!
//! - The \e oscSource parameter is the oscillator source chosen with
//!   \b SYSCTL_OSCSRC_OSC2, \b SYSCTL_OSCSRC_XTAL, \b SYSCTL_OSCSRC_XTAL_SE
//!    or \b SYSCTL_OSCSRC_OSC1.
//!
//! - The \e pllMult parameter is a bitwise OR of \b SYSCTL_IMULT(x) where x is
//!   a value from 1 to 127 and one of the following fractional values:
//!   \b SYSCTL_FMULT_0, \b SYSCTL_FMULT_1_4, \b SYSCTL_FMULT_1_2, or
//!   \b SYSCTL_FMULT_3_4.
//!
//!  \return Returns \b true if the DCCSTATUS error flag is not set.
//!    Otherwise, returns \b false.
//
//*****************************************************************************
extern bool
SysCtl_isPLLValid(uint32_t oscSource, uint32_t pllMult);
//*****************************************************************************
//
//! Configures the external oscillator for the clocking of the device.
//!
//! This function configures the external oscillator (XTAL) to be used for the
//! clocking of the device in crystal mode. It follows the procedure to turn on
//! the oscillator, wait for it to power up, and select it as the source of the
//! system clock.
//!
//! Please note that this function blocks while it waits for the XTAL to power
//! up. If the XTAL does not manage to power up properly, the function will
//! loop for a long time. It is recommended that you modify this function to
//! add an appropriate timeout and error-handling procedure.
//!
//! \return None.
//
//*****************************************************************************
extern void
SysCtl_selectXTAL(void);

//*****************************************************************************
//
//! Configures the external oscillator for the clocking of the device in
//! single-ended mode.
//!
//! This function configures the external oscillator (XTAL) to be used for the
//! clocking of the device in single-ended mode. It follows the procedure to
//! turn on the oscillator, wait for it to power up, and select it as the
//! source of the system clock.
//!
//! Please note that this function blocks while it waits for the XTAL to power
//! up. If the XTAL does not manage to power up properly, the function will
//! loop for a long time. It is recommended that you modify this function to
//! add an appropriate timeout and error-handling procedure.
//!
//! \return None.
//
//*****************************************************************************
extern void
SysCtl_selectXTALSingleEnded(void);

//*****************************************************************************
//
//! Selects the oscillator to be used for the clocking of the device.
//!
//! \param oscSource is the oscillator source to be configured.
//!
//! This function configures the oscillator to be used in the clocking of the
//! device. The \e oscSource parameter may take a value of
//! \b SYSCTL_OSCSRC_OSC2, \b SYSCTL_OSCSRC_XTAL, \b SYSCTL_OSCSRC_XTAL_SE, or
//! \b SYSCTL_OSCSRC_OSC1.
//!
//! \sa SysCtl_turnOnOsc()
//!
//! \return None.
//
//*****************************************************************************
extern void
SysCtl_selectOscSource(uint32_t oscSource);

//*****************************************************************************
//
//! Calculates the low-speed peripheral clock frequency (LSPCLK).
//!
//! \param clockInHz is the frequency of the oscillator clock source (OSCCLK).
//!
//! This function determines the frequency of the low-speed peripheral clock
//! based on the frequency of the oscillator clock source (from \e clockInHz)
//! and the PLL and clock divider configuration registers.
//!
//! \return Returns the low-speed peripheral clock frequency.
//
//*****************************************************************************
extern uint32_t
SysCtl_getLowSpeedClock(uint32_t clockInHz);

//*****************************************************************************
//
//! Get the device part parametric value
//!
//! \param parametric is the requested device parametric value
//!
//! This function gets the device part parametric value.
//!
//! The \e parametric parameter can have one the following enumerated values:
//! - \b SYSCTL_DEVICE_QUAL      - Device Qualification Status
//! - \b SYSCTL_DEVICE_PINCOUNT  - Device Pin Count
//! - \b SYSCTL_DEVICE_INSTASPIN - Device InstaSPIN Feature Set
//! - \b SYSCTL_DEVICE_FLASH     - Device Flash size (KB)
//! - \b SYSCTL_DEVICE_FAMILY    - Device Family
//! - \b SYSCTL_DEVICE_PARTNO    - Device Part Number
//! - \b SYSCTL_DEVICE_CLASSID   - Device Class ID
//!
//! \note This API is applicable only for the CPU1 subsystem.
//!
//! \return Returns the specified parametric value.
//
//*****************************************************************************
extern uint16_t
SysCtl_getDeviceParametric(SysCtl_DeviceParametric parametric);

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//****************************************************************************

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // SYSCTL_H
