//###########################################################################
//
// FILE:    hw_memmap.h
//
// TITLE:   Macros defining the memory map of the C28x.
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

#ifndef HW_MEMMAP_H
#define HW_MEMMAP_H

//*****************************************************************************
//
// The following are defines for the base address of the memories and
// peripherals.
//
//*****************************************************************************
#define ADCARESULT_BASE             0x00000B00U // ADCA Result Registers
#define ADCBRESULT_BASE             0x00000B20U // ADCB Result Registers
#define ADCCRESULT_BASE             0x00000B40U // ADCC Result Registers

#if defined(__TMS320C28XX_CLA2__)
#define CLA1_ONLY_BASE              0x00000C00U // CLA Accessible Registers
#else // __TMS320C28XX__
#define CPUTIMER0_BASE              0x00000C00U // CPU Timer 0 Registers
#define CPUTIMER1_BASE              0x00000C08U // CPU Timer 1 Registers
#define CPUTIMER2_BASE              0x00000C10U // CPU Timer 2 Registers
#endif // defined(__TMS320C28XX_CLA2__)

#define PIECTRL_BASE                0x00000CE0U // PIE Registers
#define PIEVECTTABLE_BASE           0x00000D00U // PIE Vector Table
#define DMA_BASE                    0x00001000U // DMA Control Registers
#define DMA_CH1_BASE                0x00001020U // DMA Channel Registers
#define DMA_CH2_BASE                0x00001040U // DMA Channel Registers
#define DMA_CH3_BASE                0x00001060U // DMA Channel Registers
#define DMA_CH4_BASE                0x00001080U // DMA Channel Registers
#define DMA_CH5_BASE                0x000010A0U // DMA Channel Registers
#define DMA_CH6_BASE                0x000010C0U // DMA Channel Registers
#define CLA1_BASE                   0x00001400U // CPU1.CLA1 Registers
#define CLB1_BASE                   0x00003000U // CLB 1 Logic Config Registers
#define CLB1_LOGICCFG_BASE          0x00003000U // CLB 2 Logic Config Registers
#define CLB1_LOGICCTL_BASE          0x00003100U // CLB 1 Logic Control Registers
#define CLB1_DATAEXCH_BASE          0x00003200U // CLB 1 Data Exchange Registers
#define CLB2_BASE                   0x00003400U // CLB 2 Logic Config Registers
#define CLB2_LOGICCFG_BASE          0x00003400U // CLB 2 Logic Config Registers
#define CLB2_LOGICCTL_BASE          0x00003500U // CLB 2 Logic Control Registers
#define CLB2_DATAEXCH_BASE          0x00003600U // CLB 2 Data Exchange Registers
#define CLB3_BASE                   0x00003800U // CLB 3 Logic Config Registers
#define CLB3_LOGICCFG_BASE          0x00003800U // CLB 3 Logic Config Registers
#define CLB3_LOGICCTL_BASE          0x00003900U // CLB 3 Logic Control Registers
#define CLB3_DATAEXCH_BASE          0x00003A00U // CLB 3 Data Exchange Registers
#define CLB4_BASE                   0x00003C00U // CLB 4 Logic Config Registers
#define CLB4_LOGICCFG_BASE          0x00003C00U // CLB 4 Logic Config Registers
#define CLB4_LOGICCTL_BASE          0x00003D00U // CLB 4 Logic Control Registers
#define CLB4_DATAEXCH_BASE          0x00003E00U // CLB 4 Data Exchange Registers
#define EPWM1_BASE                  0x00004000U // EPWM1
#define EPWM2_BASE                  0x00004100U // EPWM2
#define EPWM3_BASE                  0x00004200U // EPWM3
#define EPWM4_BASE                  0x00004300U // EPWM4
#define EPWM5_BASE                  0x00004400U // EPWM5
#define EPWM6_BASE                  0x00004500U // EPWM6
#define EPWM7_BASE                  0x00004600U // EPWM7
#define EPWM8_BASE                  0x00004700U // EPWM8
#define EQEP1_BASE                  0x00005100U // EQEP1
#define EQEP2_BASE                  0x00005140U // EQEP2
#define ECAP1_BASE                  0x00005200U // ECAP1
#define ECAP2_BASE                  0x00005240U // ECAP2
#define ECAP3_BASE                  0x00005280U // ECAP3
#define ECAP4_BASE                  0x000052C0U // ECAP4
#define ECAP5_BASE                  0x00005300U // ECAP5
#define ECAP6_BASE                  0x00005340U // ECAP6
#define HRCAP6_BASE                 0x00005360U // HRCAP6
#define ECAP7_BASE                  0x00005380U // ECAP7
#define HRCAP7_BASE                 0x000053A0U // HRCAP7
#define PGA1_BASE                   0x00005B00U // PGA1
#define PGA2_BASE                   0x00005B10U // PGA2
#define PGA3_BASE                   0x00005B20U // PGA3
#define PGA4_BASE                   0x00005B30U // PGA4
#define PGA5_BASE                   0x00005B40U // PGA5
#define PGA6_BASE                   0x00005B50U // PGA6
#define PGA7_BASE                   0x00005B60U // PGA7
#define DACA_BASE                   0x00005C00U // BUFDACA
#define DACB_BASE                   0x00005C10U // BUFDACB
#define CMPSS1_BASE                 0x00005C80U // CMPSS1
#define CMPSS2_BASE                 0x00005CA0U // CMPSS2
#define CMPSS3_BASE                 0x00005CC0U // CMPSS3
#define CMPSS4_BASE                 0x00005CE0U // CMPSS4
#define CMPSS5_BASE                 0x00005D00U // CMPSS5
#define CMPSS6_BASE                 0x00005D20U // CMPSS6
#define CMPSS7_BASE                 0x00005D40U // CMPSS7
#define SDFM1_BASE                  0x00005E00U // SDFM Configuration Registers
#define SPIA_BASE                   0x00006100U // SPI A Registers
#define SPIB_BASE                   0x00006110U // SPI B Registers
#define CLA1PROMCRC_BASE            0x000061C0U // CLA1PROMCRC Registers
#define PMBUSA_BASE                 0x00006400U // PMBUS A Registers
#define FSITXA_BASE                 0x00006600U // FSITX Configuration Registers
#define FSIRXA_BASE                 0x00006680U // FSIRX Configuration Registers
#define LINA_BASE                   0x00006A00U // LIN A Registers
#define LINB_BASE                   0x00006B00U // LIN B Registers
#define WD_BASE                     0x00007000U // Watchdog Registers
#define NMI_BASE                    0x00007060U // NMI Registers
#define XINT_BASE                   0x00007070U // Interrupt Control Counter Registers
#define SCIA_BASE                   0x00007200U // SCI A Registers
#define SCIB_BASE                   0x00007210U // SCI B Registers
#define I2CA_BASE                   0x00007300U // I2C A Registers
#define ADCA_BASE                   0x00007400U // ADCA Configuration Registers
#define ADCB_BASE                   0x00007480U // ADCB Configuration Registers
#define ADCC_BASE                   0x00007500U // ADCC Configuration Registers
#define INPUTXBAR_BASE              0x00007900U // GPIO Mux GPTRIP Input Select Registers
#define XBAR_BASE                   0x00007920U // X-Bar Registers
#define SYNCSOC_BASE                0x00007940U // SYNC SOC registers
#define DMACLASRCSEL_BASE           0x00007980U // DMA CLA Triggers Source Select Registers
#define EPWMXBAR_BASE               0x00007A00U // EPWM XBAR Configuration Registers
#define CLBXBAR_BASE                0x00007A40U // CLB XBAR Configuration Registers
#define OUTPUTXBAR_BASE             0x00007A80U // Output X-BAR Configuration Registers
#define GPIOCTRL_BASE               0x00007C00U // GPIO 0 to 31 Mux A Configuration Registers
#define GPIODATA_BASE               0x00007F00U // GPIO 0 to 31 Mux A Data Registers
#define CANA_BASE                   0x00048000U // CAN-A Registers
#define CANA_MSG_RAM_BASE           0x00049000U // CAN-A Message RAM
#define CANB_BASE                   0x0004A000U // CAN-B Registers
#define CANB_MSG_RAM_BASE           0x0004B000U // CAN-B Message RAM
#define DEVCFG_BASE                 0x0005D000U // Device Configuration Registers
#define CLKCFG_BASE                 0x0005D200U // Clock Configuration Registers
#define CPUSYS_BASE                 0x0005D300U // CPU System Configuration Registers
#define PERIPHAC_BASE               0x0005D500U // Peripheral Master Access Registers
#define ANALOGSUBSYS_BASE           0x0005D700U // Analog System Control Registers
#define DCC0_BASE                   0x0005E700U // Dual-Clock Comparator 0
#define ERAD_GLOBAL_BASE            0x0005E800U // Enhanced Debug Global Registers
#define ERAD_HWBP1_BASE             0x0005E900U // Enhanced Debug HW Breakpoint 1 Registers
#define ERAD_HWBP2_BASE             0x0005E908U // Enhanced Debug HW Breakpoint 2 Registers
#define ERAD_HWBP3_BASE             0x0005E910U // Enhanced Debug HW Breakpoint 3 Registers
#define ERAD_HWBP4_BASE             0x0005E918U // Enhanced Debug HW Breakpoint 4 Registers
#define ERAD_HWBP5_BASE             0x0005E920U // Enhanced Debug HW Breakpoint 5 Registers
#define ERAD_HWBP6_BASE             0x0005E928U // Enhanced Debug HW Breakpoint 6 Registers
#define ERAD_HWBP7_BASE             0x0005E930U // Enhanced Debug HW Breakpoint 7 Registers
#define ERAD_HWBP8_BASE             0x0005E938U // Enhanced Debug HW Breakpoint 8 Registers
#define ERAD_COUNTER1_BASE          0x0005E980U // Enhanced Debug Counter 1 Registers
#define ERAD_COUNTER2_BASE          0x0005E990U // Enhanced Debug Counter 2 Registers
#define ERAD_COUNTER3_BASE          0x0005E9A0U // Enhanced Debug Counter 3 Registers
#define ERAD_COUNTER4_BASE          0x0005E9B0U // Enhanced Debug Counter 4 Registers
#define DCSMBANK0_Z1_BASE           0x0005F000U // Zone 1 DCSM Registers
#define DCSMBANK0_Z2_BASE           0x0005F040U // Zone 2 DCSM Registers
#define DCSMCOMMON_BASE             0x0005F070U // Security Registers
#define DCSMCOMMON2_BASE            0x0005F080U // Security Registers
#define DCSMBANK1_Z1_BASE           0x0005F100U // Zone 1 DCSM Registers
#define DCSMBANK1_Z2_BASE           0x0005F140U // Zone 2 DCSM Registers
#define MEMCFG_BASE                 0x0005F400U // Memory config registers
#define ACCESSPROTECTION_BASE       0x0005F4C0U // Access protection registers
#define MEMORYERROR_BASE            0x0005F500U // Memory error registers
#define FLASH0CTRL_BASE             0x0005F800U // Flash control registers
#define FLASH0ECC_BASE              0x0005FB00U // Flash ECC error log registers
#define DCSMBANK0_Z1OTP_BASE        0x00078000U // Zone 1 DCSM OTP
#define DCSMBANK0_Z2OTP_BASE        0x00078200U // Zone 2 DCSM OTP
#define DCSMBANK1_Z1OTP_BASE        0x00078400U // Zone 1 DCSM OTP
#define DCSMBANK1_Z2OTP_BASE        0x00078600U // Zone 2 DCSM OTP

#endif // HW_MEMMAP_H

