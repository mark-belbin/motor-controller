//###########################################################################
//
// FILE:   adc.c
//
// TITLE:  C28x ADC driver.
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

#include "adc.h"

//*****************************************************************************
//
// Defines for locations of ADC calibration functions in OTP for use in
// ADC_setVREF(). Not intended for use by application code.
//
//*****************************************************************************
#define ADC_OFFSET_TRIM_OTP    0x70594U

//*****************************************************************************
//
// ADC_setVREF
//
//*****************************************************************************
void
ADC_setVREF(uint32_t base, ADC_ReferenceMode refMode,
            ADC_ReferenceVoltage refVoltage)
{
    uint16_t *offset;
    uint16_t moduleShiftVal;
    uint16_t offsetShiftVal;

    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));

    //
    // Assign a shift amount corresponding to which ADC module is being
    // configured.
    //
    switch(base)
    {
        case ADCA_BASE:
            moduleShiftVal = 0U;
            break;
        case ADCB_BASE:
            moduleShiftVal = 1U;
            break;
        case ADCC_BASE:
            moduleShiftVal = 2U;
            break;
        default:
            //
            // Invalid base address!!
            //
            moduleShiftVal = 0U;
            break;
    }

    //
    // Offset trim for internal VREF 3.3V is unique and stored in upper byte.
    //
    if((refMode == ADC_REFERENCE_INTERNAL) &&
       (refVoltage == ADC_REFERENCE_3_3V))
    {
        offsetShiftVal = 8U;
    }
    else
    {
        offsetShiftVal = 0U;
    }

    //
    // Set up pointer to offset trim in OTP.
    //
    offset = (uint16_t *)(ADC_OFFSET_TRIM_OTP + ((uint32_t)6U *
                                                 (uint32_t)moduleShiftVal));

    //
    // Get offset trim from OTP and write it to the register.
    //
    EALLOW;
    HWREGH(base + ADC_O_OFFTRIM) = (*offset >> offsetShiftVal) & 0xFFU;

    //
    // Configure the reference mode (internal or external).
    //
    if(refMode == ADC_REFERENCE_INTERNAL)
    {
        HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_ANAREFCTL) &=
            ~(ASYSCTL_ANAREFCTL_ANAREFASEL << moduleShiftVal);
    }
    else
    {
        HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_ANAREFCTL) |=
            ASYSCTL_ANAREFCTL_ANAREFASEL << moduleShiftVal;
    }

    //
    // Configure the reference voltage (3.3V or 2.5V).
    //
    if(refVoltage == ADC_REFERENCE_3_3V)
    {
        HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_ANAREFCTL) &=
            ~(ASYSCTL_ANAREFCTL_ANAREFA2P5SEL << moduleShiftVal);
    }
    else
    {
        HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_ANAREFCTL) |=
            ASYSCTL_ANAREFCTL_ANAREFA2P5SEL << moduleShiftVal;
    }

    EDIS;
}

//*****************************************************************************
//
// ADC_setPPBTripLimits
//
//*****************************************************************************
void
ADC_setPPBTripLimits(uint32_t base, ADC_PPBNumber ppbNumber,
                     int32_t tripHiLimit, int32_t tripLoLimit)
{
    uint32_t ppbHiOffset;
    uint32_t ppbLoOffset;

    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));
    ASSERT((tripHiLimit <= 65535) && (tripHiLimit >= -65536));
    ASSERT((tripLoLimit <= 65535) && (tripLoLimit >= -65536));

    //
    // Get the offset to the appropriate trip limit registers.
    //
    ppbHiOffset = (ADC_PPBxTRIPHI_STEP * (uint32_t)ppbNumber) +
                  ADC_O_PPB1TRIPHI;
    ppbLoOffset = (ADC_PPBxTRIPLO_STEP * (uint32_t)ppbNumber) +
                  ADC_O_PPB1TRIPLO;

    EALLOW;

    //
    // Set the trip high limit.
    //
    HWREG(base + ppbHiOffset) =
        (HWREG(base + ppbHiOffset) & ~ADC_PPBTRIP_MASK) |
        ((uint32_t)tripHiLimit & ADC_PPBTRIP_MASK);

    //
    // Set the trip low limit.
    //
    HWREG(base + ppbLoOffset) =
        (HWREG(base + ppbLoOffset) & ~ADC_PPBTRIP_MASK) |
        ((uint32_t)tripLoLimit & ADC_PPBTRIP_MASK);

    EDIS;
}
