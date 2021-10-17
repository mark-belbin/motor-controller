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
//! \file   drivers/pie/src/32b/f28x/f2802x/pie.c
//! \brief  Contains the various functions related to the peripheral interrupt
//!         expansion (PIE) object
//!
//! (C) Copyright 2015, Texas Instruments, Inc.

// **************************************************************************
// the includes

#include "TI_ESC_V1/pie.h"


// **************************************************************************
// the defines


// **************************************************************************
// the globals


// **************************************************************************
// the functions

void PIE_clearAllFlags(PIE_Handle pieHandle)
{
    PIE_Obj *pie = (PIE_Obj *)pieHandle;
    uint_least8_t groupCnt;


    for(groupCnt=0;groupCnt<12;groupCnt++)
    {
      pie->PIEIER_PIEIFR[groupCnt].IFR = 0;
    }

    return;
} // end of PIE_clearAllFlags() function


void PIE_clearAllInts(PIE_Handle pieHandle)
{
    PIE_Obj *pie = (PIE_Obj *)pieHandle;


    // set the bits
    pie->PIEACK |= 0xFFFF;

    return;
} // end of PIE_clearAllInts() function


void PIE_disable(PIE_Handle pieHandle)
{
    PIE_Obj *pie = (PIE_Obj *)pieHandle;


    // clear the bits
    pie->PIECTRL &= (~PIE_PIECTRL_ENPIE_BITS);

    return;
} // end of PIE_disable() function

void PIE_disableCaptureInt(PIE_Handle pieHandle)
{
    PIE_Obj *pie = (PIE_Obj *)pieHandle;

    // set the value
    pie->PIEIER_PIEIFR[3].IER &= ~PIE_IERx_INTx1_BITS;

    return;
} // end of PIE_disableCaptureInt() function


void PIE_disableExtInt(PIE_Handle pieHandle, const CPU_ExtIntNumber_e intNumber)
{
    PIE_Obj *pie = (PIE_Obj *)pieHandle;


    // clear the bits
    pie->XINTnCR[intNumber] &= (~PIE_XINTnCR_ENABLE_BITS);

    return;
} // end of PIE_disableExtInt() function


void PIE_disableAllInts(PIE_Handle pieHandle)
{
    PIE_Obj *pie = (PIE_Obj *)pieHandle;

    uint_least8_t groupCnt;

    for(groupCnt=0;groupCnt<12;groupCnt++)
    {
      pie->PIEIER_PIEIFR[groupCnt].IER = 0;
    }

    return;
} // end of PIE_disableAllInts() function

void PIE_disableInt(PIE_Handle pieHandle, const PIE_GroupNumber_e group, const PIE_InterruptSource_e intSource)
{
    PIE_Obj *pie = (PIE_Obj *)pieHandle;
    
    pie->PIEIER_PIEIFR[group].IER &= ~intSource;
    
    return;
}


void PIE_enable(PIE_Handle pieHandle)
{
    PIE_Obj *pie = (PIE_Obj *)pieHandle;


    // set the bits
    pie->PIECTRL |= PIE_PIECTRL_ENPIE_BITS;

    return;
} // end of PIE_enable() function


void PIE_enableAdcInt(PIE_Handle pieHandle, const ADC_IntNumber_e intNumber)
{
    PIE_Obj *pie = (PIE_Obj *)pieHandle;
    uint16_t index;
    uint16_t setValue;


    if(intNumber < ADC_IntNumber_9)
    {
      index = 9;
      setValue = 1 << intNumber;
    }
    else if(intNumber == ADC_IntNumber_9)
    {
      index = 0;
      setValue = 1 << 5;
    }
    else
    {
      index = 0;
      setValue = 1 << ((intNumber & 0x07) - 1) ;
    }

    // set the value
    pie->PIEIER_PIEIFR[index].IER |= setValue;

    return;
} // end of PIE_enableAdcInt() function

void PIE_enableCaptureInt(PIE_Handle pieHandle)
{
    PIE_Obj *pie = (PIE_Obj *)pieHandle;

    // set the value
    pie->PIEIER_PIEIFR[3].IER |= PIE_IERx_INTx1_BITS;

    return;
} // end of PIE_enableCaptureInt() function


void PIE_enableExtInt(PIE_Handle pieHandle, const CPU_ExtIntNumber_e intNumber)
{
    PIE_Obj *pie = (PIE_Obj *)pieHandle;
    uint16_t index;
    uint16_t setValue;

    
    if(intNumber < CPU_ExtIntNumber_3)
    {
      index = 0;
      setValue = 1 << (intNumber + 3);
    }
    else
    {
      index = 10;
      setValue = 1 << 0;
    }


    // set the value
    pie->PIEIER_PIEIFR[index].IER |= setValue;


    // set the bits
    pie->XINTnCR[intNumber] |= PIE_XINTnCR_ENABLE_BITS;

    return;
} // end of PIE_enableExtInt() function

void PIE_enableInt(PIE_Handle pieHandle, const PIE_GroupNumber_e group, const PIE_InterruptSource_e intSource)
{
    PIE_Obj *pie = (PIE_Obj *)pieHandle;
    
    pie->PIEIER_PIEIFR[group].IER |= intSource;
    
    return;
}


void PIE_enablePwmInt(PIE_Handle pieHandle, const PWM_Number_e pwmNumber)
{
    PIE_Obj *pie = (PIE_Obj *)pieHandle;
    uint16_t index = 2;
    uint16_t setValue = (1 << pwmNumber);


    // set the value
    pie->PIEIER_PIEIFR[index].IER |= setValue;

    return;
} // end of PIE_enablePwmInt() function

void PIE_enablePwmTzInt(PIE_Handle pieHandle, const PWM_Number_e pwmNumber)
{
    PIE_Obj *pie = (PIE_Obj *)pieHandle;
    uint16_t index = 1;
    uint16_t setValue = (1 << pwmNumber);


    // set the value
    pie->PIEIER_PIEIFR[index].IER |= setValue;

    return;
} // end of PIE_enablePwmTzInt() function

void PIE_enableTimer0Int(PIE_Handle pieHandle)
{
    PIE_Obj *pie = (PIE_Obj *)pieHandle;

    // set the value
    pie->PIEIER_PIEIFR[0].IER |= PIE_IERx_INTx7_BITS;

    return;
} // end of PIE_enableTimer0Int() function


void PIE_forceInt(PIE_Handle pieHandle, const PIE_GroupNumber_e group, const PIE_InterruptSource_e intSource)
{
    PIE_Obj *pie = (PIE_Obj *)pieHandle;
    
    pie->PIEIER_PIEIFR[group].IFR |= intSource;
    
    return;
}


uint16_t PIE_getExtIntCount(PIE_Handle pieHandle, const CPU_ExtIntNumber_e intNumber)
{
    PIE_Obj *pie = (PIE_Obj *)pieHandle;


    // get the count value
    uint16_t count = pie->XINTnCTR[intNumber];

    return(count);
} // end of PIE_getExtIntCount() function


uint16_t PIE_getIntEnables(PIE_Handle pieHandle, const PIE_GroupNumber_e group)
{
    PIE_Obj *pie = (PIE_Obj *)pieHandle;
    
    return (pie->PIEIER_PIEIFR[group].IER);
} // end of PIE_getIntEnables() function


uint16_t PIE_getIntFlags(PIE_Handle pieHandle, const PIE_GroupNumber_e group)
{
    PIE_Obj *pie = (PIE_Obj *)pieHandle;
    
    return (pie->PIEIER_PIEIFR[group].IFR);
} // end of PIE_getIntFlags() function


interrupt void PIE_illegalIsr(void)
{

    // The next two lines are placeholders
    asm(" ESTOP0");

    // endless hold loop
    for(;;);

} // end of PIE_illegalIsr() function


PIE_Handle PIE_init(void *pMemory, const size_t numBytes)
{
    PIE_Handle pieHandle;


    if(numBytes < sizeof(PIE_Obj))
    return((PIE_Handle)NULL);

    // assign the handle
    pieHandle = (PIE_Handle)pMemory;

    return(pieHandle);
} // end of PIE_init() function



void PIE_registerPieIntHandler(PIE_Handle pieHandle, 
                           const PIE_GroupNumber_e groupNumber, 
                           const PIE_SubGroupNumber_e subGroupNumber, 
                           const PIE_IntVec_t vector)
{
    PIE_Obj *pie = (PIE_Obj *)pieHandle;
    PIE_IntVec_t *intPointer;
    
    ENABLE_PROTECTED_REGISTER_WRITE_MODE;
    
    // Point to the beginning of the PIE table
    intPointer = (PIE_IntVec_t *)&(pie->ADCINT1_HP);

    // Increment pointer to the correct group
    intPointer += groupNumber * 8;

    // Increment point to the correct subgroup
    intPointer += subGroupNumber;

    // Set the vector
    *intPointer = vector;

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;
    
    return;
} // end of PIE_registerIntHandler() function


void PIE_registerSystemIntHandler(PIE_Handle pieHandle, 
                           const PIE_SystemInterrupts_e systemInt, 
                           const PIE_IntVec_t vector)
{
    PIE_Obj *pie = (PIE_Obj *)pieHandle;
    PIE_IntVec_t *intPointer;
    
    ENABLE_PROTECTED_REGISTER_WRITE_MODE;
    
    // Point to the beginning of the system interrupt table
    intPointer = (PIE_IntVec_t *)&(pie->Reset);

    // Increment point to the correct interrupt
    intPointer += systemInt;

    // Set the vector
    *intPointer = vector;
    
    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

    return;
} // end of PIE_registerIntHandler() function


void PIE_setDefaultIntVectorTable(PIE_Handle pieHandle)
{
    PIE_Obj *pie = (PIE_Obj *)pieHandle;
    PIE_IntVec_t *addr = (PIE_IntVec_t *)&(pie->INT1);
    uint16_t regCnt;
    
    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // initialize the table to PIE_illegalIsr() address
    for(regCnt=0;regCnt<120;regCnt++)
    {
      *addr++ = &PIE_illegalIsr;
    }
    
    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

    return;
} // end of PIE_setDefaultIntVectorTable() function


#ifdef _DEBUG
void PIE_setDebugIntVectorTable(PIE_Handle pieHandle)
{
    PIE_Obj *pie = (PIE_Obj *)pieHandle;
    PIE_IntVec_t *dest = (PIE_IntVec_t *)&(pie->INT1);
    PIE_IntVec_t *src = (PIE_IntVec_t *)PIE_tableDebugInit;
    uint16_t regCnt;
    
    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // initialize the table to PIE_illegalIsr() address
    for(regCnt=0;regCnt<120;regCnt++)
    {
      *dest++ = *src++;
    }
    
    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

    return;
} // end of PIE_setDebugIntVectorTable() function
#endif // enf of #ifdef DEBUG


void PIE_setExtIntPolarity(PIE_Handle pieHandle, 
                           const CPU_ExtIntNumber_e intNumber, 
                           const PIE_ExtIntPolarity_e polarity)
{
    PIE_Obj *pie = (PIE_Obj *)pieHandle;


    // clear the bits
    pie->XINTnCR[intNumber] &= (~PIE_XINTnCR_POLARITY_BITS);

    // set the bits
    pie->XINTnCR[intNumber] |= polarity;

    return;
} // end of PIE_setExtIntPolarity() function


void PIE_unregisterPieIntHandler(PIE_Handle pieHandle, 
                           const PIE_GroupNumber_e groupNumber, 
                           const PIE_SubGroupNumber_e subGroupNumber)
{
    PIE_Obj *pie = (PIE_Obj *)pieHandle;
    PIE_IntVec_t *intPointer;
    
    ENABLE_PROTECTED_REGISTER_WRITE_MODE;
    
    // Point to the beginning of the PIE table
    intPointer = (PIE_IntVec_t *)&(pie->ADCINT1_HP);

    // Increment pointer to the correct group
    intPointer += groupNumber * 8;

    // Increment point to the correct subgroup
    intPointer += subGroupNumber;

    // Set the vector
    *intPointer = PIE_illegalIsr;
    
    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

    return;
} // end of PIE_unregisterPieIntHandler() function


void PIE_unregisterSystemIntHandler(PIE_Handle pieHandle, 
                           const PIE_SystemInterrupts_e systemInt)
{
    PIE_Obj *pie = (PIE_Obj *)pieHandle;
    PIE_IntVec_t *intPointer;
    
    ENABLE_PROTECTED_REGISTER_WRITE_MODE;
    
    // Point to the beginning of the system interrupt table
    intPointer = (PIE_IntVec_t *)&(pie->Reset);

    // Increment point to the correct interrupt
    intPointer += systemInt;

    // Set the vector
    *intPointer = PIE_illegalIsr;
    
    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

    return;
} // end of PIE_unregisterSystemIntHandler() function

// end of file
