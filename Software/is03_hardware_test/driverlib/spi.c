//###########################################################################
//
// FILE:   spi.c
//
// TITLE:  C28x SPI driver.
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

#include "spi.h"

//*****************************************************************************
//
// SPI_setConfig
//
//*****************************************************************************
void
SPI_setConfig(uint32_t base, uint32_t lspclkHz, SPI_TransferProtocol protocol,
              SPI_Mode mode, uint32_t bitRate, uint16_t dataWidth)
{
    uint16_t regValue;
    uint32_t baud;

    //
    // Check the arguments.
    //
    ASSERT(SPI_isBaseValid(base));
    ASSERT(bitRate <= (lspclkHz / 4U));
    ASSERT((lspclkHz / bitRate) <= 128U);
    ASSERT((dataWidth >= 1U) && (dataWidth <= 16U));
    ASSERT((HWREGH(base + SPI_O_CCR) & SPI_CCR_SPISWRESET) == 0U);

    //
    // Set polarity and data width.
    //
    regValue = (((uint16_t)protocol << 6U) & SPI_CCR_CLKPOLARITY) |
               (dataWidth - 1U);

    HWREGH(base + SPI_O_CCR) = (HWREGH(base + SPI_O_CCR) &
                                ~(SPI_CCR_CLKPOLARITY | SPI_CCR_SPICHAR_M)) |
                               regValue;

    //
    // Set the mode and phase.
    //
    regValue = (uint16_t)mode | (((uint16_t)protocol << 2U) &
                                 SPI_CTL_CLK_PHASE);

    HWREGH(base + SPI_O_CTL) = (HWREGH(base + SPI_O_CTL) &
                                ~(SPI_CTL_TALK | SPI_CTL_MASTER_SLAVE |
                                  SPI_CTL_CLK_PHASE)) | regValue;

    //
    // Set the clock.
    //
    baud = (lspclkHz / bitRate) - 1U;
    HWREGH(base + SPI_O_BRR) = (uint16_t)baud;
}

//*****************************************************************************
//
// SPI_setBaudRate
//
//*****************************************************************************
void
SPI_setBaudRate(uint32_t base, uint32_t lspclkHz, uint32_t bitRate)
{
    uint32_t baud;

    //
    // Check the arguments.
    //
    ASSERT(SPI_isBaseValid(base));
    ASSERT(bitRate <= (lspclkHz / 4U));
    ASSERT((lspclkHz / bitRate) <= 128U);

    //
    // Set the clock.
    //
    baud = (lspclkHz / bitRate) - 1U;
    HWREGH(base + SPI_O_BRR) = (uint16_t)baud;
}

//*****************************************************************************
//
// SPI_enableInterrupt
//
//*****************************************************************************
void
SPI_enableInterrupt(uint32_t base, uint32_t intFlags)
{
    //
    // Check the arguments.
    //
    ASSERT(SPI_isBaseValid(base));

    //
    // Enable the specified non-FIFO interrupts.
    //
    if((intFlags & SPI_INT_RX_DATA_TX_EMPTY) != 0U)
    {
        HWREGH(base + SPI_O_CTL) |= SPI_CTL_SPIINTENA;
    }

    if((intFlags & SPI_INT_RX_OVERRUN) != 0U)
    {
        HWREGH(base + SPI_O_CTL) |= SPI_CTL_OVERRUNINTENA;
    }

    //
    // Enable the specified FIFO-mode interrupts.
    //
    if((intFlags & SPI_INT_TXFF) != 0U)
    {
        HWREGH(base + SPI_O_FFTX) |= SPI_FFTX_TXFFIENA;
    }

    if((intFlags & (SPI_INT_RXFF | SPI_INT_RXFF_OVERFLOW)) != 0U)
    {
        HWREGH(base + SPI_O_FFRX) |= SPI_FFRX_RXFFIENA;
    }
}

//*****************************************************************************
//
// SPI_disableInterrupt
//
//*****************************************************************************
void
SPI_disableInterrupt(uint32_t base, uint32_t intFlags)
{
    //
    // Check the arguments.
    //
    ASSERT(SPI_isBaseValid(base));

    //
    // Disable the specified non-FIFO interrupts.
    //
    if((intFlags & SPI_INT_RX_DATA_TX_EMPTY) != 0U)
    {
        HWREGH(base + SPI_O_CTL) &= ~(SPI_CTL_SPIINTENA);
    }

    if((intFlags & SPI_INT_RX_OVERRUN) != 0U)
    {
        HWREGH(base + SPI_O_CTL) &= ~(SPI_CTL_OVERRUNINTENA);
    }

    //
    // Disable the specified FIFO-mode interrupts.
    //
    if((intFlags & SPI_INT_TXFF) != 0U)
    {
        HWREGH(base + SPI_O_FFTX) &= ~(SPI_FFTX_TXFFIENA);
    }

    if((intFlags & (SPI_INT_RXFF | SPI_INT_RXFF_OVERFLOW)) != 0U)
    {
        HWREGH(base + SPI_O_FFRX) &= ~(SPI_FFRX_RXFFIENA);
    }
}

//*****************************************************************************
//
// SPI_getInterruptStatus
//
//*****************************************************************************
uint32_t
SPI_getInterruptStatus(uint32_t base)
{
    uint32_t temp = 0;

    //
    // Check the arguments.
    //
    ASSERT(SPI_isBaseValid(base));

    if((HWREGH(base + SPI_O_STS) & SPI_STS_INT_FLAG) != 0U)
    {
        temp |= SPI_INT_RX_DATA_TX_EMPTY;
    }

    if((HWREGH(base + SPI_O_STS) & SPI_STS_OVERRUN_FLAG) != 0U)
    {
        temp |= SPI_INT_RX_OVERRUN;
    }

    if((HWREGH(base + SPI_O_FFTX) & SPI_FFTX_TXFFINT) != 0U)
    {
        temp |= SPI_INT_TXFF;
    }

    if((HWREGH(base + SPI_O_FFRX) & SPI_FFRX_RXFFINT) != 0U)
    {
        temp |= SPI_INT_RXFF;
    }

    if((HWREGH(base + SPI_O_FFRX) & SPI_FFRX_RXFFOVF) != 0U)
    {
        temp |= SPI_INT_RXFF_OVERFLOW;
    }

    return(temp);
}

//*****************************************************************************
//
// SPI_clearInterruptStatus
//
//*****************************************************************************
void
SPI_clearInterruptStatus(uint32_t base, uint32_t intFlags)
{
    //
    // Check the arguments.
    //
    ASSERT(SPI_isBaseValid(base));

    //
    // Clear the specified non-FIFO interrupt sources.
    //
    if((intFlags & SPI_INT_RX_DATA_TX_EMPTY) != 0U)
    {
        HWREGH(base + SPI_O_CCR) &= ~(SPI_CCR_SPISWRESET);
        HWREGH(base + SPI_O_CCR) |= SPI_CCR_SPISWRESET;
    }

    if((intFlags & SPI_INT_RX_OVERRUN) != 0U)
    {
        HWREGH(base + SPI_O_STS) |= SPI_STS_OVERRUN_FLAG;
    }

    //
    // Clear the specified FIFO-mode interrupt sources.
    //
    if((intFlags & SPI_INT_TXFF) != 0U)
    {
        HWREGH(base + SPI_O_FFTX) |= SPI_FFTX_TXFFINTCLR;
    }

    if((intFlags & SPI_INT_RXFF) != 0U)
    {
        HWREGH(base + SPI_O_FFRX) |= SPI_FFRX_RXFFINTCLR;
    }

    if((intFlags & SPI_INT_RXFF_OVERFLOW) != 0U)
    {
        HWREGH(base + SPI_O_FFRX) |= SPI_FFRX_RXFFOVFCLR;
    }
}
