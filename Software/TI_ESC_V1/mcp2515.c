#include "TI_ESC_V1/mcp2515.h"

// **************************************************************************
// the globals

TXBn_REGS TXB[N_TXBUFFERS] = {
    {MCP_TXB0CTRL, MCP_TXB0SIDH, MCP_TXB0DATA},
    {MCP_TXB1CTRL, MCP_TXB1SIDH, MCP_TXB1DATA},
    {MCP_TXB2CTRL, MCP_TXB2SIDH, MCP_TXB2DATA}
};

RXBn_REGS RXB[N_RXBUFFERS] = {
    {MCP_RXB0CTRL, MCP_RXB0SIDH, MCP_RXB0DATA, CANINTF_RX0IF},
    {MCP_RXB1CTRL, MCP_RXB1SIDH, MCP_RXB1DATA, CANINTF_RX1IF}
};


// **************************************************************************
// function definitions


MCP2515_Handle MCP2515_init(void *pMemory, const size_t numBytes)
{
    MCP2515_Handle handle;

    if(numBytes < sizeof(MCP2515_Obj))
      return((MCP2515_Handle)NULL);

    // assign the handle
    handle = (MCP2515_Handle)pMemory;

    return(handle);
} // end of MCP2515_init() function


void MCP2515_setSpiHandle(MCP2515_Handle handle, SPI_Handle spiHandle)
{
    MCP2515_Obj *obj = (MCP2515_Obj *)handle;

    // initialize the serial peripheral interface object
    obj->spiHandle = spiHandle;

    return;
} // end of MCP2515_setSpiHandle() function


void MCP2515_setGpioHandle(MCP2515_Handle handle,GPIO_Handle gpioHandle)
{
    MCP2515_Obj *obj = (MCP2515_Obj *)handle;

    // initialize the gpio interface object
    obj->gpioHandle = gpioHandle;

    return;
} // end of MCP2515_setGpioHandle() function


void MCP2515_setGpioNumber(MCP2515_Handle handle, GPIO_Number_e gpioNumber)
{
    MCP2515_Obj *obj = (MCP2515_Obj *)handle;

    // initialize the gpio interface object
    obj->gpioCS = gpioNumber;

    return;
} // end of MCP2515_setGpioNumber() function


uint8_t MCP2515_readRegister(MCP2515_Handle handle, REGISTER reg)
{
  MCP2515_Obj *obj = (MCP2515_Obj *)handle;
  volatile uint16_t readWord;
  static volatile uint16_t WaitTimeOut = 0;
  volatile SPI_FifoStatus_e RxFifoCnt = SPI_FifoStatus_Empty;

  //Set CS to Low
  GPIO_setLow(obj->gpioHandle,obj->gpioCS);

  // reset the Rx fifo pointer to zero
  SPI_resetRxFifo(obj->spiHandle);
  SPI_enableRxFifo(obj->spiHandle);

  // write the command
  SPI_write8(obj->spiHandle, INSTRUCTION_READ);

  // write the address
  SPI_write8(obj->spiHandle, reg);

  // wait for the response to populate the RX fifo, else a wait timeout will occur
  while((RxFifoCnt < SPI_FifoStatus_1_Word) && (WaitTimeOut < 0xffff))
  {
    RxFifoCnt = SPI_getRxFifoStatus(obj->spiHandle);
    if(++WaitTimeOut > 0xfffe)
    {
        obj->RxTimeOut = true;
    }
  }

  // Read the word
  readWord = SPI_readEmu(obj->spiHandle);

  //Set CS to High
  GPIO_setHigh(obj->gpioHandle,obj->gpioCS);

  return((uint8_t)readWord);
} // end of MCP2515_readRegister() function


void MCP2515_readRegisters(MCP2515_Handle handle, REGISTER reg, uint8_t values[], const uint8_t n)
{
  MCP2515_Obj *obj = (MCP2515_Obj *)handle;
  static volatile uint16_t WaitTimeOut = 0;
  volatile SPI_FifoStatus_e RxFifoCnt = SPI_FifoStatus_Empty;

  //Set CS to Low
  GPIO_setLow(obj->gpioHandle,obj->gpioCS);

  // reset the Rx fifo pointer to zero
  SPI_resetRxFifo(obj->spiHandle);
  SPI_enableRxFifo(obj->spiHandle);

  // write the command
  SPI_write8(obj->spiHandle, INSTRUCTION_READ);

  // write the address
  SPI_write8(obj->spiHandle, reg);

  uint8_t i;
  for(i=0; i<n; i+=2) {
    // wait for the response to populate the RX fifo, else a wait timeout will occur
    while((RxFifoCnt < SPI_FifoStatus_1_Word) && (WaitTimeOut < 0xffff))
    {
        RxFifoCnt = SPI_getRxFifoStatus(obj->spiHandle);
        if(++WaitTimeOut > 0xfffe)
        {
            obj->RxTimeOut = true;
        }
    }

    // Read the 16 bit FIFO, transfer it to high and low bytes
    values[i]   = (uint8_t)(SPI_readEmu(obj->spiHandle) & 0xFF);
    values[i+1] = (uint8_t)(SPI_readEmu(obj->spiHandle) >> 8);

    // reset the Rx fifo pointer to zero
    SPI_resetRxFifo(obj->spiHandle);
    SPI_enableRxFifo(obj->spiHandle);
  }

  //Set CS to High
  GPIO_setHigh(obj->gpioHandle,obj->gpioCS);

} // end of MCP2515_readRegisters() function


void MCP2515_setRegister(MCP2515_Handle handle, REGISTER reg, const uint8_t value)
{
    MCP2515_Obj *obj = (MCP2515_Obj *)handle;
    uint16_t n;

    //Set CS to Low
    GPIO_setLow(obj->gpioHandle, obj->gpioCS);

    // reset the Rx fifo pointer to zero
    SPI_resetRxFifo(obj->spiHandle);
    SPI_enableRxFifo(obj->spiHandle);

    // write the command
    SPI_write8(obj->spiHandle, INSTRUCTION_WRITE);

    // write the address
    SPI_write8(obj->spiHandle, reg);

    // write the new value
    SPI_write8(obj->spiHandle, value);

    // wait for registers to update
    for(n=0;n<0xf;n++)
      asm(" NOP");

    //Set CS to High
    GPIO_setHigh(obj->gpioHandle, obj->gpioCS);

    return;

} // end of MCP2515_setRegister() function



void MCP2515_setRegisters(MCP2515_Handle handle, REGISTER reg, const uint8_t values[], const uint8_t n)
{
    MCP2515_Obj *obj = (MCP2515_Obj *)handle;
    uint16_t j;

    //Set CS to Low
    GPIO_setLow(obj->gpioHandle, obj->gpioCS);

    // reset the Rx fifo pointer to zero
    SPI_resetRxFifo(obj->spiHandle);
    SPI_enableRxFifo(obj->spiHandle);

    // write the command
    SPI_write8(obj->spiHandle, INSTRUCTION_WRITE);

    // write the address
    SPI_write8(obj->spiHandle, reg);

    // write the new values
    uint8_t i;
    for (i=0; i<n; i++) {
        SPI_write8(obj->spiHandle, values[i]);
    }

    // wait for registers to update
    for(j=0;j<0xf;j++)
      asm(" NOP");

    //Set CS to High
    GPIO_setHigh(obj->gpioHandle, obj->gpioCS);

    return;

} // end of MCP2515_setRegisters() function

void MCP2515_modifyRegister(MCP2515_Handle handle, REGISTER reg, const uint8_t mask, const uint8_t data)
{
    MCP2515_Obj *obj = (MCP2515_Obj *)handle;
    uint16_t n;

    //Set CS to Low
    GPIO_setLow(obj->gpioHandle, obj->gpioCS);

    // reset the Rx fifo pointer to zero
    SPI_resetRxFifo(obj->spiHandle);
    SPI_enableRxFifo(obj->spiHandle);

    // write the command
    SPI_write8(obj->spiHandle, INSTRUCTION_WRITE);

    // write the address
    SPI_write8(obj->spiHandle, reg);

    // write the mask
    SPI_write8(obj->spiHandle, mask);

    // write the new data
    SPI_write8(obj->spiHandle, data);

    // wait for registers to update
    for(n=0;n<0xf;n++)
      asm(" NOP");

    //Set CS to High
    GPIO_setHigh(obj->gpioHandle, obj->gpioCS);

    return;

} // end of MCP2515_modifyRegister() function


MCP_ERROR MCP2515_reset(MCP2515_Handle handle)
{
    MCP2515_Obj *obj = (MCP2515_Obj *)handle;
    uint32_t n;

    //Set CS to Low
    GPIO_setLow(obj->gpioHandle, obj->gpioCS);

    // reset the Rx fifo pointer to zero
    SPI_resetRxFifo(obj->spiHandle);
    SPI_enableRxFifo(obj->spiHandle);

    // write the command
    SPI_write8(obj->spiHandle, INSTRUCTION_WRITE);

    //Set CS to High
    GPIO_setHigh(obj->gpioHandle, obj->gpioCS);

    // wait 10 ms
    for(n=0;n<0x927C0;n++)
      asm(" NOP");

    uint8_t zeros[14];
    memset(zeros, 0, sizeof(zeros));

    MCP2515_setRegisters(handle, MCP_TXB0CTRL, zeros, 14);
    MCP2515_setRegisters(handle, MCP_TXB1CTRL, zeros, 14);
    MCP2515_setRegisters(handle, MCP_TXB2CTRL, zeros, 14);

    MCP2515_setRegister(handle, MCP_RXB0CTRL, 0);
    MCP2515_setRegister(handle, MCP_RXB1CTRL, 0);

    MCP2515_setRegister(handle, MCP_CANINTE, CANINTF_RX0IF | CANINTF_RX1IF | CANINTF_ERRIF | CANINTF_MERRF);

    MCP2515_modifyRegister(handle, MCP_RXB0CTRL, RXBnCTRL_RXM_MASK | RXB0CTRL_BUKT, RXBnCTRL_RXM_STDEXT | RXB0CTRL_BUKT);
    MCP2515_modifyRegister(handle, MCP_RXB1CTRL, RXBnCTRL_RXM_MASK, RXBnCTRL_RXM_STDEXT);

    return ERROR_OK;

} // end of MCP2515_reset() function


MCP_ERROR MCP2515_setMode(MCP2515_Handle handle, const CANCTRL_REQOP_MODE mode)
{
    MCP2515_modifyRegister(handle, MCP_CANCTRL, CANCTRL_REQOP, mode);

    uint32_t n = 0;
    bool modeMatch = false;

    //keep running while loop for about 10 ms
    while (n < 0x186A0) {
        uint8_t newmode = MCP2515_readRegister(handle, MCP_CANSTAT);
        newmode &= CANSTAT_OPMOD;

        modeMatch = (newmode == mode);

        if (modeMatch) {
            break;
        }
        n++;
    }

    return modeMatch ? ERROR_OK : ERROR_FAIL;

} // end of MCP2515_setMode() function


MCP_ERROR MCP2515_setConfigMode(MCP2515_Handle handle)
{
    return MCP2515_setMode(handle, CANCTRL_REQOP_CONFIG);

} // end of MCP2515_setConfigMode() function


MCP_ERROR MCP2515_setListenOnlyMode(MCP2515_Handle handle)
{
    return MCP2515_setMode(handle, CANCTRL_REQOP_LISTENONLY);

} // end of MCP2515_setListenOnlyMode() function


MCP_ERROR MCP2515_setSleepMode(MCP2515_Handle handle)
{
    return MCP2515_setMode(handle, CANCTRL_REQOP_SLEEP);

} // end of MCP2515_setSleepMode() function


MCP_ERROR MCP2515_setLoopbackMode(MCP2515_Handle handle)
{
    return MCP2515_setMode(handle, CANCTRL_REQOP_LOOPBACK);

} // end of MCP2515_setLoopbackMode() function


MCP_ERROR MCP2515_setNormalMode(MCP2515_Handle handle)
{
    return MCP2515_setMode(handle, CANCTRL_REQOP_NORMAL);

} // end of MCP2515_setNormalMode() function


uint8_t MCP2515_getStatus(MCP2515_Handle handle)
{
    MCP2515_Obj *obj = (MCP2515_Obj *)handle;
    volatile uint16_t readWord;
    static volatile uint16_t WaitTimeOut = 0;
    volatile SPI_FifoStatus_e RxFifoCnt = SPI_FifoStatus_Empty;

    //Set CS to Low
    GPIO_setLow(obj->gpioHandle,obj->gpioCS);

    // reset the Rx fifo pointer to zero
    SPI_resetRxFifo(obj->spiHandle);
    SPI_enableRxFifo(obj->spiHandle);

    // write the command
    SPI_write8(obj->spiHandle, INSTRUCTION_READ_STATUS);

    // wait for the response to populate the RX fifo, else a wait timeout will occur
    while((RxFifoCnt < SPI_FifoStatus_1_Word) && (WaitTimeOut < 0xffff))
    {
      RxFifoCnt = SPI_getRxFifoStatus(obj->spiHandle);
      if(++WaitTimeOut > 0xfffe)
      {
          obj->RxTimeOut = true;
      }
    }

    // Read the word
    readWord = SPI_readEmu(obj->spiHandle);

    //Set CS to High
    GPIO_setHigh(obj->gpioHandle,obj->gpioCS);

    return((uint8_t)readWord);

} // end of MCP2515_getStatus() function


MCP_ERROR MCP2515_setBitrate(MCP2515_Handle handle, CAN_SPEED canSpeed, CAN_CLOCK canClock)
{
    MCP_ERROR error = MCP2515_setConfigMode(handle);
    if (error != ERROR_OK) {
        return error;
    }

    uint8_t set, cfg1, cfg2, cfg3;
    set = 1;
    switch (canClock)
    {
        case (MCP_8MHZ):
        switch (canSpeed)
        {
            case (CAN_5KBPS):                                               //   5KBPS
            cfg1 = MCP_8MHz_5kBPS_CFG1;
            cfg2 = MCP_8MHz_5kBPS_CFG2;
            cfg3 = MCP_8MHz_5kBPS_CFG3;
            break;

            case (CAN_10KBPS):                                              //  10KBPS
            cfg1 = MCP_8MHz_10kBPS_CFG1;
            cfg2 = MCP_8MHz_10kBPS_CFG2;
            cfg3 = MCP_8MHz_10kBPS_CFG3;
            break;

            case (CAN_20KBPS):                                              //  20KBPS
            cfg1 = MCP_8MHz_20kBPS_CFG1;
            cfg2 = MCP_8MHz_20kBPS_CFG2;
            cfg3 = MCP_8MHz_20kBPS_CFG3;
            break;

            case (CAN_31K25BPS):                                            //  31.25KBPS
            cfg1 = MCP_8MHz_31k25BPS_CFG1;
            cfg2 = MCP_8MHz_31k25BPS_CFG2;
            cfg3 = MCP_8MHz_31k25BPS_CFG3;
            break;

            case (CAN_33KBPS):                                              //  33.333KBPS
            cfg1 = MCP_8MHz_33k3BPS_CFG1;
            cfg2 = MCP_8MHz_33k3BPS_CFG2;
            cfg3 = MCP_8MHz_33k3BPS_CFG3;
            break;

            case (CAN_40KBPS):                                              //  40Kbps
            cfg1 = MCP_8MHz_40kBPS_CFG1;
            cfg2 = MCP_8MHz_40kBPS_CFG2;
            cfg3 = MCP_8MHz_40kBPS_CFG3;
            break;

            case (CAN_50KBPS):                                              //  50Kbps
            cfg1 = MCP_8MHz_50kBPS_CFG1;
            cfg2 = MCP_8MHz_50kBPS_CFG2;
            cfg3 = MCP_8MHz_50kBPS_CFG3;
            break;

            case (CAN_80KBPS):                                              //  80Kbps
            cfg1 = MCP_8MHz_80kBPS_CFG1;
            cfg2 = MCP_8MHz_80kBPS_CFG2;
            cfg3 = MCP_8MHz_80kBPS_CFG3;
            break;

            case (CAN_100KBPS):                                             // 100Kbps
            cfg1 = MCP_8MHz_100kBPS_CFG1;
            cfg2 = MCP_8MHz_100kBPS_CFG2;
            cfg3 = MCP_8MHz_100kBPS_CFG3;
            break;

            case (CAN_125KBPS):                                             // 125Kbps
            cfg1 = MCP_8MHz_125kBPS_CFG1;
            cfg2 = MCP_8MHz_125kBPS_CFG2;
            cfg3 = MCP_8MHz_125kBPS_CFG3;
            break;

            case (CAN_200KBPS):                                             // 200Kbps
            cfg1 = MCP_8MHz_200kBPS_CFG1;
            cfg2 = MCP_8MHz_200kBPS_CFG2;
            cfg3 = MCP_8MHz_200kBPS_CFG3;
            break;

            case (CAN_250KBPS):                                             // 250Kbps
            cfg1 = MCP_8MHz_250kBPS_CFG1;
            cfg2 = MCP_8MHz_250kBPS_CFG2;
            cfg3 = MCP_8MHz_250kBPS_CFG3;
            break;

            case (CAN_500KBPS):                                             // 500Kbps
            cfg1 = MCP_8MHz_500kBPS_CFG1;
            cfg2 = MCP_8MHz_500kBPS_CFG2;
            cfg3 = MCP_8MHz_500kBPS_CFG3;
            break;

            case (CAN_1000KBPS):                                            //   1Mbps
            cfg1 = MCP_8MHz_1000kBPS_CFG1;
            cfg2 = MCP_8MHz_1000kBPS_CFG2;
            cfg3 = MCP_8MHz_1000kBPS_CFG3;
            break;

            default:
            set = 0;
            break;
        }
        break;

        case (MCP_16MHZ):
        switch (canSpeed)
        {
            case (CAN_5KBPS):                                               //   5Kbps
            cfg1 = MCP_16MHz_5kBPS_CFG1;
            cfg2 = MCP_16MHz_5kBPS_CFG2;
            cfg3 = MCP_16MHz_5kBPS_CFG3;
            break;

            case (CAN_10KBPS):                                              //  10Kbps
            cfg1 = MCP_16MHz_10kBPS_CFG1;
            cfg2 = MCP_16MHz_10kBPS_CFG2;
            cfg3 = MCP_16MHz_10kBPS_CFG3;
            break;

            case (CAN_20KBPS):                                              //  20Kbps
            cfg1 = MCP_16MHz_20kBPS_CFG1;
            cfg2 = MCP_16MHz_20kBPS_CFG2;
            cfg3 = MCP_16MHz_20kBPS_CFG3;
            break;

            case (CAN_33KBPS):                                              //  33.333Kbps
            cfg1 = MCP_16MHz_33k3BPS_CFG1;
            cfg2 = MCP_16MHz_33k3BPS_CFG2;
            cfg3 = MCP_16MHz_33k3BPS_CFG3;
            break;

            case (CAN_40KBPS):                                              //  40Kbps
            cfg1 = MCP_16MHz_40kBPS_CFG1;
            cfg2 = MCP_16MHz_40kBPS_CFG2;
            cfg3 = MCP_16MHz_40kBPS_CFG3;
            break;

            case (CAN_50KBPS):                                              //  50Kbps
            cfg1 = MCP_16MHz_50kBPS_CFG1;
            cfg2 = MCP_16MHz_50kBPS_CFG2;
            cfg3 = MCP_16MHz_50kBPS_CFG3;
            break;

            case (CAN_80KBPS):                                              //  80Kbps
            cfg1 = MCP_16MHz_80kBPS_CFG1;
            cfg2 = MCP_16MHz_80kBPS_CFG2;
            cfg3 = MCP_16MHz_80kBPS_CFG3;
            break;

            case (CAN_83K3BPS):                                             //  83.333Kbps
            cfg1 = MCP_16MHz_83k3BPS_CFG1;
            cfg2 = MCP_16MHz_83k3BPS_CFG2;
            cfg3 = MCP_16MHz_83k3BPS_CFG3;
            break; 

            case (CAN_100KBPS):                                             // 100Kbps
            cfg1 = MCP_16MHz_100kBPS_CFG1;
            cfg2 = MCP_16MHz_100kBPS_CFG2;
            cfg3 = MCP_16MHz_100kBPS_CFG3;
            break;

            case (CAN_125KBPS):                                             // 125Kbps
            cfg1 = MCP_16MHz_125kBPS_CFG1;
            cfg2 = MCP_16MHz_125kBPS_CFG2;
            cfg3 = MCP_16MHz_125kBPS_CFG3;
            break;

            case (CAN_200KBPS):                                             // 200Kbps
            cfg1 = MCP_16MHz_200kBPS_CFG1;
            cfg2 = MCP_16MHz_200kBPS_CFG2;
            cfg3 = MCP_16MHz_200kBPS_CFG3;
            break;

            case (CAN_250KBPS):                                             // 250Kbps
            cfg1 = MCP_16MHz_250kBPS_CFG1;
            cfg2 = MCP_16MHz_250kBPS_CFG2;
            cfg3 = MCP_16MHz_250kBPS_CFG3;
            break;

            case (CAN_500KBPS):                                             // 500Kbps
            cfg1 = MCP_16MHz_500kBPS_CFG1;
            cfg2 = MCP_16MHz_500kBPS_CFG2;
            cfg3 = MCP_16MHz_500kBPS_CFG3;
            break;

            case (CAN_1000KBPS):                                            //   1Mbps
            cfg1 = MCP_16MHz_1000kBPS_CFG1;
            cfg2 = MCP_16MHz_1000kBPS_CFG2;
            cfg3 = MCP_16MHz_1000kBPS_CFG3;
            break;

            default:
            set = 0;
            break;
        }
        break;

        case (MCP_20MHZ):
        switch (canSpeed)
        {
            case (CAN_33KBPS):                                              //  33.333Kbps
            cfg1 = MCP_20MHz_33k3BPS_CFG1;
            cfg2 = MCP_20MHz_33k3BPS_CFG2;
            cfg3 = MCP_20MHz_33k3BPS_CFG3;
        break;

            case (CAN_40KBPS):                                              //  40Kbps
            cfg1 = MCP_20MHz_40kBPS_CFG1;
            cfg2 = MCP_20MHz_40kBPS_CFG2;
            cfg3 = MCP_20MHz_40kBPS_CFG3;
            break;

            case (CAN_50KBPS):                                              //  50Kbps
            cfg1 = MCP_20MHz_50kBPS_CFG1;
            cfg2 = MCP_20MHz_50kBPS_CFG2;
            cfg3 = MCP_20MHz_50kBPS_CFG3;
            break;

            case (CAN_80KBPS):                                              //  80Kbps
            cfg1 = MCP_20MHz_80kBPS_CFG1;
            cfg2 = MCP_20MHz_80kBPS_CFG2;
            cfg3 = MCP_20MHz_80kBPS_CFG3;
            break;

            case (CAN_83K3BPS):                                             //  83.333Kbps
            cfg1 = MCP_20MHz_83k3BPS_CFG1;
            cfg2 = MCP_20MHz_83k3BPS_CFG2;
            cfg3 = MCP_20MHz_83k3BPS_CFG3;
        break;

            case (CAN_100KBPS):                                             // 100Kbps
            cfg1 = MCP_20MHz_100kBPS_CFG1;
            cfg2 = MCP_20MHz_100kBPS_CFG2;
            cfg3 = MCP_20MHz_100kBPS_CFG3;
            break;

            case (CAN_125KBPS):                                             // 125Kbps
            cfg1 = MCP_20MHz_125kBPS_CFG1;
            cfg2 = MCP_20MHz_125kBPS_CFG2;
            cfg3 = MCP_20MHz_125kBPS_CFG3;
            break;

            case (CAN_200KBPS):                                             // 200Kbps
            cfg1 = MCP_20MHz_200kBPS_CFG1;
            cfg2 = MCP_20MHz_200kBPS_CFG2;
            cfg3 = MCP_20MHz_200kBPS_CFG3;
            break;

            case (CAN_250KBPS):                                             // 250Kbps
            cfg1 = MCP_20MHz_250kBPS_CFG1;
            cfg2 = MCP_20MHz_250kBPS_CFG2;
            cfg3 = MCP_20MHz_250kBPS_CFG3;
            break;

            case (CAN_500KBPS):                                             // 500Kbps
            cfg1 = MCP_20MHz_500kBPS_CFG1;
            cfg2 = MCP_20MHz_500kBPS_CFG2;
            cfg3 = MCP_20MHz_500kBPS_CFG3;
            break;

            case (CAN_1000KBPS):                                            //   1Mbps
            cfg1 = MCP_20MHz_1000kBPS_CFG1;
            cfg2 = MCP_20MHz_1000kBPS_CFG2;
            cfg3 = MCP_20MHz_1000kBPS_CFG3;
            break;

            default:
            set = 0;
            break;
        }
        break;

        default:
        set = 0;
        break;
    }

    if (set) {
        MCP2515_setRegister(handle, MCP_CNF1, cfg1);
        MCP2515_setRegister(handle, MCP_CNF2, cfg2);
        MCP2515_setRegister(handle, MCP_CNF3, cfg3);
        return ERROR_OK;
    }
    else {
        return ERROR_FAIL;
    }

} // end of MCP2515_setBitrate() function


MCP_ERROR MCP2515_setClkOut(MCP2515_Handle handle, const CAN_CLKOUT divisor)
{
    if (divisor == CLKOUT_DISABLE) {
    /* Turn off CLKEN */
    MCP2515_modifyRegister(handle, MCP_CANCTRL, CANCTRL_CLKEN, 0x00);

    /* Turn on CLKOUT for SOF */
    MCP2515_modifyRegister(handle, MCP_CNF3, CNF3_SOF, CNF3_SOF);
        return ERROR_OK;
    }

    /* Set the prescaler (CLKPRE) */
    MCP2515_modifyRegister(handle, MCP_CANCTRL, CANCTRL_CLKPRE, divisor);

    /* Turn on CLKEN */
    MCP2515_modifyRegister(handle, MCP_CANCTRL, CANCTRL_CLKEN, CANCTRL_CLKEN);

    /* Turn off CLKOUT for SOF */
    MCP2515_modifyRegister(handle, MCP_CNF3, CNF3_SOF, 0x00);

    return ERROR_OK;

} // end of MCP2515_setClkOut() function



void MCP2515_prepareId(uint8_t *buffer, const bool ext, const uint32_t id)
{
    uint16_t canid = (uint16_t)(id & 0x0FFFF);

    if (ext) {
        buffer[MCP_EID0] = (uint8_t) (canid & 0xFF);
        buffer[MCP_EID8] = (uint8_t) (canid >> 8);
        canid = (uint16_t)(id >> 16);
        buffer[MCP_SIDL] = (uint8_t) (canid & 0x03);
        buffer[MCP_SIDL] += (uint8_t) ((canid & 0x1C) << 3);
        buffer[MCP_SIDL] |= TXB_EXIDE_MASK;
        buffer[MCP_SIDH] = (uint8_t) (canid >> 5);
    } else {
        buffer[MCP_SIDH] = (uint8_t) (canid >> 3);
        buffer[MCP_SIDL] = (uint8_t) ((canid & 0x07 ) << 5);
        buffer[MCP_EID0] = 0;
        buffer[MCP_EID8] = 0;
    }
} // end of MCP2515_prepareId() function


MCP_ERROR MCP2515_setFilterMask(MCP2515_Handle handle, const MASK mask, const bool ext, const uint32_t ulData)
{
    MCP_ERROR res = MCP2515_setConfigMode(handle);
    if (res != ERROR_OK) {
        return res;
    }

    uint8_t tbufdata[4];
    MCP2515_prepareId(tbufdata, ext, ulData);

    REGISTER reg;
    switch (mask) {
        case MASK0: reg = MCP_RXM0SIDH; break;
        case MASK1: reg = MCP_RXM1SIDH; break;
        default:
            return ERROR_FAIL;
    }

    MCP2515_setRegisters(handle, reg, tbufdata, 4);

    return ERROR_OK;

} // end of MCP2515_setFilterMask() function

MCP_ERROR MCP2515_setFilter(MCP2515_Handle handle, const RXF num, const bool ext, const uint32_t ulData)
{
    MCP_ERROR res = MCP2515_setConfigMode(handle);
    if (res != ERROR_OK) {
        return res;
    }

    REGISTER reg;

    switch (num) {
        case RXF0: reg = MCP_RXF0SIDH; break;
        case RXF1: reg = MCP_RXF1SIDH; break;
        case RXF2: reg = MCP_RXF2SIDH; break;
        case RXF3: reg = MCP_RXF3SIDH; break;
        case RXF4: reg = MCP_RXF4SIDH; break;
        case RXF5: reg = MCP_RXF5SIDH; break;
        default:
            return ERROR_FAIL;
    }

    uint8_t tbufdata[4];
    MCP2515_prepareId(tbufdata, ext, ulData);
    MCP2515_setRegisters(handle, reg, tbufdata, 4);

    return ERROR_OK;

} // end of MCP2515_setFilter() function


MCP_ERROR MCP2515_readMessage(MCP2515_Handle handle, can_frame *frame)
{
    MCP_ERROR rc;
    uint8_t stat = MCP2515_getStatus(handle);

    if ( stat & STAT_RX0IF ) {
        rc = MCP2515_readMessages(handle, RXB0, frame);
    } else if ( stat & STAT_RX1IF ) {
        rc = MCP2515_readMessages(handle, RXB1, frame);
    } else {
        rc = ERROR_NOMSG;
    }

    return rc;

} // end of MCP2515_readMessage() function


MCP_ERROR MCP2515_readMessages(MCP2515_Handle handle, RXBn rxbn, can_frame *frame)
{
    RXBn_REGS *rxb = &RXB[rxbn];

    uint8_t tbufdata[5];

    MCP2515_readRegisters(handle, rxb->SIDH, tbufdata, 5);

    uint32_t id = (tbufdata[MCP_SIDH]<<3) + (tbufdata[MCP_SIDL]>>5);

    if ( (tbufdata[MCP_SIDL] & TXB_EXIDE_MASK) ==  TXB_EXIDE_MASK ) {
        id = (id<<2) + (tbufdata[MCP_SIDL] & 0x03);
        id = (id<<8) + tbufdata[MCP_EID8];
        id = (id<<8) + tbufdata[MCP_EID0];
        id |= CAN_EFF_FLAG;
    }

    uint8_t dlc = (tbufdata[MCP_DLC] & DLC_MASK);
    if (dlc > CAN_MAX_DLEN) {
        return ERROR_FAIL;
    }

    uint8_t ctrl =  MCP2515_readRegister(handle, rxb->CTRL);
    if (ctrl & RXBnCTRL_RTR) {
        id |= CAN_RTR_FLAG;
    }

    frame->can_id = id;
    frame->can_dlc = dlc;

    MCP2515_readRegisters(handle, rxb->DATA, frame->data, dlc);

    MCP2515_modifyRegister(handle, MCP_CANINTF, rxb->CANINTF_RXnIF, 0);

    return ERROR_OK;

} // end of MCP2515_readMessages() function


MCP_ERROR MCP2515_sendMessage(MCP2515_Handle handle, can_frame *frame)
{
    if (frame->can_dlc > CAN_MAX_DLEN) {
        return ERROR_FAILTX;
    }

    TXBn txBuffers[N_TXBUFFERS] = {TXB0, TXB1, TXB2};
    int i;
    for (i=0; i<N_TXBUFFERS; i++) {
        TXBn_REGS *txbuf = &TXB[txBuffers[i]];
        uint8_t ctrlval = MCP2515_readRegister(handle, txbuf->CTRL);
        if ( (ctrlval & TXB_TXREQ) == 0 ) {
            return MCP2515_sendMessages(handle, txBuffers[i], frame);
        }
    }

    return ERROR_FAILTX;

} // end of MCP2515_sendMessage() function


MCP_ERROR MCP2515_sendMessages(MCP2515_Handle handle, TXBn txbn, can_frame *frame)
{
    TXBn_REGS *txbuf = &TXB[txbn];

    uint8_t data[13];

    bool ext = (frame->can_id & CAN_EFF_FLAG);
    bool rtr = (frame->can_id & CAN_RTR_FLAG);
    uint32_t id = (frame->can_id & (ext ? CAN_EFF_MASK : CAN_SFF_MASK));

    MCP2515_prepareId(data, ext, id);

    data[MCP_DLC] = rtr ? (frame->can_dlc | RTR_MASK) : frame->can_dlc;

    memcpy(&data[MCP_DATA], frame->data, frame->can_dlc);

    MCP2515_setRegisters(handle, txbuf->SIDH, data, 5 + frame->can_dlc);

    MCP2515_modifyRegister(handle, txbuf->CTRL, TXB_TXREQ, TXB_TXREQ);

    return ERROR_OK;

} // end of MCP2515_sendMessages() function


bool MCP2515_checkReceive(MCP2515_Handle handle)
{
    uint8_t res =  MCP2515_getStatus(handle);
    if ( res & STAT_RXIF_MASK ) {
        return true;
    } else {
        return false;
    }

} // end of MCP2515_checkReceive() function


bool MCP2515_checkError(MCP2515_Handle handle)
{
    uint8_t eflg =  MCP2515_getErrorFlags(handle);

    if ( eflg & EFLG_ERRORMASK ) {
        return true;
    } else {
        return false;
    }

} // end of MCP2515_checkError() function


uint8_t MCP2515_getErrorFlags(MCP2515_Handle handle)
{
    return  MCP2515_readRegister(handle, MCP_EFLG);

} // end of MCP2515_getErrorFlags() function


void MCP2515_clearRXnOVRFlags(MCP2515_Handle handle)
{
    MCP2515_modifyRegister(handle, MCP_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0);

} // end of MCP2515_clearRXnOVRFlags() function


uint8_t MCP2515_getInterrupts(MCP2515_Handle handle)
{
    return  MCP2515_readRegister(handle, MCP_CANINTF);

} // end of MCP2515_getInterrupts() function


void MCP2515_clearInterrupts(MCP2515_Handle handle)
{
    MCP2515_setRegister(handle, MCP_CANINTF, 0);

} // end of MCP2515_clearInterrupts() function


uint8_t MCP2515_getInterruptMask(MCP2515_Handle handle)
{
    return  MCP2515_readRegister(handle, MCP_CANINTE);

} // end of MCP2515_getInterruptMask() function


void MCP2515_clearTXInterrupts(MCP2515_Handle handle)
{
    MCP2515_modifyRegister(handle, MCP_CANINTF, (CANINTF_TX0IF | CANINTF_TX1IF | CANINTF_TX2IF), 0);

} // end of MCP2515_clearTXInterrupts() function


void MCP2515_clearRXnOVR(MCP2515_Handle handle)
{
	uint8_t eflg = MCP2515_getErrorFlags(handle);
	if (eflg != 0) {
	    MCP2515_clearRXnOVRFlags(handle);
	    MCP2515_clearInterrupts(handle);
		//modifyRegister(MCP_CANINTF, CANINTF_ERRIF, 0);
	}
	
} // end of MCP2515_clearRXnOVR() function


void MCP2515_clearMERR(MCP2515_Handle handle)
{
	//modifyRegister(MCP_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0);
	//clearInterrupts();
    MCP2515_modifyRegister(handle, MCP_CANINTF, CANINTF_MERRF, 0);

} // end of MCP2515_clearMERR() function


void MCP2515_clearERRIF(MCP2515_Handle handle)
{
    //modifyRegister(MCP_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0);
    //clearInterrupts();
    MCP2515_modifyRegister(handle, MCP_CANINTF, CANINTF_ERRIF, 0);

} // end of MCP2515_clearERRIF() function


