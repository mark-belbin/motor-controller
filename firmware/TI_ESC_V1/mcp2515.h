/********************************************/
/**********MCP2515 CAN Libarary**************/
/********************************************/

//Ported from the Arduino MCP2515 Library

/*
 * mcp2515.c
 *
 *  Created on: Feb 25, 2020
 *      Author: markb
 */

#ifndef _MCP2515_H_
#define _MCP2515_H_

#include "sw/drivers/spi/src/32b/f28x/f2802x/spi.h"
#include "sw/drivers/gpio/src/32b/f28x/f2802x/gpio.h"
#include "can.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef unsigned char   uint8_t;


/*
 *  Speed 8M
 */
#define MCP_8MHz_1000kBPS_CFG1 (0x00)
#define MCP_8MHz_1000kBPS_CFG2 (0x80)
#define MCP_8MHz_1000kBPS_CFG3 (0x80)

#define MCP_8MHz_500kBPS_CFG1 (0x00)
#define MCP_8MHz_500kBPS_CFG2 (0x90)
#define MCP_8MHz_500kBPS_CFG3 (0x82)

#define MCP_8MHz_250kBPS_CFG1 (0x00)
#define MCP_8MHz_250kBPS_CFG2 (0xB1)
#define MCP_8MHz_250kBPS_CFG3 (0x85)

#define MCP_8MHz_200kBPS_CFG1 (0x00)
#define MCP_8MHz_200kBPS_CFG2 (0xB4)
#define MCP_8MHz_200kBPS_CFG3 (0x86)

#define MCP_8MHz_125kBPS_CFG1 (0x01)
#define MCP_8MHz_125kBPS_CFG2 (0xB1)
#define MCP_8MHz_125kBPS_CFG3 (0x85)

#define MCP_8MHz_100kBPS_CFG1 (0x01)
#define MCP_8MHz_100kBPS_CFG2 (0xB4)
#define MCP_8MHz_100kBPS_CFG3 (0x86)

#define MCP_8MHz_80kBPS_CFG1 (0x01)
#define MCP_8MHz_80kBPS_CFG2 (0xBF)
#define MCP_8MHz_80kBPS_CFG3 (0x87)

#define MCP_8MHz_50kBPS_CFG1 (0x03)
#define MCP_8MHz_50kBPS_CFG2 (0xB4)
#define MCP_8MHz_50kBPS_CFG3 (0x86)

#define MCP_8MHz_40kBPS_CFG1 (0x03)
#define MCP_8MHz_40kBPS_CFG2 (0xBF)
#define MCP_8MHz_40kBPS_CFG3 (0x87)

#define MCP_8MHz_33k3BPS_CFG1 (0x47)
#define MCP_8MHz_33k3BPS_CFG2 (0xE2)
#define MCP_8MHz_33k3BPS_CFG3 (0x85)

#define MCP_8MHz_31k25BPS_CFG1 (0x07)
#define MCP_8MHz_31k25BPS_CFG2 (0xA4)
#define MCP_8MHz_31k25BPS_CFG3 (0x84)

#define MCP_8MHz_20kBPS_CFG1 (0x07)
#define MCP_8MHz_20kBPS_CFG2 (0xBF)
#define MCP_8MHz_20kBPS_CFG3 (0x87)

#define MCP_8MHz_10kBPS_CFG1 (0x0F)
#define MCP_8MHz_10kBPS_CFG2 (0xBF)
#define MCP_8MHz_10kBPS_CFG3 (0x87)

#define MCP_8MHz_5kBPS_CFG1 (0x1F)
#define MCP_8MHz_5kBPS_CFG2 (0xBF)
#define MCP_8MHz_5kBPS_CFG3 (0x87)

/*
 *  speed 16M
 */
#define MCP_16MHz_1000kBPS_CFG1 (0x00)
#define MCP_16MHz_1000kBPS_CFG2 (0xD0)
#define MCP_16MHz_1000kBPS_CFG3 (0x82)

#define MCP_16MHz_500kBPS_CFG1 (0x00)
#define MCP_16MHz_500kBPS_CFG2 (0xF0)
#define MCP_16MHz_500kBPS_CFG3 (0x86)

#define MCP_16MHz_250kBPS_CFG1 (0x41)
#define MCP_16MHz_250kBPS_CFG2 (0xF1)
#define MCP_16MHz_250kBPS_CFG3 (0x85)

#define MCP_16MHz_200kBPS_CFG1 (0x01)
#define MCP_16MHz_200kBPS_CFG2 (0xFA)
#define MCP_16MHz_200kBPS_CFG3 (0x87)

#define MCP_16MHz_125kBPS_CFG1 (0x03)
#define MCP_16MHz_125kBPS_CFG2 (0xF0)
#define MCP_16MHz_125kBPS_CFG3 (0x86)

#define MCP_16MHz_100kBPS_CFG1 (0x03)
#define MCP_16MHz_100kBPS_CFG2 (0xFA)
#define MCP_16MHz_100kBPS_CFG3 (0x87)

#define MCP_16MHz_80kBPS_CFG1 (0x03)
#define MCP_16MHz_80kBPS_CFG2 (0xFF)
#define MCP_16MHz_80kBPS_CFG3 (0x87)

#define MCP_16MHz_83k3BPS_CFG1 (0x03)
#define MCP_16MHz_83k3BPS_CFG2 (0xBE)
#define MCP_16MHz_83k3BPS_CFG3 (0x07)

#define MCP_16MHz_50kBPS_CFG1 (0x07)
#define MCP_16MHz_50kBPS_CFG2 (0xFA)
#define MCP_16MHz_50kBPS_CFG3 (0x87)

#define MCP_16MHz_40kBPS_CFG1 (0x07)
#define MCP_16MHz_40kBPS_CFG2 (0xFF)
#define MCP_16MHz_40kBPS_CFG3 (0x87)

#define MCP_16MHz_33k3BPS_CFG1 (0x4E)
#define MCP_16MHz_33k3BPS_CFG2 (0xF1)
#define MCP_16MHz_33k3BPS_CFG3 (0x85)

#define MCP_16MHz_20kBPS_CFG1 (0x0F)
#define MCP_16MHz_20kBPS_CFG2 (0xFF)
#define MCP_16MHz_20kBPS_CFG3 (0x87)

#define MCP_16MHz_10kBPS_CFG1 (0x1F)
#define MCP_16MHz_10kBPS_CFG2 (0xFF)
#define MCP_16MHz_10kBPS_CFG3 (0x87)

#define MCP_16MHz_5kBPS_CFG1 (0x3F)
#define MCP_16MHz_5kBPS_CFG2 (0xFF)
#define MCP_16MHz_5kBPS_CFG3 (0x87)

/*
 *  speed 20M
 */
#define MCP_20MHz_1000kBPS_CFG1 (0x00)
#define MCP_20MHz_1000kBPS_CFG2 (0xD9)
#define MCP_20MHz_1000kBPS_CFG3 (0x82)

#define MCP_20MHz_500kBPS_CFG1 (0x00)
#define MCP_20MHz_500kBPS_CFG2 (0xFA)
#define MCP_20MHz_500kBPS_CFG3 (0x87)

#define MCP_20MHz_250kBPS_CFG1 (0x41)
#define MCP_20MHz_250kBPS_CFG2 (0xFB)
#define MCP_20MHz_250kBPS_CFG3 (0x86)

#define MCP_20MHz_200kBPS_CFG1 (0x01)
#define MCP_20MHz_200kBPS_CFG2 (0xFF)
#define MCP_20MHz_200kBPS_CFG3 (0x87)

#define MCP_20MHz_125kBPS_CFG1 (0x03)
#define MCP_20MHz_125kBPS_CFG2 (0xFA)
#define MCP_20MHz_125kBPS_CFG3 (0x87)

#define MCP_20MHz_100kBPS_CFG1 (0x04)
#define MCP_20MHz_100kBPS_CFG2 (0xFA)
#define MCP_20MHz_100kBPS_CFG3 (0x87)

#define MCP_20MHz_83k3BPS_CFG1 (0x04)
#define MCP_20MHz_83k3BPS_CFG2 (0xFE)
#define MCP_20MHz_83k3BPS_CFG3 (0x87)

#define MCP_20MHz_80kBPS_CFG1 (0x04)
#define MCP_20MHz_80kBPS_CFG2 (0xFF)
#define MCP_20MHz_80kBPS_CFG3 (0x87)

#define MCP_20MHz_50kBPS_CFG1 (0x09)
#define MCP_20MHz_50kBPS_CFG2 (0xFA)
#define MCP_20MHz_50kBPS_CFG3 (0x87)

#define MCP_20MHz_40kBPS_CFG1 (0x09)
#define MCP_20MHz_40kBPS_CFG2 (0xFF)
#define MCP_20MHz_40kBPS_CFG3 (0x87)

#define MCP_20MHz_33k3BPS_CFG1 (0x0B)
#define MCP_20MHz_33k3BPS_CFG2 (0xFF)
#define MCP_20MHz_33k3BPS_CFG3 (0x87)

//More Defines
#define CANCTRL_REQOP  (0xE0)
#define CANCTRL_ABAT   (0x10)
#define CANCTRL_OSM    (0x08)
#define CANCTRL_CLKEN  (0x04)
#define CANCTRL_CLKPRE (0x03)

#define CANSTAT_OPMOD  (0xE0)
#define CANSTAT_ICOD   (0x0E)

#define CNF3_SOF       (0x80)

#define TXB_EXIDE_MASK (0x08)
#define DLC_MASK       (0x0F)
#define RTR_MASK       (0x40)

#define RXBnCTRL_RXM_STD    (0x20)
#define RXBnCTRL_RXM_EXT    (0x40)
#define RXBnCTRL_RXM_STDEXT (0x00)
#define RXBnCTRL_RXM_MASK   (0x60)
#define RXBnCTRL_RTR        (0x08)
#define RXB0CTRL_BUKT       (0x04)

#define MCP_SIDH  (0)
#define MCP_SIDL  (1)
#define MCP_EID8  (2)
#define MCP_EID0  (3)
#define MCP_DLC   (4)
#define MCP_DATA  (5)

#define STAT_RXIF_MASK (3)

#define SPI_CLOCK (10000000) // 10MHz

#define N_TXBUFFERS  (3)
#define N_RXBUFFERS  (2)

#define EFLG_ERRORMASK (0xF8)

typedef enum
{
  MCP_20MHZ,
  MCP_16MHZ,
  MCP_8MHZ
} CAN_CLOCK;

typedef enum
{
  CAN_5KBPS,
  CAN_10KBPS,
  CAN_20KBPS,
  CAN_31K25BPS,
  CAN_33KBPS,
  CAN_40KBPS,
  CAN_50KBPS,
  CAN_80KBPS,
  CAN_83K3BPS,
  CAN_95KBPS,
  CAN_100KBPS,
  CAN_125KBPS,
  CAN_200KBPS,
  CAN_250KBPS,
  CAN_500KBPS,
  CAN_1000KBPS
} CAN_SPEED;

typedef enum
{
  CLKOUT_DISABLE = -1,
  CLKOUT_DIV1 = 0x0,
  CLKOUT_DIV2 = 0x1,
  CLKOUT_DIV4 = 0x2,
  CLKOUT_DIV8 = 0x3,
} CAN_CLKOUT;

typedef enum
{
  ERROR_OK        = 0,
  ERROR_FAIL      = 1,
  ERROR_ALLTXBUSY = 2,
  ERROR_FAILINIT  = 3,
  ERROR_FAILTX    = 4,
  ERROR_NOMSG     = 5
} MCP_ERROR;

typedef enum
{
  MASK0,
  MASK1
} MASK;

typedef enum
{
  RXF0 = 0,
  RXF1 = 1,
  RXF2 = 2,
  RXF3 = 3,
  RXF4 = 4,
  RXF5 = 5
} RXF;

typedef enum
{
  RXB0 = 0,
  RXB1 = 1
} RXBn;

typedef enum
{
  TXB0 = 0,
  TXB1 = 1,
  TXB2 = 2
} TXBn;

typedef enum
{  // uint8_t
  CANINTF_RX0IF = 0x01,
  CANINTF_RX1IF = 0x02,
  CANINTF_TX0IF = 0x04,
  CANINTF_TX1IF = 0x08,
  CANINTF_TX2IF = 0x10,
  CANINTF_ERRIF = 0x20,
  CANINTF_WAKIF = 0x40,
  CANINTF_MERRF = 0x80
} CANINTF;

typedef enum
{  // uint8_t
  EFLG_RX1OVR = (1<<7),
  EFLG_RX0OVR = (1<<6),
  EFLG_TXBO   = (1<<5),
  EFLG_TXEP   = (1<<4),
  EFLG_RXEP   = (1<<3),
  EFLG_TXWAR  = (1<<2),
  EFLG_RXWAR  = (1<<1),
  EFLG_EWARN  = (1<<0)
} EFLG;


typedef enum
{  // uint8_t
  CANCTRL_REQOP_NORMAL     = 0x00,
  CANCTRL_REQOP_SLEEP      = 0x20,
  CANCTRL_REQOP_LOOPBACK   = 0x40,
  CANCTRL_REQOP_LISTENONLY = 0x60,
  CANCTRL_REQOP_CONFIG     = 0x80,
  CANCTRL_REQOP_POWERUP    = 0xE0
} CANCTRL_REQOP_MODE;


typedef enum
{  // uint8_t
  STAT_RX0IF = (1<<0),
  STAT_RX1IF = (1<<1)
} STAT;


typedef enum
{ // uint8_t
  TXB_ABTF   = 0x40,
  TXB_MLOA   = 0x20,
  TXB_TXERR  = 0x10,
  TXB_TXREQ  = 0x08,
  TXB_TXIE   = 0x04,
  TXB_TXP    = 0x03
} TXBnCTRL;

typedef enum
{ // uint8_t
  INSTRUCTION_WRITE       = 0x02,
  INSTRUCTION_READ        = 0x03,
  INSTRUCTION_BITMOD      = 0x05,
  INSTRUCTION_LOAD_TX0    = 0x40,
  INSTRUCTION_LOAD_TX1    = 0x42,
  INSTRUCTION_LOAD_TX2    = 0x44,
  INSTRUCTION_RTS_TX0     = 0x81,
  INSTRUCTION_RTS_TX1     = 0x82,
  INSTRUCTION_RTS_TX2     = 0x84,
  INSTRUCTION_RTS_ALL     = 0x87,
  INSTRUCTION_READ_RX0    = 0x90,
  INSTRUCTION_READ_RX1    = 0x94,
  INSTRUCTION_READ_STATUS = 0xA0,
  INSTRUCTION_RX_STATUS   = 0xB0,
  INSTRUCTION_RESET       = 0xC0
} INSTRUCTION;

typedef enum
{
  MCP_RXF0SIDH = 0x00,
  MCP_RXF0SIDL = 0x01,
  MCP_RXF0EID8 = 0x02,
  MCP_RXF0EID0 = 0x03,
  MCP_RXF1SIDH = 0x04,
  MCP_RXF1SIDL = 0x05,
  MCP_RXF1EID8 = 0x06,
  MCP_RXF1EID0 = 0x07,
  MCP_RXF2SIDH = 0x08,
  MCP_RXF2SIDL = 0x09,
  MCP_RXF2EID8 = 0x0A,
  MCP_RXF2EID0 = 0x0B,
  MCP_CANSTAT  = 0x0E,
  MCP_CANCTRL  = 0x0F,
  MCP_RXF3SIDH = 0x10,
  MCP_RXF3SIDL = 0x11,
  MCP_RXF3EID8 = 0x12,
  MCP_RXF3EID0 = 0x13,
  MCP_RXF4SIDH = 0x14,
  MCP_RXF4SIDL = 0x15,
  MCP_RXF4EID8 = 0x16,
  MCP_RXF4EID0 = 0x17,
  MCP_RXF5SIDH = 0x18,
  MCP_RXF5SIDL = 0x19,
  MCP_RXF5EID8 = 0x1A,
  MCP_RXF5EID0 = 0x1B,
  MCP_TEC      = 0x1C,
  MCP_REC      = 0x1D,
  MCP_RXM0SIDH = 0x20,
  MCP_RXM0SIDL = 0x21,
  MCP_RXM0EID8 = 0x22,
  MCP_RXM0EID0 = 0x23,
  MCP_RXM1SIDH = 0x24,
  MCP_RXM1SIDL = 0x25,
  MCP_RXM1EID8 = 0x26,
  MCP_RXM1EID0 = 0x27,
  MCP_CNF3     = 0x28,
  MCP_CNF2     = 0x29,
  MCP_CNF1     = 0x2A,
  MCP_CANINTE  = 0x2B,
  MCP_CANINTF  = 0x2C,
  MCP_EFLG     = 0x2D,
  MCP_TXB0CTRL = 0x30,
  MCP_TXB0SIDH = 0x31,
  MCP_TXB0SIDL = 0x32,
  MCP_TXB0EID8 = 0x33,
  MCP_TXB0EID0 = 0x34,
  MCP_TXB0DLC  = 0x35,
  MCP_TXB0DATA = 0x36,
  MCP_TXB1CTRL = 0x40,
  MCP_TXB1SIDH = 0x41,
  MCP_TXB1SIDL = 0x42,
  MCP_TXB1EID8 = 0x43,
  MCP_TXB1EID0 = 0x44,
  MCP_TXB1DLC  = 0x45,
  MCP_TXB1DATA = 0x46,
  MCP_TXB2CTRL = 0x50,
  MCP_TXB2SIDH = 0x51,
  MCP_TXB2SIDL = 0x52,
  MCP_TXB2EID8 = 0x53,
  MCP_TXB2EID0 = 0x54,
  MCP_TXB2DLC  = 0x55,
  MCP_TXB2DATA = 0x56,
  MCP_RXB0CTRL = 0x60,
  MCP_RXB0SIDH = 0x61,
  MCP_RXB0SIDL = 0x62,
  MCP_RXB0EID8 = 0x63,
  MCP_RXB0EID0 = 0x64,
  MCP_RXB0DLC  = 0x65,
  MCP_RXB0DATA = 0x66,
  MCP_RXB1CTRL = 0x70,
  MCP_RXB1SIDH = 0x71,
  MCP_RXB1SIDL = 0x72,
  MCP_RXB1EID8 = 0x73,
  MCP_RXB1EID0 = 0x74,
  MCP_RXB1DLC  = 0x75,
  MCP_RXB1DATA = 0x76
} REGISTER;

typedef struct _TXBn_REGS_ {
    REGISTER CTRL;
    REGISTER SIDH;
    REGISTER DATA;
} TXBn_REGS;

typedef struct _RXBn_REGS_ {
    REGISTER CTRL;
    REGISTER SIDH;
    REGISTER DATA;
    CANINTF  CANINTF_RXnIF;
} RXBn_REGS;

//! \brief Defines the MCP2515 object
//!
typedef struct _MCP2515_Obj_
{
  SPI_Handle       spiHandle;                  //!< the handle for the serial peripheral interface
  GPIO_Handle      gpioHandle;                 //!< the gpio handle that is connected to the MCP2515 CS pin
  GPIO_Number_e    gpioCS;                     //!< the gpio number that is connected to the MCP2515 CS pin
  bool             RxTimeOut;                  //!< the timeout flag for the RX fifo of the SPI
} MCP2515_Obj;


//! \brief Defines the MCP2515 handle
//!
typedef struct _MCP2515_Obj_ *MCP2515_Handle;


// **************************************************************************
// function prototypes


//! \brief     Initializes the MCP2515 object
//! \param[in] pMemory   A pointer to the memory for the MCP2515 object
//! \param[in] numBytes  The number of bytes allocated for the MCP2515 object, bytes
//! \return    The MCP2515 object handle
extern MCP2515_Handle MCP2515_init(void *pMemory, const size_t numBytes);


//! \brief     Sets the SPI handle in the MCP2515
//! \param[in] handle     The MCP2515 handle
//! \param[in] spiHandle  The SPI handle to use
void MCP2515_setSpiHandle(MCP2515_Handle handle, SPI_Handle spiHandle);


//! \brief     Sets the GPIO handle in the MCP2515
//! \param[in] handle       The MCP2515 handle
//! \param[in] gpioHandle   The GPIO handle to use
void MCP2515_setGpioHandle(MCP2515_Handle handle,GPIO_Handle gpioHandle);


//! \brief     Sets the CS GPIO number in the MCP2515
//! \param[in] handle       The MCP2515 handle
//! \param[in] gpioHandle   The GPIO number to use
void MCP2515_setGpioNumber(MCP2515_Handle handle, GPIO_Number_e gpioNumber);


//! \brief     Reads data from a MCP2515 register
//! \param[in] handle   The MCP2515 handle
//! \param[in] regAddr  The register address
//! \return    The data value
extern uint8_t MCP2515_readRegister(MCP2515_Handle handle, REGISTER reg);


//! \brief     Reads data from multiple MCP2515 registers
//! \param[in] handle   The MCP2515 handle
//! \param[in] regAddr  The register address
//! \param[in] n        The number of registers
extern void MCP2515_readRegisters(MCP2515_Handle handle, REGISTER reg, uint8_t values[], const uint8_t n);


//! \brief     Writes data to a MCP2515 register
//! \param[in] handle   The MCP2515 handle
//! \param[in] reg      The register name
//! \param[in] value    The data value
extern void MCP2515_setRegister(MCP2515_Handle handle, REGISTER reg, const uint8_t value);


//! \brief     Writes data to multiple MCP2515 register
//! \param[in] handle   The MCP2515 handle
//! \param[in] reg      The register name
//! \param[in] values   The data values
extern void MCP2515_setRegisters(MCP2515_Handle handle, REGISTER reg, const uint8_t values[], const uint8_t n);


//! \brief     Modifies individual bits of a MCP2515 register
//! \param[in] handle   The MCP2515 handle
//! \param[in] reg      The register name
//! \param[in] mask     The data mask
//! \param[in] data     The data value
extern void MCP2515_modifyRegister(MCP2515_Handle handle, REGISTER reg, const uint8_t mask, const uint8_t data);


//! \brief     Modifies individual bits of a MCP2515 register
//! \param[in] handle   The MCP2515 handle
//! \return    Any MCP Errors
extern MCP_ERROR MCP2515_reset(MCP2515_Handle handle);


//! \brief     Sets operating mode of MCP2515
//! \param[in] handle   The MCP2515 handle
//! \param[in] mode     The control mode
//! \return    Any MCP Errors
extern MCP_ERROR MCP2515_setMode(MCP2515_Handle handle, const CANCTRL_REQOP_MODE mode);


//! \brief     Sets MCP2515 to config mode
//! \param[in] handle   The MCP2515 handle
//! \param[in] mode     The control mode
//! \return    Any MCP Errors
extern MCP_ERROR MCP2515_setConfigMode(MCP2515_Handle handle);


//! \brief     Sets MCP2515 to listen only mode
//! \param[in] handle   The MCP2515 handle
//! \param[in] mode     The control mode
//! \return    Any MCP Errors
extern MCP_ERROR MCP2515_setListenOnlyMode(MCP2515_Handle handle);


//! \brief     Sets MCP2515 to sleep mode
//! \param[in] handle   The MCP2515 handle
//! \param[in] mode     The control mode
//! \return    Any MCP Errors
extern MCP_ERROR MCP2515_setSleepMode(MCP2515_Handle handle);


//! \brief     Sets MCP2515 to loopback mode
//! \param[in] handle   The MCP2515 handle
//! \param[in] mode     The control mode
//! \return    Any MCP Errors
extern MCP_ERROR MCP2515_setLoopbackMode(MCP2515_Handle handle);


//! \brief     Sets MCP2515 to normal mode
//! \param[in] handle   The MCP2515 handle
//! \param[in] mode     The control mode
//! \return    Any MCP Errors
extern MCP_ERROR MCP2515_setNormalMode(MCP2515_Handle handle);


//! \brief     Get's the read status of the MCP2515
//! \param[in] handle   The MCP2515 handle
//! \return    MCP2515 read status
extern uint8_t MCP2515_getStatus(MCP2515_Handle handle);


//! \brief     Set's the bitrate of the MCP2515
//! \param[in] handle   The MCP2515 handle
//! \param[in] canSpeed The CAN kb/s value
//! \param[in] canClock The MCP2515 external clock value
//! \return    MCP ERROR
extern MCP_ERROR MCP2515_setBitrate(MCP2515_Handle handle, CAN_SPEED canSpeed, CAN_CLOCK canClock);


//! \brief     Set's the clock of the MCP2515
//! \param[in] handle   The MCP2515 handle
//! \param[in] divisor  The CAN clock divisor
//! \return    MCP ERROR
extern MCP_ERROR MCP2515_setClkOut(MCP2515_Handle handle, CAN_CLKOUT divisor);


//! \brief     Prepares CAN ID
//! \param[in] buffer
//! \param[in] ext      Extended CAN msg flag
//! \param[in] id       CAN ID
extern void MCP2515_prepareId(uint8_t *buffer, const bool ext, const uint32_t id);


//! \brief     Set's the CAN ID filter mask
//! \param[in] handle   The MCP2515 handle
//! \param[in] num      The CAN mask number
//! \param[in] ext      Extended CAN msg flag
//! \param[in] ulData   CAN ID Mask data
//! \return    MCP ERROR
extern MCP_ERROR MCP2515_setFilterMask(MCP2515_Handle handle, const MASK num, const bool ext, const uint32_t ulData);


//! \brief     Set's the CAN ID filter
//! \param[in] handle   The MCP2515 handle
//! \param[in] num      The RXF number
//! \param[in] ext      Extended CAN msg flag
//! \param[in] ulData   CAN ID Mask data
//! \return    MCP ERROR
extern MCP_ERROR MCP2515_setFilter(MCP2515_Handle handle, const RXF num, const bool ext, const uint32_t ulData);


//! \brief     Reads a CAN message
//! \param[in] handle   The MCP2515 handle
//! \param[in] frame    The CAN frame
//! \return    MCP ERROR
extern MCP_ERROR MCP2515_readMessage(MCP2515_Handle handle, can_frame *frame);


//! \brief     Reads a multibyte CAN message
//! \param[in] handle   The MCP2515 handle
//! \param[in] rxbn     The MCP2515 RX buffers
//! \param[in] frame    The CAN frame
//! \return    MCP ERROR
extern MCP_ERROR MCP2515_readMessages(MCP2515_Handle handle, RXBn rxbn, can_frame *frame);


//! \brief     Writes a CAN message
//! \param[in] handle   The MCP2515 handle
//! \param[in] frame    The CAN frame
//! \return    MCP ERROR
extern MCP_ERROR MCP2515_sendMessage(MCP2515_Handle handle, can_frame *frame);


//! \brief     Writes a multibyte CAN message
//! \param[in] handle   The MCP2515 handle
//! \param[in] txbn     The MCP2515 RX buffers
//! \param[in] frame    The CAN frame
//! \return    MCP ERROR
extern MCP_ERROR MCP2515_sendMessages(MCP2515_Handle handle, TXBn txbn, can_frame *frame);


//! \brief     Checks for message recieved
//! \param[in] handle   The MCP2515 handle
//! \return    bool
extern bool MCP2515_checkReceive(MCP2515_Handle handle);


//! \brief     Checks for an error
//! \param[in] handle   The MCP2515 handle
//! \return    bool
extern bool MCP2515_checkError(MCP2515_Handle handle);


//! \brief     Retrieves any error flags
//! \param[in] handle   The MCP2515 handle
//! \return    error flags
extern uint8_t MCP2515_getErrorFlags(MCP2515_Handle handle);


//! \brief     Clears RX buffer flags
//! \param[in] handle   The MCP2515 handle
extern void MCP2515_clearRXnOVRFlags(MCP2515_Handle handle);


//! \brief     Gets Interrupts
//! \param[in] handle   The MCP2515 handle
//! \return    interrupts
extern uint8_t MCP2515_getInterrupts(MCP2515_Handle handle);


//! \brief     Gets Interrupt Mask
//! \param[in] handle   The MCP2515 handle
//! \return    interrupt masks
extern uint8_t MCP2515_getInterruptMask(MCP2515_Handle handle);


//! \brief     Clears Interrupts
//! \param[in] handle   The MCP2515 handle
extern void MCP2515_clearInterrupts(MCP2515_Handle handle);


//! \brief     Clears TX Interrupts
//! \param[in] handle   The MCP2515 handle
extern void MCP2515_clearTXInterrupts(MCP2515_Handle handle);


//! \brief     Clears RX buffer
//! \param[in] handle   The MCP2515 handle
extern void MCP2515_clearRXnOVR(MCP2515_Handle handle);


//! \brief     Clears MERR
//! \param[in] handle   The MCP2515 handle
extern void MCP2515_clearMERR(MCP2515_Handle handle);


//! \brief     Clears ERRIF
//! \param[in] handle   The MCP2515 handle
extern void MCP2515_clearERRIF(MCP2515_Handle handle);


#endif
