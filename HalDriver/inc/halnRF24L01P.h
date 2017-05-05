#ifndef __HALNRF24L01P_H
#define __HALNRF24L01P_H

#include "stm32f10x.h"
#include "halMCUWait.h"


#define TX_FIFO_COUNT   3
#define RX_FIFO_COUNT   3
 
#define TX_FIFO_SIZE    32
#define RX_FIFO_SIZE    32

#define SPI_MAX_DATA_RATE         (10000000)

////////////////////////////////////////////////////////////////////////////////
//SPI COMMAND
////////////////////////////////////////////////////////////////////////////////
#define SPI_CMD_R_REGRESTER       (0x00)        //5LSB => Reg Map Addr 
#define SPI_CMD_W_REGRESTER       (0x20)        //5LSB => Reg Map Addr
#define SPI_CMD_R_RX_PAYLOAD      (0x61)        //Rx Mode, max 32 bytes
#define SPI_CMD_W_TX_PAYLOAD      (0xa0)        //max 32 bytes
#define SPI_CMD_FLUSH_TX          (0xe1)        
#define SPI_CMD_FLUSH_RX          (0xe2)        //not executed during transmission of ack
#define SPI_CMD_REUSE_TX_PL       (0xe3)        
#define SPI_CMD_R_RX_PL_WID       (0x60)        //Read Rx payload width for the top R_RX_PAYLOAD in the Rx FIFO
#define SPI_CMD_W_ACK_PAYLOAD     (0xa8)        //Rx mode
#define SPI_CMD_W_TX_PAYLOAD_NOACK (0xb0)        //Tx mode, disable auto ack on this specific packet
#define SPI_CMD_NOP               (0xff)        //used to read status

////////////////////////////////////////////////////////////////////////////////
//SPI REGISTER
////////////////////////////////////////////////////////////////////////////////
#define REG_CONFIG                (0x00)
#define REG_EN_AA                 (0x01)
#define REG_EN_RXADDR             (0x02)
#define REG_SETUP_AW              (0x03)
#define REG_SETUP_RETR            (0x04)
#define REG_RF_CH                 (0x05)
#define REG_RF_SETUP              (0x06)
#define REG_STATUS                (0x07)
#define REG_OBSERVE_TX            (0x08)
#define REG_RPD                   (0x09)
#define REG_RX_ADDR_P0            (0x0a)
#define REG_RX_ADDR_P1            (0x0b)
#define REG_RX_ADDR_P2            (0x0c)
#define REG_RX_ADDR_P3            (0x0d)
#define REG_RX_ADDR_P4            (0x0e)
#define REG_RX_ADDR_P5            (0x0f)
#define REG_TX_ADDR               (0x10)
#define REG_RX_PW_P0              (0x11)
#define REG_RX_PW_P1              (0x12)
#define REG_RX_PW_P2              (0x13)
#define REG_RX_PW_P3              (0x14)
#define REG_RX_PW_P4              (0x15)
#define REG_RX_PW_P5              (0x16)
#define REG_FIFO_STATUS           (0x17)
#define REG_DYNPD                 (0x1c)
#define REG_FEATURE               (0x1d)

#define PRIM_RX        (1<<0)
#define PWR_UP         (1<<1)
#define CRCO           (1<<2)
#define EN_CRC         (1<<3)
#define MASK_MAX_RT    (1<<4)
#define MASK_TX_DS     (1<<5)
#define MASK_RX_DR     (1<<6)
 
#define EN_AA_P0       (1<<0) 
#define EN_AA_P1       (1<<1)
#define EN_AA_P2       (1<<2)
#define EN_AA_P3       (1<<3)
#define EN_AA_P4       (1<<4)
#define EN_AA_P5       (1<<5)

#define EN_RRX_P0      (1<<0)
#define EN_RRX_P1      (1<<1)
#define EN_RRX_P2      (1<<2)
#define EN_RRX_P3      (1<<3)
#define EN_RRX_P4      (1<<4)
#define EN_RRX_P5      (1<<5)

#define AW_0           (1<<0)
#define AW_1           (1<<1)
#define AW_2           (1<<2)
 
#define ARD_0          (1<<4)
#define ARD_1          (1<<5)
#define ARD_2          (1<<6)
#define ARD_3          (1<<7)
#define ARC_0          (1<<0)
#define ARC_1          (1<<1)
#define ARC_2          (1<<2)
#define ARC_3          (1<<3)

#define CONT_WAVE      (1<<7)
#define RF_DR_LOW      (1<<5)
#define PLL_LOCK       (1<<4)
#define RF_DR_HIGH_0   (1<<3)
#define RF_DR_HIGH_1   (1<<4)
#define RF_DR_HIGH_2   (1<<5)
#define RF_POWER_0     (1<<1)
#define RF_POWER_1     (1<<2)
#define RF_POWER_2     (1<<3)

#define STATUS_TX_FULL (1<<0)
#define RX_P_NO_0      (1<<1)
#define RX_P_NO_1      (1<<2)
#define RX_P_NO_2      (1<<3)
#define MAX_RT         (1<<4)
#define TX_DS          (1<<5)
#define RX_DR          (1<<6)

#define PLOS_CNT_0     (1<<4)
#define PLOS_CNT_1     (1<<5)
#define PLOS_CNT_2     (1<<6)
#define PLOS_CNT_3     (1<<7)
#define ARC_CNT_0      (1<<0)
#define ARC_CNT_1      (1<<1)
#define ARC_CNT_2      (1<<2)
#define ARC_CNT_3      (1<<3)

#define RPD            (1<<0)

#define FIFO_STATUS_TX_REUSE       (1<<6)
#define TX_FULL        (1<<5)
#define TX_EMPTY       (1<<4)
#define RX_FULL        (1<<1)
#define RX_EMPTY       (1<<0)

#define DPL_P0         (1<<0)
#define DPL_P1         (1<<1)
#define DPL_P2         (1<<2)
#define DPL_P3         (1<<3)
#define DPL_P4         (1<<4)
#define DPL_P5         (1<<5)

#define EN_DPL         (1<<2)
#define EN_ACK_PAY     (1<<1)
#define EN_DYN_ACK     (1<<0)

#define TIMING_Tundef2pd_us        100000   // 100mS
#define TIMING_Tstby2a_us          130      // 130uS
#define TIMING_Thce_us             10       // 10uS
#define TIMING_Tpd2stby_us         4500     // 4.5mS worst case
#define TIMING_Tpece2csn_us        4        // 4uS

#define  CEPinSetHigh()       GPIO_SetBits(GPIOD, GPIO_Pin_3); halMCUWaitUS(TIMING_Thce_us)
#define  CEPinSetLow()        GPIO_ResetBits(GPIOD, GPIO_Pin_3);
#define  CEPinRead()          GPIO_ReadOutputDataBit(GPIOD, GPIO_Pin_3)

#define  IRQPinRead()         GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_5)

#define  EnterStandByMode()   CEPinSetLow()
#define  EnterTxRxMode()      CEPinSetHigh()

extern void halnRF24L01PInit(void);

extern uint8_t halnRF24L01PRdRegByte(uint8_t reg);
extern uint8_t halnRF24L01PWrRegByte(uint8_t reg, uint8_t data);
extern uint8_t halnRF24L01PRdRegPacket(uint8_t reg, uint8_t* data, uint8_t len);
extern uint8_t halnRF24L01PWrRegPacket(uint8_t reg, uint8_t* data, uint8_t len);

extern void halnRF24L01PTxPacket(uint8_t* data, uint8_t len);
extern void halnRF24L01PRxPacket(void);
extern uint8_t halnRF24L01PBroadcastPacket(uint8_t* data, uint8_t len);

#endif
