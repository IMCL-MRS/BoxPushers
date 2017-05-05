#ifndef __HALSPI_H
#define __HALSPI_H


#include "stm32f10x.h"

#define  SPI1_EXT_ENABLE()   GPIO_ResetBits(GPIOC, GPIO_Pin_12)
#define  SPI1_EXT_DISABLE()  GPIO_SetBits(GPIOC, GPIO_Pin_12)

#define  SPI1_EXT_STATUS()   GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_12)

#define  SPI_MPU_ENABLE()    GPIO_ResetBits(GPIOG, GPIO_Pin_9)
#define  SPI_MPU_DISABLE()   GPIO_SetBits(GPIOG, GPIO_Pin_9)

#define  SPI1_MPU_STATUS()   GPIO_ReadOutputDataBit(GPIOG, GPIO_Pin_9)

#define  SPI_RF_ENABLE()     GPIO_ResetBits(GPIOD, GPIO_Pin_4)
#define  SPI_RF_DISABLE()    GPIO_SetBits(GPIOD, GPIO_Pin_4)

#define  SPI1_RF_STATUS()    GPIO_ReadOutputDataBit(GPIOD, GPIO_Pin_4)

extern void halSPI1Init(void);

extern void halSPI1WrByte(uint8_t data);
extern void halSPI1WrPacket(uint8_t* data, uint8_t len);

extern uint8_t halSPI1RdByte(void);
extern void halSPI1RdPacket(uint8_t* data, uint8_t len);

extern void halSPI1WrByteDMA(uint8_t data);
extern void halSPI1WrPacketDMA(uint8_t* data, uint8_t len);

extern uint8_t halSPI1WrRdByte(uint8_t data);

extern uint8_t halSPI1BusBusy(void);

#endif
