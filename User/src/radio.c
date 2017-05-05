#include "radio.h"


////////////////////////////////////////////////////////////////////////////////
//PARA FOR MEASURE DIS
////////////////////////////////////////////////////////////////////////////////
extern volatile uint8_t  activeBeacon;                  //Only one beacon is active at a time (0 no active)
extern volatile uint16_t beaconTimeRecStart;            //Time that active beacon receive Radio

extern uint8_t spi1EXTBroken, spi1MPUBroken;

////////////////////////////////////////////////////////////////////////////////
//PARA FOR RF COMMUNICATION
////////////////////////////////////////////////////////////////////////////////
extern xQueueHandle xQueueHandleRFRx;      
uint8_t RFRxBuf[32];                  //Buffer store Radio Rx data
volatile uint8_t RFTxDone=0;          //indicate RF Tx success

void EXTI5ISR_RF(void) {
  uint8_t status;
  uint8_t pipeOffset=0;
  uint8_t dataLen=0;
  
  if (SPI1_EXT_STATUS() == Bit_RESET) {
    spi1EXTBroken = 1;
    SPI1_EXT_DISABLE();
  }
  
  if (SPI1_MPU_STATUS() == Bit_RESET) {
    spi1MPUBroken = 1;
    SPI_MPU_DISABLE();
  }
  
  status = halnRF24L01PWrRegByte(SPI_CMD_W_REGRESTER + REG_STATUS, MAX_RT+TX_DS+RX_DR); //清除标志位
  if ((status&MAX_RT) == MAX_RT) {
    asm ("NOP");
  }
  if ((status &TX_DS) == TX_DS) {
    RFTxDone = 1;
    halnRF24L01PWrRegByte(SPI_CMD_FLUSH_TX, 0xFF);
    asm ("NOP");
  }
  if ((status&RX_DR) == RX_DR) {
    pipeOffset = (0x07 & (status >> 1));                                                //获取当前产生中断的PIPE
    dataLen = halnRF24L01PRdRegByte(SPI_CMD_R_REGRESTER + REG_RX_PW_P0 + pipeOffset);   //获取当前PIPE中数据长度
    halnRF24L01PRdRegPacket(SPI_CMD_R_RX_PAYLOAD, RFRxBuf, dataLen);                    //获取数据
    halnRF24L01PWrRegByte(SPI_CMD_FLUSH_RX, 0xFF);
    if ((RFRxBuf[0] == (60000&0xFF)) &&  (RFRxBuf[1] == ((60000>>8)&0xFF))){
      TIM_SetCounter(TIM3, 0);
      TIM_Cmd(TIM3, ENABLE);
      activeBeacon = 1;
    }
    else if ((RFRxBuf[0] == (60001&0xFF)) &&  (RFRxBuf[1] == ((60001>>8)&0xFF))){
      TIM_SetCounter(TIM3, 0);
      TIM_Cmd(TIM3, ENABLE);
      activeBeacon = 2;
    }
    else if ((RFRxBuf[0] == (60002&0xFF)) &&  (RFRxBuf[1] == ((60002>>8)&0xFF))){
      TIM_SetCounter(TIM3, 0);
      TIM_Cmd(TIM3, ENABLE);
      activeBeacon = 3;
    }
    else if ((RFRxBuf[0] == (60003&0xFF)) &&  (RFRxBuf[1] == ((60003>>8)&0xFF))){
      TIM_SetCounter(TIM3, 0);
      TIM_Cmd(TIM3, ENABLE);
      activeBeacon = 4;
    }
    else {
      portBASE_TYPE xHigherPriorityTaskWoken;
      xHigherPriorityTaskWoken = pdFALSE;
      xHigherPriorityTaskWoken = xQueueSendFromISR(xQueueHandleRFRx, RFRxBuf, &xHigherPriorityTaskWoken);
      if( xHigherPriorityTaskWoken == pdTRUE ) {
        portEND_SWITCHING_ISR(xHigherPriorityTaskWoken );
      }
    }
  }
}



