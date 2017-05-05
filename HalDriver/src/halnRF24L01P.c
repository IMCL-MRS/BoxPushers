#include "halnRF24L01P.h"
#include "halSPI.h"
#include "string.h"

uint8_t TX_ADDRESS[5]  = {0x34,0x43,0x10,0x10,0x01};

////////////////////////////////////////////////////////////////////////////////
//PA11  -> RADIO_IRQ
//PC5   -> RADIO_CE
//PB1   -> SPI_RF
//PB13  -> SPI1_SCLK
//PB14  -> SPI1_MISO
//PB15  -> SPI1_MOSI
////////////////////////////////////////////////////////////////////////////////
void halnRF24L01PInit(void){
  //1. RCC
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,  ENABLE);
  //2. GPIO
  GPIO_InitTypeDef GPIO_InitStructure;                                             
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;                                 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                
  GPIO_Init(GPIOD, &GPIO_InitStructure);                                    
  GPIO_ResetBits(GPIOD, GPIO_Pin_3);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;                                 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  
  //3. EXIT
  EXTI_InitTypeDef EXTI_InitStructure;
  EXTI_InitStructure.EXTI_Line    = GPIO_Pin_5;                                 //外部中断线
  EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;                        //中断模式
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;                       //中断触发方式
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;                                     //打开中断
  EXTI_Init(&EXTI_InitStructure);
  
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource5);
  EXTI_ClearFlag(EXTI_Line5);
  //NVIC
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;           
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;         
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  //////////////////////////////////////////////////////////////////////////////
  CEPinSetLow();
  uint8_t rxfifo[32];
  halnRF24L01PRdRegPacket(SPI_CMD_R_RX_PAYLOAD, rxfifo, 32);                   
  halnRF24L01PWrRegByte(SPI_CMD_FLUSH_RX, 0xFF);
  halnRF24L01PWrRegByte(SPI_CMD_FLUSH_TX, 0xFF);
  halnRF24L01PWrRegByte(SPI_CMD_W_REGRESTER + REG_STATUS, MAX_RT+TX_DS+RX_DR); 
  //////////////////////////////////////////////////////////////////////////////
  
  halnRF24L01PRxPacket();
}

uint8_t halnRF24L01PRdRegByte(uint8_t reg) {
  uint8_t value=0, cmd=0;
  
  uint8_t ceState;
  
  ceState = CEPinRead();
  CEPinSetLow();
  
  cmd = (SPI_CMD_R_REGRESTER + (reg&0x1F));
  SPI_RF_ENABLE();
  halSPI1WrByte(cmd);
  value = halSPI1RdByte();
  SPI_RF_DISABLE();
  
  if (ceState !=  Bit_RESET) {
    CEPinSetHigh();
  }
  
  return value;
}

uint8_t halnRF24L01PWrRegByte(uint8_t reg, uint8_t data) {  
  uint8_t cmd;
  uint8_t status;
  
  uint8_t ceState;
  
  ceState = CEPinRead();
  CEPinSetLow();
  
  cmd = (SPI_CMD_W_REGRESTER + (reg&0x1F));
  SPI_RF_ENABLE();
  status = halSPI1WrRdByte(cmd); 
  halSPI1WrByte(data);
  SPI_RF_DISABLE();
  

  if (ceState !=  Bit_RESET) {
    CEPinSetHigh();
  }
  
  return status;
}

uint8_t halnRF24L01PRdRegPacket(uint8_t reg, uint8_t* data, uint8_t len){
  uint8_t status;
  
  uint8_t ceState;
  
  ceState = CEPinRead();
  CEPinSetLow();
  
  SPI_RF_ENABLE();
  status = halSPI1WrRdByte(reg);
  halSPI1RdPacket(data, len);
  SPI_RF_DISABLE();
  
  if (ceState !=  Bit_RESET) {
    CEPinSetHigh();
  }
  
  return status;
}

uint8_t halnRF24L01PWrRegPacket(uint8_t reg, uint8_t* data, uint8_t len){
  uint8_t status;
  
  uint8_t ceState;
  
  ceState = CEPinRead();
  CEPinSetLow();
  
  SPI_RF_ENABLE();
  status = halSPI1WrRdByte(reg);
  halSPI1WrPacket(data, len);
  SPI_RF_DISABLE();
  
  if (ceState !=  Bit_RESET) {
    CEPinSetHigh();
  }
  
  return status;
}

void halnRF24L01PSetTxPower(uint8_t tp) {
  
}
void halnRF24L01PSetDataRate(uint8_t dr) {

}
void halnRF24L01PSetChannel(uint8_t hz) {

}
void halnRF24L01PSetTxAddr(uint8_t* addr, uint8_t len) {

}
void halnRF24L01PSetRxAddr(uint8_t* addr, uint8_t len) {

}

uint8_t halnRF24L01PBroadcastPacket(uint8_t* data, uint8_t len) {
  if (halnRF24L01PRdRegByte(REG_RPD)==RPD) {
    
  }
  halnRF24L01PTxPacket(data, len);
  return 1;
}

void halnRF24L01PTxPacket(uint8_t* data, uint8_t len) {
  uint8_t dataLen;
  uint8_t dataInput[32];
  memset(dataInput, 0, 32);
  if (len >32) {
    dataLen=32;
  }
  else {
    dataLen=len;
  }
  memcpy(dataInput, data, dataLen);
  CEPinSetLow();
  halnRF24L01PWrRegByte(SPI_CMD_FLUSH_TX, 0xFF);                                //清除TX FIFO
  halnRF24L01PWrRegByte(SPI_CMD_W_REGRESTER + REG_STATUS, MAX_RT+TX_DS);        //清除标志位
  halnRF24L01PWrRegByte(SPI_CMD_W_REGRESTER + REG_FEATURE,(1<<1));              //使能EN_DYN_ACK
  halnRF24L01PWrRegByte(SPI_CMD_W_REGRESTER + REG_EN_AA, 0x00);                 //禁止ACK
  halnRF24L01PWrRegByte(SPI_CMD_W_REGRESTER + REG_SETUP_RETR, 0x00);            //禁止重传
  halnRF24L01PWrRegPacket(SPI_CMD_W_REGRESTER+REG_TX_ADDR, TX_ADDRESS, 5);      //TX 地址 -> 目标地址
  halnRF24L01PWrRegByte(SPI_CMD_W_REGRESTER + REG_RF_CH,   100);                //通道选择(2.4G+100M)HZ
  halnRF24L01PWrRegByte(SPI_CMD_W_REGRESTER + REG_RF_SETUP,(1<<3)+(3<<1));      //2Mbps+0dBm  
  halnRF24L01PWrRegPacket(SPI_CMD_W_TX_PAYLOAD_NOACK, dataInput, 32);           //写入TX_FIFO
  halnRF24L01PWrRegByte(SPI_CMD_W_REGRESTER + REG_CONFIG, 0x0E);                //Tx MODE + CRC16
  CEPinSetHigh();
}

void halnRF24L01PRxPacket(void) {
  CEPinSetLow();
  halnRF24L01PWrRegByte(SPI_CMD_FLUSH_RX, 0xFF);                                //清除TX FIFO
  halnRF24L01PWrRegByte(SPI_CMD_W_REGRESTER + REG_STATUS, RX_DR);               //清除标志位
  halnRF24L01PWrRegByte(SPI_CMD_W_REGRESTER + REG_EN_AA, 0x00);                 //禁止ACK
  halnRF24L01PWrRegByte(SPI_CMD_W_REGRESTER + REG_SETUP_RETR, 0x00);            //禁止重传
  halnRF24L01PWrRegPacket(SPI_CMD_W_REGRESTER + REG_RX_ADDR_P0, TX_ADDRESS, 5); //RX PIPE0 地址 -> 目标地址
  halnRF24L01PWrRegByte(SPI_CMD_W_REGRESTER + REG_EN_RXADDR, 0x01);             //使能PIPE0
  halnRF24L01PWrRegByte(SPI_CMD_W_REGRESTER + REG_RX_PW_P0, 32);                //静态数据长度
  halnRF24L01PWrRegByte(SPI_CMD_W_REGRESTER + REG_RF_CH,   100);                //通道选择(2.4G+100M)HZ
  halnRF24L01PWrRegByte(SPI_CMD_W_REGRESTER + REG_RF_SETUP,(1<<3)+(3<<1));      //2Mbps+0dBm
  halnRF24L01PWrRegByte(SPI_CMD_W_REGRESTER + REG_CONFIG, 0x0F);                //Rx MODE + CRC16
  CEPinSetHigh();
}

__weak void EXTI5ISR_RF(void) {
  if (halSPI1BusBusy()==1) {
    return;
  }
  halnRF24L01PWrRegByte(SPI_CMD_W_REGRESTER + REG_STATUS, MAX_RT+TX_DS+RX_DR);  //清除标志位
}
