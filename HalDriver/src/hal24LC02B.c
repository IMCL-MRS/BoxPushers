#include "hal24LC02B.h"

////////////////////////////////////////////////////////////////////////////////
//24LC02B 需要最大5ms的写入周期
////////////////////////////////////////////////////////////////////////////////
void hal24LC02BInit(void) {
  
}

bool hal24LC02BByteWrite(uint8_t addr, uint8_t data){
  halI2CStart();
  halI2CWriteByte(CONTROL_CODE + CHIP_SELECT_ADDR + R_W_WRITE); 
  if (halI2CReadAck()==false) {
    halI2CStop();
    return false;
  }
  halI2CWriteByte(addr);
  if (halI2CReadAck()==false) {
    halI2CStop();
    return false;
  }
  halI2CWriteByte(data);
  if (halI2CReadAck()==false) {
    halI2CStop();
    return false;
  }
  halI2CStop();
  //halMCUWaitMS(5);
  return true;
}

bool hal24LC02BPageWrite(uint8_t addr, uint8_t* data, uint8_t len){
  uint8_t i;
  
  halI2CStart();
  halI2CWriteByte(CONTROL_CODE + CHIP_SELECT_ADDR + R_W_WRITE); 
  if (halI2CReadAck()==false) {
    halI2CStop();
    return false;
  }
  halI2CWriteByte(addr);
  if (halI2CReadAck()==false) {
    halI2CStop();
    return false;
  }
  for (i=0;i<len;i++) {
    halI2CWriteByte(*(data+i));
    if (halI2CReadAck()==false) {
      halI2CStop();
      return false;
    }
  }
  halI2CStop();
  
  halMCUWaitMS(5);
  
  return true;
}

uint8_t hal24LC02BCurAddrRead(void) {
  uint8_t data;
  
  halI2CStart();
  halI2CWriteByte(CONTROL_CODE + CHIP_SELECT_ADDR + R_W_READ); 
  if (halI2CReadAck()==false) {
    halI2CStop();
    return false;
  }
  data = halI2CReadByte();
  halI2CWriteNAck();
  halI2CStop();

  return data;
}

bool hal24LC02BRandomRead(uint8_t addr, uint8_t* data) {

  halI2CStart();
  halI2CWriteByte(CONTROL_CODE + CHIP_SELECT_ADDR + R_W_WRITE); 
  if (halI2CReadAck()==false) {
    halI2CStop();
    return false;
  }
  halI2CWriteByte(addr);
  if (halI2CReadAck()==false) {
    halI2CStop();
    return false;
  }
  
  halI2CStart();
  halI2CWriteByte(CONTROL_CODE + CHIP_SELECT_ADDR + R_W_READ); 
  if (halI2CReadAck()==false) {
    halI2CStop();
    return false;
  }
  *data = halI2CReadByte();
  halI2CWriteNAck();
  halI2CStop();
  
  return true;
}

bool hal24LC02BSeqRead(uint8_t addr, uint8_t* data, uint8_t len) {
  uint8_t i;
  
  halI2CStart();
  halI2CWriteByte(CONTROL_CODE + CHIP_SELECT_ADDR + R_W_WRITE); 
  if (halI2CReadAck()==false) {
    halI2CStop();
    return false;
  }
  halI2CWriteByte(addr);
  if (halI2CReadAck()==false) {
    halI2CStop();
    return false;
  }

  halI2CStart();
  halI2CWriteByte(CONTROL_CODE + CHIP_SELECT_ADDR + R_W_READ); 
  if (halI2CReadAck()==false) {
    halI2CStop();
    return false;
  }
  for (i=0;i<len;i++) {
    *(data+i) = halI2CReadByte();
    if (i==(len-1)){
      halI2CWriteNAck();
    }
    else {
      halI2CWriteAck();
    }
  }
  halI2CStop();
  return true;
}



