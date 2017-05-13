#include "DEMOTask.h"
#include "halBeep.h"
#include "halADC12.h"
#include "halSPI.h"
#include "halMPU9250.h"
#include "hal24LC02B.h"
#include "halMPU9250.h"
#include "halI2S.h"

#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "bottomBoard.h"
#include "ultraSound.h"

#include "halnRF24L01P.h"
#include "basicMotion.h"
#include "CRC16.h"

void vPushBoxTask( void *pvParameters){
   static int16_t angle ;
   angle = RobotAngle2North();
   asm("NOP");
   while(1)
     vTaskDelay(100);
}

void DEMOTask( void *pvParameters ){
  SetRobotSpeed(0, 0);
  vTaskDelay(1000);
  bool obstacleFind = false;
  bool grooveFind = false;
  uint8_t infSensor, bat1248, infSensor_bp, cnt;
  int16_t sL,sR;
  type_coordinate tp;
  for(;;) {
    for(;;) { // break if obstacle found
      while (GetRobotBStatus(&infSensor, &sL, &sR, &bat1248)==0){
        vTaskDelay(20);
      }
      if (infSensor) { // obstacle found, beep
        SetRobotSpeed(0, 0);
        halBeepOn(3951);
        vTaskDelay(20);
        halBeepOff();
        obstacleFind = true;
        break;
      }
      tp = RobotGetPosition();
      if ( (tp.x > 130) && (tp.y > 15) && (tp.x < 210) && (tp.y < 125) ){
        SetRobotSpeed(50, 50);
        vTaskDelay(200);
      } else {
        RobotRotate(20, 120);
        SetRobotSpeed(50, 50);
        vTaskDelay(1000);
      }
    }
    // obstacle is found
    for(;;) {
      // rotate until nothing ahead and something leftside
      cnt = 0;
      while ( (infSensor & 0x80) || !(infSensor & 0x2) || !(infSensor & 0x4) ) {
        RobotRotate(20, 10);
        cnt ++;
        while (GetRobotBStatus(&infSensor, &sL, &sR, &bat1248) == 0) {
          vTaskDelay(20);
        }
        if (cnt > 36) {
          SetRobotSpeed(0, 0);
          obstacleFind = false;
          break;
        }
      }
      if (obstacleFind == false) { // sensor error, find obstacle again
        break;
      }
      RobotRotate(20, -13);
      
      // go straight until nothing leftside
      SetRobotSpeed(10, 10);
      while ( (infSensor & 0x2) || (infSensor & 0x4)) {
        vTaskDelay(50);
        while (GetRobotBStatus(&infSensor, &sL, &sR, &bat1248) == 0) {
          vTaskDelay(20);
        }
      }
      SetRobotSpeed(0, 0);
      
      for (;;) {
        // Go straight a little bit
        SetRobotSpeed(10, 10);
        vTaskDelay(1500);
        SetRobotSpeed(0, 0);
      
        // turn left 90 degree
        RobotRotate(20, -90);
        SetRobotSpeed(0, 0);
        
        // go directly until something leftside
        SetRobotSpeed(10, 10);
        while (!(infSensor & 0x2) ) {
          vTaskDelay(50);
          while (GetRobotBStatus(&infSensor, &sL, &sR, &bat1248) == 0) {
            vTaskDelay(20);
          }
        }
        halBeepOn(2100);
        vTaskDelay(50);
        halBeepOff();
//        SetRobotSpeed(0, 0);
        
        // go straight for 0.5s
//        SetRobotSpeed(10, 10);
        vTaskDelay(500);
        
        // go straight until nothing leftside
        SetRobotSpeed(10, 10);
        grooveFind = false;
        for(;;) {
          //vTaskDelay(50);
          halBeepOn(2100);
          vTaskDelay(50);
          //halBeepOff();
          while (GetRobotBStatus(&infSensor, &sL, &sR, &bat1248) == 0) {
            vTaskDelay(20);
          }
          if ( ( infSensor & 0xc1 ) == 0xc1 ) {
            halBeepOff();
            SetRobotSpeed(0, 0);
            grooveFind = true;
            break;
          }
          if (!(infSensor & 0x2) ) {
            halBeepOff();
            SetRobotSpeed(0, 0);
            break;
          }
          //cnt = 0;
          //infSensor_bp = infSensor;
          //while (infSensor_bp) {
            //if (infSensor_bp & 1) cnt ++;
            //infSensor_bp >>= 1;
          //}
          //if (cnt > 5) {
          //  grooveFind = true;
          //  break;
          //}
        }
        if (grooveFind) {
          break;
        }
      }
      halBeepOn(2100);
      vTaskDelay(20);
      halBeepOff();
      SetRobotSpeed(0, 0);
      // Push the box outside
      SetRobotSpeed(30, 30);
      for (;;) {
        tp = RobotGetPosition();
        if((tp.x <= 123) || (tp.y <= 8) || (tp.x >= 217) || (tp.y >= 132)) {
          break;
        }
        vTaskDelay(100);
      }
      SetRobotSpeed(0, 0);
      halBeepOn(2100);
      vTaskDelay(50);
      halBeepOff();
      // counter-clockwise rotate
      RobotRotate(20, -40);
      // Move backward for 14cm
      SetRobotSpeed(-20, -20);
      vTaskDelay(10000);
      // beep forever
      halBeepOn(2100);
      vTaskDelay(50);
      halBeepOff();
      SetRobotSpeed(0, 0);
      for(;;)
        vTaskDelay(1000);
    }
  }
}




typedef struct type_coPacket{
  uint16_t id;
  int16_t  cx;
  int16_t  cy;
  uint16_t crc16Res;
}type_coPacket;
void COORDINATECALITask(void *pvParameters) {
  
  while(1){
    type_coordinate rP;
    rP = RobotGetPosition();
    
    uint8_t tx[32];
    type_coPacket* p = (type_coPacket*)tx;

    p->id = 1;
    p->cx = rP.x;
    p->cy = rP.y;
    p->crc16Res = CRC16(tx, 6);
    
    extern xQueueHandle xQueueHandleRFTx, xQueueHandleRFRx;
    while (xQueueSendToBack(xQueueHandleRFTx, tx, portMAX_DELAY)!=pdPASS)  {
      vTaskDelay(5);
    }
    
    halSetLedStatus(LED_RED, LED_TOGGLE);

    vTaskDelay(500);
    
  }
}


#define CC_BUF_SIZE   (1000)
typedef struct type_cPacket{
  uint16_t id;
  int16_t  cx;
  int16_t  cy;
  int16_t  cz;
  uint16_t crc16Res;
}type_cPacket;

void COMPASSCALITask(void* pvParameters) {
  
  static int16_t  cx[CC_BUF_SIZE], cy[CC_BUF_SIZE], cxt, cyt,czt;
  static uint16_t cxy=0;
  
  SetRobotSpeed(5, -5);
  while(1) {
    portTickType xLastWakeTime; 
    xLastWakeTime = xTaskGetTickCount(); 
    
    while(halMPU9250RdCompassX(&cxt)==0) {
      vTaskDelay(5);
    }
    while(halMPU9250RdCompassY(&cyt)==0) {
      vTaskDelay(5);
    }
    while(halMPU9250RdCompassZ(&czt)==0) {
      vTaskDelay(5);
    }

    cx[cxy] = cxt;
    cy[cxy] = cyt;
    
    uint8_t tx[32];
    type_cPacket* p = (type_cPacket*)tx;

    p->id = 1;
    p->cx = cxt;
    p->cy = cyt;
    p->cz = czt;
    p->crc16Res = CRC16(tx, 8);
    
    extern xQueueHandle xQueueHandleRFTx, xQueueHandleRFRx;
    while (xQueueSendToBack(xQueueHandleRFTx, tx, portMAX_DELAY)!=pdPASS)  {
      vTaskDelay(5);
    }
    
    halSetLedStatus(LED_RED, LED_TOGGLE);

    cxy++;
    if (cxy==CC_BUF_SIZE) {
      cxy = 0;
      SetRobotSpeed(0, 0);
    }
    
    vTaskDelayUntil(&xLastWakeTime, 50);  
  }
}

void INFOTask( void *pvParameters ){
  while(1) {
    vTaskDelay(200);
    uint8_t infSensor, bat1248;
    int16_t sL,sR;
    GetRobotBStatus(&infSensor, &sL, &sR, &bat1248);
    if ( ((infSensor & (1<<0)) == (1<<0)) ||
        ((infSensor & (1<<6)) == (1<<6)) ||
          ((infSensor & (1<<7)) == (1<<7)) ) 
    {
      SetRobotSpeed(-50, -50);
    }
    else if ( ((infSensor & (1<<2)) == (1<<2)) ||
             ((infSensor & (1<<3)) == (1<<3)) ||
               ((infSensor & (1<<4)) == (1<<4)) )
    {
      SetRobotSpeed(50, 50);
    }
    else {
      SetRobotSpeed(0, 0);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
//INFERAD SENSOR AND BAT
////////////////////////////////////////////////////////////////////////////////
void BOTTOMTask( void *pvParameters ){
  while(1) {
    vTaskDelay(200);
    static uint8_t infSensor, bat1248;
    int16_t sL,sR;
    GetRobotBStatus(&infSensor, &sL, &sR, &bat1248);
    if ( ((infSensor & (1<<0)) == (1<<0)) ||
          ((infSensor & (1<<6)) == (1<<6)) ||
            ((infSensor & (1<<7)) == (1<<7)) ) 
    {
      SetRobotSpeed(-50, -50);
    }
    
    else if ( ((infSensor & (1<<2)) == (1<<2)) ||
               ((infSensor & (1<<3)) == (1<<3)) ||
                 ((infSensor & (1<<4)) == (1<<4)) )
    {
      SetRobotSpeed(50, 50);
    }
    
    else {
      SetRobotSpeed(0, 0);
    }

  }
}


////////////////////////////////////////////////////////////////////////////////
//MOTOR
////////////////////////////////////////////////////////////////////////////////
void MOTORTask( void *pvParameters ){
  while(1) {
    
    vTaskDelay(1000);
   
    SetRobotSpeed(50, 50);
    vTaskDelay(1000);
    SetRobotSpeed(-50, -50);
    vTaskDelay(1000);
    SetRobotSpeed(25, -25);
    vTaskDelay(1000);
  }
}

////////////////////////////////////////////////////////////////////////////////
//MICRO PHONE SENSOR
////////////////////////////////////////////////////////////////////////////////
void MICROPHONETask( void *pvParameters ){
  while(1) {
    static uint16_t* p;
    
    halI2SStartSample();
    vTaskDelay(200);
    halI2SStopSample();
    p=halI2SReadMicroPhone();
    
    uint16_t i;
    static int32_t mRes[I2S_DMA_BUF_SIZE/4];
    uint32_t TData;
    
    i=0;
    while (i<I2S_DMA_BUF_SIZE-4) {
      TData = (((*(p+i) << 16) + *(p+i+1)) << 1);
      mRes[i/4] = *(int32_t*)(&TData)/128;
      i+=4;
    }
    asm ("NOP");
    //mRes store the microphone record result
  }
}

////////////////////////////////////////////////////////////////////////////////
//COMPASS SENSOR
////////////////////////////////////////////////////////////////////////////////
#define BUF_SIZE  1
void COMPASS_Task( void *pvParameters ){
  while(1) {
    #define   SIZE   500
    static int16_t  cX[SIZE], out[SIZE];
    static uint16_t cXP=0;
    uint16_t i;
    
    static int16_t max=-32767, min=32767;
    
    //SetRobotSpeed(5, -5);
    while (1) {
      while (halMPU9250RdCompassX(cX+cXP)==0){
        vTaskDelay(5);
      }
      if (cXP>=10) {
        out[cXP-10] = (cX[cXP-10]+cX[cXP-9]+cX[cXP-8]
                      +cX[cXP-7]+cX[cXP-6]+cX[cXP-5]
                        +cX[cXP-4]+cX[cXP-3]+cX[cXP-2]
                          +cX[cXP-1])/10;
      }
      cXP++;
      if (cXP==SIZE) {
        cXP= 0;
        for (i=0;i<SIZE;i++) {
          if (cX[i]>max) {
            max=cX[i];
          }
          if (cX[i]<min) {
            min=cX[i];
          }
        }
        asm ("NOP");
        break;
      }
    }
    /*
    static int16_t testCompass;
    while (1) {
      while (halMPU9250RdCompassX(&testCompass)==0){
        vTaskDelay(2);
      }
      if(abs(max-testCompass) > 20) {
        SetRobotSpeed(-10, 10);
      }
      else {
        SetRobotSpeed(0, 0);
      }
      vTaskDelay(10);
    }
    */
  }
}
////////////////////////////////////////////////////////////////////////////////
//GYRO SENSOR
////////////////////////////////////////////////////////////////////////////////
void GYRO_Task(void *pvParameters) {  
  while(1) {
    static float gyroZ;
    gyroZ = halMPU9250RdGyroZ();
    if (gyroZ > 5) {
      halBeepOn(2093);
      vTaskDelay(20);
      halBeepOff();
      vTaskDelay(980);
    }
    else if (gyroZ <-5){
      halBeepOn(3951);
      vTaskDelay(20);
      halBeepOff();
      vTaskDelay(980);  
    }
    else {
      vTaskDelay(50);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
//ACCEL SENSOR
////////////////////////////////////////////////////////////////////////////////
void ACCEL_Task( void *pvParameters ) {  
  while(1) {
    
    static float accelZ;
    
    accelZ = halMPU9250RdAccelZ();

    if (accelZ > 0.5) {
      halBeepOn(2093);
      vTaskDelay(20);
      halBeepOff();
      vTaskDelay(980);
    }
    else if (accelZ <-0.5){
      halBeepOn(3951);
      vTaskDelay(20);
      halBeepOff();
      vTaskDelay(980);  
    }
    else {
      vTaskDelay(50);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
//2 K EEPROM -> 256 BYTEs 2K BITS
////////////////////////////////////////////////////////////////////////////////
void EEPROM_Task( void *pvParameters ) {
  
  while (1) {
    uint8_t i;
    static uint8_t inData[100], outData[100];
    for (i=0;i<100;i++) {
      inData[i]  = i;
      outData[i] = 0;
    }
    
    for (i=0;i<100;i++) { 
      while (hal24LC02BByteWrite(100+i, inData[i])==false);
    }    
    
    for (i=0;i<100;i++) { 
      while (hal24LC02BRandomRead(100+i, outData+i)==false);
    }
    
    for (i=0;i<100;i++) {
      if (inData[i] != outData[i]) {
        break;
      }
    }
    if (i==100) {
      halSetLedStatus(LED_YELLOW, LED_TOGGLE);
    }
    vTaskDelay(250);
  }
}

////////////////////////////////////////////////////////////////////////////////
//Light sensor
//100HZ noise, simply filter raw data....
////////////////////////////////////////////////////////////////////////////////
void LIGHTSENSORTask( void *pvParameters )  {
  while(1)  {
    static uint16_t lightSensorData[10];
    static uint8_t p=0;
    uint8_t i;
    uint32_t sum;
    
    lightSensorData[p++] = halADC1GetData(0);
    if (p == 10){
      p =0 ;
      sum = 0;
      for (i=0;i<10;i++) {
        sum+=lightSensorData[i];
      }
      if ((sum/10)<800) {
        halBeepOn(2093);
        vTaskDelay(50);
        halBeepOff();
        vTaskDelay(950);
      }
    }
    vTaskDelay(1);
  }
}

////////////////////////////////////////////////////////////////////////////////
//BUTTON AND LED
////////////////////////////////////////////////////////////////////////////////
extern xSemaphoreHandle xBSB1,xBSB2;

//BUTTON 1
void EXTI3ISR_BUTTON(void){
  portBASE_TYPE xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE; 
  xHigherPriorityTaskWoken = xSemaphoreGiveFromISR(xBSB1, &xHigherPriorityTaskWoken);
  if( xHigherPriorityTaskWoken == pdTRUE ) {
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken );
  }
}
//BUTTON 2
void EXTI4ISR(void){
  portBASE_TYPE xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE; 
  xHigherPriorityTaskWoken = xSemaphoreGiveFromISR(xBSB2, &xHigherPriorityTaskWoken);
  if( xHigherPriorityTaskWoken == pdTRUE ) {
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken );
  }  
}

void BUTTON_LED_DEMOTask( void *pvParameters )  {
  
  portBASE_TYPE xStatus;
  while(1)  {
   xStatus = pdFALSE ;
   xStatus = xSemaphoreTake(xBSB1, 10);
   if (xStatus == pdPASS) {
     halSetLedStatus(LED_RED, LED_TOGGLE);
   }
   
   xStatus = pdFALSE ;
   xStatus = xSemaphoreTake(xBSB2, 10);
   if (xStatus == pdPASS) {
     //start
     halSetLedStatus(LED_GREEN, LED_TOGGLE);
     //end
   }
   // BUTTON 3... POLLING...
   if (GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_5) == Bit_RESET) {
     halSetLedStatus(LED_YELLOW, LED_TOGGLE);
     while(GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_5) == Bit_RESET);
   }
  }
}

////////////////////////////////////////////////////////////////////////////////
//BEEP DEMO
////////////////////////////////////////////////////////////////////////////////
//已知最低音的6（拉）的频率是220，设公比为q，那么：7的频率就是220*q*q 。
//乘了两个q，是因为6、7之间隔了个黑键。以此类推，两个6之间，共有12个琴键，所以：
//低音区的6（拉）的频率就是：220*q*q*q*q*q*q*q*q*q*q*q*q = 440。
//马上就可以看出，12个q相乘，等于： 440 / 220 = 2。
//那么就可以求出公比：q = 2的12次方根 = 1.059463094。
//用这个公比，和已知的220进行计算，可以得出全部琴键所对应的频率
////////////////////////////////////////////////////////////////////////////////
//2/4   四分音符为1拍  DELAY_1_4   定义DELAY_1_4为1/4拍的时间长度
#define DELAY_1_4   (100)
const int16_t musicTable[] = {
  -6, -6, -5, -6, -6, 1,  1,  2,  1, -6,  0xFF,
   1,  1, -5,  1,  2, 3,  5,  5,  3,  2,  3, 0xFF, 
   6,  6,  5,  3,  3, 1, -6, -6, -6,  3,  2, 3,    2, 0xFF,
   3,  3,  5,  3,  2, 3,  2,  1, -6, -5, -6, 0xFF, 
   3,  3,  5,  3,  3, 5,  5,  6,  8,  6,  5, 6, 8, 6, 0xFF,
};

//每个时间单位为1/4拍
const uint16_t delayTable[] = {
  4,2,2,4,2,2,4,2,2,4,4,
  4,2,2,2,2,2,2,2,2,4,4,4,
  2,2,2,2,2,2,2,2,2,2,4,1,1,2,4,
  2,2,2,2,2,2,2,2,4,4,4,4,
  2,2,2,2,2,2,2,2,2,2,4,1,1,2,4,
};
////////////////////////////////////////////////////////////////////////////
// do、re、mi、fa、so、la、ti  Table
////////////////////////////////////////////////////////////////////////////
//static uint16_t SoundLTable[7] = {1047,1175,1329,1397,1568,1760,1976};    //LOW 1234567
//static uint16_t SoundTable[7] = {2093,2349,2637,2794,3136,3520,3951};     //1234567
//static uint16_t SoundHTable[3] = {4186,4699,5274};                        //HIGH 123
const uint16_t SoundTable[18] = {
  1976,1760,1568,1397,1329,1175,1047, 0,   //LOW 7654321 -> 0-> no meaning 
  2093,2349,2637,2794,3136,3520,3951,      //1234567
  4186,4699,5274                           //HIGH 123
};                         

void BEEP_DEMOTask( void *pvParameters )  {
  while(1)  {
    uint16_t i, len;
    len = sizeof (musicTable);
    
    for (i=0;i<len;i++) {
      if (musicTable[i] == 0xFF) {
        halBeepOff();
      }
      else {
        halBeepOn(SoundTable[musicTable[i]+7]);      //halBeepOn(FREQUENCY) 
      }
      vTaskDelay(delayTable[i] * DELAY_1_4);
    }
    vTaskDelay(100000);
  }
}
