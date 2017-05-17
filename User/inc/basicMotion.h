#ifndef __BASICMOTION_H
#define __BASICMOTION_H

#include "stm32f10x.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "stdbool.h"


#define DISTANCE_B1_2_B2         (1310)   
#define DISTANCE_B_2_GROUND      (1750) 

////////////////////////////////////////////////////////////////////////////////
//UNIT: 0.1mm
//PI  : 3.1415926*100
////////////////////////////////////////////////////////////////////////////////
#define DIS_WHEEL_2_WHEEL        (600)
#define WHEEL_DIA                (200)
#define PI                       (314)

////////////////////////////////////////////////////////////////////////////////
//机器人头方向与北方夹角-180 到 180度, 顺时针为正
////////////////////////////////////////////////////////////////////////////////
#define ROBOT_3

#ifdef ROBOT_1
#define COMPASS_X_CALI_PARA        (25.0f)
#define COMPASS_Y_CALI_PARA        (-161.0f)
#define MAG_RATIO                  (388/454.0f)
#endif // ROBOT_1
#ifdef ROBOT_3
#define COMPASS_X_CALI_PARA        (-46.5f)
#define COMPASS_Y_CALI_PARA        (227.5f)
#define MAG_RATIO                  (347/471.0f)
#endif // ROBOT_3

#ifdef ROBOT_4
#define COMPASS_X_CALI_PARA        (64.5f)
#define COMPASS_Y_CALI_PARA        (0)
#define MAG_RATIO                  (363/418.0f)
#endif // ROBOT_4

//#define COMPASS_X_CALI_PARA        (129.0f)
//#define COMPASS_Y_CALI_PARA        (382.0f)

typedef struct type_coordinate{
  int16_t x;
  int16_t y;
}type_coordinate;

extern type_coordinate RobotGetPosition(void);
extern void RobotGoCircleRight(int16_t s, uint16_t r);
extern void RobotGoCircleLeft(int16_t s, uint16_t r);
extern void RobotRotate(int16_t s, int16_t angle);
extern bool RobotInRange(int32_t x1, int32_t y1, int32_t x2, int32_t y2);
extern int16_t RobotAngle2North(void);
extern int16_t CalibrateNorth2X(void);
extern int16_t RobotTowardDst(int32_t x, int32_t y,int16_t s);
extern int16_t ReadAngle2North();
extern void GotoWaypoint(int32_t x, int32_t y);
extern int8_t whichSide(int32_t x, int32_t y);
extern int16_t getStatus();
extern int16_t setStatus(int16_t _status);
#endif

