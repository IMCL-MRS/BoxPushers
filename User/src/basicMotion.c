#include "basicMotion.h"
#include "ultraSound.h"
#include "bottomBoard.h"
#include "math.h"
#include "halMPU9250.h"
#include "stdlib.h"
#include "halBeep.h"
//0��180�ȵ�cos��
const float cosTable[] = {
1.0f,
0.999847695161587f,
0.999390827039876f,
0.998629534801318f,
0.997564050342896f,
0.996194698221486f,
0.994521895554995f,
0.992546151895304f,
0.990268069073048f,
0.987688341014302f,
0.984807753529195f,
0.981627184072550f,
0.978147601476602f,
0.974370065655880f,
0.970295727284350f,
0.965925827444907f,
0.961261697251328f,
0.956304757442804f,
0.951056517951169f,
0.945518577440958f,
0.939692622822441f,
0.933580428737769f,
0.927183857020410f,
0.920504856128005f,
0.913545460548859f,
0.906307790182209f,
0.898794049692488f,
0.891006527837756f,
0.882947596772531f,
0.874619711325199f,
0.866025408250255f,
0.857167305455581f,
0.848048101205006f,
0.838670573296391f,
0.829037578215481f,
0.819152050265795f,
0.809017000674805f,
0.798635516676694f,
0.788010760571947f,
0.777145968764087f,
0.766044450773835f,
0.754709588230999f,
0.743144833844394f,
0.731353710350118f,
0.719339809438486f,
0.707106790659974f,
0.694658380310478f,
0.681998370296251f,
0.669130616978852f,
0.656059040000461f,
0.642787621089918f,
0.629320402849847f,
0.615661487525243f,
0.601815035753891f,
0.587785265298989f,
0.573576449764381f,
0.559192917292776f,
0.544639049247351f,
0.529919278877149f,
0.515038089966664f,
0.500000015470041f,
0.484809636130293f,
0.469471579083961f,
0.453990516451646f,
0.438371163914833f,
0.422618279279450f,
0.406736661026595f,
0.390731146850871f,
0.374606612186780f,
0.358367968723613f,
0.342020162909309f,
0.325568174443714f,
0.309017014761716f,
0.292371725506718f,
0.275637376994901f,
0.258819066670756f,
0.241921917554356f,
0.224951076680834f,
0.207911713532541f,
0.190809018464382f,
0.173648201122772f,
0.156434488858737f,
0.139173125135607f,
0.121869367931806f,
0.104528488139224f,
0.0871557679576513f,
0.0697564992857674f,
0.0523359821091799f,
0.0348995228859955f,
0.0174524329304235f,
0.0f,
-0.0174523793487923f,
-0.0348994693288475f,
-0.0523359285928295f,
-0.0697564458265162f,
-0.0871557145717835f,
-0.104528434843001f,
-0.121869314741463f,
-0.139173072067346f,
-0.156434435928723f,
-0.173648148347128f,
-0.190808965859184f,
-0.207911661113814f,
-0.224951024464544f,
-0.241921865556409f,
-0.258819014906991f,
-0.275637325481086f,
-0.292371674258544f,
-0.309016963794794f,
-0.325568123773569f,
-0.342020112551376f,
-0.358367918693231f,
-0.374606562499189f,
-0.390731097521206f,
-0.406736612069882f,
-0.422618230710602f,
-0.438371115748646f,
-0.453990468702791f,
-0.469471531766982f,
-0.484809589259604f,
-0.499999969059918f,
-0.515038044031245f,
-0.529919233430426f,
-0.544639004303169f,
-0.559192872864824f,
-0.573576405866192f,
-0.587785221943936f,
-0.601814992955179f,
-0.615661445295910f,
-0.629320361202755f,
-0.642787580037754f,
-0.656058999555731f,
-0.669130577153874f,
-0.681998331103157f,
-0.694658341761206f,
-0.707106752766267f,
-0.719339772211887f,
-0.731353673801966f,
-0.743144797985823f,
-0.754709553072930f,
-0.766044416326979f,
-0.777145935038937f,
-0.788010727578775f,
-0.798635484425551f,
-0.809016969175515f,
-0.819152019527951f,
-0.829037548248448f,
-0.838670544109297f,
-0.848048072806742f,
-0.857167277854796f,
-0.866025381455358f,
-0.874619685344351f,
-0.882947571613646f,
-0.891006503508499f,
-0.898794026200268f,
-0.906307767534183f,
-0.913545438751925f,
-0.920504835188804f,
-0.927183836945319f,
-0.933580409532904f,
-0.939692604493651f,
-0.945518559993827f,
-0.951056501391011f,
-0.956304741774664f,
-0.961261682479978f,
-0.965925813574846f,
-0.970295714319804f,
-0.974370053600798f,
-0.978147590334656f,
-0.981627173847135f,
-0.984807744223424f,
-0.987688332631010f,
-0.990268061614789f,
-0.992546145364349f,
-0.994521889953335f,
-0.996194693550827f,
-0.997564046604660f,
-0.998629531996644f,
-0.999390825169618f,
-0.999847694226315f,
-0.999999999999999f,
};
////////////////////////////////////////////////////////////////////////////////
//����mmλ��
////////////////////////////////////////////////////////////////////////////////
type_coordinate  robotCoordinate;
type_coordinate RobotGetPosition(void){
  int32_t d1, d2;
  type_coordinate  tc;
  
  d1 = GetDistance(1);
  d2 = GetDistance(2);
  
  tc.y = (d1*d1 - d2*d2 + DISTANCE_B1_2_B2*DISTANCE_B1_2_B2)/(2*DISTANCE_B1_2_B2);
  if ((d1*d1 - DISTANCE_B1_2_B2*DISTANCE_B1_2_B2 - tc.y*tc.y) < 0){
    halBeepOn(2093);
    vTaskDelay(20);
    halBeepOff();
    return robotCoordinate;
  }
  else {
    tc.x = sqrt(d1*d1 - DISTANCE_B1_2_B2*DISTANCE_B1_2_B2 - tc.y*tc.y);
  }
  
  tc.y = (tc.y)/(10);
  tc.x = (tc.x)/(10);
  
  robotCoordinate.x = tc.x;
  robotCoordinate.y = tc.y;
  
  return tc;
}

void RobotGoStrait(int16_t ls, int16_t rs) {
  SetRobotSpeed(ls, rs);
}

void RobotGoCircleLeft(int16_t s, uint16_t r) {
  int16_t sl, sr;
  sl = s-(DIS_WHEEL_2_WHEEL>>2)/r;
  sr = s+(DIS_WHEEL_2_WHEEL>>2)/r;
  SetRobotSpeed(sl, sr);
}

void RobotGoCircleRight(int16_t s, uint16_t r) {
  int16_t sl, sr;
  sl = s+(DIS_WHEEL_2_WHEEL>>2)/r;
  sr = s-(DIS_WHEEL_2_WHEEL>>2)/r;
  SetRobotSpeed(sl, sr);
}

void RobotGoTo(uint16_t x, uint16_t y) {
  RobotTowardDst(x,y);
  RobotGoStrait(50, 50);  
  SetRobotSpeed(0, 0);
}
////////////////////////////////////////////////////////////////////////////////
//˳ʱ��Ϊ������, ��ʱ��Ϊ������
////////////////////////////////////////////////////////////////////////////////
void RobotRotate(int16_t s, int16_t angle) {
  static uint32_t waitTime=0;
  int16_t sl, sr, sm;
  int32_t ang;
  
  SetRobotSpeed(0, 0);
  
  if (s<0) {
    sm = -s;
  }
  else {
    sm = s;
  }
  
  if (angle>0) {
    sl = s;
    sr = -s;
    ang = angle;
  }
  else {
    ang = -angle;
    sl = -s;
    sr = s;
  }
  
  waitTime = (uint32_t)(310*ang/sm);
  
  SetRobotSpeed(sl, sr);
  vTaskDelay(waitTime);
  SetRobotSpeed(0, 0);
  asm("NOP");
}

#define ANGLE_COORDINATE_NORTH   (60)
int16_t RobotTowardDst(int32_t x, int32_t y) {
  type_coordinate cp;
  int32_t edge;
  int32_t disY, disX;
  int16_t angCoor;
  float acosValue;
  uint16_t i;
  int16_t r2North;
  int16_t rotateAngle=0;
  
  cp = RobotGetPosition();
  
  disX = x - cp.x;
  disY = y - cp.y;
  
  edge = (int32_t)(sqrt(disX*disX + disY*disY));
  
  acosValue = (float)disX/edge;
  
  for (i=0; i<sizeof(cosTable); i++) {
    if (acosValue >= cosTable[i]){
      break;
    }
  }
  angCoor = i;
  
  if (disY<0) {
    angCoor = angCoor;
  }
  else {
    angCoor = -angCoor;
  }
  
  r2North = RobotAngle2North();
  
  //����������ͷ��ָ���ǵ�X��������� 
  r2North = r2North + ANGLE_COORDINATE_NORTH;
  
  if (r2North>=180) {
    r2North = r2North-360;
  }
  
  rotateAngle = angCoor-r2North;
  
  if (rotateAngle > 180)  {
    rotateAngle = rotateAngle-360;
  }
  
  if (rotateAngle < -180)  {
    rotateAngle = rotateAngle+360;
  }
  
  RobotRotate(20, rotateAngle);
  
  asm ("NOP");
  return rotateAngle;
}

void RobotFollowLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
  
}

void RobotFollowCircle(uint16_t x, uint16_t y, uint16_t r, int16_t s) {

}


void RobotFindObstacle(void) {
  
}

bool RobotInRange(int32_t x1, int32_t y1, int32_t x2, int32_t y2) {
  type_coordinate tp;
  
  tp = RobotGetPosition();
  
  if ((tp.x > x1) && (tp.y > y1) && (tp.x < x2) && (tp.y < y2)) {
    return true;
  }
  return false;
}

int16_t RobotAngle2North(void) {
  float edge, acosValue;
  int16_t cxt, cyt;
  uint16_t i;
  int16_t angle;
  
  while(halMPU9250RdCompassX(&cxt)==0) {
    vTaskDelay(5);
  }
  
  while(halMPU9250RdCompassY(&cyt)==0) {
    vTaskDelay(5);
  }
  
  cxt = cxt - COMPASS_X_CALI_PARA;
  cyt = cyt - COMPASS_Y_CALI_PARA;
  
  edge = sqrt(cxt*cxt + cyt*cyt);
  //1. ����acos�޷�����
  if (fabs(cxt) > edge) {
    if (cxt>0) {
      return 180;
    }
    if (cxt<0) {
      return 0;
    }
  }
  
  //2. ����Ƕ�
  acosValue = (float)cxt/edge;
  
  for (i=0; i<sizeof(cosTable); i++) {
    if (acosValue >= cosTable[i]){
      break;
    }
  }
  
  if (cyt < 0) {
    angle = i-180;
  }
  else {
    angle = 180-i;
  }

  return angle;
}

                  
                  
                  