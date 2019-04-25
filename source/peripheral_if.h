#ifndef _PERIPHERAL_IF_
#define _PERIPHERAL_IF_
//#include "ad7124_defs.h"

typedef struct{
  int32_t mul_a;
  int32_t mul_b;
  int32_t zero_x;
  int32_t zero_y;
}storedParam_t;

typedef struct sysParam_s{
  double mul_a;
  double mul_b;
  int32_t zero_x;
  int32_t zero_y;
}sysParam_t;


typedef enum{
  SYS_IDLE,
  SYS_NORMAL,
  SYS_GO_SLEEP,
  SYS_SLEEP,
  SYS_GO_WKUP,
}sysstate_t;

#define NOF_TORQUE_HISTORY      4 //cycles
typedef struct{
  bool isPositive;
  bool valid;
  bool dirChanged;
  bool defined;
  uint8_t index;
  double avgTorq[NOF_TORQUE_HISTORY];
  uint16_t cycleCount;
  double cycleTorq;
}torque_data_t;

typedef struct{
  sysstate_t state;
  torque_data_t torque;
  uint16_t power;
  int32_t rpm;
  double dTorque;
  double staticTorque;
  uint8_t actChannel;
  bool goStop;
}appParam_t;


extern sysParam_t sysParam;
extern appParam_t appParam;

void peripheral_init(void);
void peripheral_deinit(void);

int32_t getPeriod(void);
int32_t getTicks(void);
int32_t getRPM(void);
uint16_t getTorque(void);
uint16_t getPower(void);

void peripheralVars_init(void);
//int8_t shouldSleep(void);
void goSleep(void);
int8_t readySleep(void);
void peripheral_start();
void peripheral_stop();
sysstate_t sysGetState(void);


#endif