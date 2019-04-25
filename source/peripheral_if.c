#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "nrf_spim.h"
#include "nrf_drv_spi.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "sdk_config.h"
#include "peripheral_if.h"
#include "bmi160_defs.h"
#include "bmi160_cmd.h"
#include "gd17003.h"


extern int32_t adResult[];

#define BMI160_GYRO_ODR         25
#define SENSORDATA_BUFFER_SIZE  BMI160_GYRO_ODR*3
#define DPS_PER_LSB     0.061
#define REV_PER_SEC     DPS_PER_LSB/360.
#define LSB_TO_RPM      DPS_PER_LSB/60
#define LSB_TO_REV_MS   DPS_PER_LSB*1000000/360 // us per lsb
#define LSB_TO_TICK_MS   DPS_PER_LSB*1000/360/0.488 // us per lsb


#define ADC_BUFFER_SZ   16
#define ADC_SAMPLE_RATE 40
#define ADC_TORQUE_SZ   ADC_SAMPLE_RATE*3       // 3-seconds
typedef struct{
  int32_t v1;
  int32_t v2;
  uint16_t cntr;
}adc_data_t;

static struct bmi160_sensor_data accel[SENSORDATA_BUFFER_SIZE];
static struct bmi160_sensor_data gyro[SENSORDATA_BUFFER_SIZE];
static struct bmi160_sensor_data accel_aveg,gyro_aveg;
uint16_t bmi160_sensor_index = 0;
int32_t ticks, period;

#define INACTIVE_THRESHOLD      BMI160_GYRO_ODR*200     // bmi byro odr(25)*4
uint32_t inactiveCntr;
appParam_t appParam;
uint8_t bmidata[12];

double mvv[2];
int32_t raw[2];
#define NOF_ADC_HISTORY 16
uint8_t adc_raw_index = 0;
adc_data_t adc_raw_history[NOF_ADC_HISTORY];



double calTorque(int32_t mv1, int32_t mv2)
{
  double v1, v2, t;
  v1 = (double)(mv1 - sysParam.zero_x)/8388608./128.*1000.;
  v2 = (double)(mv2 - sysParam.zero_y)/8388608./128.*1000.;
  t = sysParam.mul_a*(v1 + sysParam.mul_b*v2);
  return t;
}

static union bmi160_int_status intstatus;
enum bmi160_int_status_sel int_status_sel;
static uint16_t intCntr = 0;
static uint8_t orientCntr = 0;




void peripheral_deinit(void)
{
//  nrf_drv_spi_uninit(&ad7124_spi);
//  nrf_drv_spi_uninit(&bmi_spi);  
//  nrf_gpio_cfg_default(AD7124_CS_PIN);
//  nrf_gpio_cfg_default(BMI160_CS_PIN);
}
void peripheral_init(void)
{
  APP_ERROR_CHECK(nrf_drv_gpiote_init());
  // spim0


  
  
  appParam.torque.isPositive = false;
  appParam.torque.valid = false;
  appParam.torque.defined = false;
  appParam.torque.cycleCount = 0;
  appParam.torque.cycleTorq = 0;
  appParam.goStop = false;
  //simulateTorque(50,40,20,2);


}



void peripheral_start(void)
{
  //bmi160_cmd_pwup(&bmi160);
  //bmi160_cmd_testRead(&bmi160);
  //bmi160_cmd_config_int_am(&bmi160,0);
  //bmi160_cmd_config_int(&bmi160,1);  
  //nrf_drv_gpiote_in_event_enable(BMI160_INT1_PIN,true);
  //nrf_drv_gpiote_in_event_enable(BMI160_INT2_PIN,true);
  inactiveCntr = 0;
  appParam.state = SYS_NORMAL;
}

void peripheral_stop(void)
{
  
}

void goSleep(void)
{
//  if(ad7124_selected() == 1){
//    nrf_drv_gpiote_in_event_enable(BMI160_INT1_PIN,false);
//    bmi160_cmd_pwdn(&bmi160);
//    bmi160_cmd_config_int_am(&bmi160,1);
//    bmi160_cmd_config_int(&bmi160,0); 
//    appParam.state = SYS_SLEEP;
//    peripheral_deinit();
//  }
}

// function for ANT+ I/F
int32_t getTicks(void)
{
    return ticks;
}

// get the period per revolution
int32_t avgx, avgy, avgz;
int32_t getPeriod(void)
{
  period = 0;
  double v;
  avgx = avgy = avgz = 0;
  for(uint16_t i=0;i<SENSORDATA_BUFFER_SIZE;i++){
    avgx += gyro[i].x;
    avgy += gyro[i].y;
    avgz += gyro[i].z<0?-gyro[i].z:gyro[i].z;
  }
  avgx /= SENSORDATA_BUFFER_SIZE;
  avgy /= SENSORDATA_BUFFER_SIZE;
  avgz /= SENSORDATA_BUFFER_SIZE;
  
  v = avgz * REV_PER_SEC;
  appParam.rpm = 60*v;
  if(appParam.rpm > 255) appParam.rpm = 0;
  v = 1.0/v;
  v *= 2048;
  period = (int32_t)(v);
 
  if(period < 400) period = 0;
  if(period > 12800) period = 0;
  
  
  return period;
}

int32_t getRPM(void)
{
  
  return appParam.rpm;
}

//enum {
//  DIR_NONE,
//  DIR_UP,
//  DIR_DOWN
//};

#define REPORT_STATIC



uint16_t getTorque(void)
{ 
#ifdef REPORT_STATIC
  return lround(appParam.staticTorque*32); 
#else
  double dv = 0;
  uint8_t act = 0;
  for(uint8_t i=0;i < NOF_TORQUE_HISTORY;i++){
    if(appParam.torque.avgTorq[i] > 0){
      act++;
      dv += appParam.torque.avgTorq[i];
    }
  }
  if(act){
    appParam.dTorque = 2*dv/ act;
  }else{
    appParam.dTorque = 0;
  }
  appParam.power = (uint16_t)lround(appParam.dTorque*appParam.rpm*1000./9549.);
  return lround(appParam.dTorque*32); 
#endif
  
 
}

uint16_t getPower(void)
{
  return appParam.power;
}

void peripheralVars_init(void)
{
  inactiveCntr = 0;
}


//int8_t readySleep(void)
//{
//  
//  return (ad7124.state == AD7124_READY)?1:0;
//}

sysstate_t sysGetState(void)
{
  return appParam.state;
}


int32_t getAdcRaw(uint8_t ch)
{
  int32_t sum = 0;
  uint8_t i;
  for(i=0;i<NOF_ADC_HISTORY;i++){
    sum += (ch==0)?adc_raw_history[i].v1:adc_raw_history[i].v2;
  }
  sum /= NOF_ADC_HISTORY;
    
  return sum;
}

//void setAdcOffset(int32_t x, int32_t y)
//{
//}