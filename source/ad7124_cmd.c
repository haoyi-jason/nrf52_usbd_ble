#include "ad7124.h"
#include "ad7124_defs.h"
#include "ad7124_cmd.h"
#include "nrf_spim.h"
#include "nrf_drv_spi.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "sdk_config.h"
#include "gd17003.h"
#include "peripheral_if.h"

int8_t ad7124_spi_read(uint8_t reg_adr, uint8_t *b, uint16_t n);
int8_t ad7124_spi_write(uint8_t reg_adr, uint8_t *b, uint16_t n);
int8_t ad7124_select(uint8_t st);
int8_t ad7124_intctrl(uint8_t st);
void ad7124_delay(uint32_t period);
int8_t ad7124_conversion_done(void *p);
void ad7124_int_isr(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);


static const nrf_drv_spi_t ad7124_spi = NRF_DRV_SPI_INSTANCE(0);
static int32_t adResult[16];

ad7124_channel_t channels[] = {
  {.u={3,2,0,0,ADC_BIT_ENABLE}},
  {.u={4,5,0,0,ADC_BIT_ENABLE}},
  {.u={11,10,0,0,ADC_BIT_DISABLE}},
  {.u={15,14,0,0,ADC_BIT_DISABLE}},
  {.u={9,8,0,0,ADC_BIT_DISABLE}},
  {.u={11,10,0,0,ADC_BIT_DISABLE}},
  {.u={13,12,0,0,ADC_BIT_DISABLE}},
  {.u={15,14,0,0,ADC_BIT_DISABLE}},
  {.u={15,14,0,0,ADC_BIT_DISABLE}},
  {.u={15,14,0,0,ADC_BIT_DISABLE}},
  {.u={15,14,0,0,ADC_BIT_DISABLE}},
  {.u={15,14,0,0,ADC_BIT_DISABLE}},
  {.u={15,14,0,0,ADC_BIT_DISABLE}},
  {.u={15,14,0,0,ADC_BIT_DISABLE}},
  {.u={15,14,0,0,ADC_BIT_DISABLE}},
  {.u={15,14,0,0,ADC_BIT_DISABLE}},
};


static ad7124_setup_t setups[] = {
  {.u={ADC_PGA_X128,ADC_REF_REFEXT1,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BURNOUT_OFF,ADC_BIT_ENABLE}},
  {.u={ADC_PGA_X128,ADC_REF_REFEXT1,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BURNOUT_OFF,ADC_BIT_ENABLE}},
  {.u={ADC_PGA_X1,ADC_REF_REFEXT1,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BURNOUT_OFF,ADC_BIT_ENABLE}},
  {.u={ADC_PGA_X1,ADC_REF_REFINT,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BURNOUT_OFF,ADC_BIT_DISABLE}},
  {.u={ADC_PGA_X1,ADC_REF_REFINT,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BURNOUT_OFF,ADC_BIT_DISABLE}},
  {.u={ADC_PGA_X1,ADC_REF_REFINT,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BURNOUT_OFF,ADC_BIT_DISABLE}},
  {.u={ADC_PGA_X1,ADC_REF_REFINT,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BURNOUT_OFF,ADC_BIT_DISABLE}},
  {.u={ADC_PGA_X1,ADC_REF_REFINT,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BURNOUT_OFF,ADC_BIT_DISABLE}},
  {.u={ADC_PGA_X1,ADC_REF_REFINT,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BURNOUT_OFF,ADC_BIT_DISABLE}},
};


static ad7124_filter_t filters[] = {
  {.u={59,ADC_BIT_DISABLE,0,ADC_BIT_DISABLE,ADC_FILER_SINC4}}, // 40 SPS
  {.u={24,ADC_BIT_DISABLE,0,ADC_BIT_DISABLE,ADC_FILER_SINC4}},
  {.u={24,ADC_BIT_DISABLE,0,ADC_BIT_DISABLE,ADC_FILER_SINC4}},
  {.u={24,ADC_BIT_DISABLE,0,ADC_BIT_DISABLE,ADC_FILER_SINC4}},
  {.u={24,ADC_BIT_DISABLE,0,ADC_BIT_DISABLE,ADC_FILER_SINC4}},
  {.u={24,ADC_BIT_DISABLE,0,ADC_BIT_DISABLE,ADC_FILER_SINC4}},
  {.u={24,ADC_BIT_DISABLE,0,ADC_BIT_DISABLE,ADC_FILER_SINC4}},
  {.u={24,ADC_BIT_DISABLE,0,ADC_BIT_DISABLE,ADC_FILER_SINC4}},
};  
current_drive_t ad7124_iDrv[] = {
  {0,ADC_IOUT_500UA},
  {11,ADC_IOUT_OFF},
};


ad7124_config_t ad7124cfg = {
  AD7124_CHANNEL_SCAN,
//  &channels[0],
//  setups,
//  filters,
//  ad7124_iDrv,
//  0x0,
//  0x0
};

//ad7124_config_t ad7124cfg;

ad7124_dev_t ad7124 = {
  .chip_id = 0x12,
  .results = adResult,
  .sample = {0,8,2,2,2,2},
  .goStop = false,
  .state = AD7124_STOP,
  .currentChannel = 0,
  .control = {
    .u={
      .ref_en = ADC_BIT_ENABLE,
      .csb_en = ADC_BIT_DISABLE,
      .data_status = ADC_BIT_ENABLE,
      .cont_read = ADC_BIT_DISABLE,
      .drdy_del = ADC_BIT_ENABLE,
      .clk_sel = ADC_CLKSEL_CLK_DIS,
      .mode = AD7124_MODE_CONTINUE,
      .power_mode = AD7124_FULL_POWER
    }
  },
  ad7124_conversion_done,
  &ad7124cfg,
  ad7124_select,
  ad7124_intctrl,
  ad7124_spi_read,
  ad7124_spi_write,
  ad7124_delay
};


int8_t ad7124cmd_init(void)
{
  memcpy((uint8_t*)ad7124.config->channels,(uint8_t*)channels,sizeof(ad7124_channel_t)*16);
  memcpy((uint8_t*)ad7124.config->setups,(uint8_t*)setups,sizeof(ad7124_setup_t)*8);
  memcpy((uint8_t*)ad7124.config->filters,(uint8_t*)filters,sizeof(ad7124_filter_t)*8);
  memcpy((uint8_t*)ad7124.config->iDrv,(uint8_t*)ad7124_iDrv,sizeof(current_drive_t)*2);
  ad7124.config->psw = 0;
  ad7124.config->vBiasMask = 0x0;
  
  nrf_drv_spi_config_t spiad7124_config = NRF_DRV_SPI_DEFAULT_CONFIG;
  spiad7124_config.ss_pin = NRF_DRV_SPI_PIN_NOT_USED;
  spiad7124_config.miso_pin = AD7124_MISO_PIN;
  spiad7124_config.mosi_pin = AD7124_MOSI_PIN;
  spiad7124_config.sck_pin = AD7124_SCK_PIN;
  spiad7124_config.frequency = NRF_DRV_SPI_FREQ_1M;
  spiad7124_config.mode = NRF_DRV_SPI_MODE_3;
  APP_ERROR_CHECK(nrf_drv_spi_init(&ad7124_spi, &spiad7124_config, NULL, NULL));
  
  // gpio for spi
  nrf_gpio_cfg_output(AD7124_CS_PIN);
  nrf_gpio_pin_set(AD7124_CS_PIN);
  
  //  // interrupt config
  nrf_drv_gpiote_in_config_t in_cfg_ad7124 = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
  in_cfg_ad7124.pull = NRF_GPIO_PIN_PULLUP;
  APP_ERROR_CHECK(nrf_drv_gpiote_in_init(AD7124_MISO_PIN,&in_cfg_ad7124,ad7124_int_isr));
  
  ad7124_init(&ad7124);
  
}

int8_t ad7124cmd_conversion()
{
  for(uint8_t i=0;i<8;i++){
    ad7124.config->channels[i].u.enable = ADC_BIT_DISABLE;
  }
  appParam.actChannel = 0;
  ad7124.config->channels[appParam.actChannel].u.enable = ADC_BIT_ENABLE;
  ad7124.config->iDrv[0].iout = ADC_IOUT_500UA;
  ad7124_setmode(AD7124_MODE_CONTINUE,&ad7124);  
}

int8_t ad7124cmd_goSleep()
{
      ad7124.config->iDrv[0].iout = ADC_IOUT_OFF;
    ad7124_setmode(AD7124_MODE_POWER_DOWN,&ad7124);

}


int8_t ad7124_spi_read(uint8_t reg_adr, uint8_t *b, uint16_t n)
{
  ret_code_t err_code;
  uint8_t rx[16];
  err_code = nrf_drv_spi_transfer(&ad7124_spi, &reg_adr, 
                    1, rx, n+1);
  memcpy(b,&rx[1],n);
  return AD7124_OK;
}

int8_t ad7124_spi_write(uint8_t reg_adr, uint8_t *b, uint16_t n)
{
  ret_code_t err_code;
  uint8_t tx[16];
  //nrf_gpio_pin_clear(BMI160_CS_PIN);
  tx[0] = reg_adr;
  memcpy(&tx[1],b,n);
  err_code = nrf_drv_spi_transfer(&ad7124_spi, tx, 
                    n+1, NULL, 0);
  return AD7124_OK;
}

int8_t ad7124_select(uint8_t st)
{
  int8_t ret = AD7124_OK;
  if(st)
    nrf_gpio_pin_set(AD7124_CS_PIN);
  else
    nrf_gpio_pin_clear(AD7124_CS_PIN);
  return ret;
}

int8_t ad7124_selected()
{
  return nrf_gpio_pin_out_read(AD7124_CS_PIN);
}

int8_t ad7124_intctrl(uint8_t st)
{
  int8_t ret = AD7124_OK;
  if(st){
    // enable interrupt
    nrf_drv_gpiote_in_event_enable(AD7124_MISO_PIN, true);
  }
  else{
    nrf_drv_gpiote_in_event_disable(AD7124_MISO_PIN);
  }
  return ret;
}

void ad7124_delay(uint32_t period)
{
    nrf_delay_ms(period);
}

int8_t ad7124_conversion_done(void *p)
{
//  ad7124.chipSel(1);
//  ad7124.config->channels[appParam.actChannel].u.enable = ADC_BIT_DISABLE;
//  if(appParam.actChannel == 0){
//    appParam.actChannel = 1;
//  }
//  else{
//    double tt;
//    appParam.actChannel = 0;
//    // read latest raw data on each adc interrupt, these data was saved when gyro drdy
//    raw[0] = ((uint32_t)(adResult[0]) >> 8) - 8388608;
//    raw[1] = ((uint32_t)(adResult[1]) >> 8) - 8388608;
//    adc_raw_history[adc_raw_index].v1 = ((uint32_t)(adResult[0]) >> 8) - 8388608;
//    adc_raw_history[adc_raw_index].v2 = ((uint32_t)(adResult[1]) >> 8) - 8388608;
//    adc_raw_index++;
//    if(adc_raw_index == NOF_ADC_HISTORY)
//      adc_raw_index = 0;
//
//    tt = calTorque(raw[0],raw[1]);
//    appParam.staticTorque = tt;
//    if(appParam.torque.defined){
//      if(tt > 0){ // positive
//        if(appParam.torque.isPositive){ // stay positive
//          if(appParam.torque.valid){
//            appParam.torque.cycleTorq+=tt;
//            appParam.torque.cycleCount++;
//          }
//        }
//        else{ // from negative to positive
//          if(appParam.torque.valid){ // count cycle
//            if(appParam.torque.cycleCount){
//              appParam.torque.avgTorq[appParam.torque.index] = 
//                appParam.torque.cycleTorq/appParam.torque.cycleCount;
//              appParam.torque.cycleCount = 0;
//              appParam.torque.cycleTorq = 0;
//              if(appParam.torque.avgTorq[appParam.torque.index] < 1.0)
//                appParam.torque.avgTorq[appParam.torque.index] = 0;
////              if(appParam.rpm == 0)
////                appParam.torque.avgTorq[appParam.torque.index] = 0;
//              appParam.torque.index++;
//              if(appParam.torque.index == NOF_TORQUE_HISTORY)
//                appParam.torque.index = 0;
//            }
//          }
//          appParam.torque.valid = true;
//          appParam.torque.isPositive = true;
//          appParam.torque.cycleTorq+=tt;
//          appParam.torque.cycleCount++;
//        }
//      }
//      else{ // to negative
//        if(appParam.torque.valid){
//          appParam.torque.cycleTorq+=tt;
//          appParam.torque.cycleCount++;
//        }
//        appParam.torque.isPositive = false;
//      }
//    }else{
//      appParam.torque.isPositive = (tt >=0);
//      appParam.torque.defined = true;
//    }
//    
//  }
//  ad7124.config->channels[appParam.actChannel].u.enable = ADC_BIT_ENABLE;
//  ad7124_config_channel(&ad7124);
//  if(appParam.state != SYS_GO_SLEEP){ // if not issue go sleep, start again
//    ad7124.chipSel(0);
//    return AD7124_OK;
//  }
//  else{
//    ad7124_stop(&ad7124);
//    return AD7124_STOP_OP;
//  }
}

void ad7124_int_isr(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
  ad7124_int_handler(&ad7124);
}