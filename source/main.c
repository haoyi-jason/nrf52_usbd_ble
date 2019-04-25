/**
 * This software is subject to the ANT+ Shared Source License
 * www.thisisant.com/swlicenses
 * Copyright (c) Dynastream Innovations, Inc. 2012
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or
 * without modification, are permitted provided that the following
 * conditions are met:
 * 1) Redistributions of source code must retain the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer.
 * 
 * 2) Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials
 *    provided with the distribution.
 * 
 * 3) Neither the name of Dynastream nor the names of its
 *    contributors may be used to endorse or promote products
 *    derived from this software without specific prior
 *    written permission.
 * 
 * The following actions are prohibited:
 * 1) Redistribution of source code containing the ANT+ Network
 *    Key. The ANT+ Network Key is available to ANT+ Adopters.
 *    Please refer to http://thisisant.com to become an ANT+
 *    Adopter and access the key.
 * 
 * 2) Reverse engineering, decompilation, and/or disassembly of
 *    software provided in binary form under this license.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE HEREBY
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; DAMAGE TO ANY DEVICE, LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE. SOME STATES DO NOT ALLOW
 * THE EXCLUSION OF INCIDENTAL OR CONSEQUENTIAL DAMAGES, SO THE
 * ABOVE LIMITATIONS MAY NOT APPLY TO YOU.
 * 
 */

/**@file
 * @defgroup ant_bpwr_sensor_main ANT Bicycle Power sensor example
 * @{
 * @ingroup nrf_ant_bicycle_power
 *
 * @brief Example of ANT Bicycle Power profile display.
 *
 * Before compiling this example for NRF52, complete the following steps:
 * - Download the S212 SoftDevice from <a href="https://www.thisisant.com/developer/components/nrf52832" target="_blank">thisisant.com</a>.
 * - Extract the downloaded zip file and copy the S212 SoftDevice headers to <tt>\<InstallFolder\>/components/softdevice/s212/headers</tt>.
 * If you are using Keil packs, copy the files into a @c headers folder in your example folder.
 * - Make sure that @ref ANT_LICENSE_KEY in @c nrf_sdm.h is uncommented.
 */

//#define ANT_LICENSE_KEY "3831-521d-7df9-24d8-eff3-467b-225f-a00e" // This is an EVALUATION license key - DO NOT USE FOR COMMERCIAL PURPOSES


#include <stdio.h>
#include <string.h>
#include "nrf.h"
#include "nrf_soc.h"
#include "bsp.h"
#include "hardfault.h"
//#include "app_error.h"
#include "nordic_common.h"
#include "app_timer.h"
#include "nrf_delay.h"
#include "math.h"
#include "nrf_section.h"
#include "nrf_fstorage_sd.h"
#include "fds.h"
//#include "nrf_drv_gpiote.h"
//#include "app_gpiote.h"
#include "nrf_pwr_mgmt.h"
//#i//nclude "ant_boot_settings_api.h"

//#include "ad7124_cmd.h"
//#include "ad7124_config.h"
//#include "bmi160.h"
//#include "bmi160_config.h"
//#include "bmi160_cmd.h"
#include "peripheral_if.h"

//#include "ant_ota.h"
//#include "ant_stack_config.h"
//#include "ant_bpwr.h"
//#include "ant_state_indicator.h"
//#include "ant_key_manager.h"
//#include "ant_bpwr_simulator.h"
#include "app_ble_uart.h"

#define PARM_FILE_ID    0x5329
#define PARM_REC_KEY    0x9235




//#define NRF_LOG_MODULE_NAME APP17003
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_sdh.h"
   
#include "usbd_ble_uart_peripheral.h"

#define MODIFICATION_TYPE_BUTTON 0 /* predefined value, MUST REMAIN UNCHANGED */
#define MODIFICATION_TYPE_AUTO   1 /* predefined value, MUST REMAIN UNCHANGED */

#if (MODIFICATION_TYPE != MODIFICATION_TYPE_BUTTON) \
    && (MODIFICATION_TYPE != MODIFICATION_TYPE_AUTO)
    #error Unsupported value of MODIFICATION_TYPE.
#endif

#ifndef SENSOR_TYPE // can be provided as preprocesor global symbol
    #define SENSOR_TYPE (TORQUE_NONE)
#endif

#define APP_TIMER_PRESCALER         0 /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE     0x04 /**< Size of timer operation queues. */
#define BPWR_CHANNEL_NUMBER         0x00 /**< Channel number assigned to Bicycle Power profile. */
#define ANTPLUS_NETWORK_NUMBER      0       /**< Network number. */
#define CALIBRATION_DATA            0x55AAu /**< General calibration data value. */

#define POLL_INTERVAL   1000      // ms
#define POLL_TICKS      (POLL_INTERVAL*2048)/1000
// ms to tick, measure interval of 100 ms
#define MEASURE_INTERVAL		APP_TIMER_TICKS(POLL_INTERVAL, APP_TIMER_PRESCALER)  /**< measurement interval (ticks). */

bool appTimerStopped =false;

#pragma location = 0x6F000    // last page before bootloader
//__root storedParam_t storedParam;
sysParam_t sysParam;
      
/** @snippet [ANT BPWR TX Instance] */
//void ant_bpwr_evt_handler(ant_bpwr_profile_t * p_profile, ant_bpwr_evt_t event);
//void ant_bpwr_calib_handler(ant_bpwr_profile_t * p_profile, ant_bpwr_page1_data_t * p_page1);

static uint8_t testRPM = 0;
static uint8_t testTorque = 0;


static uint16_t timeout_secs = 0;

//app_gpiote_user_id_t m_app_gpiote_my_id;
//app_gpiote_user_id_t  id_ad7124;
//app_gpiote_user_id_t  id_adxl335;

//BPWR_SENS_CHANNEL_CONFIG_DEF(m_ant_bpwr,
//                             BPWR_CHANNEL_NUMBER,
//                             CHAN_ID_TRANS_TYPE,
//                             CHAN_ID_DEV_NUM,
//                             ANTPLUS_NETWORK_NUMBER);
//BPWR_SENS_PROFILE_CONFIG_DEF(m_ant_bpwr,
//                            (ant_bpwr_torque_t)(SENSOR_TYPE),
//                            ant_bpwr_calib_handler,
//                            ant_bpwr_evt_handler);

//ant_bpwr_profile_t m_ant_bpwr;
/** @snippet [ANT BPWR TX Instance] */

//ant_bpwr_simulator_t m_ant_bpwr_simulator;    /**< Simulator used to simulate profile data. */


APP_TIMER_DEF(m_measure_timer_id);

uint16_t torque_temp;

bool m_is_ready = false;
bool m_sysoff_started;
bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event);
//NRF_PWR_MGMT_REGISTER_HANDLER(m_app_shutdown_handler) = app_shutdown_handler;

bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
  uint32_t err_code;

//  NRF_LOG_INFO("APP_SHUTDOWN Handler\n");
//
//  switch (event)
//  {
//    case NRF_PWR_MGMT_EVT_PREPARE_SYSOFF:
//      NRF_LOG_INFO("NRF_PWR_MGMT_EVT_PREPARE_SYSOFF\n");
////      if(sysGetState() != SYS_SLEEP){
////        goSleep();
////      }
//      break;
//
//    case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP:
//      NRF_LOG_INFO("NRF_PWR_MGMT_EVT_PREPARE_WAKEUP\n");
//      UNUSED_VARIABLE(err_code);
////      if(sysGetState() == SYS_GO_WKUP){
////        peripheral_start();
////      }
//      break;
//
//    case NRF_PWR_MGMT_EVT_PREPARE_DFU:
//      NRF_LOG_INFO("NRF_PWR_MGMT_EVT_PREPARE_WAKEUP\n");
//      NRF_LOG_ERROR("Entering DFU is not supported by this example.\r\n");
//      APP_ERROR_HANDLER(NRF_ERROR_API_NOT_IMPLEMENTED);
//      break;
//  }

  return true;
}


/**@brief Function for dispatching an ANT stack event to all modules with an ANT stack event handler.
 *
 * @details This function is called from the ANT Stack event interrupt handler after an ANT stack
 *          event has been received.
 *
 * @param[in] p_ant_evt  ANT stack event.
 *
 * @snippet [ANT BPWR TX Profile handling] */
//void ant_evt_dispatch(ant_evt_t * p_ant_evt)
//{
//  //ant_bpwr_sens_evt_handler(&m_ant_bpwr, p_ant_evt);
//  //ant_state_indicator_evt_handler(p_ant_evt);
//  ota_channel_event_handle(p_ant_evt);
//}
/** @snippet [ANT BPWR TX Profile handling] */


/**@brief Function for handling bsp events.
 */
/** @snippet [ANT BPWR simulator button] */
//void bsp_evt_handler(bsp_event_t event)
//{
//  switch (event)
//  {
//    case BSP_EVENT_KEY_0:
////			ant_bpwr_simulator_increment(&m_ant_bpwr_simulator);
//      break;
//
//    case BSP_EVENT_KEY_1:
////			ant_bpwr_simulator_decrement(&m_ant_bpwr_simulator);
//      break;
//
//    case BSP_EVENT_KEY_2:
//      ant_bpwr_calib_response(&m_ant_bpwr);
//      break;
//    default:
//      break;
//  }
//}
/** @snippet [ANT BPWR simulator button] */

/**@brief Function for handling ANT BPWR events.
 */
/** @snippet [ANT BPWR simulator call] */
uint32_t sndCntr = 0;
//void ant_bpwr_evt_handler(ant_bpwr_profile_t * p_profile, ant_bpwr_evt_t event)
//{
//  uint32_t test = 0;
//  int16_t retv;
//  int16_t polls;
//  sndCntr++;
//  switch (event)
//  {
//    case ANT_BPWR_PAGE_1_UPDATED:
//      //test = 1;
//      m_ant_bpwr.page_1.data.general_calib = 99;
//      break;
//            /* fall through */
//    case ANT_BPWR_PAGE_16_UPDATED:
//      m_ant_bpwr.page_16.update_event_count++;
//      //m_ant_bpwr.page_18.update_event_count = m_ant_bpwr.page_16.update_event_count;
//      
//      //m_ant_bpwr.page_18.period += getPeriod();
//      m_ant_bpwr.common.instantaneous_cadence = getRPM();
//      m_ant_bpwr.page_16.instantaneous_power = getPower();
////      m_ant_bpwr.page_18.accumulated_torque += (uint16_t)getTorque();
//      break;
//            /* fall through */
//    case ANT_BPWR_PAGE_17_UPDATED:
//      test = 3;
//      break;
//            /* fall through */
//    case ANT_BPWR_PAGE_18_UPDATED:
//      m_ant_bpwr.page_18.update_event_count++;
//      m_ant_bpwr.page_18.period += getPeriod();
//      m_ant_bpwr.page_18.accumulated_torque += (uint16_t)getTorque();
//      // for test
////      m_ant_bpwr.page_16.instantaneous_power = (uint16_t)lround(torque*testRPM*1000./9549);
//      //m_ant_bpwr.page_16.instantaneous_power = (uint8_t)lround(testTorque*testRPM*1000./9549.);
//
//      break;
//            /* fall through */
//    case ANT_BPWR_PAGE_80_UPDATED:
//      test = 5;
//      break;
//            /* fall through */
//    case ANT_BPWR_PAGE_81_UPDATED:
//      test = 6;
//      break;
//    default:
//      ant_bpwr_simulator_one_iteration(&m_ant_bpwr_simulator, event);
//  }
//   
//}
/** @snippet [ANT BPWR simulator call] */


/**@brief Function for handling ANT BPWR events.
 */
/** @snippet [ANT BPWR calibration] */
//void ant_bpwr_calib_handler(ant_bpwr_profile_t * p_profile, ant_bpwr_page1_data_t * p_page1)
//{
//  switch (p_page1->calibration_id)
//  {
//    case ANT_BPWR_CALIB_ID_MANUAL:
//      m_ant_bpwr.BPWR_PROFILE_calibration_id     = ANT_BPWR_CALIB_ID_MANUAL_SUCCESS;
//      m_ant_bpwr.BPWR_PROFILE_general_calib_data = CALIBRATION_DATA;
//      sysParam.zero_x = getAdcRaw(0);
//      sysParam.zero_y = getAdcRaw(1);
//      //setAdcOffset(sysParam.zero_x, sysParam.zero_y);
//      
//      //fdsWriteParam();
//      //ota_saveparam();
//      writeOTAActivity(OTA_SAVEPARAM);
//      ant_bpwr_calib_response(p_profile);
//      break;
//
//    case ANT_BPWR_CALIB_ID_AUTO:
//      m_ant_bpwr.BPWR_PROFILE_calibration_id     = ANT_BPWR_CALIB_ID_MANUAL_SUCCESS;
//      m_ant_bpwr.BPWR_PROFILE_auto_zero_status   = p_page1->auto_zero_status;
//      m_ant_bpwr.BPWR_PROFILE_general_calib_data = CALIBRATION_DATA;
//      
//      break;
//    case ANT_BPWR_CALIB_ID_CUSTOM_REQ:
//      m_ant_bpwr.BPWR_PROFILE_calibration_id = ANT_BPWR_CALIB_ID_CUSTOM_REQ_SUCCESS;
//      memcpy(m_ant_bpwr.BPWR_PROFILE_custom_calib_data, p_page1->data.custom_calib,
//      sizeof (m_ant_bpwr.BPWR_PROFILE_custom_calib_data));
//      break;
//    case ANT_BPWR_CALIB_ID_CUSTOM_UPDATE:
//      m_ant_bpwr.BPWR_PROFILE_calibration_id = ANT_BPWR_CALIB_ID_CUSTOM_UPDATE_SUCCESS;
//      memcpy(m_ant_bpwr.BPWR_PROFILE_custom_calib_data, p_page1->data.custom_calib,
//      sizeof (m_ant_bpwr.BPWR_PROFILE_custom_calib_data));
//      break;
//    default:
//      break;
//  }
//}
/** @snippet [ANT BPWR calibration] */


uint32_t		m_flag_measure = 0;

static void measure_timeout_handler(void * p_context)
{
  UNUSED_PARAMETER(p_context);
  uint8_t u8;
  m_flag_measure = 1;
  sysstate_t state = sysGetState();
  switch(state){
  case SYS_IDLE:
  case SYS_NORMAL:
    break;
  case SYS_GO_SLEEP:
    goSleep();
    NRF_LOG_INFO("Go Sleep\r\n");
    break;
  case SYS_SLEEP:
    // shut down system, wakeup by gpiote
    
    if(!appTimerStopped){
      app_timer_stop_all();
      appTimerStopped = true;
    }
    break;
  case SYS_GO_WKUP:
    NRF_LOG_INFO("Go Wakeup\r\n");
    peripheral_start();

    break;
    
  }
  NRF_LOG_FLUSH();

}
/**
 * @brief Function for setup all thinks not directly associated with ANT stack/protocol.
 * @desc Initialization of: @n
 *         - app_tarce for debug.
 *         - app_timer, pre-setup for bsp.
 *         - bsp for signaling LEDs and user buttons.
 */
static void utils_setup(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    // Initialize and start a single continuous mode timer, which is used to update the event time
    // on the main data page.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

//    err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
//                        bsp_evt_handler);
//    APP_ERROR_CHECK(err_code);
//
//    err_code = ant_state_indicator_init(m_ant_bpwr.channel_number, BPWR_SENS_CHANNEL_TYPE);
//    APP_ERROR_CHECK(err_code);

    //  uint32_t err_code;
//
//  // Initialize and start a single continuous mode timer, which is used to update the event time
//  // on the main data page.
//  APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, NULL);
//
//
//  // Create timers.
////  err_code = app_timer_create(&m_measure_timer_id,
////                          APP_TIMER_MODE_REPEATED,
////                          measure_timeout_handler);
////  APP_ERROR_CHECK(err_code);
////  err_code = app_timer_start(m_measure_timer_id, MEASURE_INTERVAL, NULL);
////  APP_ERROR_CHECK(err_code);
////
//  err_code = NRF_LOG_INIT(NULL);
//  APP_ERROR_CHECK(err_code);

//  err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
//                  APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
//                  bsp_evt_handler);
//  APP_ERROR_CHECK(err_code);
//
//  ret_code_t ret_code = nrf_pwr_mgmt_init(APP_TIMER_TICKS(1000,APP_TIMER_PRESCALER));
//  APP_ERROR_CHECK(ret_code);
}


/**@brief Function for the BPWR simulator initialization.
 */
//void simulator_setup(void)
//{
//	/** @snippet [ANT BPWR simulator init] */
//	const ant_bpwr_simulator_cfg_t simulator_cfg =
//	{
//		.p_profile   = &m_ant_bpwr,
//		.sensor_type = (ant_bpwr_torque_t)(SENSOR_TYPE),
//	};
//
//	/** @snippet [ANT BPWR simulator init] */
//
//#if MODIFICATION_TYPE == MODIFICATION_TYPE_AUTO
//	/** @snippet [ANT BPWR simulator auto init] */
//	ant_bpwr_simulator_init(&m_ant_bpwr_simulator, &simulator_cfg, true);
//	/** @snippet [ANT BPWR simulator auto init] */
//#else
//	/** @snippet [ANT BPWR simulator button init] */
//	ant_bpwr_simulator_init(&m_ant_bpwr_simulator, &simulator_cfg, false);
//	/** @snippet [ANT BPWR simulator button init] */
//#endif
//}

static void sys_evt_dispatch(uint32_t sys_evt)
{
//  fs_sys_event_handler(sys_evt);
  ant_boot_settings_event(sys_evt);
}

static uint32_t sd_evt_handler(void)
{
  uint32_t ulEvent;
  while(sd_evt_get(&ulEvent) != NRF_ERROR_NOT_FOUND){
    ant_boot_settings_event(ulEvent);
  }
  
  return 0;
}

/**
 * @brief Function for ANT stack initialization.
 *
 * @details Initializes the SoftDevice and the ANT event interrupt.
 */
static void softdevice_setup(void)
{
//  uint32_t err_code;
//
//  nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
//
//  err_code = softdevice_ant_evt_handler_set(ant_evt_dispatch);
//  APP_ERROR_CHECK(err_code);
//
//  err_code = softdevice_handler_init(&clock_lf_cfg, NULL, 0, NULL);
//
//  APP_ERROR_CHECK(err_code);
//
//  err_code = ant_stack_static_config(); // set ant resource
//  APP_ERROR_CHECK(err_code);
//
//  err_code = ant_plus_key_set(ANTPLUS_NETWORK_NUMBER);
//  APP_ERROR_CHECK(err_code);
//
//  err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
//  APP_ERROR_CHECK(err_code);

//    ret_code_t err_code = nrf_sdh_enable_request();
//    APP_ERROR_CHECK(err_code);
//
//    ASSERT(nrf_sdh_is_enabled());
//
//    err_code = nrf_sdh_ant_enable();
//    APP_ERROR_CHECK(err_code);
//
//    err_code = ant_plus_key_set(ANTPLUS_NETWORK_NUM);
//    APP_ERROR_CHECK(err_code);  
}


/**
 * @brief Function for Bicycle Power profile initialization.
 *
 * @details Initializes the Bicycle Power profile and open ANT channel.
 */
//static void profile_setup(void)
//{
///** @snippet [ANT BPWR TX Profile Setup] */
//  uint32_t err_code;
//
//  err_code = ant_bpwr_sens_init(&m_ant_bpwr,
//                          BPWR_SENS_CHANNEL_CONFIG(m_ant_bpwr),
//                          BPWR_SENS_PROFILE_CONFIG(m_ant_bpwr));
//  APP_ERROR_CHECK(err_code);
//
//  // fill manufacturer's common data page.
//  m_ant_bpwr.page_80 = ANT_COMMON_page80(BPWR_HW_REVISION,
//                          BPWR_MANUFACTURER_ID,
//                          BPWR_MODEL_NUMBER);
//  // fill product's common data page.
//  m_ant_bpwr.page_81 = ANT_COMMON_page81(BPWR_SW_REVISION_MAJOR,
//                          BPWR_SW_REVISION_MINOR,
//                          BPWR_SERIAL_NUMBER);
//
//  m_ant_bpwr.BPWR_PROFILE_auto_zero_status = ANT_BPWR_AUTO_ZERO_OFF;
//
//  ant_channel_tx_broadcast_setup();
//  
//  
//  err_code = ant_bpwr_sens_open(&m_ant_bpwr);
//  APP_ERROR_CHECK(err_code);
//
//  //err_code = ant_state_indicator_channel_opened();
//  //APP_ERROR_CHECK(err_code);
///** @snippet [ANT BPWR TX Profile Setup] */
//}


//==============================================================

//static void fs_evt_handler(fs_evt_t const * const evt, fs_ret_t result)
//{
//    if (result != FS_SUCCESS)
//    {
//        // An error occurred.
//    }
//}


static volatile uint8_t writeflag = 0;
volatile uint8_t fdReady = 0;

static void fds_evt_handler(fds_evt_t const * const p_fds_evt)
{
  switch(p_fds_evt->id){
  case FDS_EVT_INIT:
    if(p_fds_evt->result == FDS_SUCCESS)
      fdReady = 1;
    else
      fdReady = 0;
    break;
  case FDS_EVT_WRITE:
    writeflag = 1;
    break;
  case FDS_EVT_UPDATE:
    if(p_fds_evt->result == FDS_SUCCESS)
    {
      fdReady = 1;
    }
    break;
  default:
    break;
  }
}


/**@brief Function to execute while waiting for the wait burst busy flag
*/
static void event_burst_wait_handle(void)
{
    // No implementation needed
}

static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


#define USE_SD   1
uint32_t loopCount = 0;
uint32_t hclk_run;
//static EventQueue eventQueue(16*32);
#define FPU_EXCEPTION_MASK 0x0000009F 
/**@brief Function for application main entry, does not return.
 */
int main(void)
{
  

  //utils_setup();
//  NRF_LOG_INFO("Wakeup from sleep 1\r\n");
//  NRF_LOG_ERROR("Wakeup from sleep 2\r\n");
//  NRF_LOG_DEBUG("Wakeup from sleep 3\r\n");
//  NRF_LOG_ERROR("Wakeup from sleep 4\r\n");
//  NRF_LOG_FLUSH();
  //peripheral_init();
  app_ble_nus_init();
  //usbd_ble_uart_init();

//  ad7124cmd_init();
//  bmi160_cmd_init();
  
  //peripheral_start();
  
 // app_ble_nus_init();

//  while(1){
//    //getTorque();
//  }
//#if USE_SD
//  softdevice_setup();  
//  profile_setup();
//  m_ant_bpwr.common.instantaneous_cadence = 0xff;
//#endif
  
    for (;;)
    {
        if (NRF_LOG_PROCESS() == false)
        {
            nrf_pwr_mgmt_run();
        }
    } 
  
  
//  for (;; )
//  {
//    loopCount++;
//    uint8_t state = sysGetState();
//    //sd_clock_hfclk_is_running(&hclk_run);
//    switch(state){
//    case SYS_GO_WKUP:
//      NRF_LOG_INFO("Wakeup from sleep\r\n");
//      peripheral_start();
//      if(appTimerStopped){
//        app_timer_start(m_measure_timer_id, MEASURE_INTERVAL, NULL);
//        appTimerStopped = false;
//      }
//      break;
//    case SYS_GO_SLEEP:
//      NRF_LOG_INFO("Enter sleep\r\n");
//      goSleep();
//#if USE_SD
//      sd_power_dcdc_mode_set(NRF_POWER_DCDC_DISABLE);
//      sd_power_system_off();
//#endif
//      break;
//    case SYS_SLEEP:
//#if USE_SD
//      sd_app_evt_wait();
//#endif
//      break;
//    default:
//      if(NRF_LOG_PROCESS() == false){
//        __set_FPSCR(__get_FPSCR()  & ~(FPU_EXCEPTION_MASK));      
//        (void) __get_FPSCR();
//        NVIC_ClearPendingIRQ(FPU_IRQn);   
//#if USE_SD
//        //sd_nvic_ClearPendingIRQ(FPU_IRQn);
//        sd_app_evt_wait();
//#endif
//        switch(readOTAActivity()){
//        case OTA_BOOT:
//          NRF_LOG_INFO("Restart in bootloader\r\n");
//          restart_in_bootloader();
//          break;
//        case OTA_SAVEPARAM:
//          //NRF_LOG_INFO("Save param\r\n");
//          ota_saveparam();
//          break;
//        }
//      }
//    }
//    
//  }
}


/**
 *@}
 **/
