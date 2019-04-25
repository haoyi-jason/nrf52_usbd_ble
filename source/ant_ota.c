#include <stdbool.h>
#include <stdint.h>
#include "app_error.h"
#include "nrf.h"
#include "ant_interface.h"
#include "ant_parameters.h"
#include "nrf_soc.h"
#include "nrf_sdm.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "ant_error.h"
//#include "ant_boot_settings_api.h"
#include <string.h>
#include "ant_stack_config.h"
#include "app_util_platform.h"
//#include "ant_stack_handler_types.h"
#include "ant_boot_settings_api.h"
#include "nrf_log.h"
#include "peripheral_if.h"
#include "boards.h"
#include "ant_ota.h"

extern sysParam_t sysParam;


// Channel configuration.
#define ANT_CHANNEL_NUMBER              0x01                 /**< ANT Channel Number*/
#define ANT_RF_FREQUENCY                0x39u                /**< Channel RF Frequency = (2400 + 50)MHz */
#define ANT_CHANNEL_PERIOD              8192u                /**< Channel period 4 Hz. */
#define ANT_EXT_ASSIGN                  0x00                 /**< ANT Ext Assign. */
#define ANT_NETWORK_KEY                {0xE8, 0xE4, 0x21, 0x3B, 0x55, 0x7A, 0x67, 0xC1}      /**< ANT public network key. */
#define ANT_NETWORK_NUMBER              0x01                 /**< Network Number */
#define ANT_TRANSMIT_POWER              0u                   /**< ANT Custom Transmit Power (Invalid/Not Used). */

// Channel ID configuration.
#define ANT_DEV_TYPE                    0x20u                /**< Device type = 32. */
#define ANT_TRANS_TYPE                  0x05u                /**< Transmission type. */
#define ANT_DEV_NUM                     (NRF_FICR->DEVICEID[0])         /**< Device number. */

// Test broadcast data
#define BROADCAST_PAYLOAD               {0x01, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xFF, 0xEE}
#define BROADCAST_DATA_BUFFER_SIZE      8

// Version string
#define VERSION_STRING              "BFM1.00B01"

// Message definitions
#define COMMAND_ID                      0x02u
#define COMMAND_RESTART_BOOTLOADER      0x01u


static const uint8_t m_version_string[] = VERSION_STRING; // Version string
static bool restart = false;
static uint32_t otaActivity = 0x0;

/**@brief Reset the device, and start bootloader
*/
void restart_in_bootloader(void)
{
    uint32_t err_code;
    static ant_boot_settings_t ant_boot_settings;

    err_code = ant_boot_settings_clear(&ant_boot_settings); // Clears and set FFs to the memory block
    APP_ERROR_CHECK(err_code);
    memcpy((void*) ant_boot_settings.app_version, m_version_string, sizeof(m_version_string));
    ant_boot_settings.app_size = 350000;                      // Estimated current application size used to try to preserve itself
    err_code = ant_boot_settings_save(&ant_boot_settings);
    APP_ERROR_CHECK(err_code);
    ant_boot_settings_validate(1);                          // Sets flag to indicate restarting in bootloader mode. Must be done last before the reset!!!
    NVIC_SystemReset();
}

void ota_saveparam(void)
{
  uint32_t err_code;
  storedParam_t param = {0xffffffff,0xffffffff,0xffffffff,0xffffffff};
  
  NRF_LOG_INFO("Save param\r\n");
  
  err_code = ant_boot_param_clear(&param);
  APP_ERROR_CHECK(err_code);
  
  param.mul_a = (int32_t)(sysParam.mul_a*1000000);
  param.mul_b = (int32_t)(sysParam.mul_b*1000000);
  param.zero_x = sysParam.zero_x;
  param.zero_y = sysParam.zero_y;
  
  err_code = ant_boot_param_save(&param);
  APP_ERROR_CHECK(err_code);
  
  NRF_LOG_INFO("Save param done!\r\n");
}

bool checkOTA(void)
{
  return restart;
}

uint32_t readOTAActivity(void)
{
  uint32_t ret = otaActivity;
  otaActivity = 0x0;
  return ret;
}

void writeOTAActivity(uint32_t v)
{
  otaActivity = v;
}

/**@brief Function for handling ANT TX channel events.
 *
 * @param[in] event The received ANT event to handle.
 * @param[in] p_ant_message The ANT message structure
 */
void ota_channel_event_handle(ant_evt_t* p_ant_event)
{
    uint8_t page_num;
    uint8_t command;
    int32_t tmp;
//    ANT_MESSAGE *p_ant_message = (ANT_MESSAGE*)p_ant_event->msg.evt_buffer;
    ANT_MESSAGE *p_ant_message = (ANT_MESSAGE*)&p_ant_event->message;
    switch (p_ant_event->event)
    {
        case EVENT_RX:
            switch (p_ant_message->ANT_MESSAGE_ucMesgID)
            {
                case MESG_BROADCAST_DATA_ID:
                case MESG_ACKNOWLEDGED_DATA_ID:
                    page_num = p_ant_message->ANT_MESSAGE_aucPayload[0];
                    command = p_ant_message->ANT_MESSAGE_aucPayload[7];
                    switch(page_num){
                    case COMMAND_ID:
                      if (page_num == COMMAND_ID && command == COMMAND_RESTART_BOOTLOADER)
                      {
                        NRF_LOG_INFO("Restart in bootloader\r\n");
                          //restart_in_bootloader();
                        restart = true;
                        otaActivity = OTA_BOOT;
                      }
                      break;
                    case 0xa1: // write mul_a
                      tmp = (p_ant_message->ANT_MESSAGE_aucPayload[1] << 24) |
                            (p_ant_message->ANT_MESSAGE_aucPayload[2] << 16) |
                            (p_ant_message->ANT_MESSAGE_aucPayload[3] << 8) |
                            (p_ant_message->ANT_MESSAGE_aucPayload[4] << 0);
                      sysParam.mul_a = (double)(tmp/1000000.);
                      break;
                    case 0xa2:
                      tmp = (p_ant_message->ANT_MESSAGE_aucPayload[1] << 24) |
                            (p_ant_message->ANT_MESSAGE_aucPayload[2] << 16) |
                            (p_ant_message->ANT_MESSAGE_aucPayload[3] << 8) |
                            (p_ant_message->ANT_MESSAGE_aucPayload[4] << 0);
                      sysParam.mul_b = (double)(tmp/1000000.);
                      break;
                    case 0xe5: // save parameter
                      otaActivity = OTA_SAVEPARAM;
                      break;
                    }
                    break;
            }
            break;
        default:
            break;
    }
}


/**@brief Function for setting up the ANT module to be ready for TX broadcast.
 */
void ant_channel_tx_broadcast_setup(void)
{
    uint32_t err_code;
    uint8_t  m_network_key[] = ANT_NETWORK_KEY;
    uint8_t m_broadcast_data[] = BROADCAST_PAYLOAD;

    // Set Network Key.
    err_code = sd_ant_network_address_set(ANT_NETWORK_NUMBER, (uint8_t*)m_network_key);
    APP_ERROR_CHECK(err_code);

    // Assign Channel
    err_code = sd_ant_channel_assign(ANT_CHANNEL_NUMBER,
                                     CHANNEL_TYPE_MASTER,
                                     ANT_NETWORK_NUMBER,
                                     ANT_EXT_ASSIGN);

    APP_ERROR_CHECK(err_code);
//
//    // Set Channel ID.
    err_code = sd_ant_channel_id_set(ANT_CHANNEL_NUMBER,
                                     ANT_DEV_NUM,
                                     ANT_DEV_TYPE,
                                     ANT_TRANS_TYPE);
    APP_ERROR_CHECK(err_code);

//    // Set Channel Period
    err_code = sd_ant_channel_period_set(ANT_CHANNEL_NUMBER, ANT_CHANNEL_PERIOD);
    APP_ERROR_CHECK(err_code);
//
//    // Set RF Frequency
    err_code = sd_ant_channel_radio_freq_set(ANT_CHANNEL_NUMBER, ANT_RF_FREQUENCY);
    APP_ERROR_CHECK(err_code);
//
//    //Set Tx Power
    err_code = sd_ant_channel_radio_tx_power_set(ANT_CHANNEL_NUMBER, RADIO_TX_POWER_LVL_3, ANT_TRANSMIT_POWER);
    APP_ERROR_CHECK(err_code);

    // Setup broadcast payload
    err_code = sd_ant_broadcast_message_tx(ANT_CHANNEL_NUMBER,
                                          BROADCAST_DATA_BUFFER_SIZE,
                                          m_broadcast_data);

    if (err_code != NRF_ANT_ERROR_CHANNEL_IN_WRONG_STATE)
    {
        APP_ERROR_CHECK(err_code);
    }
    
    //err_code = ant_fs_key_set(ANT_CHANNEL_NUMBER);
   // APP_ERROR_CHECK(err_code);
    // Open channel
    err_code = sd_ant_channel_open(ANT_CHANNEL_NUMBER);
    APP_ERROR_CHECK(err_code);
}



