#include "nrf_sdh_ant.h"

enum ota_activity_e{
  OTA_BOOT = 1,
  OTA_SAVEPARAM = 2
};

void ant_channel_tx_broadcast_setup(void);
void ota_channel_event_handle(ant_evt_t* p_ant_event);
bool checkOTA(void);
void restart_in_bootloader(void);
uint32_t readOTAActivity(void);
void ota_saveparam(void);
void writeOTAActivity(uint32_t v);


