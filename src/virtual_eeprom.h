/*
    custom virtual EEPROM to read/write flash memory
*/

#ifndef __VIR_EEPROM_H__
#define __VIR_EEPROM_H__
#include <EEPROM.h>

#define STORAGE_SIZE      200
#define EEPROM_BASE_ADDR  0x00
#define SSID_MAX_LENGTH   0x14
#define PWD_MAX_LENGTH    0x14
#define VARIABLE1_ADDR    40
#define VARIABLE2_ADDR    70
#define VARIABLE3_ADDR    100
#define VARIABLE4_ADDR    130

size_t eeprom_write_wifi(const char *wf_ssid, const char *wf_password);
size_t eeprom_write_wifi(const char *combine_ssid_pwd);
size_t eeprom_read_wifi(char *ssid_buf, uint8_t ssid_len, char *pwd_buf, uint8_t pwd_len);
void eeprom_erase_memory(void);
void eeprom_erase_memory(uint32_t address, uint32_t length);
void eeprom_init_memory(void);
size_t eeprom_write_data(uint8_t address, uint8_t *data, uint8_t length);

#endif 
