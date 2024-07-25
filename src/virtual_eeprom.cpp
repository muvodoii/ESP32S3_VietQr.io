#include "virtual_eeprom.h"

void eeprom_init_memory(void) {
    if (EEPROM.begin(STORAGE_SIZE)) {
        Serial.print("EEPROM initialized, length = ");
        Serial.println(EEPROM.length());
    } else {
        Serial.println("EEPROM init failure!");
    }
}

size_t eeprom_write_wifi(const char *wf_ssid, const char *wf_password) {
  size_t res;
  char foo_buffer[SSID_MAX_LENGTH + PWD_MAX_LENGTH];
  memset(foo_buffer, 0, sizeof(foo_buffer));
  sprintf(foo_buffer, "%s %s", wf_ssid, wf_password);
  res = EEPROM.writeString(EEPROM_BASE_ADDR, foo_buffer);
  EEPROM.commit();
  return res;
}

size_t eeprom_write_wifi(const char *combine_ssid_pwd) {
  size_t res;
  res = EEPROM.writeString(EEPROM_BASE_ADDR, combine_ssid_pwd);
  EEPROM.commit();
  return res;
}

size_t eeprom_read_wifi(char *ssid_buf, uint8_t ssid_len, char *pwd_buf, uint8_t pwd_len) {
  size_t res = 0;
  char foo_buffer[SSID_MAX_LENGTH + PWD_MAX_LENGTH];
  memset(ssid_buf, 0, ssid_len);
  memset(pwd_buf, 0, pwd_len);
  res = EEPROM.readString(EEPROM_BASE_ADDR, foo_buffer, sizeof(foo_buffer));
  if (!res) {
    return 0;
  } else {
    sscanf(foo_buffer, "%s %s", ssid_buf, pwd_buf);
    return res;
  }
} 

void eeprom_erase_memory(void) {
    int i;
    for (i = 0; i < STORAGE_SIZE; i++)
        EEPROM.write(i , 0x00);
    EEPROM.commit();
}

void eeprom_erase_memory(uint32_t address, uint32_t length) {
  int i;
  for (i = address; i < (address + length); i++)
        EEPROM.write(i , 0x00);
  EEPROM.commit();
}

size_t eeprom_write_data(uint8_t address, uint8_t *data, uint8_t length) {
  size_t res;
  res = EEPROM.writeBytes(address, data, length);
  EEPROM.commit();
  return res;
}
