/* 
 * INL-01 Main 
 * Author : DIGNSYS Inc.
 */
#include <EEPROM.h>
#define EEPROM_SIZE 512

void setup() {
  Serial.begin(115200);

  if (!EEPROM.begin(EEPROM_SIZE)) {
      Serial.println("Failed to initialise EEPROM");
      return;
  }
}

void loop() {
  uint8_t data;
  long randomValue = random();

  Serial.println("EEPROM Read/Write test");

  Serial.println("Read data from EEPROM");
  for(int i=0; i<10; i++){
    Serial.printf("Read Data[%d]: %02x\r\n", i, EEPROM.read(i));
  }

  Serial.println("Write new data to EEPROM");
  for(int i=0; i<10; i++){
    data = (uint8_t) (randomValue + i);
    Serial.printf("New Write Data[%d]: %02x\r\n", i, data);
    EEPROM.write(i, data);
  }
  EEPROM.commit();
  delay(3000);
}