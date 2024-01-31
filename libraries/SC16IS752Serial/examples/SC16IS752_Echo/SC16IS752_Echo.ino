/* 
 * SC16IS752 2 channel serial UART test
 * HW : PMS04, Based on ESP32
 * Author : DIGNSYS Inc.
 */
#include <SC16IS752Serial.h>

#define PIN_I2C_SDA       23
#define PIN_I2C_SCL       22

SC16IS752Serial serial0 = SC16IS752Serial(0);
SC16IS752Serial serial1 = SC16IS752Serial(1);

// Run this once at power on or reset.
void setup(void)
{
  // Arduino USART setup.
  Serial.begin(115200);

  // Display USART header on Arduino.
  Serial.println();
  Serial.println("Maxim Integrated MAX4830 testing.");
  Serial.println("(C) 2023 HB Ahn.");

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
  Wire.setClock(400000);  // 400kHz

  // SC16IS752 UART setup.
  serial0.begin(9600);
  serial0.begin(9600);  
}

// Process this loop whenever we see a serial event or interrupt from the MAX14830
void loop(void)
{
  // Display USART header on MAX3100.
  serial0.println("SC16IS752 Serial channel 0 testing.");
  serial0.println("(C) 2023 DIGNSYS Inc.");  

  // Display USART header on MAX3100.
  serial1.println("SC16IS752 Serial channel 1 testing.");
  serial1.println("(C) 2023 DIGNSYS Inc.");  

  sleep(1000);
}
