/* 
 * INL-05 Main 
 * Author : DIGNSYS Inc.
 */

#include <Arduino.h>
#include <SC16IS752Serial.h>
#include <SoftWire.h>
#include <Wire.h>
#include <SPI.h>
#include <Ethernet_Generic.h>
#include <EEPROM.h>
#include <USB.h>
#include <USBHIDKeyboard.h>
#include <EspUsbHost.h>
#include <esp_log.h>

#define VERSION_INL05_FW  "20240415"

#define PIN_LED_STATUS      10  // 20
#define PIN_W5500_RST       40
#define PIN_ETH_CS          39
#define PIN_ETH_INT         38
#define PIN_ETH_MISO        37
#define PIN_ETH_SCLK        36
#define PIN_ETH_MOSI        35
#define PIN_I2C1_SDA        15
#define PIN_I2C1_SCL        16
#define PIN_I2C2_SDA        42
#define PIN_I2C2_SCL        41
#define PIN_RS232_RX        18
#define PIN_RS232_TX        17
#define PIN_RS485_RX        48
#define PIN_RS485_TX        47
#define PIN_BOOT            0
#define PIN_SW_I2C_SDA      6
#define PIN_SW_I2C_SCL      5
#define PIN_SC16IS752_RST   7
#define PIN_RELAY_OUT1      21
#define PIN_RELAY_OUT2      14
#define PIN_MAX485_DE       11  //46
#define PIN_OTG_VBUS_EN     3
#define PIN_USB_ID          12
#define PIN_VBUS_DEC        13
#define PIN_ADC_IN0         1
#define PIN_ADC_IN1         2
#define PIN_ADC_IN3         4
#define PIN_ADC_IN8         9

#define USING_TX_INV_TR
#ifdef USING_TX_INV_TR
// Using TX inverting transistor
#define MAX485_DIR_SEND     LOW
#define MAX485_DIR_RECEIVE  HIGH
#else
// Not using TX inverting transistor
#define MAX485_DIR_SEND     HIGH
#define MAX485_DIR_RECEIVE  LOW
#endif
#define OTG_VBUS_ENABLE     LOW   // Active Low
#define OTG_VBUS_DISABLE    HIGH

// Soft I2C
// 0x38 + A2,A1,A0
#define I2C_ADDR_PCF_OUT    0x39  //0x21  // 39
#define I2C_ADDR_PCF_SW     0x3b  //0x38  //0x23  // 3b
// 0x20 + A2,A1,A0
#define I2C_ADDR_MCP_IN     0x22
#define I2C_ADDR_RTC        0x68

#define I2C_ADDR_USB        0x3d
#define USB_DET_REG_ID      0x01
#define USB_DET_REG_CTRL    0x02
#define USB_DET_REG_INT     0x03
#define USB_DET_REG_STATUS  0x04

uint8_t sw_tx_buf[16];
uint8_t sw_rx_buf[16];
uint8_t str_cur_date[9] = {0,};
uint8_t str_cur_time[9] = {0,};

//#define PCF_HW_I2C_TEST

#ifdef PCF_HW_I2C_TEST
SoftWire sw(9, 10);
#else
SoftWire sw(PIN_SW_I2C_SDA, PIN_SW_I2C_SCL);
#endif

//#define USING_SW_I2C_TO_DUART

#ifdef USING_SW_I2C_TO_DUART
uint8_t sw_duart_tx_buf[16];
uint8_t sw_duart_rx_buf[16];
SoftWire sw_duart(PIN_I2C1_SDA, PIN_I2C1_SCL);
#endif

void printTwoDigit(int n);
void readTime(void);

SC16IS752Serial serial0 = SC16IS752Serial(0, SC16IS752_SADDR1);
SC16IS752Serial serial1 = SC16IS752Serial(1, SC16IS752_SADDR1);

int i2c_read(uint8_t addr, uint8_t reg, uint8_t* pdata, uint8_t dlen);
int i2c_read_wo(uint8_t addr, uint8_t reg, uint8_t* pdata, uint8_t dlen);
int i2c_write(uint8_t addr, uint8_t reg, uint8_t* pdata, uint8_t dlen);
int i2c_read_sw(uint8_t addr, uint8_t reg, uint8_t* pdata, uint8_t dlen);
int i2c_write_sw(uint8_t addr, uint8_t reg, uint8_t* pdata, uint8_t dlen);
#ifdef USING_SW_I2C_TO_DUART
int i2c_read_sw_duart(uint8_t addr, uint8_t reg, uint8_t* pdata, uint8_t dlen);
int i2c_write_sw_duart(uint8_t addr, uint8_t reg, uint8_t* pdata, uint8_t dlen);
#endif

#define LED_TOGGLE_PERIOD   500  // msec
int gv_led_status = HIGH;
unsigned long gv_prev_millis;
unsigned long gv_cur_millis;
void led_toggle(void);
void led_toggle_check(void);

void gpio_exp8_conf(void);
void gpio_exp8_read(uint8_t addr, uint8_t* pdata);
void gpio_exp8_read_m(uint8_t addr, uint8_t* pdata);
void gpio_exp8_write(uint8_t addr, uint8_t data);
void gpio_exp_conf(void);
void gpio_exp_read(uint16_t* pdata);
void gpio_exp_write(uint16_t data);

#define RELAY_ON    1
#define RELAY_OFF   0
void relay_on_off(uint8_t pnum, uint8_t value);

void write_duart_reg(uint8_t address, uint8_t channel, uint8_t reg, uint8_t value);
uint8_t read_duart_reg(uint8_t address, uint8_t channel, uint8_t reg);

uint8_t nc_mac[] = {
  0x00, 0x08, 0xDC, 0x00, 0x00, 0x00
};
IPAddress nc_ip(192, 168, 1, 35);
IPAddress nc_dns(192, 168, 1, 1);
IPAddress nc_gateway(192, 168, 1, 1);
IPAddress nc_subnet(255, 255, 255, 0);

SPIClass* hspi;
DhcpClass* dhcp = new DhcpClass();

USBHIDKeyboard Keyboard;

class MyEspUsbHost : public EspUsbHost {
  void onKeyboardKey(uint8_t ascii, uint8_t keycode, uint8_t modifier) {
    if (' ' <= ascii && ascii <= '~') {
      Serial.printf("%c", ascii);
    } else if (ascii == '\r') {
      Serial.println();
    }
  };
};
MyEspUsbHost usbHost;

#define EEPROM_SIZE 512

enum {
  E_USB_MODE_READY,
  E_USB_MODE_DEVICE,
  E_USB_MODE_HOST,
};

enum {
  E_USB_ROLE_DEVIVE,
  E_USB_ROLE_HOST,
  E_USB_ROLE_DRP,
  E_USB_ROLE_DRP2,
};

int8_t gv_usb_mode = E_USB_MODE_READY;
int8_t gv_usbh_init = 0;

void sub_test_a(void);    // LED Test
void sub_test_b(void);    // Button Test
void sub_test_c(void);    // USB OTG Test
void sub_test_l(void);    // RTC Test
void sub_test_m(void);    // IO Expander
void sub_test_n(void);    // W5500 Network Function Test
void sub_test_o(void);    // UART TX Function Test
void sub_test_p(void);    // UART RX Function Test
void sub_test_q(void);    // UART TX/RX Function Test
void sub_test_r(void);    // EEPROM Test

void prt_cmd_info_all(void){

  Serial.println("a: LED Test");
  Serial.println("b: Button Test");
  Serial.println("c: USB OTG Test");
  Serial.println("l: RTC Test");
  Serial.println("m: IO & Expander Test");
  Serial.println("n: Network Function Test");
  Serial.println("o: UART TX Test");
  Serial.println("p: UART RX Test");
  Serial.println("q: UART TX/RX Test");
  Serial.println("r: EEPROM Test");
  Serial.println();
}

void prt_cmd_info_a(void){

}

void prt_cmd_info_b(void){

}

void prt_cmd_info_c(void){

  Serial.println("7: DRP Mode");
  Serial.println("Host Test: ");
  Serial.println("  1) Plug OTG Cable");
  Serial.println("  2) Plug Keyboard");
  Serial.println("  3) Typing & Check");
  Serial.println("  4) Unplug Keyboard and Cable");
  Serial.println("  5) Press # to return to main loop");
  Serial.println("Device Test: ");
  Serial.println("  1) Connect to Host PC with C-type USB Cable");
  Serial.println("  2) Typing & Check : input abc and check bcd printed");
  Serial.println("  3) Unplug C-type USB Cable");
  Serial.println("  4) Press # to return to main loop");
  Serial.println();
}

void prt_cmd_info_l(void){

  Serial.println("3: set (time value should be set in the source code)");
  Serial.println("4: Read RTC time value");
  Serial.println();
}

void prt_cmd_info_m(void){

  Serial.println("1: Read NDA-08V DIP Switch (ON: Low, OFF: High)");
  Serial.println("2: Write Output value of J6[OUTPUT] (ON: Low, OFF: High)");
  Serial.println("3: Read Input value of J2(12V, Upper Byte), J4(5V, Lower Byte)[INPUT]");
  Serial.println("4: Control Relay 1, 2");
  Serial.println("8: Read Analog value of J9[ADC/GPIO]");
  Serial.println("9: Read Digital value of J9[ADC/GPIO]");
  Serial.println();
}

void prt_cmd_info_n(void){

  Serial.println("3: Initialize network setting");
  Serial.println("4: Check network port status");
  Serial.println("7: Set DHCP");
  Serial.println();
}

void prt_cmd_info_o(void){

  Serial.println("RS232 TX or RS485 A/B TX of J12");
  Serial.println("  Press # to return to main loop during transmitting data");
  Serial.println();
}

void prt_cmd_info_p(void){

  Serial.println("RS232 RX or RS485 A/B RX of J12");
  Serial.println("  Press # to return to main loop during receiving data");
  Serial.println();
}

void prt_cmd_info_q(void){

  Serial.println("1: J12, 2: N.A., 3: J14, 4: J15");
  Serial.println();
}

void prt_cmd_info_r(void){

  Serial.println("0: Read 512 bytes, 1: Write 512 sequential data");
  Serial.println();
}

void setup() {

  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, PIN_RS232_RX, PIN_RS232_TX);  // RS232
  Serial2.begin(115200, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX);  // RS485, RS232 TTL
  //Serial2.begin(57600, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX);  // RS485, RS232 TTL
  //Serial2.begin(19200, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX);  // RS485, RS232 TTL
  //Serial2.begin(9600, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX);  // RS485, RS232 TTL

  pinMode(PIN_OTG_VBUS_EN, OUTPUT);
  digitalWrite(PIN_OTG_VBUS_EN, OTG_VBUS_DISABLE);
  pinMode(PIN_USB_ID, INPUT);
  pinMode(PIN_VBUS_DEC, INPUT);

  pinMode(PIN_MAX485_DE, OUTPUT);
  digitalWrite(PIN_MAX485_DE, MAX485_DIR_SEND);

  pinMode(PIN_LED_STATUS, OUTPUT);

  pinMode(PIN_W5500_RST, OUTPUT);
  digitalWrite(PIN_W5500_RST, LOW);
  delay(100);
  digitalWrite(PIN_W5500_RST, HIGH);
  delay(100);

  // Reset SC16IS752
  pinMode(PIN_SC16IS752_RST, OUTPUT);
  digitalWrite(PIN_SC16IS752_RST, LOW);
  delay(100);
  digitalWrite(PIN_SC16IS752_RST, HIGH);
  delay(100);

  // Relay
  pinMode(PIN_RELAY_OUT1, OUTPUT);
  pinMode(PIN_RELAY_OUT2, OUTPUT);

  // SW I2C Initialization
  sw.setTxBuffer(sw_tx_buf, sizeof(sw_tx_buf));
  sw.setRxBuffer(sw_rx_buf, sizeof(sw_rx_buf));
  sw.setDelay_us(5);
  sw.setTimeout(1000);
  sw.begin();

  // GPIO Expander
  gpio_exp8_conf();

  hspi = new SPIClass(HSPI);
  hspi->begin(PIN_ETH_SCLK, PIN_ETH_MISO, PIN_ETH_MOSI, PIN_ETH_CS);

#ifdef PCF_HW_I2C_TEST
  Wire.begin(PIN_SW_I2C_SDA, PIN_SW_I2C_SCL, 400000);
#else
  //Wire.begin(PIN_I2C1_SDA, PIN_I2C1_SCL, 400000);
  Wire.begin(PIN_I2C1_SDA, PIN_I2C1_SCL);
  //Wire.setClock(50000);
  Wire.setClock(400000);
  //Wire.begin(PIN_I2C2_SDA, PIN_I2C2_SCL);
  Serial.printf("I2C Clock: %0d\r\n", Wire.getClock());  // Default 100k
#endif
  //Wire1.begin(PIN_I2C2_SDA, PIN_I2C2_SCL, 400000);

#ifdef USING_SW_I2C_TO_DUART
  sw_duart.setTxBuffer(sw_duart_tx_buf, sizeof(sw_duart_tx_buf));
  sw_duart.setRxBuffer(sw_duart_rx_buf, sizeof(sw_duart_rx_buf));
  sw_duart.setDelay_us(5);
  sw_duart.setTimeout(1000);
  sw_duart.begin();
#endif

  // SC16IS752 UART setup. (begin is excuted here instead of pzem creation)
  serial0.begin(SC_REF_BAUDRATE);
  serial1.begin(SC_REF_BAUDRATE);

  // initialize EEPROM with predefined size
  EEPROM.begin(EEPROM_SIZE);

  // USB Device 
  //Keyboard.begin();
  //USB.begin();

  // USB Host
  //usbHost.begin();
  //usbHost.setHIDLocal(HID_LOCAL_US);

  //esp_log_level_set("EspUsbHost", ESP_LOG_VERBOSE);
}

void loop() {

  Serial.println();
  Serial.println("INL-05 testing.");
  Serial.println("(C) 2024 Dignsys");
  Serial.printf("VERSION: %s\r\n\r\n", VERSION_INL05_FW);

  prt_cmd_info_all();
  gv_prev_millis = gv_cur_millis = millis();

  char c;
  while(c != 'x') {
    Serial.println();
    Serial.printf("Input Command: ");
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        if(isalnum(c)){
          break;
        }
      }
      delay(100);
      led_toggle_check();
    }
    Serial.printf("%c", c);
    Serial.println();

    switch(c) {
      case 'a': 
        sub_test_a();
        break;
      case 'b': 
        sub_test_b();
        break;
      case 'c':
        sub_test_c();
        break;
      case 'l':
        sub_test_l();
        break;
      case 'm':
        sub_test_m();
        break;
      case 'n':
        sub_test_n();
        break;
      case 'o':
        sub_test_o();
        break;
      case 'p':
        sub_test_p();
        break;
      case 'q':
        sub_test_q();
        break;
      case 'r':
        sub_test_r();
        break;
      default:
        break;
    }
  }
  Serial.println("loop exit!");

}

int i2c_read(uint8_t addr, uint8_t reg, uint8_t* pdata, uint8_t dlen){

  int ret = 0;

  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission();
  
  Wire.requestFrom(addr, (uint8_t)dlen);

  if (Wire.available()) {
    for(int i = 0; i < dlen; i++) {
      pdata[i] = Wire.read();
    }
  }

  return ret;
}

int i2c_read_wo(uint8_t addr, uint8_t reg, uint8_t* pdata, uint8_t dlen){

  int ret = 0;

  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  
  Wire.requestFrom(addr, (size_t)dlen, (bool) false);

  if (Wire.available()) {
    for(int i = 0; i < dlen; i++) {
      pdata[i] = Wire.read();
    }
  }

  return ret;
}

int i2c_write(uint8_t addr, uint8_t reg, uint8_t* pdata, uint8_t dlen){

  int ret = 0;

  Wire.beginTransmission(addr);
  Wire.write(reg);
  for(int i = 0; i < dlen; i++) {
    Wire.write(pdata[i]);
  }
  Wire.endTransmission();

  return ret;
}

int i2c_read_sw(uint8_t addr, uint8_t reg, uint8_t* pdata, uint8_t dlen){

  int ret = 0;

  sw.beginTransmission(addr);
  sw.write(reg);
  sw.endTransmission();
  
  sw.requestFrom(addr, (uint8_t)dlen);

  if (sw.available()) {
    for(int i = 0; i < dlen; i++) {
      pdata[i] = sw.read();
    }
  }

  return ret;
}

int i2c_write_sw(uint8_t addr, uint8_t reg, uint8_t* pdata, uint8_t dlen){

  int ret = 0;

  sw.beginTransmission(addr);
  sw.write(reg);
  for(int i = 0; i < dlen; i++) {
    sw.write(pdata[i]);
  }
  sw.endTransmission();

  return ret;
}
#ifdef USING_SW_I2C_TO_DUART
int i2c_read_sw_duart(uint8_t addr, uint8_t reg, uint8_t* pdata, uint8_t dlen){

  int ret = 0;

  sw_duart.beginTransmission(addr);
  sw_duart.write(reg);
  sw_duart.endTransmission();
  
  sw_duart.requestFrom(addr, (uint8_t)dlen);

  if (sw_duart.available()) {
    for(int i = 0; i < dlen; i++) {
      pdata[i] = sw_duart.read();
    }
  }

  return ret;
}

int i2c_write_sw_duart(uint8_t addr, uint8_t reg, uint8_t* pdata, uint8_t dlen){

  int ret = 0;

  sw_duart.beginTransmission(addr);
  sw_duart.write(reg);
  for(int i = 0; i < dlen; i++) {
    sw_duart.write(pdata[i]);
  }
  sw_duart.endTransmission();

  return ret;
}
#endif

void gpio_exp8_conf(void){
#ifndef PCF_HW_I2C_TEST
  sw.beginTransmission(I2C_ADDR_PCF_SW);
  sw.write(0x00);
  sw.endTransmission();

  sw.beginTransmission(I2C_ADDR_PCF_OUT);
  sw.write(0x00);
  sw.endTransmission();
#else
  Wire.beginTransmission(I2C_ADDR_PCF_SW);
  Wire.write(0xff);
  Wire.endTransmission();

  Wire.beginTransmission(I2C_ADDR_PCF_OUT);
  Wire.write(0x00);
  Wire.endTransmission();
#endif
}

void gpio_exp8_read(uint8_t addr, uint8_t* pdata){

  uint8_t rdata[3] = {0,};
  uint8_t cnt = 0;
  uint8_t idx = 0;
  uint8_t rslt = 0;
#ifndef PCF_HW_I2C_TEST
  sw.beginTransmission(addr);
  sw.write(0xff);
  sw.endTransmission();

  rslt = sw.requestFrom(addr, (uint8_t)2);
  //Serial.printf("Request Result: Addr: 0x%02x, 0x%02x\r\n", addr, rslt);

  while(1){
    if (sw.available()) {
      rdata[idx++] = sw.read();
      //Serial.printf("I2C read 0x%02x at cnt %d\r\n", rdata[cnt], cnt);
      if(idx >= 2){
        //Serial.printf("rdata: 0x%02x, 0x%02x\r\n", rdata[0], rdata[1]);
        break;
      }
    }
    delay(1);
    if(++cnt > 100){
      Serial.println("SW I2C read time-out!");
      Serial.printf("rdata: 0x%02x, 0x%02x\r\n", rdata[0], rdata[1]);
      break;
    }
  }
#else
  Wire.begin();
  rslt = Wire.requestFrom(addr, (uint8_t)1);
  Serial.printf("Request Result: Addr: 0x%02x, 0x%02x\r\n", addr, rslt);

  while(1){
    if (Wire.available()) {
      rdata[0] = Wire.read();
      Serial.printf("I2C read at cnt %d\r\n", cnt);
      break;
    }
    delay(1);
    if(++cnt > 100){
      Serial.println("HW I2C read time-out!");
      rdata[0] = Wire.read();
      break;
    }
  }
#endif
  *pdata = rdata[0];

}

void gpio_exp8_read_m(uint8_t addr, uint8_t* pdata){

  uint8_t ack = 0;
  uint8_t rdata[9] = {0,};

  sw.sdaLow();
  delayMicroseconds(50);
  sw.sclLow();
  delayMicroseconds(50);

  sw.sdaLow();  // A6
  delayMicroseconds(50);
  sw.sclHigh();
  delayMicroseconds(50);
  sw.sclLow();
  delayMicroseconds(50);

  sw.sdaHigh();  // A5
  delayMicroseconds(50);
  sw.sclHigh();
  delayMicroseconds(50);
  sw.sclLow();
  delayMicroseconds(50);

  sw.sdaHigh();  // A4
  delayMicroseconds(50);
  sw.sclHigh();
  delayMicroseconds(50);
  sw.sclLow();
  delayMicroseconds(50);

  sw.sdaHigh();  // A3
  delayMicroseconds(50);
  sw.sclHigh();
  delayMicroseconds(50);
  sw.sclLow();
  delayMicroseconds(50);

  sw.sdaLow();  // A2
  delayMicroseconds(50);
  sw.sclHigh();
  delayMicroseconds(50);
  sw.sclLow();
  delayMicroseconds(50);

  sw.sdaHigh();  // A1
  delayMicroseconds(50);
  sw.sclHigh();
  delayMicroseconds(50);
  sw.sclLow();
  delayMicroseconds(50);

  sw.sdaHigh();  // A0
  delayMicroseconds(50);
  sw.sclHigh();
  delayMicroseconds(50);
  sw.sclLow();
  delayMicroseconds(50);

  sw.sdaHigh();  // RE
  delayMicroseconds(50);
  sw.sclHigh();
  delayMicroseconds(50);
  sw.sclLow();
  delayMicroseconds(50);

  sw.sclLow();
  delayMicroseconds(50);
  sw.sclHigh();
  delayMicroseconds(50);
  sw.sdaLow();
  delayMicroseconds(50);
  sw.sdaHigh();
  delayMicroseconds(500);

  ack = sw.readSda(&sw);

  for(int i; i < 8; i++){
    sw.sclLow();
    delayMicroseconds(50);
    sw.sclHigh();
    delayMicroseconds(50);
    rdata[i] = sw.readSda(&sw);
  }

  sw.sclLow();
  delayMicroseconds(50);
  sw.sclHigh();
  delayMicroseconds(50);
  
  Serial.printf("EOT[%d]: SDA: %d, SCL: %d\r\n", ack, sw.readSda(&sw), sw.readScl(&sw));

  Serial.printf("Data: %d, %d, %d, %d, %d, %d, %d, %d\r\n", 
    rdata[0], rdata[1], rdata[2], rdata[3], rdata[4], rdata[5], rdata[6], rdata[7]);

  *pdata = rdata[0];

}

void gpio_exp8_write(uint8_t addr, uint8_t data){
#ifndef PCF_HW_I2C_TEST
  sw.beginTransmission(addr);
  sw.write(data);
  sw.endTransmission();
#else  
  Wire.beginTransmission(addr);
  Wire.write(data);
  Wire.endTransmission();
#endif
}

void gpio_exp_conf(void){

  uint8_t data[3] = {0,};

  data[0] = 0xff;
  data[1] = 0xff;

  // IN/OUT set
  i2c_write_sw(I2C_ADDR_MCP_IN, 0x00, data, 2);

}

void gpio_exp_read(uint16_t* pdata){

  uint8_t rdata[3] = {0,};

  i2c_read_sw(I2C_ADDR_MCP_IN, 0x12, rdata, 2);

  *pdata = rdata[1] * 0x100 + rdata[0];

}

void gpio_exp_write(uint16_t data){

  uint8_t wdata[3] = {0,};

  wdata[0] = (uint8_t) (data & 0x00ff);
  wdata[1] = (uint8_t) ((data>>8) & 0x00ff);

  i2c_write_sw(I2C_ADDR_MCP_IN, 0x14, wdata, 2);

}

void relay_on_off(uint8_t pnum, uint8_t value){

  digitalWrite(pnum, value);

}

void write_duart_reg(uint8_t address, uint8_t channel, uint8_t reg, uint8_t value){

  Wire.beginTransmission(address);
  Wire.write((reg<<3)|(channel<<1));
  Wire.write(value);
  Wire.endTransmission();

}

uint8_t read_duart_reg(uint8_t address, uint8_t channel, uint8_t reg){

  uint8_t rslt;
  uint8_t rdata[3] = {0,};
  uint8_t cnt = 0;

  Wire.beginTransmission(address);
  Wire.write((reg<<3)|(channel<<1));
  Wire.endTransmission();

  rslt = Wire.requestFrom(address, (uint8_t)1);
  Serial.printf("Request Result: Addr: 0x%02x, 0x%02x\r\n", address, rslt);

  while(1){
    if (Wire.available()) {
      rdata[0] = Wire.read();
      Serial.printf("I2C read at cnt %d\r\n", cnt);
      break;
    }
    delay(1);
    if(++cnt > 100){
      Serial.println("HW I2C read time-out!");
      rdata[0] = Wire.read();
      break;
    }
  }

  return rdata[0];

}

// Print with leading zero, as expected for time
void printTwoDigit(int n)
{
  if (n < 10) {
    Serial.print('0');
  }
  Serial.print(n);
}

void readTime(void)
{
  // Ensure register address is valid
  sw.beginTransmission(I2C_ADDR_RTC);
  sw.write(uint8_t(0)); // Access the first register
  sw.endTransmission();

  uint8_t registers[7]; // There are 7 registers we need to read from to get the date and time.
  int numBytes = sw.requestFrom(I2C_ADDR_RTC, (uint8_t)7);
  for (int i = 0; i < numBytes; ++i) {
    registers[i] = sw.read();
  }
  if (numBytes != 7) {
    Serial.print("Read wrong number of bytes: ");
    Serial.println((int)numBytes);
    return;
  }

  int tenYear = (registers[6] & 0xf0) >> 4;
  int unitYear = registers[6] & 0x0f;
  int year = (10 * tenYear) + unitYear;

  int tenMonth = (registers[5] & 0x10) >> 4;
  int unitMonth = registers[5] & 0x0f;
  int month = (10 * tenMonth) + unitMonth;

  int tenDateOfMonth = (registers[4] & 0x30) >> 4;
  int unitDateOfMonth = registers[4] & 0x0f;
  int dateOfMonth = (10 * tenDateOfMonth) + unitDateOfMonth;

  // Reading the hour is messy. See the datasheet for register details!
  bool twelveHour = registers[2] & 0x40;
  bool pm = false;
  int unitHour;
  int tenHour;
  if (twelveHour) {
    pm = registers[2] & 0x20;
    tenHour = (registers[2] & 0x10) >> 4;
  } else {
    tenHour = (registers[2] & 0x30) >> 4;
  }
  unitHour = registers[2] & 0x0f;
  int hour = (10 * tenHour) + unitHour;
  if (twelveHour) {
    // 12h clock? Convert to 24h.
    hour += 12;
  }

  int tenMinute = (registers[1] & 0xf0) >> 4;
  int unitMinute = registers[1] & 0x0f;
  int minute = (10 * tenMinute) + unitMinute;

  int tenSecond = (registers[0] & 0xf0) >> 4;
  int unitSecond = registers[0] & 0x0f;
  int second = (10 * tenSecond) + unitSecond;

  // ISO8601 is the only sensible time format
  Serial.print("Time: ");
  Serial.print(year);
  Serial.print('-');
  printTwoDigit(month);
  Serial.print('-');
  printTwoDigit(dateOfMonth);
  Serial.print('T');
  printTwoDigit(hour);
  Serial.print(':');
  printTwoDigit(minute);
  Serial.print(':');
  printTwoDigit(second);
  Serial.println();

  sprintf((char*) str_cur_date, "%02d-%02d-%02d", year, month, dateOfMonth);
  sprintf((char*) str_cur_time, "%02d:%02d:%02d", hour, minute, second);
  Serial.printf("cur_date: %s\r\n", str_cur_date);
  Serial.printf("cur_time: %s\r\n", str_cur_time);
}

void setRS485Dir(bool dir) {

  if(dir == MAX485_DIR_SEND) {
    digitalWrite(PIN_MAX485_DE, MAX485_DIR_SEND);
  } else if(dir == MAX485_DIR_RECEIVE) {
    digitalWrite(PIN_MAX485_DE, MAX485_DIR_RECEIVE);
  } else {
    Serial.println("Invalid set of MAX485");
  }
}

void led_toggle(void) {

  if(gv_led_status == HIGH){
    gv_led_status = LOW;
  } else {
    gv_led_status = HIGH;
  }
  digitalWrite(PIN_LED_STATUS, gv_led_status);
}

void led_toggle_check(void) {

  gv_cur_millis = millis();
  if((gv_cur_millis - gv_prev_millis) > LED_TOGGLE_PERIOD){
    led_toggle();
    gv_prev_millis = gv_cur_millis;
  }
}

void sub_test_a(void) {

  Serial.println("Sub-test A - LED");

  char c = 0;
  uint8_t led_status = 0;

  Serial.println("Press q to quit: ");

  while(1) {
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)) Serial.println(c);
    }
    if((c == 'q') || (c == '#')){
      Serial.println("Quit loop");
      break;
    }
    if(led_status) {
      led_status = 0;
      Serial.println("LED Off");
    } else {
      led_status = 1;
      Serial.println("LED On");
    }
    digitalWrite(PIN_LED_STATUS, led_status);
    delay(1000);
  }
}

void sub_test_b(void) {

  Serial.println("Sub-test B - Button");

  char c = 0;
  uint8_t button_status = 0;

  Serial.println("Press q to quit: ");

  while(1) {
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)) Serial.println(c);
    }
    if((c == 'q') || (c == '#')){
      Serial.println("Quit loop");
      break;
    }
    button_status = digitalRead(PIN_BOOT);
    Serial.printf("Button: %d\r\n", button_status);
    delay(1000);
  }
}

void sub_test_c(void) {

  uint8_t data[5] = {0,};
  int numBytes;
  char c, c_out;
  uint8_t usb_id, usb_id_prev, usb_vbus_det;
  uint8_t usb_pol, usb_port_status, usb_chg_current, usb_vbus;
  uint8_t usb_host_prc = 0;
  uint8_t usb_device_prc = 0;

  Serial.println("Sub-test C - USB-OTG");
  prt_cmd_info_c();

  Serial.print("Input Test Number: ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)) break;
    }
    delay(100);
  }
  Serial.println(c);

  if(c == '0') {  // 

    i2c_read_sw(I2C_ADDR_USB, 0x01, data, 4);
    Serial.printf("Device ID[0x%02x], Control[0x%02x], Interrupt[0x%02x], CC status[0x%02x]\r\n", 
      data[0], data[1], data[2], data[3]);
    Serial.printf("USB_ID[%d], VBUS_DET[%d]\r\n", digitalRead(PIN_USB_ID), digitalRead(PIN_VBUS_DEC));
    Serial.println("* Control ===");
    if((data[1]&0x06) == 0x00){
      Serial.println("Port: Device(SNK)");
    } else if((data[1]&0x06) == 0x02){
      Serial.println("Port: Host(SRC)");
    } else {
      Serial.println("Port: Dual Role");
    }
    Serial.println("* Interrupt ===");
    if((data[2]&0x02)){
      Serial.println("Detach Event");
    } else if((data[2]&0x01)){
      Serial.println("Attach Event");
    } else {
      Serial.println("No Event");
    }
    Serial.println("* CC Status ===");
    if((data[3]&0x80)){
      Serial.println("VBUS detected");
    } else {
      Serial.println("VBUS not detected");
    }
    if((data[3]&0x60) == 0x20){
      Serial.println("Charging current detection - Default current mode");
    } else if((data[3]&0x60 == 0x40)){
      Serial.println("Charging current detection - Medium current mode");
    } else if((data[3]&0x60 == 0x60)){
      Serial.println("Charging current detection - High current mode");
    } else {
      Serial.println("Charging current detection - Standby");
    }
    if((data[3]&0x1c) == 0x04){
      Serial.println("Attached port status - Device");
    } else if((data[3]&0x1c) == 0x08){
      Serial.println("Attached port status - Host");
    } else if((data[3]&0x1c) == 0x0c){
      Serial.println("Attached port status - Audio Adapter Accessory");
    } else if((data[3]&0x1c) == 0x10){
      Serial.println("Attached port status - Debug Accessory");
    } else if((data[3]&0x1c) == 0x14){
      Serial.println("Attached port status - Device with Active Cable");
    } else if((data[3]&0x1c) == 0x00){
      Serial.println("Attached port status - Standby");
    } else {
      Serial.println("Attached port status - Unknown");
    }
    if((data[3]&0x03) == 0x00){
      Serial.println("Plug polarity - Standby");
    } else if((data[3]&0x03) == 0x01){
      Serial.println("Plug polarity - CC1 makes connection");
    } else if((data[3]&0x03) == 0x02){
      Serial.println("Plug polarity - CC2 makes connection");
    } else {
      Serial.println("Plug polarity - Undetermined");
    }

  } else if(c == '1') {

    Keyboard.begin();
    USB.begin();

  } else if(c == '2') {

    while(1) {
      Serial.println("Input data: ");
      while(1){
        if(Serial.available()) {
          c = Serial.read();
          if(isalnum(c) || (c == '#')) break;
        }
        delay(100);
      }
      if(!isalnum(c)){
        Serial.println("Quit data input");
        break;
      }
      Serial.println(c);
      
      c_out = c + 1;

      Keyboard.write(c_out);
    }
  } else if(c == '3') {

    // Device Mode
    data[0] = E_USB_ROLE_DEVIVE << 1;  //0x00;
    i2c_write_sw(I2C_ADDR_USB, USB_DET_REG_CTRL, data, 1);

    digitalWrite(PIN_OTG_VBUS_EN, OTG_VBUS_DISABLE);
    i2c_read_sw(I2C_ADDR_USB, USB_DET_REG_ID, data, 4);
    Serial.printf("Status[%02x-%02x-%02x-%02x]: 0x%02x\r\n", data[0], data[1], data[2], data[3], data[3]);

  } else if(c == '4') {

    // Host Mode
    data[0] = E_USB_ROLE_HOST << 1;  //0x02;
    i2c_write_sw(I2C_ADDR_USB, USB_DET_REG_CTRL, data, 1);

//    digitalWrite(PIN_OTG_VBUS_EN, OTG_VBUS_ENABLE);
    digitalWrite(PIN_OTG_VBUS_EN, OTG_VBUS_DISABLE);
    i2c_read_sw(I2C_ADDR_USB, USB_DET_REG_ID, data, 4);
    Serial.printf("Status[%02x-%02x-%02x-%02x]: 0x%02x\r\n", data[0], data[1], data[2], data[3], data[3]);

  } else if(c == '5') {

    usbHost.begin();
    usbHost.setHIDLocal(HID_LOCAL_US);

  } else if(c == '6') {

    usb_id_prev = 0xff;
    Serial.printf("Is Ready: %d\r\n", usbHost.isReady);
    while(1) {

      if(Serial.available()) {
        c = Serial.read();
        if(c == '#'){
          Serial.println("Quit USB-Host Task");
          break;
        }
      }
      usb_id = digitalRead(PIN_USB_ID);
      if(usb_id != usb_id_prev){
        if(!usb_id){
          digitalWrite(PIN_OTG_VBUS_EN, OTG_VBUS_ENABLE);
          Serial.println("VBUS applied");
        } else {
          digitalWrite(PIN_OTG_VBUS_EN, OTG_VBUS_DISABLE);
          Serial.println("VBUS removed");
        }
        usb_id_prev = usb_id;
      }

      usbHost.task();
    }
  } else if(c == '7') {

    // Dual Role (DRP)
    data[0] = E_USB_ROLE_DRP << 1;
    i2c_write_sw(I2C_ADDR_USB, USB_DET_REG_CTRL, data, 1);
    while(1) {

      if(Serial.available()) {
        c = Serial.read();
        if(c == '#'){
          Serial.println("Quit CC-Status Check");
          break;
        }
      }
      
      i2c_read_sw(I2C_ADDR_USB, USB_DET_REG_ID, data, 4);
      usb_pol = data[3]&0x03;
      usb_port_status = (data[3]>>2)&0x07;
      usb_chg_current = (data[3]>>5)&0x03;
      usb_vbus = (data[3]>>7)&0x01;
      usb_id = digitalRead(PIN_USB_ID);
      usb_vbus_det = digitalRead(PIN_VBUS_DEC);

#if 1
      if((!usb_host_prc) && (!usb_device_prc)) {
        Serial.printf("Data: %02x-%02x-%02x-%02x, VBUS: 0x%02x, Current: 0x%02x, Port: 0x%02x, Pol: 0x%02x\r\n",
          data[0], data[1], data[2], data[3], usb_vbus, usb_chg_current, usb_port_status, usb_pol);
        Serial.printf("USB_ID: %d, VBUS_DET: %d\r\n", usb_id, usb_vbus_det);
      }
#endif

      if(gv_usb_mode == E_USB_MODE_READY) {

        if(usb_vbus_det) {
          data[0] = E_USB_ROLE_DEVIVE << 1;
          i2c_write_sw(I2C_ADDR_USB, USB_DET_REG_CTRL, data, 1);
          gv_usb_mode = E_USB_MODE_DEVICE;
          digitalWrite(PIN_OTG_VBUS_EN, OTG_VBUS_DISABLE);
          Serial.println("--- Goto USB Device Mode ----------");
          Serial.println("Input Keyboard Data :");
          Keyboard.begin();
          //Serial.println("usb device stage 1");
          USB.begin();
          //Serial.println("usb device stage 2");
          usb_device_prc = 1;
        } else if(!usb_id) {
          data[0] = E_USB_ROLE_HOST << 1;
          i2c_write_sw(I2C_ADDR_USB, USB_DET_REG_CTRL, data, 1);
          gv_usb_mode = E_USB_MODE_HOST;
          delay(500);
          digitalWrite(PIN_OTG_VBUS_EN, OTG_VBUS_ENABLE);
          delay(500);
          Serial.println("--- Goto USB Host Mode ----------");
          if(!gv_usbh_init) {
            usbHost.begin();
            //Serial.println("host stage 1");
            usbHost.setHIDLocal(HID_LOCAL_US);
            //Serial.println("host stage 2");
            gv_usbh_init = 1;
          }
          usb_host_prc = 1;
        }
      } else if(gv_usb_mode == E_USB_MODE_DEVICE) {

        if(!usb_vbus_det) {
          data[0] = E_USB_ROLE_DRP << 1;
          i2c_write_sw(I2C_ADDR_USB, USB_DET_REG_CTRL, data, 1);
          gv_usb_mode = E_USB_MODE_READY;
          digitalWrite(PIN_OTG_VBUS_EN, OTG_VBUS_DISABLE);
          Serial.println();
          Serial.println("--- Goto USB Ready ----------");
          USB.~ESPUSB();
          Keyboard.end();
          usb_device_prc = 0;
        } else {

          if(Serial.available()) {
            c = Serial.read();
            if(isalnum(c)) {
              c_out = c + 1;
              Keyboard.write(c_out);
              Serial.print(c);
            }
          }
        }
      } else if(gv_usb_mode == E_USB_MODE_HOST) {

        if(usb_id) {
          data[0] = E_USB_ROLE_DRP << 1;
          i2c_write_sw(I2C_ADDR_USB, USB_DET_REG_CTRL, data, 1);
          gv_usb_mode = E_USB_MODE_READY;
          digitalWrite(PIN_OTG_VBUS_EN, OTG_VBUS_DISABLE);
          Serial.println();
          Serial.println("--- Goto USB Ready ----------");
          usb_host_prc = 0;
        } else {
          //Serial.println("host stage 3");
          usbHost.task();
          //Serial.println("host stage 4");
        }
      }

      if((!usb_host_prc) && (!usb_device_prc)) delay(1000);
    }

  } else {
    Serial.println("Invalid Test Number");
    return;
  }
}

void sub_test_l(void) {

  uint8_t data[8] = {0,};
  int numBytes;
  char c;
  Serial.println("Sub-test L - RTC");
  prt_cmd_info_l();

  Serial.print("Input Test Number: ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)) break;
    }
    delay(100);
  }
  Serial.println(c);

  if(c == '0') {  // Read RTC 8 Bytes
    sw.beginTransmission(I2C_ADDR_RTC);
    sw.write(uint8_t(0)); // Access the first register
    sw.endTransmission();

    numBytes = sw.requestFrom(I2C_ADDR_RTC, (uint8_t)8);
    for (int i = 0; i < numBytes; ++i) {
      data[i] = sw.read();
      Serial.print("data["); Serial.print(i, DEC); Serial.print("]: "); Serial.println(data[i], HEX);
    }
    if (numBytes != 8) {
      Serial.print("Read wrong number of bytes: ");
      Serial.println((int)numBytes);
      return;
    }
  } else if (c == '1') {  // Enable RTC Clock
    sw.beginTransmission(I2C_ADDR_RTC);
    sw.write(uint8_t(0));
    sw.write(uint8_t(0));
    sw.endTransmission();
  } else if (c == '2') {  // Disable RTC Clock
    sw.beginTransmission(I2C_ADDR_RTC);
    sw.write(uint8_t(0));
    sw.write(uint8_t(0x80));
    sw.endTransmission();
  } else if (c == '3') {  // Set RTC Time
    data[0] = 0*0x10;   // 10 Seconds
    data[0] += 0;       // Seconds

    data[1] = 5*0x10;   // 10 Minutes
    data[1] += 7;       // Minutes

    //data[2] = 0x40;   // 12-Hour Mode
    data[2] = 0x00;     // 24-Hour Mode

    data[2] += 0*0x10;  // 10 Hours
    data[2] += 9;       // Hours

    data[3] = 2;        // Day (1~7), Monday first

    data[4] = 3*0x10;   // 10 Date
    data[4] += 0;       // Date

    data[5] = 0*0x10;   // 10 Month
    data[5] += 1;       // Month

    data[6] = 2*0x10;   // 10 Year
    data[6] += 4;       // Year
    sw.beginTransmission(I2C_ADDR_RTC);
    sw.write(uint8_t(0));
    for (int i = 0; i < 8; ++i) {
      sw.write(data[i]);
    }
    sw.endTransmission();
  } else if (c == '4') {  // Read Time
    readTime();
  } else {
    Serial.println("Invalid Test Number");
    return;
  }

}

void sub_test_m(void) {

  uint16_t data;
  uint8_t rdata[2];
  uint8_t tdata;
  uint8_t val[2];
  int numBytes;
  char c;
  uint8_t reg;
  Serial.println("Sub-test M - IO Expander");
  prt_cmd_info_m();

  Serial.print("Input Test Number: ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)) break;
    }
    delay(100);
  }
  Serial.println(c);

  if(c == '0') {  // Read IO
    //gpio_exp8_read(I2C_ADDR_PCF_SW, rdata);
    //Serial.printf("pcf_data: 0x%02x\r\n", rdata[0]);

    gpio_exp8_read_m(I2C_ADDR_PCF_SW, rdata);
    Serial.printf("pcf_data_m: 0x%02x\r\n", rdata[0]);

#if 0
    for(int i=1; i<9; i++){
      if(get_swtich_val(i) == SWITCH_ON){
        Serial.print("Switch"); Serial.print(i); Serial.println(": ON");
      } else {
        Serial.print("Switch"); Serial.print(i); Serial.println(": OFF");
      }
    }
#endif
  } else if(c == '1') {
    gpio_exp8_read(I2C_ADDR_PCF_SW, rdata);
    Serial.printf("pcf_data: 0x%02x\r\n", rdata[0]);

  } else if(c == '2') {
    Serial.print("Input PCF OUT Data (e.g. aa): ");
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        if(isalnum(c)) break;
      }
      delay(100);
    }
    c = tolower(c);
    if((c >= '0') && (c <= '9')) {
      tdata = (c - '0')*0x10;
    } else if ((c >= 'a') && (c <= 'f')) {
      tdata = (c - 'a' + 0xa)*0x10;
    } else {
      Serial.println("Invalid input");
      return;
    }
    Serial.print(c);
  
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        if(isalnum(c)) break;
      }
      delay(100);
    }
    c = tolower(c);
    if((c >= '0') && (c <= '9')) {
      tdata |= (c - '0');
    } else if ((c >= 'a') && (c <= 'f')) {
      tdata |= (c - 'a' + 0xa);
    } else {
      Serial.println("Invalid input");
      return;
    }
    Serial.println(c);

    gpio_exp8_write(I2C_ADDR_PCF_OUT, tdata);
    Serial.printf("pcf_data_write: 0x%02x\r\n", tdata);

    //gpio_exp8_read(I2C_ADDR_PCF_OUT, rdata);
    //Serial.printf("pcf_data_read: 0x%02x\r\n", rdata[0]);

  } else if(c == '3') {
    gpio_exp_read(&data);
    Serial.printf("mcp_data: 0x%04x\r\n", data);

  } else if(c == '4') {  // Relay On/Off
    Serial.print("Relay Number[1,2]: ");
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        if(isalnum(c)) break;
      }
      delay(100);
    }
    if(c == '1'){
      rdata[0] = PIN_RELAY_OUT1;
    } else if(c == '2'){
      rdata[0] = PIN_RELAY_OUT2;
    } else {
      Serial.println("Invalid Relay Number!");
      return;
    }
    Serial.println(c);

    Serial.print("Relay [1: ON, 0: OFF]: ");
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        if(isalnum(c)) break;
      }
      delay(100);
    }
    if(c == '0'){
      rdata[1] = RELAY_OFF;
    } else if(c == '1'){
      rdata[1] = RELAY_ON;
    } else {
      Serial.println("Invalid Relay Operation!");
      return;
    }
    Serial.println(c);

    relay_on_off(rdata[0], rdata[1]);

#if 0
    for(int i=1; i<5; i++){
      led_pm(i, LED_OFF);
    }

    for(int i=1; i<5; i++){
      led_pm(i, LED_ON);
      delay(500);
      led_pm(i, LED_OFF);
      delay(500);
    }

    led_fail(LED_ON);
    delay(500);
    led_fail(LED_OFF);
    delay(500);

    led_tx(LED_ON);
    delay(500);
    led_tx(LED_OFF);
    delay(500);

    led_rx(LED_ON);
    delay(500);
    led_rx(LED_OFF);
    delay(500);
#endif
  } else if(c == '5') {  // Data Read
    Serial.print("Input Register Address: ");
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        if(isalnum(c)) break;
      }
      delay(100);
    }
    c = tolower(c);
    if((c >= '0') && (c <= '9')) {
      reg = (c - '0')*0x10;
    } else if ((c >= 'a') && (c <= 'f')) {
      reg = (c - 'a' + 0xa)*0x10;
    } else {
      Serial.println("Invalid input");
      return;
    }
    Serial.print(c);
  
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        if(isalnum(c)) break;
      }
      delay(100);
    }
    c = tolower(c);
    if((c >= '0') && (c <= '9')) {
      reg |= (c - '0');
    } else if ((c >= 'a') && (c <= 'f')) {
      reg |= (c - 'a' + 0xa);
    } else {
      Serial.println("Invalid input");
      return;
    }
    Serial.println(c);

    //i2c_read(SC16IS752_SADDR0, reg, rdata, 2);
#ifdef USING_SW_I2C_TO_DUART
    i2c_read_sw_duart(SC16IS752_SADDR0, 0x00, rdata, 1);
#else
    rdata[0] = read_duart_reg(SC16IS752_SADDR0, SC16IS752_CHANNEL_A, reg);
#endif
    Serial.print("data[0]: "); Serial.println(rdata[0], HEX);
    //Serial.print("data[1]: "); Serial.println(rdata[1], HEX);

  } else if(c == '6') {  // Data Write
    Serial.print("Input Register Address: ");
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        if(isalnum(c)) break;
      }
      delay(100);
    }
    c = tolower(c);
    if((c >= '0') && (c <= '9')) {
      reg = (c - '0')*0x10;
    } else if ((c >= 'a') && (c <= 'f')) {
      reg = (c - 'a' + 0xa)*0x10;
    } else {
      Serial.println("Invalid input");
      return;
    }
    Serial.print(c);
  
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        if(isalnum(c)) break;
      }
      delay(100);
    }
    c = tolower(c);
    if((c >= '0') && (c <= '9')) {
      reg |= (c - '0');
    } else if ((c >= 'a') && (c <= 'f')) {
      reg |= (c - 'a' + 0xa);
    } else {
      Serial.println("Invalid input");
      return;
    }
    Serial.println(c);

    Serial.print("Input Value to write: ");
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        if(isalnum(c)) break;
      }
      delay(100);
    }
    c = tolower(c);
    if((c >= '0') && (c <= '9')) {
      val[0] = (c - '0')*0x10;
    } else if ((c >= 'a') && (c <= 'f')) {
      val[0] = (c - 'a' + 0xa)*0x10;
    } else {
      Serial.println("Invalid input");
      return;
    }
    Serial.print(c);
  
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        if(isalnum(c)) break;
      }
      delay(100);
    }
    c = tolower(c);
    if((c >= '0') && (c <= '9')) {
      val[0] |= (c - '0');
    } else if ((c >= 'a') && (c <= 'f')) {
      val[0] |= (c - 'a' + 0xa);
    } else {
      Serial.println("Invalid input");
      return;
    }
    Serial.println(c);

    Serial.print("Input High Byte Value to write: ");
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        if(isalnum(c)) break;
      }
      delay(100);
    }
    c = tolower(c);
    if((c >= '0') && (c <= '9')) {
      val[1] = (c - '0')*0x10;
    } else if ((c >= 'a') && (c <= 'f')) {
      val[1] = (c - 'a' + 0xa)*0x10;
    } else {
      Serial.println("Invalid input");
      return;
    }
    Serial.print(c);
  
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        break;
      }
      delay(100);
    }
    c = tolower(c);
    if((c >= '0') && (c <= '9')) {
      val[1] |= (c - '0');
    } else if ((c >= 'a') && (c <= 'f')) {
      val[1] |= (c - 'a' + 0xa);
    } else {
      Serial.println("Invalid input");
      return;
    }
    Serial.println(c);

    i2c_write(SC16IS752_SADDR0, reg, val, 2);

  } else if(c == '7') {  // Dual Uart LED
#if 0  
    for(int i=2; i<6; i++){
      dual_uart_led_set(i, LED_OFF);
    }

    for(int i=2; i<6; i++){
      dual_uart_led_set(i, LED_ON);
      delay(500);
      dual_uart_led_set(i, LED_OFF);
      delay(500);
    }
#endif
  } else if(c == '8') {  // ADC Input
    uint16_t adc_in0, adc_in1, adc_in3, adc_in8;
    int adc_mvolt0, adc_mvolt1, adc_mvolt3, adc_mvolt8;

    analogReadResolution(12);

    pinMode(PIN_ADC_IN0, ANALOG);
    pinMode(PIN_ADC_IN1, ANALOG);
    pinMode(PIN_ADC_IN3, ANALOG);
    pinMode(PIN_ADC_IN8, ANALOG);

    adc_in0 = analogRead(PIN_ADC_IN0);
    adc_in1 = analogRead(PIN_ADC_IN1);
    adc_in3 = analogRead(PIN_ADC_IN3);
    adc_in8 = analogRead(PIN_ADC_IN8);

    adc_mvolt0 = analogReadMilliVolts(PIN_ADC_IN0);
    adc_mvolt1 = analogReadMilliVolts(PIN_ADC_IN1);
    adc_mvolt3 = analogReadMilliVolts(PIN_ADC_IN3);
    adc_mvolt8 = analogReadMilliVolts(PIN_ADC_IN8);

    Serial.printf("ADC Input: %d, %d, %d, %d\r\n", adc_in0, adc_in1, adc_in3, adc_in8);
    Serial.printf("ADC Input Voltage (mV): %d, %d, %d, %d\r\n", adc_mvolt0, adc_mvolt1, adc_mvolt3, adc_mvolt8);

  } else if(c == '9') {  // GPIO Input
    uint16_t gpio_in0, gpio_in1, gpio_in3, gpio_in8;

    pinMode(PIN_ADC_IN0, INPUT);
    pinMode(PIN_ADC_IN1, INPUT);
    pinMode(PIN_ADC_IN3, INPUT);
    pinMode(PIN_ADC_IN8, INPUT);

    gpio_in0 = digitalRead(PIN_ADC_IN0);
    gpio_in1 = digitalRead(PIN_ADC_IN1);
    gpio_in3 = digitalRead(PIN_ADC_IN3);
    gpio_in8 = digitalRead(PIN_ADC_IN8);

    Serial.printf("GPIO Input: %d, %d, %d, %d\r\n", gpio_in0, gpio_in1, gpio_in3, gpio_in8);
  } else if(c == 'a') {  // PCM_Control HIGH
    Serial.println("PCM Control to HIGH - AC Power-ON");
    //digitalWrite(PIN_PCM_CONTROL, HIGH);
  } else if(c == 'b') {  // PCM_Control LOW
    Serial.println("PCM Control to LOW - AC Power-OFF");
    //digitalWrite(PIN_PCM_CONTROL, LOW);
  } else if(c == 'c') {
    Serial.print("System will be restart in 10 seconds");
    for(int i=0; i<10; i++){
      Serial.print(".");
      delay(1000);
    }
    Serial.println();
    ESP.restart();
  } else {
    Serial.println("Invalid Test Number");
    return;
  }

}

void sub_test_n(void) {

  uint8_t data;
  int numBytes;
  char c;
  Serial.println("Sub-test N - W5500");
  prt_cmd_info_n();

  Serial.print("Input Test Number: ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)){
        break;
      }
    }
    delay(100);
  }
  Serial.println(c);

  if(c == '0') {  // 
    // W5500
    pinMode(PIN_ETH_CS, OUTPUT);
    pinMode(PIN_ETH_SCLK, OUTPUT);
    pinMode(PIN_ETH_MISO, INPUT);
    pinMode(PIN_ETH_MOSI, OUTPUT);
    digitalWrite(PIN_ETH_CS, HIGH);
    digitalWrite(PIN_ETH_SCLK, LOW);

    Serial.print("pCUR_SPI->pinSS(): "); Serial.println(pCUR_SPI->pinSS(), DEC);

    SPI.begin(PIN_ETH_SCLK, PIN_ETH_MISO, PIN_ETH_MOSI, PIN_ETH_CS);
    pinMode(SPI.pinSS(), OUTPUT);

    Ethernet.init(PIN_ETH_CS);

    Serial.print("pCUR_SPI->pinSS(): "); Serial.println(pCUR_SPI->pinSS(), DEC);

    Serial.print("w5500: "); Serial.println(w5500, DEC);
    Serial.print("getChip(): "); Serial.println(Ethernet.getChip(), DEC);

  } else if(c == '1') {  // 

    pinMode(SS, OUTPUT);
    pinMode(SCK, OUTPUT);
    pinMode(MISO, INPUT);
    pinMode(MOSI, OUTPUT);
    digitalWrite(SS, HIGH);
    digitalWrite(SCK, LOW);

    SPI.begin(SCK, MISO, MOSI, SS);
    //SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    SPI.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0));  
    //SPI.beginTransaction(SPISettings(80000000, MSBFIRST, SPI_MODE0));  
    digitalWrite(SS, LOW);
    for(int i=0; i < 20; i++) {
      SPI.transfer(0x55);
    }
    digitalWrite(SS, HIGH);
    SPI.endTransaction();

  } else if(c == '2') {  // 

    pinMode(SS, OUTPUT);
    digitalWrite(SS, HIGH);

    SPIClass* vspi = new SPIClass(HSPI);

    vspi->begin(PIN_ETH_SCLK, PIN_ETH_MISO, PIN_ETH_MOSI, PIN_ETH_CS);
    vspi->beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0));
    digitalWrite(SS, LOW);
    for(int i=0; i < 20; i++) {
      vspi->transfer(0x55);
    }
    digitalWrite(SS, HIGH);
    vspi->endTransaction();
    delete vspi;

  } else if(c == '3') {  // 
    Ethernet.init(PIN_ETH_CS);
    hspi->setFrequency(40000000);
    pCUR_SPI = hspi;
    Ethernet.begin(nc_mac, nc_ip, nc_dns, nc_gateway, nc_subnet);
    hspi->setFrequency(40000000);
    Ethernet._pinRST = PIN_W5500_RST;
    Ethernet._pinCS = PIN_ETH_CS;
    Ethernet.setHostname("INL05_001");
    Ethernet.setRetransmissionCount(3);
    Ethernet.setRetransmissionTimeout(4000);

    Serial.print("getChip(): "); Serial.println(Ethernet.getChip(), DEC);
    Serial.print("localIP(): "); Serial.println(Ethernet.localIP());

  } else if(c == '4') {
    Serial.print("linkStatus: "); Serial.println(Ethernet.linkStatus(), DEC); //LINK_ON, LINK_OFF
    Serial.print("PhyState: "); Serial.println(Ethernet.phyState(), HEX);
    Serial.print("HardwareStatus: "); Serial.println(Ethernet.hardwareStatus());
    Serial.print("speed: "); Serial.println(Ethernet.speed(), DEC);
    Serial.print("duplex: "); Serial.println(Ethernet.duplex(), DEC);
    Serial.print("localIP(): "); Serial.println(Ethernet.localIP());
    Serial.print("dnsServerIP: "); Serial.println(Ethernet.dnsServerIP());
    Serial.print("gatewayIP(): "); Serial.println(Ethernet.gatewayIP());
    Serial.print("subnetMask(): "); Serial.println(Ethernet.subnetMask());
    Serial.print("hostName(): "); Serial.println(Ethernet.hostName());
    Serial.print("_pinRST: "); Serial.println(Ethernet._pinRST);
    Serial.print("_pinCS: "); Serial.println(Ethernet._pinCS);
    Serial.print("_maxSockNum: "); Serial.println(Ethernet._maxSockNum);

  } else if(c == '5') {
    unsigned int localPort = 1883;
    char packetBuffer[255];
    int packetSize;
    int len;
    
    EthernetUDP Udp;
    Udp.begin(localPort);

    while(1) {
      packetSize = Udp.parsePacket();

      if (packetSize)
      {
        Serial.print(F("Received packet of size "));
        Serial.println(packetSize);
        Serial.print(F("From "));
        IPAddress remoteIp = Udp.remoteIP();
        Serial.print(remoteIp);
        Serial.print(F(", port "));
        Serial.println(Udp.remotePort());

        // read the packet into packetBufffer
        len = Udp.read(packetBuffer, 255);

        if (len > 0)
        {
          packetBuffer[len] = 0;
        }

        Serial.println(F("Contents:"));
        Serial.println(packetBuffer);
      }

      if(Serial.available()) {
        c = Serial.read();
        if((c == 'q') || (c == '#')) {
          Serial.println("Exit Udp Receive");
          break;
        }
      }
      delay(100);
    }
  } else if(c == '6') {
    unsigned int localPort = 8080;
    unsigned int serverPort = 1883;
    int ret;
    size_t wsize;
    //IPAddress server_ip(192, 168, 1, 149);
    IPAddress server_ip(192, 168, 1, 187);
    
    EthernetUDP Udp;
    Udp.begin(localPort);

    ret = Udp.beginPacket(server_ip, serverPort);
    Serial.print("Return of beginPacket: "); Serial.println(ret, DEC);
    wsize = Udp.write("hello from esp");
    Serial.print("Return of write: "); Serial.println(wsize);
    ret = Udp.endPacket();
    Serial.print("Return of endPacket: "); Serial.println(ret, DEC);

  } else if(c == '7') {
    dhcp->beginWithDHCP(nc_mac);
    Serial.print("localIP(): "); Serial.println(dhcp->getLocalIp());
    Ethernet.setLocalIP(dhcp->getLocalIp());
  } else if(c == '8') {
    Serial.print("DhcpServerIp(): "); Serial.println(dhcp->getDhcpServerIp());
    Serial.print("localIP(): "); Serial.println(dhcp->getLocalIp());
  } else if(c == '9') {
    unsigned int localPort = 8888;
    //const char timeServer[] = "time.nist.gov";
    const char timeServer[] = "time.windows.com";
    const int NTP_PACKET_SIZE = 48;
    byte packetBuffer[NTP_PACKET_SIZE];
    int inPacketSize = 0;
    int outPacketSize = 0;

    EthernetUDP Udp;
    Udp.begin(localPort);

    while(1) {

      // send NTP packet
      memset(packetBuffer, 0, NTP_PACKET_SIZE);
      packetBuffer[0] = 0b11100011;  // LI, Version, Mode
      packetBuffer[1] = 0;  // Stratum, or type of clock
      packetBuffer[2] = 6;  // Polling Interval
      packetBuffer[3] = 0xEC;  // Peer Clock Precision
      packetBuffer[12] = 49;
      packetBuffer[13] = 0x4E;
      packetBuffer[14] = 49;
      packetBuffer[15] = 52;

      Udp.beginPacket(timeServer, 123);
      outPacketSize = Udp.write(packetBuffer, NTP_PACKET_SIZE);
      Serial.printf("outPacketSize = %d\r\n", outPacketSize);
      Udp.endPacket();

      delay(1000);
      inPacketSize = Udp.parsePacket();
      Serial.printf("inPacketSize = %d\r\n", inPacketSize);
      if(inPacketSize) {
      //if(Udp.parsePacket()) {
        Udp.read(packetBuffer, NTP_PACKET_SIZE);

        unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
        unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);

        unsigned long secsSince1900 = highWord << 16 | lowWord;
        Serial.print("Seconds since Jan 1 1900 = ");
        Serial.println(secsSince1900);

        Serial.print("Unix time = ");
        const unsigned long seventyYears = 2208988800UL;
        unsigned long epoch = secsSince1900 - seventyYears;
        Serial.println(epoch);
        
        Serial.print("The UTC time is ");
        Serial.print((epoch % 86400L) / 3600);
        Serial.print(':');
        if(((epoch % 3600) / 60) < 10) {
          Serial.print('0');
        }
        Serial.print((epoch % 3600) / 60);
        Serial.print(':');
        if((epoch % 60) < 10) {
          Serial.print('0');
        }
        Serial.println(epoch % 60);

        time_t rawtime;
        struct tm ts;
        char buf[80];

        rawtime = epoch;
        ts = *localtime(&rawtime);
        strftime(buf, sizeof(buf), "%a %Y-%m-%d %H:%M:%S %Z", &ts);
        Serial.printf("NTP GMT Time is %s\r\n", buf);
        Serial.printf("NTP Local Time is: %d-%d-%d %d-%d-%d\r\n", 
          ts.tm_year + 1900, ts.tm_mon + 1, ts.tm_mday, ts.tm_hour + 9, ts.tm_min, ts.tm_sec);
      }
      delay(10000);
      Ethernet.maintain();

      if(Serial.available()) {
        c = Serial.read();
        if((c == 'q') || (c == '#')) {
          Serial.println("Exit NTP Receive Loop");
          break;
        }
      }

    }
    Udp.stop();

  } else {
    Serial.println("Invalid Test Number");
    return;
  }

}

void sub_test_o(void) {

  uint8_t uport = 1;
  uint8_t dtype = 0;
  char c;
  uint8_t data = 0;
  Serial.println("Sub-test O - UART TX");
  prt_cmd_info_o();

  Serial.print("Select Port (1:RS232, 2:RS485): ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)) break;
    }
    delay(100);
  }
  Serial.println(c);

  if(c == '1') {
    uport = 1;
    Serial1.flush();
  } else if(c == '2') {
    uport = 2;
    Serial2.flush();
    setRS485Dir(MAX485_DIR_SEND);
  } else {
    Serial.println("Invalid port number");
    return;
  }

  while(1) {
    Serial.println("Input data: ");
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        if(isalnum(c) || (c == '#')) break;
      }
      delay(100);
    }
    if(!isalnum(c)){
      Serial.println("Quit data input");
      break;
    }
    Serial.println(c);
    
    if(uport == 1) {
      Serial1.write(c);
    } else if(uport == 2) {
      Serial2.write(c);
    }

  }

}

void sub_test_p(void) {

  uint8_t uport = 1;
  uint8_t dtype = 0;
  char c;
  uint8_t data[128] = {0,};
  uint16_t length, rsize;
  Serial.println("Sub-test P - UART RX");
  prt_cmd_info_p();

  Serial.print("Select Port (1:RS232, 2:RS485): ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)) break;
    }
    delay(100);
  }
  Serial.println(c);

  if(c == '1') {
    uport = 1;
    Serial1.flush();
  } else if(c == '2') {
    uport = 2;
    Serial2.flush();
    setRS485Dir(MAX485_DIR_RECEIVE);
  } else {
    Serial.println("Invalid port number");
    return;
  }

  Serial.println("Received Data: ");

  while(1) {
    if(uport == 1) {
      length = Serial1.available();
      if(length) {
        if(length > 128) length = 128;
      }
      rsize = Serial1.read(data, length);
      for(int i=0; i < (int) rsize; i++) {
        Serial.println((char) data[i]);
      }
    } else if(uport == 2) {
      length = Serial2.available();
      if(length) {
        if(length > 128) length = 128;
      }
      rsize = Serial2.read(data, length);
      for(int i=0; i < (int) rsize; i++) {
        Serial.println((char) data[i]);
      }
    }

    if(Serial.available()) {
      c = Serial.read();
      if((c == 'q') || (c == '#')) {
        Serial.println("Exit Data Receive");
        break;
      }
    }
    delay(100);
  }

}

void sub_test_q(void) {

  uint8_t uport = 1;
  uint8_t d_in = 0;
  char c;
  uint8_t rdata[128] = {0,};
  uint8_t wdata[128] = {0,};
  uint16_t rlen, rsize;
  Serial.println("Sub-test Q - UART TX/RX");
  prt_cmd_info_q();

  Serial.print("Select Port (1:RS232, 2:RS232 TTL, 3:RS232_A, 4:RS232_B): ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)) break;
    }
    delay(100);
  }
  Serial.println(c);

  if(c == '1') {
    uport = 1;
    Serial1.flush();
  } else if(c == '2') {
    uport = 2;
    Serial2.flush();
    //setRS485Dir(MAX485_DIR_SEND);
  } else if(c == '3') {
    uport = 3;
  } else if(c == '4') {
    uport = 4;
  } else {
    Serial.println("Invalid port number");
    return;
  }

  Serial.println("Input data: ");
  while(1) {
    if(Serial.available()) {
      c = Serial.read();

      if(c == '#'){
        Serial.println("Quit data input");
        break;
      }
      if(isalnum(c)) {
        d_in = 1;
        Serial.print("d_in: "); Serial.println(c);
      }
    }

    if(uport == 1) {
      rlen = Serial1.available();
      if(rlen){
        if(rlen > 128) rlen = 128;
        rsize = Serial1.read(rdata, rlen);
        for(int i=0; i < (int) rsize; i++) {
          Serial.print("r_in["); Serial.print(i); Serial.print("]: "); Serial.println((char) rdata[i]);
        }
      } else if (d_in) {
        Serial1.write(c);
        d_in = 0;
      }
    } else if(uport == 2) {
      rlen = Serial2.available();
      if(rlen){
        if(rlen > 128) rlen = 128;
        rsize = Serial2.read(rdata, rlen);
        for(int i=0; i < (int) rsize; i++) {
          Serial.print("r_in["); Serial.print(i); Serial.print("]: "); Serial.println((char) rdata[i]);
        }
      } else if (d_in) {
        Serial2.write(c);
        d_in = 0;
      }
    } else if(uport == 3) {
      if(rlen){
        if(rlen > 128) rlen = 128;
        memset(rdata, 0x00, sizeof(rdata));
        rsize = serial0.readData(rdata, 1);
        for(int i=0; i < (int) rsize; i++) {
          Serial.print("r_in["); Serial.print(i); Serial.print("]: "); Serial.println((char) rdata[i]);
        }
        if(rsize){
          rlen -= rsize;
        }
      } else if (d_in) {
        memset(wdata, 0x00, sizeof(wdata));
        wdata[0] = c;
        serial0.writeData(wdata, 1);
        d_in = 0;
        rlen = 1;
      }
    } else if(uport == 4) {
      if(rlen){
        if(rlen > 128) rlen = 128;
        memset(rdata, 0x00, sizeof(rdata));
        rsize = serial1.readData(rdata, 1);
        for(int i=0; i < (int) rsize; i++) {
          Serial.print("r_in["); Serial.print(i); Serial.print("]: "); Serial.println((char) rdata[i]);
        }
        if(rsize){
          rlen -= rsize;
        }
      } else if (d_in) {
        memset(wdata, 0x00, sizeof(wdata));
        wdata[0] = c;
        serial1.writeData(wdata, 1);
        d_in = 0;
        rlen = 1;
      }
    }
    delay(100);
  }

}

void sub_test_r(void) {

  uint8_t data;
  char c;
  Serial.println("Sub-test R - EEPROM");
  prt_cmd_info_r();

  Serial.print("Input Test Number: ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      break;
    }
    delay(100);
  }
  Serial.println(c);

  if(c == '0') {
    for(int i=0; i<EEPROM_SIZE; i++){
      Serial.printf("Data[%d]: %02x\r\n", i, EEPROM.read(i));
    }
  } else if(c == '1') {
    for(int i=0; i<EEPROM_SIZE; i++){
      data = (uint8_t) (i % 256);
      EEPROM.write(i, data);
    }
    EEPROM.commit();
  } else {
    Serial.println("Invalid Test Number");
    return;
  }

}
