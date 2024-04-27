/* 
 * INL-01 Main 
 * Author : DIGNSYS Inc.
 */

#define PIN_LED             20

#define PIN_UART_RX0        18
#define PIN_UART_TX0        17
#define PIN_UART_RX1        48
#define PIN_UART_TX1        47
#define PIN_BOOT            0

void setup() {

  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, PIN_UART_RX0, PIN_UART_TX0);  // RS232
  Serial2.begin(115200, SERIAL_8N1, PIN_UART_RX1, PIN_UART_TX1);  // RS232 TTL

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
}

void loop() {
  uint8_t rdata[128] = {0,};
  uint16_t rlen, rsize;

  Serial.println();
  Serial.println("Send test packet");

  Serial1.print("RS232 packet test");
  Serial1.flush();
  delay(100);
  rlen = Serial1.available();
  if(rlen){
    if(rlen > 128) rlen = 128;
     rsize = Serial1.read(rdata, rlen);
     for(int i=0; i < (int) rsize; i++) {
       Serial.print("r_in["); Serial.print(i); Serial.print("]: "); Serial.println((char) rdata[i]);
     }
  }
  delay(3000);
}