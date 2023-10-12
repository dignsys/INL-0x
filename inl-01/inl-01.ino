/* 
 * INL-01 Sample code
 * Author : DIGNSYS Inc.
 */
#include <Arduino.h>
#include <SPI.h>
#include <Ethernet_Generic.h>

#define PIN_LED             20
#define PIN_W5500_RST       40
#define PIN_ETH_CS          39
#define PIN_ETH_INT         38
#define PIN_ETH_MISO        37
#define PIN_ETH_SCLK        36
#define PIN_ETH_MOSI        35

uint8_t nc_mac[] = {
  0x00, 0x08, 0xDC, 0x00, 0x00, 0x00
//  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress nc_ip(192, 168, 1, 35);
IPAddress nc_dns(192, 168, 1, 1);
IPAddress nc_gateway(192, 168, 1, 1);
IPAddress nc_subnet(255, 255, 255, 0);

SPIClass* hspi;
DhcpClass* dhcp = new DhcpClass();

void sub_test_a(void);
void sub_test_n(void);

void setup() {

  Serial.begin(115200);

  pinMode(PIN_LED, OUTPUT);

  pinMode(PIN_W5500_RST, OUTPUT);
  digitalWrite(PIN_W5500_RST, LOW);
  delay(100);
  digitalWrite(PIN_W5500_RST, HIGH);
  delay(100);

  hspi = new SPIClass(HSPI);
  hspi->begin(PIN_ETH_SCLK, PIN_ETH_MISO, PIN_ETH_MOSI, PIN_ETH_CS);


}

void loop() {

  Serial.println();
  Serial.println("INL-01 testing.");
  Serial.println("(C) 2023 Dignsys"); Serial.println();

  char c;
  while(c != 'x') {
    Serial.printf("Input Command: ");
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        if(isalnum(c)){
          break;
        }
      }
      delay(100);
    }
    Serial.printf("%c", c);
    Serial.println();

    switch(c) {
      case 'a': 
        sub_test_a();
        break;
      case 'n':
        sub_test_n();
        break;
      default:
        break;
    }
  }
  Serial.println("loop exit!");

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
    if(c == 'q'){
      Serial.println("Quit loop");
      break;
    }
    if(led_status) {
      led_status = 0;
    } else {
      led_status = 1;
    }
    digitalWrite(PIN_LED, led_status);
    delay(1000);
  }
}

void sub_test_n(void) {

  uint8_t data;
  int numBytes;
  char c;
  Serial.println("Sub-test N - W5500");

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
#if 1  
    pinMode(SS, OUTPUT);
    pinMode(SCK, OUTPUT);
    pinMode(MISO, INPUT);
    pinMode(MOSI, OUTPUT);
    digitalWrite(SS, HIGH);
    digitalWrite(SCK, LOW);

    SPI.begin(SCK, MISO, MOSI, SS);
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(SS, LOW);
    for(int i=0; i < 20; i++) {
      SPI.transfer(0x55);
    }
    digitalWrite(SS, HIGH);
#else
    SPI.endTransaction();
    pinMode(PIN_ETH_CS, OUTPUT);
    pinMode(PIN_ETH_SCLK, OUTPUT);
    pinMode(PIN_ETH_MISO, INPUT);
    pinMode(PIN_ETH_MOSI, OUTPUT);
    digitalWrite(PIN_ETH_CS, HIGH);
    digitalWrite(PIN_ETH_SCLK, LOW);

    SPI.begin(PIN_ETH_SCLK, PIN_ETH_MISO, PIN_ETH_MOSI, PIN_ETH_CS);
    pinMode(SPI.pinSS(), OUTPUT);
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(PIN_ETH_CS, LOW);
    for(int i=0; i < 20; i++) {
      SPI.transfer(0x55);
    }
    digitalWrite(PIN_ETH_CS, HIGH);
    SPI.endTransaction();
#endif
  } else if(c == '2') {  // 
#if 1
    pinMode(SS, OUTPUT);
    digitalWrite(SS, HIGH);

    SPIClass* vspi = new SPIClass(HSPI);

    //vspi->begin();
    //vspi->begin(41, 42, 40, 39);
    vspi->begin(PIN_ETH_SCLK, PIN_ETH_MISO, PIN_ETH_MOSI, PIN_ETH_CS);
    vspi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(SS, LOW);
    for(int i=0; i < 20; i++) {
      vspi->transfer(0x55);
    }
    digitalWrite(SS, HIGH);
    vspi->endTransaction();
    delete vspi;
#endif
  } else if(c == '3') {  // 
    Ethernet.init(PIN_ETH_CS);
#if 0
    Serial.println("ETH begin start");
    Ethernet.begin(nc_mac, hspi);
    Serial.println("ETH begin end");
    Ethernet.setLocalIP(nc_ip);
    Ethernet.setDnsServerIP(nc_dns);
    Ethernet.setGatewayIP(nc_gateway);
    Ethernet.setSubnetMask(nc_subnet);
#else
    pCUR_SPI = hspi;
    Ethernet.begin(nc_mac, nc_ip, nc_dns, nc_gateway, nc_subnet);
    Ethernet._pinRST = PIN_W5500_RST;
    Ethernet._pinCS = PIN_ETH_CS;
    Ethernet.setHostname("INL-01_001");
    Ethernet.setRetransmissionCount(3);
    Ethernet.setRetransmissionTimeout(4000);
#endif

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
        if(c == 'q') {
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
    
  } else {
    Serial.println("Invalid Test Number");
    return;
  }

}
