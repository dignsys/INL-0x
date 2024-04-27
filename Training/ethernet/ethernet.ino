/* 
 * INL-01 Main 
 * Author : DIGNSYS Inc.
 */
#include <SPI.h>
#include <Ethernet_Generic.h>

#define VERSION_INL01_FW    "20240418"
#define PIN_LED             20
#define PIN_W5500_RST       40
#define PIN_ETH_CS          39
#define PIN_ETH_INT         38
#define PIN_ETH_MISO        37
#define PIN_ETH_SCLK        36
#define PIN_ETH_MOSI        35
SPIClass* hspi;

uint8_t nc_mac[] = {  0x00, 0x08, 0xDC, 0x00, 0x00, 0x00 };
IPAddress nc_ip(192, 168, 0, 23);
IPAddress nc_dns(192, 168, 0, 1);
IPAddress nc_gateway(192, 168, 0, 1);
IPAddress nc_subnet(255, 255, 255, 0);
DhcpClass* dhcp = new DhcpClass();

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

  Serial.println();
  Serial.println("INL-01 testing.");
  Serial.println("(C) 2023 Dignsys");
  Serial.printf("VERSION: %s\r\n\r\n", VERSION_INL01_FW);
}

void loop() {
  digitalWrite(PIN_LED, LOW); // Status LED ON
  Serial.println("*** W5500 Ethernet Controller ***");
  Serial.println("* Initialize Ethernet Interface with SPI");
  Ethernet.init(PIN_ETH_CS);
  hspi->setFrequency(40000000);
  pCUR_SPI = hspi;
  Ethernet.begin(nc_mac, nc_ip, nc_dns, nc_gateway, nc_subnet);
  hspi->setFrequency(40000000);
  Ethernet._pinRST = PIN_W5500_RST;
  Ethernet._pinCS = PIN_ETH_CS;
  Ethernet.setHostname("INL-01_001");
  Ethernet.setRetransmissionCount(3);
  Ethernet.setRetransmissionTimeout(4000);

  Serial.println("* Display chup information and saved IP address");
  Serial.print("\tgetChip(): "); Serial.println(Ethernet.getChip(), DEC);
  Serial.print("\tlocalIP(): "); Serial.println(Ethernet.localIP());

  delay(1000);

  Serial.println("* Display network port status");
  Serial.print("\tlinkStatus: "); Serial.println(Ethernet.linkStatus(), DEC); //LINK_ON, LINK_OFF
  Serial.print("\tPhyState: "); Serial.println(Ethernet.phyState(), HEX);
  Serial.print("\tHardwareStatus: "); Serial.println(Ethernet.hardwareStatus());
  Serial.print("\tspeed: "); Serial.println(Ethernet.speed(), DEC);
  Serial.print("\tduplex: "); Serial.println(Ethernet.duplex(), DEC);
  Serial.print("\tlocalIP(): "); Serial.println(Ethernet.localIP());
  Serial.print("\tdnsServerIP: "); Serial.println(Ethernet.dnsServerIP());
  Serial.print("\tgatewayIP(): "); Serial.println(Ethernet.gatewayIP());
  Serial.print("\tsubnetMask(): "); Serial.println(Ethernet.subnetMask());
  Serial.print("\thostName(): "); Serial.println(Ethernet.hostName());
  Serial.print("\t_pinRST: "); Serial.println(Ethernet._pinRST);
  Serial.print("\t_pinCS: "); Serial.println(Ethernet._pinCS);
  Serial.print("\t_maxSockNum: "); Serial.println(Ethernet._maxSockNum);
  delay(1000);

  Serial.println("* Get IP address from DHCP server");
  dhcp->beginWithDHCP(nc_mac);
  Serial.print("\tDhcpServerIp(): "); Serial.println(dhcp->getDhcpServerIp());
  Serial.print("\tlocalIP(): "); Serial.println(dhcp->getLocalIp());
  Ethernet.setLocalIP(dhcp->getLocalIp());
  digitalWrite(PIN_LED, HIGH); // Status LED OFF
  delay(5000);
}