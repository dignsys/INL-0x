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

uint8_t nc_mac[] = {  0x00, 0x08, 0xDC, 0x00, 0x00, 0x01 };
IPAddress nc_ip(192, 168, 0, 23);
IPAddress nc_dns(192, 168, 0, 1);
IPAddress nc_gateway(192, 168, 0, 1);
IPAddress nc_subnet(255, 255, 255, 0);
DhcpClass* dhcp = new DhcpClass();

char timeServer[]         = "time.nist.gov";  // NTP server
unsigned int localPort    = 2390;             // local port to listen for UDP packets
const int NTP_PACKET_SIZE = 48;       // NTP timestamp is in the first 48 bytes of the message
const int UDP_TIMEOUT     = 2000;     // timeout in miliseconds to wait for an UDP packet to arrive
byte packetBuffer[NTP_PACKET_SIZE];   // buffer to hold incoming and outgoing packets

// A UDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

// send an NTP request to the time server at the given address
void sendNTPpacket(char *ntpSrv)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)

  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(ntpSrv, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

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

  Serial.println("Initialize Ethernet Interface with SPI");
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

  delay(1000);
  Serial.println("Display Ethernet Link status");
  Serial.print("\tgetChip(): "); Serial.println(Ethernet.getChip(), DEC);
  Serial.print("\tlinkStatus: "); Serial.println(Ethernet.linkReport());
  Serial.print("\tspeed: "); Serial.println(Ethernet.speedReport());
  Serial.print("\tduplex: "); Serial.println(Ethernet.duplexReport());
  Serial.print("\tlocalIP(): "); Serial.println(Ethernet.localIP());
  Udp.begin(localPort);
}

void loop() {
  Serial.println();
  sendNTPpacket(timeServer); // send an NTP packet to a time server

  // wait for a reply for UDP_TIMEOUT miliseconds
  unsigned long startMs = millis();

  while (!Udp.available() && (millis() - startMs) < UDP_TIMEOUT) {}

  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();

  if (packetSize)
  {
    Serial.print(F("UDP Packet received, size "));
    Serial.println(packetSize);
    Serial.print(F("From "));
    IPAddress remoteIp = Udp.remoteIP();
    Serial.print(remoteIp);
    Serial.print(F(", port "));
    Serial.println(Udp.remotePort());

    // We've received a packet, read the data from it into the buffer
    Udp.read(packetBuffer, NTP_PACKET_SIZE);

    // the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:
    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);

    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;

    Serial.print(F("Seconds since Jan 1 1900 = "));
    Serial.println(secsSince1900);

    // now convert NTP time into )everyday time:
    Serial.print(F("Unix time = "));
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;
    // print Unix time:
    Serial.println(epoch);

    // print the hour, minute and second:
    Serial.print(F("The UTC time is "));    // UTC is the time at Greenwich Meridian (GMT)
    Serial.print((epoch  % 86400L) / 3600 + 9); // print the hour (86400 equals secs per day)
                                                // GMT+9
    Serial.print(F(":"));

    if (((epoch % 3600) / 60) < 10)
    {
      // In the first 10 minutes of each hour, we'll want a leading '0'
      Serial.print(F("0"));
    }

    Serial.print((epoch  % 3600) / 60); // print the minute (3600 equals secs per minute)
    Serial.print(F(":"));

    if ((epoch % 60) < 10)
    {
      // In the first 10 seconds of each minute, we'll want a leading '0'
      Serial.print(F("0"));
    }

    Serial.println(epoch % 60); // print the second
  }

  // wait ten seconds before asking for the time again
  delay(10000);
}