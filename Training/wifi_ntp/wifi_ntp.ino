#include "WiFi.h"
#define NTP_SERVER  "pool.ntp.org"
#define UTC_OFFSET  0
#define UTC_OFFSET_DST  0

void printLocalTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Connection Err");
    return;
  }

  Serial.print(&timeinfo, "%H:%M:%S ");
  Serial.println(&timeinfo, "%d/%m/%Y  %Z");
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.print("Connecting to WiFi");
  WiFi.begin("your_ap", "your_password");
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("RRSI: ");
  Serial.println(WiFi.RSSI());

  configTime(UTC_OFFSET, UTC_OFFSET_DST, NTP_SERVER);
}

void loop() {
  // put your main code here, to run repeatedly:
  printLocalTime();
  delay(2000);
}
