/* 
 * INL-01 Main 
 * Author : DIGNSYS Inc.
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Ethernet_Generic.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <LittleFS.h>
#include <WebServer.h>
#include <ESPmDNS.h>

#define VERSION_INL01_FW  "20240412"

#define PIN_LED             20
#define PIN_W5500_RST       40
#define PIN_ETH_CS          39
#define PIN_ETH_INT         38
#define PIN_ETH_MISO        37
#define PIN_ETH_SCLK        36
#define PIN_ETH_MOSI        35
#define PIN_I2C0_SDA        15
#define PIN_I2C0_SCL        16
#define PIN_I2C1_SDA        42
#define PIN_I2C1_SCL        41
#define PIN_UART_RX0        18
#define PIN_UART_TX0        17
#define PIN_UART_RX1        48
#define PIN_UART_TX1        47
#define PIN_BOOT            0
#define PIN_ADC_IN0         1
#define PIN_ADC_IN1         2
#define PIN_ADC_IN3         4
#define PIN_ADC_IN4         5
#define PIN_ADC_IN5         6
#define PIN_ADC_IN6         7
#define PIN_ADC_IN7         8
#define PIN_ADC_IN8         19

uint8_t nc_mac[] = {
  0x00, 0x08, 0xDC, 0x00, 0x00, 0x00
};
IPAddress nc_ip(192, 168, 1, 35);
IPAddress nc_dns(192, 168, 1, 1);
IPAddress nc_gateway(192, 168, 1, 1);
IPAddress nc_subnet(255, 255, 255, 0);

SPIClass* hspi;
DhcpClass* dhcp = new DhcpClass();

#define EEPROM_SIZE 512

//#define WIFI_FS_TEST

const char* ssid = "********";
const char* password = "********";

WebServer server(80);
volatile bool filesystemOK = false;

//holds the current upload
File fsUploadFile;
String FileName;

String HTML = "<form method='post' action='/upload' enctype='multipart/form-data'><input type='file' name='upload'><input type='submit' value='Upload'></form>";  

void sub_test_a(void);    // LED Test
void sub_test_b(void);    // Button Test
void sub_test_m(void);    // IO & ADC
void sub_test_n(void);    // W5500 Network Function Test
void sub_test_o(void);    // UART Function Test
void sub_test_r(void);    // EEPROM Test
void sub_test_s(void);    // LittleFS & Web

void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\r\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("- failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println(" - not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.path(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("\tSIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void createDir(fs::FS &fs, const char * path){
    Serial.printf("Creating Dir: %s\n", path);
    if(fs.mkdir(path)){
        Serial.println("Dir created");
    } else {
        Serial.println("mkdir failed");
    }
}

void removeDir(fs::FS &fs, const char * path){
    Serial.printf("Removing Dir: %s\n", path);
    if(fs.rmdir(path)){
        Serial.println("Dir removed");
    } else {
        Serial.println("rmdir failed");
    }
}

void readFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\r\n", path);

    File file = fs.open(path);
    if(!file || file.isDirectory()){
        Serial.println("- failed to open file for reading");
        return;
    }

    Serial.println("- read from file:");
    while(file.available()){
        Serial.write(file.read());
    }
    file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\r\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("- failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("- file written");
    } else {
        Serial.println("- write failed");
    }
    file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\r\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("- failed to open file for appending");
        return;
    }
    if(file.print(message)){
        Serial.println("- message appended");
    } else {
        Serial.println("- append failed");
    }
    file.close();
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
    Serial.printf("Renaming file %s to %s\r\n", path1, path2);
    if (fs.rename(path1, path2)) {
        Serial.println("- file renamed");
    } else {
        Serial.println("- rename failed");
    }
}

void deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\r\n", path);
    if(fs.remove(path)){
        Serial.println("- file deleted");
    } else {
        Serial.println("- delete failed");
    }
}

// SPIFFS-like write and delete file, better use #define CONFIG_LITTLEFS_SPIFFS_COMPAT 1
void writeFile2(fs::FS &fs, const char * path, const char * message){
    if(!fs.exists(path)){
		if (strchr(path, '/')) {
            Serial.printf("Create missing folders of: %s\r\n", path);
			char *pathStr = strdup(path);
			if (pathStr) {
				char *ptr = strchr(pathStr, '/');
				while (ptr) {
					*ptr = 0;
					fs.mkdir(pathStr);
					*ptr = '/';
					ptr = strchr(ptr+1, '/');
				}
			}
			free(pathStr);
		}
    }

    Serial.printf("Writing file to: %s\r\n", path);
    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("- failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("- file written");
    } else {
        Serial.println("- write failed");
    }
    file.close();
}

void deleteFile2(fs::FS &fs, const char * path){
    Serial.printf("Deleting file and empty folders on path: %s\r\n", path);

    if(fs.remove(path)){
        Serial.println("- file deleted");
    } else {
        Serial.println("- delete failed");
    }

    char *pathStr = strdup(path);
    if (pathStr) {
        char *ptr = strrchr(pathStr, '/');
        if (ptr) {
            Serial.printf("Removing all empty folders on path: %s\r\n", path);
        }
        while (ptr) {
            *ptr = 0;
            fs.rmdir(pathStr);
            ptr = strrchr(pathStr, '/');
        }
        free(pathStr);
    }
}

void testFileIO(fs::FS &fs, const char * path){
    Serial.printf("Testing file I/O with %s\r\n", path);

    static uint8_t buf[512];
    size_t len = 0;
    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("- failed to open file for writing");
        return;
    }

    size_t i;
    Serial.print("- writing" );
    uint32_t start = millis();
    for(i=0; i<2048; i++){
        if ((i & 0x001F) == 0x001F){
          Serial.print(".");
        }
        file.write(buf, 512);
    }
    Serial.println("");
    uint32_t end = millis() - start;
    Serial.printf(" - %u bytes written in %u ms\r\n", 2048 * 512, end);
    file.close();

    file = fs.open(path);
    start = millis();
    end = start;
    i = 0;
    if(file && !file.isDirectory()){
        len = file.size();
        size_t flen = len;
        start = millis();
        Serial.print("- reading" );
        while(len){
            size_t toRead = len;
            if(toRead > 512){
                toRead = 512;
            }
            file.read(buf, toRead);
            if ((i++ & 0x001F) == 0x001F){
              Serial.print(".");
            }
            len -= toRead;
        }
        Serial.println("");
        end = millis() - start;
        Serial.printf("- %u bytes read in %u ms\r\n", flen, end);
        file.close();
    } else {
        Serial.println("- failed to open file for reading");
    }
}

void InitFilesystem() {
  // Initialize LittleFS
  if (!LittleFS.begin(false /* false: Do not format if mount failed */)) {
    Serial.println("Failed to mount LittleFS");
    if (!LittleFS.begin(true /* true: format */)) {
      Serial.println("Failed to format LittleFS");
    } else {
      Serial.println("LittleFS formatted successfully");
      filesystemOK = true;
    }
  } else { // Initial mount success
    filesystemOK = true;
    Serial.println("LittleFS Initial mount success");
    listDir(LittleFS, "/", 3);
    //readFile(LittleFS, "/mydir/hello1.txt");
    //readFile(LittleFS, "/mydir/hello2.txt");
  }
}

size_t LittleFSFilesize(const char* filename) {
  auto file = LittleFS.open(filename, "r");
  size_t filesize = file.size();
  // Don't forget to clean up!
  file.close();
  return filesize;
}

std::string ReadFileToString(const char* filename) {
  auto file = LittleFS.open(filename, "r");
  size_t filesize = file.size();
  // Read into temporary Arduino String
  String data = file.readString();
  // Don't forget to clean up!
  file.close();
  return std::string(data.c_str(), data.length());
}

//format bytes
String formatBytes(size_t bytes) {
  if (bytes < 1024) {
    return String(bytes) + "B";
  } else if (bytes < (1024 * 1024)) {
    return String(bytes / 1024.0) + "KB";
  } else if (bytes < (1024 * 1024 * 1024)) {
    return String(bytes / 1024.0 / 1024.0) + "MB";
  } else {
    return String(bytes / 1024.0 / 1024.0 / 1024.0) + "GB";
  }
}

String getContentType(String filename) {
  if (server.hasArg("download")) {
    return "application/octet-stream";
  } else if (filename.endsWith(".htm")) {
    return "text/html";
  } else if (filename.endsWith(".html")) {
    return "text/html";
  } else if (filename.endsWith(".css")) {
    return "text/css";
  } else if (filename.endsWith(".js")) {
    return "application/javascript";
  } else if (filename.endsWith(".png")) {
    return "image/png";
  } else if (filename.endsWith(".gif")) {
    return "image/gif";
  } else if (filename.endsWith(".jpg")) {
    return "image/jpeg";
  } else if (filename.endsWith(".ico")) {
    return "image/x-icon";
  } else if (filename.endsWith(".xml")) {
    return "text/xml";
  } else if (filename.endsWith(".pdf")) {
    return "application/x-pdf";
  } else if (filename.endsWith(".zip")) {
    return "application/x-zip";
  } else if (filename.endsWith(".gz")) {
    return "application/x-gzip";
  } else if (filename.endsWith(".png")) {
    return "image/wav";
  }
  return "text/plain";
}

bool exists(String path){
  bool yes = false;  
  File file = LittleFS.open(path, "r");
  if(!file.isDirectory()){
    yes = true;
  }
  file.close();
  return yes;
}

bool handleFileRead(String path) {
  Serial.println("handleFileRead: " + path);
  if (path.endsWith("/")) {
    path += "index.htm";
  }
  String contentType = getContentType(path);
  String pathWithGz = path + ".gz";
  if (exists(pathWithGz) || exists(path)) {
    if (exists(pathWithGz)) {
      path += ".gz";
    }
    File file = LittleFS.open(path, "r");
    server.streamFile(file, contentType);
    file.close();
    return true;
  }
  return false;
}

void handleFileUpload() {  
  if (server.uri() != "/upload") {
    return;
  }
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    String filename = upload.filename;
    FileName = upload.filename;
    if (!filename.startsWith("/")) {
      filename = "/" + filename;
    }
    Serial.print("handleFileUpload Name: "); Serial.println(filename);
    fsUploadFile = LittleFS.open(filename, "w");
    filename = String();
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    Serial.print("handleFileUpload Data: "); Serial.println(upload.currentSize);
    if (fsUploadFile) {
      fsUploadFile.write(upload.buf, upload.currentSize);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (fsUploadFile) {
      fsUploadFile.close();
    }
    Serial.print("handleFileUpload Size: "); Serial.println(upload.totalSize);
    server.send(200, "text/html", "<h1>Completed, " + FileName + "!</h1><p>Upload successful</p>");
  }
}

void handleFileDelete() {
  Serial.println("handleFileDelete");
  if (server.args() == 0) {
    return server.send(500, "text/plain", "BAD ARGS");
  }
  String path = server.arg(0);
  Serial.println("handleFileDelete: " + path);
  if (path == "/") {
    return server.send(500, "text/plain", "BAD PATH");
  }
  if (!exists(path)) {
    return server.send(404, "text/plain", "FileNotFound");
  }
  LittleFS.remove(path);
  server.send(200, "text/plain", "");
  path = String();
}

void handleFileCreate() {
  Serial.println("handleFileCreate");
  if (server.args() == 0) {
    return server.send(500, "text/plain", "BAD ARGS");
  }
  String path = server.arg(0);
  Serial.println("handleFileCreate: " + path);
  if (path == "/") {
    return server.send(500, "text/plain", "BAD PATH");
  }
  if (exists(path)) {
    return server.send(500, "text/plain", "FILE EXISTS");
  }
  File file = LittleFS.open(path, "w");
  if (file) {
    file.close();
  } else {
    return server.send(500, "text/plain", "CREATE FAILED");
  }
  server.send(200, "text/plain", "");
  path = String();
}

  /*
   * Example : http://192.168.0.132/list?dir=/mydir
   *           Arg = dir, Value=/mydir
   */
void handleFileList() {  
  if (!server.hasArg("dir")) {
    server.send(500, "text/plain", "BAD ARGS");
    return;
  }

  String path = server.arg("dir");
  Serial.println("handleFileList: " + path);

  File root = LittleFS.open(path);
  path = String();

  String output = "[";
  if(root.isDirectory()){
      File file = root.openNextFile();
      while(file){
          if (output != "[") {
            output += ',';
          }
          output += "{\"type\":\"";
          output += (file.isDirectory()) ? "dir" : "file";
          output += "\",\"name\":\"";
          output += String(file.path()).substring(1);
          output += "\"}";
          file = root.openNextFile();
      }
  }
  output += "]";
  server.send(200, "text/json", output);
}

void handleRoot() {
  //server.send(200, "text/plain", "Hello from esp32!");
  server.send(200, "text/html", HTML);
}

void handleNotFound() {
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
}

void setup() {

  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, PIN_UART_RX0, PIN_UART_TX0);  // RS232
  Serial2.begin(115200, SERIAL_8N1, PIN_UART_RX1, PIN_UART_TX1);  // RS232 TTL

  pinMode(PIN_LED, OUTPUT);

  pinMode(PIN_W5500_RST, OUTPUT);
  digitalWrite(PIN_W5500_RST, LOW);
  delay(100);
  digitalWrite(PIN_W5500_RST, HIGH);
  delay(100);

  hspi = new SPIClass(HSPI);
  hspi->begin(PIN_ETH_SCLK, PIN_ETH_MISO, PIN_ETH_MOSI, PIN_ETH_CS);

  Wire.begin(PIN_I2C0_SDA, PIN_I2C0_SCL, 400000);
  Wire1.begin(PIN_I2C1_SDA, PIN_I2C1_SCL, 400000);

  // initialize EEPROM with predefined size
  EEPROM.begin(EEPROM_SIZE);

#ifdef WIFI_FS_TEST

  InitFilesystem();

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);        
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  if (MDNS.begin("esp32")) {
    Serial.println("MDNS responder started");
  }

  server.on("/", handleRoot);

  //list directory
  server.on("/list", HTTP_GET, handleFileList);

  //load editor
  server.on("/edit", HTTP_GET, []() {
    if (!handleFileRead("/edit.htm")) {
      server.send(404, "text/plain", "FileNotFound");
    }
  });

  //create file
  server.on("/edit", HTTP_PUT, handleFileCreate);

  //delete file
  server.on("/edit", HTTP_DELETE, handleFileDelete);

  //first callback is called after the request has ended with all parsed arguments
  //second callback handles file uploads at that location
  server.on("/upload", HTTP_POST, []() {
    server.send(200, "text/plain", "");    
  }, handleFileUpload);

  server.on("/inline", []() {
    server.send(200, "text/plain", "this works as well");
  });

  server.onNotFound(handleNotFound);

  //get heap status, analog input value and all GPIO statuses in one json call
  server.on("/all", HTTP_GET, []() {
    String json = "{";
    json += "\"heap\":" + String(ESP.getFreeHeap());
    json += ", \"analog\":" + String(analogRead(A0));
    json += ", \"gpio\":" + String((uint32_t)(0));
    json += "}";
    server.send(200, "text/json", json);
    json = String();
  });

  server.begin();
  Serial.println("HTTP server started");
#endif

}

void loop() {

  Serial.println();
  Serial.println("INL-01 testing.");
  Serial.println("(C) 2024 Dignsys");
  Serial.printf("VERSION: %s\r\n\r\n", VERSION_INL01_FW);

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
      case 'b': 
        sub_test_b();
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
      case 'r':
        sub_test_r();
        break;
      case 's':
        sub_test_s();
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
      Serial.println("LED Off");
    } else {
      led_status = 1;
      Serial.println("LED On");
    }
    digitalWrite(PIN_LED, led_status);
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
      Serial.println(c);
    }
    if(c == 'q'){
      Serial.println("Quit loop");
      break;
    }
    button_status = digitalRead(PIN_BOOT);
    Serial.printf("Button: %d\r\n", button_status);
    delay(1000);
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
  Serial.println("Sub-test M - IO & ADC");

  Serial.print("Input Test Number: ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)) break;
    }
    delay(100);
  }
  Serial.println(c);

  if(c == '0') {  // ADC Input
    uint16_t adc_in0, adc_in1;
    int adc_mvolt0, adc_mvolt1;

    analogReadResolution(12);

    pinMode(PIN_ADC_IN0, ANALOG);
    pinMode(PIN_ADC_IN1, ANALOG);

    adc_in0 = analogRead(PIN_ADC_IN0);
    adc_in1 = analogRead(PIN_ADC_IN1);

    adc_mvolt0 = analogReadMilliVolts(PIN_ADC_IN0);
    adc_mvolt1 = analogReadMilliVolts(PIN_ADC_IN1);

    Serial.printf("ADC Input: %d, %d\r\n", adc_in0, adc_in1);
    Serial.printf("ADC Input Voltage (mV): %d, %d\r\n", adc_mvolt0, adc_mvolt1);

  } else if(c == '1') {  // GPIO Input
    uint16_t gpio_in0, gpio_in1;

    pinMode(PIN_ADC_IN0, INPUT);
    pinMode(PIN_ADC_IN1, INPUT);

    gpio_in0 = digitalRead(PIN_ADC_IN0);
    gpio_in1 = digitalRead(PIN_ADC_IN1);

    Serial.printf("GPIO Input: %d, %d\r\n", gpio_in0, gpio_in1);

  } else if(c == '2') {  // ADC Input
    uint16_t adc_in3, adc_in4, adc_in5, adc_in6, adc_in7, adc_in8;

    analogReadResolution(12);

    pinMode(PIN_ADC_IN3, ANALOG);
    pinMode(PIN_ADC_IN4, ANALOG);
    pinMode(PIN_ADC_IN5, ANALOG);
    pinMode(PIN_ADC_IN6, ANALOG);
    pinMode(PIN_ADC_IN7, ANALOG);
    pinMode(PIN_ADC_IN8, ANALOG);

    adc_in3 = analogRead(PIN_ADC_IN3);
    adc_in4 = analogRead(PIN_ADC_IN4);
    adc_in5 = analogRead(PIN_ADC_IN5);
    adc_in6 = analogRead(PIN_ADC_IN6);
    adc_in7 = analogRead(PIN_ADC_IN7);
    adc_in8 = analogRead(PIN_ADC_IN8);

    Serial.printf("ADC Input: %d, %d, %d, %d, %d, %d\r\n", adc_in3, adc_in4, adc_in5, adc_in6, adc_in7, adc_in8);

  } else if(c == '3') {  // GPIO Input
    uint16_t gpio_in3, gpio_in4, gpio_in5, gpio_in6, gpio_in7, gpio_in8;

    pinMode(PIN_ADC_IN3, INPUT);
    pinMode(PIN_ADC_IN4, INPUT);
    pinMode(PIN_ADC_IN5, INPUT);
    pinMode(PIN_ADC_IN6, INPUT);
    pinMode(PIN_ADC_IN7, INPUT);
    pinMode(PIN_ADC_IN8, INPUT);

    gpio_in3 = digitalRead(PIN_ADC_IN3);
    gpio_in4 = digitalRead(PIN_ADC_IN4);
    gpio_in5 = digitalRead(PIN_ADC_IN5);
    gpio_in6 = digitalRead(PIN_ADC_IN6);
    gpio_in7 = digitalRead(PIN_ADC_IN7);
    gpio_in8 = digitalRead(PIN_ADC_IN8);

    Serial.printf("GPIO Input: %d, %d, %d, %, %d, %d\r\n", gpio_in3, gpio_in4, gpio_in5, gpio_in6, gpio_in7, gpio_in8);

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
    //SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    SPI.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0));  
    //SPI.beginTransaction(SPISettings(80000000, MSBFIRST, SPI_MODE0));  
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

    vspi->begin(PIN_ETH_SCLK, PIN_ETH_MISO, PIN_ETH_MOSI, PIN_ETH_CS);
    vspi->beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0));
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
    hspi->setFrequency(40000000);
    pCUR_SPI = hspi;
    Ethernet.begin(nc_mac, nc_ip, nc_dns, nc_gateway, nc_subnet);
    hspi->setFrequency(40000000);
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

void sub_test_o(void) {

  uint8_t uport = 1;
  uint8_t d_in = 0;
  char c;
  uint8_t rdata[128] = {0,};
  uint16_t rlen, rsize;
  Serial.println("Sub-test O - UART TX/RX");

  Serial.print("Select Port (1:RS232, 2:RS232 TTL): ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      break;
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
  } else {
    Serial.println("Invalid port number");
    return;
  }

  Serial.println("Input data: ");
  while(1) {
    if(Serial.available()) {
      c = Serial.read();
      d_in = 1;

      if(!isalnum(c)){
        Serial.println("Quit data input");
        break;
      }
      Serial.print("d_in: "); Serial.println(c);
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
    }
    delay(100);
  }

}

void sub_test_r(void) {

  uint8_t data;
  char c;
  Serial.println("Sub-test R - EEPROM");

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

void sub_test_s(void) {

  uint8_t data;
  char c;
  int r_data = 0;
  Serial.println("Sub-test S - LiitleFS & Web");

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
    LittleFS.begin();
    LittleFS.format();
  } else if(c == '1') {
    writeFile(LittleFS, "/data.txt", "My ESP32 Data\r\n");
    readFile(LittleFS, "/data.txt");
  } else if(c == '2') {
    r_data = random(0, 1000);
    appendFile(LittleFS, "/data.txt", (String(r_data)+ "\r\n").c_str());
    readFile(LittleFS, "/data.txt");
  } else if(c == '3') {
    File file = LittleFS.open("/data.txt");
    if(!file || file.isDirectory()){
        Serial.println("- failed to open file for reading");
        return;
    }

    Serial.println("- read from file:");
#if 0
    while(file.available()){
        Serial.printf("%02x\r\n", file.read());  // 0x0d, 0x0a
    }
#elif 1
    int i=0;
    while(file.available()){
        Serial.printf("[%d]:%s\r\n", i++, file.readStringUntil(0x0d));
        file.read();
    }
#endif
    file.close();

  } else if(c == '4') {
    Serial.println("Enter data:");
#if 0
    while (Serial.available() == 0) {}     //wait for data available
    String teststr = Serial.readString();  //read until timeout
    //String teststr = Serial.readStringUntil();  //read until timeout
    teststr.trim();                        // remove any \r \n whitespace at the end of the String
    if (teststr == "red") {
      Serial.println("A primary color");
    } else {
      Serial.println("Something else");
    }
#elif 1
    char cbuf[256] = {0,};
    int idx = 0;
    while(1){
      if(Serial.available()) {
        cbuf[idx] = Serial.read();
        Serial.print(cbuf[idx]);
        if(cbuf[idx]=='\n') {
          cbuf[idx] = 0;
          break;
        }
        idx++;
      }
    }
    Serial.printf("Input Data String: %s\r\n", cbuf);
#endif
  } else if(c == '5') {
    Serial.println("Press q to quit: ");
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        Serial.println(c);
      }
      if(c == 'q') {
        Serial.println("Quit loop");
        break;
      }
      server.handleClient();
      delay(2);
    }
  } else {
    Serial.println("Invalid Test Number");
    return;
  }

}
