# INL-0x Series (Arduino)

## Install Arduino IDE
- Download Arduino IDE

  참조: <https://www.arduino.cc/en/software>
- Install Additional Board Manager URLs for ESP32

  Arduino IDE : File --> Preferences --> Addtional board manager URLs에 하기의 json 파일 경로를 추가

  - Stable release link
  ```
  https://espressif.github.io/arduino-esp32/package_esp32_index.json
  ```

  참조: <https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html>

## Setup Board and Libraries
- Setup Board Type

  Arduino IDE : Tools --> Board --> esp32 --> ESP32S3 Dev Module 선택

- Standard Arduino Library 추가

  Arduino IDE : Tools --> Manage Libraries... --> Library Manager

  Install이 필요한 Library : AsyncDelay, Ethernet_Generic, SoftWire

- User Arduino Library (.ZIP) 추가

  Arduino IDE : Sketch --> Include Library --> Add .ZIP Libraries...

  Install이 필요한 (.ZIP) Library : SC16IS752Serial-pms-x.x.x.zip, EspUsbHost-inl-x.x.x.zip

  [*] EspUsbHost는 Arduino IDE에서 설치하지 마시고, 반드시 ZIP Library를 설치하는 것이  필요함.

## Sample Source Code for Hardware Basic Operation
- 실행 방법

  각각 sub_test_*() 함수로 구분되어 작성되어 있고, Serial Monitor의 Message 창에 해당하는 함수의 명령어를 입력하여 실행할 수 있음. 예를 들어, sub_test_a( )는 'a'를 입력하고 'Enter'키를 입력함으로써, 실행됨.
- Sub Test Function List
  - sub_test_a( ) : LED - LED를 주기적으로 On/Off Toggle하여 보드의 기본 동작을 확인하기 위한 함수
  - sub_test_n( ) : W5500 - W5500를 설정하고 Ethernet 통신을 시험하기 위한 함수

- LAN Test

  기 정의된 VSPI(pspi)를 이용하여 Ethernet 객체를 초기화함.

  MAC, IP 등을 설정하고 Link Status를 확인함

  이후, UDP Packet를 이용하여, Server, Client 통신이 가능함

  Test Command ‘n’를 입력하고, 세부 Command를 추가로 입력하여 W5500 (Ethernet Controller) 동작을 확인함

  (n+Enter+3+Enter)를 입력하여 Static IP를 설정하고, (n+Enter+4+Enter)를 입력하여 LINK Status를 확인함

  LINK ON(1)이 되었다면, UDP Server/Client 구성이 가능함.


  ```
  void sub_test_n(void) {
    ~~~ 중략 ~~~
    Serial.println("Sub-test N - W5500");
    Serial.print("Input Test Number: ");
    ~~~ 중략 ~~~

    } else if(c == '3') {  // IP Set
      Ethernet.init(PIN_ETH_CS);
      pCUR_SPI = pspi;
      Ethernet.begin(nc_mac, nc_ip, nc_dns, nc_gateway, nc_subnet);
      ~~~ 중략 ~~~
      Serial.print("getChip(): "); Serial.println(Ethernet.getChip(), DEC);
      Serial.print("localIP(): "); Serial.println(Ethernet.localIP());

    } else if(c == '4') {
      Serial.print("linkStatus: "); Serial.println(Ethernet.linkStatus(), DEC); 
      Serial.print("PhyState: "); Serial.println(Ethernet.phyState(), HEX);
      Serial.print("HardwareStatus: "); 
            Serial.println(Ethernet.hardwareStatus());
      Serial.print("speed: "); Serial.println(Ethernet.speed(), DEC);
      Serial.print("duplex: "); Serial.println(Ethernet.duplex(), DEC);
      Serial.print("dnsServerIP: "); Serial.println(Ethernet.dnsServerIP());
      ~~~ 중략 ~~~
    } else if(c == '6') {
      unsigned int localPort = 8080;
      unsigned int serverPort = 1883;
      int ret;
      size_t wsize;
      IPAddress server_ip(192, 168, 1, 149);
    
      EthernetUDP Udp;
      Udp.begin(localPort);

      ret = Udp.beginPacket(server_ip, serverPort);
      Serial.print("Return of beginPacket: "); Serial.println(ret, DEC);
      wsize = Udp.write("hello from esp");
      Serial.print("Return of write: "); Serial.println(wsize);
      ret = Udp.endPacket();
      Serial.print("Return of endPacket: "); Serial.println(ret, DEC);
      ~~~ 중략 ~~~
    }
  }
  ```
