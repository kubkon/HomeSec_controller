#include <WiFi101.h>
#include <WiFiUdp.h>
#include <cstdint>
#include "credentials.h"

// WiFi
int wifi_status = WL_IDLE_STATUS;

// UDP
IPAddress remote(192,168,0,12);
unsigned int port = 8888;
WiFiUDP udp;

// Reed switch
int reed_pin = A3;
int reed_value = 0;

// PIR
int pir_pin = 2;
int pir_state = LOW;
int pir_value = 0;

// packet vars
#define UPDATE_P     0x1
#define TRIGGER_P    0x2
#define REED         0x1
#define PIR_DETECTED 0x2
#define PIR_ENDED    0x4

#define PACKET_LENGTH 2
byte packet_buffer[PACKET_LENGTH];


void setup() {
  // wait for serial; for debugging only
  Serial.begin(9600);
  while(!Serial) {
    ; // wait for serial for connect
  }

  // attempt to connect to WiFi network
  while (wifi_status != WL_CONNECTED) {
    Serial.print("Attempting to connect to ");
    Serial.println(ssid);
    wifi_status = WiFi.begin(ssid, pass);

    // wait 1 second for connection:
    delay(1000);
  }

  Serial.print("Connected to ");
  Serial.print(WiFi.SSID());
  Serial.print(" with IP address ");
  Serial.println(WiFi.localIP());

  // setup UDP
  udp.begin(port);

  // setup PIR
  pinMode(pir_pin, INPUT);
}

void loop() {
  reed_value = analogRead(reed_pin);
  if (reed_value <= 10) {
    Serial.println("Door was open!");
    publish((uint8_t)TRIGGER_P, (uint8_t)REED);
  }

  pir_value = digitalRead(pir_pin);
  if (pir_value == HIGH) {
    if (pir_state == LOW) {
      Serial.println("Motion detected!");
      publish((uint8_t)TRIGGER_P, (uint8_t)PIR_DETECTED);
      pir_state = HIGH;
    }
  }
  else {
    if (pir_state == HIGH) {
      Serial.println("Motion ended!");
      publish((uint8_t)TRIGGER_P, (uint8_t)PIR_ENDED);
      pir_state = LOW;
    }
  }
}

void publish(uint8_t type, uint8_t value) {
  memset(packet_buffer, 0, PACKET_LENGTH);
  packet_buffer[0] = type;
  packet_buffer[1] = value;
  udp.beginPacket(remote, port);
  udp.write(packet_buffer, PACKET_LENGTH);
  udp.endPacket();
}

