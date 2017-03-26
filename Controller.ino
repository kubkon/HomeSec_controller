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
#define REED_MAX_VALUE 1023
int reed_pin = A3;
int reed_state = LOW;
int reed_value = 0;

// PIR
int pir_pin = 2;
int pir_state = LOW;
int pir_value = 0;

// packet vars
#define REED         0x1
#define PIR          0x2

#define REED_OPEN    0x1
#define REED_CLOSED  0x2

#define PIR_DETECTED 0x1
#define PIR_ENDED    0x2

#define PACKET_LENGTH 4
byte packet_buffer[PACKET_LENGTH];


void setup() {
  // wait for serial; for debugging only
  Serial.begin(9600);
  /* while(!Serial) { */
  /*   ; // wait for serial for connect */
  /* } */

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
  if (reed_value <= REED_MAX_VALUE / 2) {
    if (reed_state == LOW) {
      Serial.println("Door was open!");
      publish_reed((uint8_t)REED_OPEN, reed_value);
      reed_state = HIGH;
    }
  }
  else {
    if (reed_state == HIGH) {
      Serial.println("Door was closed!");
      publish_reed((uint8_t)REED_CLOSED, reed_value);
      reed_state = LOW;
    }
  }

  pir_value = digitalRead(pir_pin);
  if (pir_value == HIGH) {
    if (pir_state == LOW) {
      Serial.println("Motion detected!");
      publish_pir((uint8_t)PIR_DETECTED);
      pir_state = HIGH;
    }
  }
  else {
    if (pir_state == HIGH) {
      Serial.println("Motion ended!");
      publish_pir((uint8_t)PIR_ENDED);
      pir_state = LOW;
    }
  }
}

void publish_reed(uint8_t state, int value) {
  memset(packet_buffer, 0, PACKET_LENGTH);
  packet_buffer[0] = (uint8_t)REED;
  packet_buffer[1] = state;
  packet_buffer[2] = ((uint8_t)(value >> 8)) & 0x3;
  packet_buffer[3] = ((uint8_t)value) & 0xFF;
  udp.beginPacket(remote, port);
  udp.write(packet_buffer, PACKET_LENGTH);
  udp.endPacket();
}

void publish_pir(uint8_t state) {
  memset(packet_buffer, 0, PACKET_LENGTH);
  packet_buffer[0] = (uint8_t)PIR;
  packet_buffer[1] = state;
  udp.beginPacket(remote, port);
  udp.write(packet_buffer, PACKET_LENGTH);
  udp.endPacket();
}

