/* #include <SPI.h> */
#include <WiFi101.h>
#include <WiFiUdp.h>
#include "credentials.h"

// WiFi vars
int wifi_status = WL_IDLE_STATUS;

// UDP vars
IPAddress remote(192,168,0,12);
unsigned int port = 8888;
WiFiUDP udp;
const unsigned PACKET_LENGTH = 48;
byte packet_buffer[PACKET_LENGTH];

unsigned long last_connection_time = 0;
const unsigned long interval = 60L * 1000L; // delay update of 60seconds

int reed_pin = A3;
int reed_value = 0;

void setup() {
  // wait for serial; for debugging only
  Serial.begin(9600);
  while(!Serial) {
    ; // wait for serial for connect
  }
}

void loop() {
  if (millis() - last_connection_time > interval) {
    last_connection_time = send_update();
  }

  reed_value = analogRead(reed_pin);
  Serial.println(reed_value);
}

unsigned long send_update() {
  // attempt to connect to WiFi network
  while (wifi_status != WL_CONNECTED) {
    /* Serial.print("Attempting to connect to "); */
    /* Serial.println(ssid); */
    wifi_status = WiFi.begin(ssid, pass);

    // wait 1 second for connection:
    delay(1000);
  }

  /* Serial.print("Connected to "); */
  /* Serial.print(WiFi.SSID()); */
  /* Serial.print(" with IP address "); */
  /* Serial.println(WiFi.localIP()); */

  /* Serial.println("Sending packet..."); */

  memset(packet_buffer, 0, PACKET_LENGTH);
  packet_buffer[0] = 'U';
  packet_buffer[1] = 'P';
  packet_buffer[2] = 'D';
  packet_buffer[3] = 'A';
  packet_buffer[4] = 'T';
  packet_buffer[5] = 'E';

  udp.begin(port);
  udp.beginPacket(remote, port);
  udp.write(packet_buffer, PACKET_LENGTH);
  udp.endPacket();

  unsigned long now = millis();

  delay(1000);

  // wifi off
  WiFi.end();
  wifi_status = WiFi.status();

  return now;
}

