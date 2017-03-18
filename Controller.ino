#include <SPI.h>
#include <WiFi101.h>
#include "credentials.h"

// WiFi vars
int wifi_status = WL_IDLE_STATUS;

void setup() {
  // wait for serial; for debugging only
  Serial.begin(9600);
  while(!Serial) {
    ; // wait for serial for connect
  }

  // attempt to connect to WiFi network
  while (wifi_status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    wifi_status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }

  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
}

void loop() {
  printWiFiStatus();
}

void printWiFiStatus() {
  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

