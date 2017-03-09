#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUDP.h>

// Ethernet vars
byte mac[] = {
  0x90, 0xA2, 0xDA, 0x0F, 0x7E, 0xFA
};
EthernetClient client;

EthernetUDP udp;
IPAddress remote(192,168,0,12); // address of the server
unsigned int local_port = 8888;
char output_buffer[UDP_TX_PACKET_MAX_SIZE];

// PIR vars
int pir_pin = 2;
int pir_state = LOW;
int pir_value = 0;

void setup() {
  // wait for serial; for debugging only
  Serial.begin(9600);
  while(!Serial) {;}

  // set up the Ethernet
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP!");
    for (;;) {;}
    // TODO try static IP
    // otherwise, lit up an LED to denote the status and
    // request hard reboot
  }
  // and UDP
  udp.begin(local_port);

  // set up the PIR sensor
  pinMode(pir_pin, INPUT);
}

void loop() {
  // maintain DHCP lease
  maintain_dhcp();

  pir_value = digitalRead(pir_pin);   // read input value
  if (pir_value == HIGH) {            // check if the input is HIGH
    if (pir_state == LOW) {
      // we have just turned on
      Serial.println("Motion detected!");
      publish_trigger();
      pir_state = HIGH;
    }
  } else {
    if (pir_state == HIGH){
      // we have just turned of
      Serial.println("Motion ended!");
      pir_state = LOW;
    }
  }
}

void maintain_dhcp() {
  switch(Ethernet.maintain()) {
    case 1:
    case 3:
      // TODO handle DHCP failure
      Serial.println("Error: DHCP failure");
      break;
    case 2:
    case 4:
    default:
      break; 
  }
}

void publish_trigger() {
    output_buffer[0] = 'A';
    output_buffer[1] = 'L';
    output_buffer[2] = 'E';
    output_buffer[3] = 'R';
    output_buffer[4] = 'T';

    udp.beginPacket(remote, local_port);
    udp.write(output_buffer, UDP_TX_PACKET_MAX_SIZE);
    udp.endPacket();

    memset(output_buffer, 0, UDP_TX_PACKET_MAX_SIZE);
}

