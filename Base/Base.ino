#include <SPI.h>
#include <LoRa.h>
#include "SSD1306.h"
 
SSD1306  display(0x3c, 4, 15);
 
#define SS      18
#define RST     14
#define DI0     26
#define BAND    915E6
 
void setup() {
  pinMode(16,OUTPUT);
  digitalWrite(16, LOW);    // set GPIO16 low to reset OLED
  delay(50); 
  digitalWrite(16, HIGH);
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
   
  Serial.begin(115200);
  while (!Serial);
  delay(1000);
  Serial.println("LoRa Receiver"); 
  display.drawString(5,5,"LoRa Receiver"); 
  display.display();
  SPI.begin(5,19,27,18);
  LoRa.setPins(SS,RST,DI0);
   
  if (!LoRa.begin(BAND)) {
    display.drawString(5,25,"Starting LoRa failed!");
    while (1);
  }
  Serial.println("LoRa Initial OK!");
  display.drawString(5,25,"LoRa Initializing OK!");
  display.display();
}
void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packets
    Serial.print("Received packet. ");
    while (LoRa.available()) {
    String data = LoRa.readString();
    Serial.print(data);
    }
    // print RSSI of packet
    Serial.print(" with RSSI ");
    Serial.println(LoRa.packetRssi());
    display.drawString(20, 45, "RSSI:  ");
    display.drawString(70, 45, (String)LoRa.packetRssi());
    display.display();
  }
}
