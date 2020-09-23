#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <HardwareSerial.h>
#include<TinyGPS.h>
#include<Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include "MPU9250.h"
#include "SSD1306Wire.h"

float lat, lon, vel;
unsigned long data, hora;
unsigned short sat;

int Altitude;
int ultimaAltitude;
int Apogeo;
int SoloAltitude;
int AlturaMaxima;

Adafruit_BMP280 bmp;
HardwareSerial SerialGPS(1);
TinyGPS GPS;
SSD1306Wire display (0x3c, 4, 15);
MPU9250 IMU(Wire, 0x68);

int status;

void setup() {
  Serial.begin(115200);
  Serial.println("Serial iniciada");

  Serial.println("Iniciando bmp");
  if (!bmp.begin(0x76)) {
    Serial.println(F("Sensor BMP280 não foi identificado! Verifique as conexões.")); //IMPRIME O TEXTO NO MONITOR SERIAL
    while (1);
  }
  Serial.println("bmp iniciado");

  Serial.println("iniciando gps");
  SerialGPS.begin(9600, SERIAL_8N1, 36, 37); //Esp32 lora 0 = RX, 22 = TX
  Serial.println("Gps iniciado");

  Serial.println("starting mpu");
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }

  IMU.readSensor();
  SoloAltitude = bmp.readAltitude(1013.25);


  LoRa.setPins(18, 14, 26);

  if (!LoRa.begin(868E6)) {
    Serial.println("Error");
    delay(1000);
  }

  Serial.println("Sending data packet...");
}
void loop() {

  getAltitude(); // carrega altitude atual
  if (Altitude - ultimaAltitude <= -1) // verifica por diminuicao de 1 metro na altitude
  {
    Serial.println("                                         Verificar 1 ");
    delay(150); // Gravidade e 100ms per meter, 150ms é suficiente
    getAltitude; // carrega nova altitude
    if (Altitude - ultimaAltitude <= -2) // verifica se nova altitude é 1 metro menos que ultima medida
    {
      Serial.println("                                         Verificar 2");
      delay(150); // Gravidade e 100ms per meter, 150ms é suficiente
      getAltitude; // carrega nova altitude
      if (Altitude - ultimaAltitude <= -3) // verifica se nova altitude é 2 metro menos que ultima medida
      {
        Serial.println("                         PARAQUEDAS");
        Apogeo = ultimaAltitude;
        Apogeo = Apogeo - 3;
        AlturaMaxima = Apogeo - SoloAltitude;
        Serial.print(F("Paraquedas acionado, altura maxima do foguete foi "));
        Serial.print(AlturaMaxima);
        Serial.println(F("metros"));
      }
      else
      {
        ultimaAltitude = Altitude; // Leitura falsa. configura ultima altitude lida
        Serial.println("                                             Hummm, 3");
      }
    }
    else
    {
      ultimaAltitude = Altitude; // Leitura falsa. configura ultima altitude lida
      Serial.println("                                                 Hummm, 2");
    }
  }
  else
  {
    ultimaAltitude = Altitude; // Leitura falsa. configura ultima altitude lida
    Serial.println("                                                 Hummmm, l");
  }

  Serial.print("Sending packet: ");
  LoRa.beginPacket();
  LoRa.print(String("///////BMP//////"));
  LoRa.print(String(bmp.readTemperature()));
  LoRa.print(String(bmp.readPressure() * 0.000145));
  LoRa.print(String(Altitude));
  LoRa.print(String(ultimaAltitude));
  LoRa.print(String(SoloAltitude));
  LoRa.print(String(Apogeo));
  LoRa.print(String(AlturaMaxima));
  IMU.readSensor();
  LoRa.print(String("///////MPU//////"));
  LoRa.print(String(IMU.getAccelX_mss(), 6));
  LoRa.print(String(IMU.getAccelY_mss(), 6));
  LoRa.print(String(IMU.getAccelZ_mss(), 6));
  LoRa.print(String(IMU.getGyroX_rads(), 6));
  LoRa.print(String(IMU.getGyroY_rads(), 6));
  LoRa.print(String(IMU.getGyroZ_rads(), 6));
  LoRa.print(String(IMU.getMagX_uT(), 6));
  LoRa.print(String(IMU.getMagY_uT(), 6));
  LoRa.print(String(IMU.getMagZ_uT(), 6));
  LoRa.print(String(IMU.getTemperature_C(), 6));
  LoRa.print(String("///////GPS//////"));
  LoRa.print(String(lat, 6));
  LoRa.print(String(lon, 6));
  LoRa.print(String(vel));
  LoRa.print(String(sat));
  LoRa.endPacket();
}
