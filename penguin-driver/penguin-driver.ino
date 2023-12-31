/*
Integrated driver for the Penguin Electronics Payload

to do list:
- negative altitudes overflow, messed up in encoding (include sign bit?)
- increase encoding efficiency by using direct binary?
- clean up if(Serial) since they are all unnecessary
*/

#include <SPI.h>
#include <RH_RF95.h>

#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

#include <SPI.h>
#include <SdFat.h>
#include <Adafruit_SPIFlash.h>

#include <Adafruit_NeoPixel.h>

// for flashTransport definition
#include "flash_config.h"

Adafruit_SPIFlash flash(&flashTransport);

// custom serial port for GPS
#include <Arduino.h>   // required before wiring_private.h
#include "wiring_private.h" // pinPeripheral() function

Uart GPSSerial (&sercom4, A2, A1, SERCOM_RX_PAD_1, UART_TX_PAD_0);
void SERCOM4_Handler()
{
  GPSSerial.IrqHandler();
}

// file system object from SdFat
FatVolume fatfs;

#define RFM95_RST 11  // "A"
#define RFM95_CS 10   // "B"
#define RFM95_INT 6   // "D"

#define RF95_FREQ 915.0
RH_RF95 rf95(RFM95_CS, RFM95_INT);

float SEALEVELPRESSURE_HPA = 1001.01657;
#define DATA_FILE "bmp390.csv"

File32 csvFile;
Adafruit_BMP3XX bmp;
Adafruit_NeoPixel rgb(1, 8);  //for status updates during testing

void rgbGreen() {
  rgb.setPixelColor(0, 0, 255, 255);
  rgb.show();
}

void rgbError() {
  rgb.setPixelColor(0, 255, 0, 0);
  rgb.show();
}

void setup() {
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  while (!Serial) delay(1);
  delay(100);
  for (int i = 0; i < 20; i++) {
    Serial.print("/");
  }
  Serial.print("\n\n");
  Serial.println("Welcome to CAI Rocketry Software");
  Serial.println("Driver for Penguin-class electronics payload");
  Serial.println();
  for (int i = 0; i < 20; i++) {
    Serial.print("/");
  }
  Serial.print("\n\n");

  Serial.print("Radio Client Initializing...");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if (!rf95.init()) {
    Serial.println("failed");
    while (1)
      ;
  }
  Serial.println("success");

  rf95.setFrequency(RF95_FREQ);
  Serial.print("frequency setting: ");
  Serial.print(RF95_FREQ);
  Serial.println("MHz");

  //Serial.println("enter power setting [5-23]:");
  //while (!Serial.available())
  //  ;
  //int i = 0;
  //char c;
  //char num[5];
  //while (Serial.available() && i < 5) {  //get user defined power input
   // c = Serial.read();
   // num[i] = c;
   // i++;
 // }
  //int power;
  //power = atoi(num);
  rf95.setTxPower(10, false);
  Serial.print("power set to ");
  Serial.print(10);
  Serial.println(" dBm");

  Serial.println("Radio configured");
  Serial.println("Initializing Barometer/Altimiter/Thermometer...");

  String input;
  input = "D";
  // Serial.println("use default sea level definition or set at current pressure? d/c");
  // while (!Serial.available()) {
  //   delay(10);
  // }
  // input = Serial.readString();
  if (bmp.begin_I2C()) {
    Serial.println("valid BMP3 sensor found");
    if (input.equals("c")) {
      Serial.print("setting to sensor value... ");
      delay(10);
      if (!bmp.performReading()) {
        Serial.println("failed");
      } else {
        SEALEVELPRESSURE_HPA = (bmp.pressure);
        Serial.println("success");
      }
    }
  } else {
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    rgbError();
    while (1)
      ;
  }
  Serial.print("Initializing GPS... ");
  GPSSerial.begin(9600);
  pinPeripheral(A1, PIO_SERCOM_ALT);
  pinPeripheral(A2, PIO_SERCOM_ALT);

  // GPSSerial.println("$PMTK220,2000*"); // config command for gps
  // while(!GPSSerial.available());
  // char ackBuf[16];int i;
  // while(GPSSerial.available()) {
  //   char c = GPSSerial.read();
  //   ackBuf[i] = c;
  //   i++;
  //   if (c == '\n' || i == 16) break;
  // }
  // if (strcmp(ackBuf, "$PMTK010,003*2C\n")) { // gps replying successfully set to 0.2hz update
  //   Serial.println("failed");
  //   while(1);
  // }
  Serial.println("success");

  Serial.print("Configuring flash memory... ");
  if (!flash.begin()) {
    Serial.println("failed");
    rgbError();
    while (1)
      ;
  }
  if (Serial) {
    Serial.println("success");
  }

  // First call begin to mount the filesystem.  Check that it returns true
  // to make sure the filesystem was mounted.
  if (!fatfs.begin(&flash)) {
    Serial.println("Error, failed to mount newly formatted filesystem");
    Serial.println("Was the flash chip formatted with the fatfs_format example?");
    rgbError();
    while (1)
      ;
  }
  Serial.println("mounted filesystem");

  Serial.print("formatting csv for data recording... ");
  csvFile = fatfs.open("bmp390.csv", FILE_WRITE | O_CREAT | O_TRUNC);
  if (csvFile) {
    csvFile.println("sensor data: ");
    csvFile.println("\"altitude\",\"pressure (hPa)\",\"temperature\"");
    Serial.println("data file first line written");
  } else {
    Serial.println("failed to start write to data file!");
    rgbError();
    while (1);
  }
  csvFile.close();
  Serial.println("all systems are configured, beginning transmissions...");
}

 
//xxxx.xx,yyyy.yy,zz.zz
void encodeSend(float bmpData[3]) {
  // xxyy.zz
  float data1 = bmpData[0]; //float
  float data2 = bmpData[1];
  float data3 = bmpData[2];

  // each float needs to be broken up into integer sections: 1234.56 -> [12], [34], [56]
  
  // xxyy.zz
  uint8_t num1pt1 = (uint8_t)(floor(data1 / 100)); //xx
  uint8_t num1pt2 = (uint8_t)(floor(data1 - num1pt1*100)); //yy  
  uint8_t num1pt3 = (uint8_t)(100*(data1 - num1pt1*100 - num1pt2)); //zz

  //xxyy.zz
  uint8_t num2pt1 = (uint8_t)(floor(data2 / 100)); //xx
  uint8_t num2pt2 = (uint8_t)(floor(data2 - num2pt1*100)); //yy  
  uint8_t num2pt3 = (uint8_t)(100*(data2 - num2pt1*100 - num2pt2)); //zz
  

  uint8_t num3pt1 = (uint8_t)(floor(data3)); //xx
  uint8_t num3pt2 = (uint8_t)((data3 - num3pt1)*100); //yy

  uint8_t data[] = {num1pt1, num1pt2, num1pt3, num2pt1, num2pt2, num2pt3, num3pt1, num3pt2};
  
  Serial.println("encoded message to send: ");
  for (int i = 0; i < 8; i++) {
    Serial.print(data[i]);
    Serial.print(" ");
  }
  Serial.println();
  
  rf95.send(data, sizeof(data));
  rf95.waitPacketSent();

  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (rf95.waitAvailableTimeout(500)) {
    if (rf95.recv(buf, &len)) {
      if (!strcmp((char *)buf, "data recieved!!")) {
        Serial.println("transmit success");
      } else {
        Serial.print("unexpected reply: ");
        Serial.println((char *)buf);
      }
    } else {
      Serial.println("data recv failed");
    }
  } else {
    Serial.println("no response from server during data transfer!");
    return;
  }
}

// checks if NMEA sentence is RMC (Recommended Minimum Coordinates) aka GPS data
bool isRMC(char * sent) {
  char msgType[6];
  for (int i = 0; i < 6; i++) {
    msgType[i] = sent[i];
  }
  return !strcmp(msgType, "$GNRMC");
}

// buffers to store NMEA data in case 
// new data can't be read next loop
char newestNMEA[128];
char newestRMC[128];
void loop() {
  if (Serial && Serial.available()) {
    Serial.println("You are typing something");
    String input = Serial.readString();
    if (input.equals("kill")) {
      Serial.println("terminating datalogging...");
      rgb.setPixelColor(0, 0, 0, 255);
      rgb.show();
      while (1)
        ;
    } else {
      Serial.println("input ignored");
    }
  }


  ////////////////// BMP390 //////////////////

  if (!bmp.performReading()) {
    Serial.println("reading error");
    rgbError();
    delay(10);
    return;
  }

  float altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  float pressure = bmp.pressure / 100.0;
  float temperature = bmp.temperature;
  if (!altitude || !pressure || !temperature) {
    Serial.println("data measurement failure");
  }
  float dataPacket[] = {altitude, pressure, temperature};

  csvFile = fatfs.open("bmp390.csv", FILE_WRITE | O_CREAT);

  if (csvFile) {
    rgbGreen();
    csvFile.print(altitude);
    csvFile.print(",");
    csvFile.print(pressure);
    csvFile.print(",");
    csvFile.println(temperature);
  } else {
    Serial.println("failed to write data to file");
    rgbError();
  }
  csvFile.close();
  Serial.print("Altitude: "); Serial.println(altitude);
  Serial.print("Pressure: "); Serial.println(pressure);
  Serial.print("Temperature: "); Serial.println(temperature);

  //////////////////  GPS   //////////////////

  bool reading = GPSSerial.available();
  //Serial.println(reading ? "true" : "false");
  char buff[10];
  // for (int i = 0; i < 10; i++) {
  //   char c = GPSSerial.read();
  //   buff[i] = c;
  // }
  // Serial.println(buff);
  
  int i = 0;
  char sentence[128];
  while (reading) {
    char c = GPSSerial.read();
    if (c == '$') {
      i = 0;
      // clear buffer since restarting to read
      for (i; i < 128; i++) {
        sentence[i] = '\0';
      }
      sentence[0] = '$';
      i = 1;
    } else if (c == '*') { //  <<<<<<<<<<<  sentence is done and while loop terminates
      //Serial.println("nmea done");
      sentence[i] = c;
      reading = false; //  <<<<<<<<<<<
    } else if (i >= 128) {
      //Serial.println("overflow");
      // clear buffer since restarting to read
      for (i; i < 128; i++) {
        sentence[i] = '\0';
      }
      i = 0;
    } else {
      sentence[i] = c;
      i++;
    }
  }

  if (sentence[0] == '$') { // hack to avoid keeping garbled data
    strcpy(newestNMEA, sentence);
    if (isRMC(newestNMEA)) { // update coordinates sentence if necessary
      strcpy(newestRMC, newestNMEA);
    }
  }
  Serial.print("newest NMEA: ");
  Serial.println(newestNMEA);
  Serial.print("newest RMC: ");
  Serial.println(newestRMC);

  return;
  //////////// Radio Transmission ////////////

  // handshake message
  uint8_t statusMessage[] = "Data Incoming";
  rf95.send(statusMessage, sizeof(statusMessage));

  Serial.print("sending request... ");
  rf95.waitPacketSent();

  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  // wait for response
  if (rf95.waitAvailableTimeout(3000)) {
    if (rf95.recv(buf, &len)) {
      // check proper handshake response
      if (!strcmp((char *)buf, "ready for data")) {
          Serial.println("handshake complete");
          encodeSend(dataPacket);
        }
      else {
        Serial.print("error, recieved message: ");
        Serial.println((char *)buf);
      }
    } else {
      Serial.println("ready check recv failed");
    }
  } else {
    Serial.println("no response from server during ready check!");
  }

  delay(1000);
}
