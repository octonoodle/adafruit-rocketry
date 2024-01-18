#include <RH_RF95.h>
#include <SPI.h>
#include <SdFat.h>
#include <Adafruit_SPIFlash.h>

#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_NeoPixel.h>

// for flashTransport definition
#include "/Users/auren/Documents/engineering/Arduino/libraries/flash_config.h"
Adafruit_SPIFlash flash(&flashTransport);

// file system object from SdFat
FatVolume fatfs;

#define DATA_FILE "bmp390.csv"
File32 csvFile;
#define CONFIG_FILE "gps-config.txt"
File32 configFile;

// custom serial port for GPS
#include <Arduino.h>   // required before wiring_private.h
#include "wiring_private.h" // pinPeripheral() function

Uart GPSSerial (&sercom4, A2, A1, SERCOM_RX_PAD_1, UART_TX_PAD_0);
void SERCOM4_Handler() { GPSSerial.IrqHandler(); }

#define RFM95_RST 11  // "A"
#define RFM95_CS 10   // "B"
#define RFM95_INT 6   // "D"
#define RF95_FREQ 915.0
RH_RF95 rf95(RFM95_CS, RFM95_INT);

float SEALEVELPRESSURE_HPA = 1001.01657;
Adafruit_BMP3XX bmp;
Adafruit_NeoPixel rgb(1, 8);  //for status updates during testing

// important flag: this controls whether the chip is running
// in radio transmit mode (1) or gps data collection mode (0)
bool TRANSMIT_MODE;

void setup() {
  serialInit();
  bmpInit();
  flashInit();
  if (TRANSMIT_MODE) {
    rf95Init();
    transmitData();
  } else {
    gpsInit();
    parseAndStore();
    reboot();
    Serial.println("you won't read this");
  }
}

void loop() {
  if (TRANSMIT_MODE) {
    // this will terminate with a reboot once data is successfully transmitted
    transmitData();
  } else {
    delay(100);
  }
}

// reboot the chip (same as pressing RST button)
void reboot() {
  Serial.println("REBOOTING...");
  delay(50);
  __disable_irq();
  NVIC_SystemReset();
  delay(5000);
  // try to reboot again forever
  reboot();
}

// ~~~~~~~~~~~~ SETUP ~~~~~~~~~~~~~~
void serialInit() {
  Serial.begin(115200);
  while (!Serial) delay(1);
  delay(100);
  for (int i = 0; i < 20; i++) {
    Serial.print("/");
  }
  Serial.print("\n\n");
  Serial.println("Welcome to CAI Rocketry Software");
  Serial.println("Driver for Penguin-class electronics payload");
  delay(500);
  Serial.println();
  for (int i = 0; i < 20; i++) {
    Serial.print("/");
  }
  Serial.print("\n\n");
}
void bmpInit() {
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
    while (1);
  }
}
void flashInit() {
  Serial.print("Configuring flash memory... ");
  if (!flash.begin()) {
    Serial.println("failed");
    while (1);
  }
  Serial.println("success");

  // First call begin to mount the filesystem.  Check that it returns true
  // to make sure the filesystem was mounted.
  if (!fatfs.begin(&flash)) {
    Serial.println("Error, failed to mount newly formatted filesystem");
    Serial.println("Was the flash chip formatted with the fatfs_format example?");
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
    while (1);
  }
  csvFile.close();

  Serial.print("config file... ");
  configFile  = fatfs.open(CONFIG_FILE, FILE_READ);
  if (configFile) {
    TRANSMIT_MODE = true;
    Serial.println("found");
    Serial.println("Starting in Radio Mode");
  } else {
    TRANSMIT_MODE = false;
    Serial.println("missing");
    Serial.println("Starting in GPS Mode");
  }
  configFile.close();
}
void rf95Init() {
  Serial.print("Radio Client Initializing...");
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if (!rf95.init()) {
    Serial.println("failed");
    while (1);
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
}
void gpsInit() {
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
}

// ~~~~~~~~~~~~~~~~~~~~~~~~ RADIO TRANSMISSION ~~~~~~~~~~~~~~~~~~~~~~~~~~~
void transmitData() {
  if (Serial && Serial.available()) {
    Serial.println("You are typing something");
    String input = Serial.readString();
    if (input.equals("kill")) {
      Serial.println("terminating datalogging...");\
      while (1)
        ;
    } else {
      Serial.println("input ignored");
    }
  }

  if (!bmp.performReading()) {
    Serial.println("reading error");
    delay(10);
    return;
  }

  // collect data from BMP390 sensor array
  float altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  float pressure = bmp.pressure / 100.0;
  float temperature = bmp.temperature;
  if (!altitude || !pressure || !temperature) {
    Serial.println("data measurement failure");
  }
  float dataPacket[] = {altitude, pressure, temperature};

  csvFile = fatfs.open("bmp390.csv", FILE_WRITE | O_CREAT);

  if (csvFile) {
    csvFile.print(altitude);
    csvFile.print(",");
    csvFile.print(pressure);
    csvFile.print(",");
    csvFile.println(temperature);
  } else {
    Serial.println("failed to write data to file");
  }
  csvFile.close();
  Serial.print("Altitude: "); Serial.println(altitude);
  Serial.print("Pressure: "); Serial.println(pressure);
  Serial.print("Temperature: "); Serial.println(temperature);

  uint8_t statusMessage[] = "Data Incoming";
  rf95.send(statusMessage, sizeof(statusMessage));

  Serial.print("sending request... ");
  rf95.waitPacketSent();

  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (rf95.waitAvailableTimeout(3000)) {
    if (rf95.recv(buf, &len)) {
      if (!strcmp((char *)buf, "ready for data")) {
          Serial.println("handshake complete");
          sendAndWait(dataPacket);
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

//xxxx.xx,yyyy.yy,zz.zz
void sendAndWait(float bmpData[3]) {
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
      if (!strcmp((char *)buf, "data recieved!!")) { // END CONDITION: TRANSMIT SUCCESS AND REBOOT
        Serial.println("transmit success");
        fatfs.remove(CONFIG_FILE);
        reboot();
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

// ~~~~~~~~~~~~~~~~~~~~~~~~ GPS LOOP ~~~~~~~~~~~~~~~~~~~~~~~~~~~
void parseAndStore() {
  Serial.println("read gps serial here");
  configFile = fatfs.open(CONFIG_FILE, FILE_WRITE | O_CREAT);
  configFile.close();
}