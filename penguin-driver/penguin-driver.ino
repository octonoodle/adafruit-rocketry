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
  }
}

void loop() {
  if (TRANSMIT_MODE) {
    // this will terminate with a reboot once data is successfully transmitted
    transmitData();
  } else {
    // this too
    parseAndStore();
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
    csvFile.println("\"date\",\"time\",\"altitude (m)\",\"pressure (hPa)\",\"temperature (°C)\",\"longitude (°)\",\"latitude (°)\",\"fix valid\"");
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

  // recover and parse gps
  float longitude; // decimal degrees
  float latitude; // decimal degrees
  bool validFix;
  int time; // in format 123456 HHMMSS
  int date; // in format 123456 MMDDYY

  configFile = fatfs.open(CONFIG_FILE, FILE_READ);
  if (!configFile) {
    Serial.println("error, failed to read gps data file");
  } else {
    // parse raw nmea sentence
    char rmcSentence[128];
    configFile.readBytes(rmcSentence, sizeof(rmcSentence));
    Serial.println("to decode:");
    Serial.println(rmcSentence);
    rmcDecode(rmcSentence, sizeof(rmcSentence), &longitude, &latitude, &validFix, &time, &date); // extract data and put into variables
    Serial.print("parsed longitude: "); Serial.println(longitude, 4);
    Serial.print("parsed latitude: "); Serial.println(latitude, 4);
    Serial.print("parsed fix status: "); Serial.println(validFix ? "valid" : "invalid");
    Serial.print("time: "); 
    Serial.print((int)floor(time/10000)); Serial.print(':');
    Serial.print((int)floor(time % 10000 /100)); Serial.print(':'); 
    Serial.println(time % 100);
    Serial.print("date: "); 
    Serial.print((int)floor(date % 10000 /100)); Serial.print('/'); 
    Serial.print((int)floor(date/10000)); Serial.print('/');
    Serial.println(date % 100);
  }
  configFile.close();

  Serial.print("Altitude: "); Serial.println(altitude);
  Serial.print("Pressure: "); Serial.println(pressure);
  Serial.print("Temperature: "); Serial.println(temperature);

  csvFile = fatfs.open("bmp390.csv", FILE_WRITE | O_CREAT);
  // record data to local file
  if (csvFile) {
    csvFile.print("\"");
    csvFile.print((int)floor(date % 10000 /100)); csvFile.print('/'); 
    csvFile.print((int)floor(date/10000)); csvFile.print('/');
    csvFile.print(date % 100);
    csvFile.print("\",\"");
    csvFile.print((int)floor(time/10000)); csvFile.print(':');
    csvFile.print((int)floor(time % 10000 /100)); csvFile.print(':'); 
    csvFile.print(time % 100);
    csvFile.print("\","); 
    csvFile.print(altitude);
    csvFile.print(",");
    csvFile.print(pressure);
    csvFile.print(",");
    csvFile.print(temperature);
    csvFile.print(",");
    csvFile.print(longitude);
    csvFile.print(",");
    csvFile.print(latitude);
    csvFile.print(",");
    csvFile.println(validFix);
  } else {
    Serial.println("failed to write data to file");
  }
  csvFile.close();


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
          float dataPacket[] = {altitude, pressure, temperature, longitude, latitude, (float)validFix, (float)time, (float)date};
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
void sendAndWait(float dataSet[8]) {
  // xxyy.zz
  float alt = dataSet[0];
  float pressure = dataSet[1];
  float temp = dataSet[2];
  bool negativeAlt = alt < 0;
  float latitude = dataSet[3];
  float longitude = dataSet[4];
  bool validFix = (bool)(int)dataSet[5];
  int time = dataSet[6];
  int date = dataSet[7];

  // each float needs to be broken up into integer sections: 1234.56 -> [12], [34], [56]

  // since we already know if alt is negative, take abs to get pure digits and avoid negative overflow on unsigned int
  alt = fabs(alt);

  // xxyy.zz
  uint8_t alt1 = (uint8_t)(floor(alt / 100)); //xx
  uint8_t alt2 = (uint8_t)(floor(alt - alt1*100)); //yy  
  uint8_t alt3 = (uint8_t)(100*(alt - alt1*100 - alt2)); //zz

  //xxyy.zz
  uint8_t pressure1 = (uint8_t)(floor(pressure / 100)); //xx
  uint8_t pressure2 = (uint8_t)(floor(pressure - pressure1*100)); //yy  
  uint8_t pressure3 = (uint8_t)(100*(pressure - pressure1*100 - pressure2)); //zz
  
  //xx.yy
  uint8_t temp1 = (uint8_t)(floor(temp)); //xx
  uint8_t temp2 = (uint8_t)((temp - temp1)*100); //yy

  uint8_t long1;
  uint8_t long2;
  uint8_t long3;

  uint8_t lat1;
  uint8_t lat2;
  uint8_t lat3;
  uint8_t lat4;

  if (validFix) {
    // aa.bbcc
    long1 = (uint8_t)(floor(longitude)); //aa 
    long2 = (uint8_t)(floor(100*(longitude - long1))); //bb
    long3 = (uint8_t)((int)(10000*longitude) % 100); //cc

    // abb.ccdd
    lat1 = (uint8_t)floor(latitude/100); //a
    lat1 = (uint8_t)(floor(latitude - lat1)); //bb
    lat2 = (uint8_t)(floor(100*(latitude - lat1 - lat2))); //cc
    lat3 = (uint8_t)((int)(10000*latitude) % 100); //dd
  }

  // xxyyzz
  uint8_t hour = floor(time/10000);
  uint8_t min = floor(time % 10000 /100);
  uint8_t sec = time % 100;
  // xxyyzz
  uint8_t day = floor(date/10000);
  uint8_t month = floor(date % 10000 /100);
  uint8_t year = date % 100;

  uint8_t encodedPackets[] = 
  {(uint8_t)negativeAlt, alt1, alt2, alt3, pressure1, pressure2, pressure3, temp1, temp2, 
  long1, long2, long3, lat1, lat2, lat3, lat4, (uint8_t)validFix,
  hour, min, sec, month, day, year};
  
  Serial.println("encoded message to send: ");
  for (int i = 0; i < sizeof(encodedPackets); i++) {
    Serial.print(encodedPackets[i]);
    Serial.print(" ");
  }
  Serial.println();
  
  rf95.send(encodedPackets, sizeof(encodedPackets));
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

// parse fields from rmc message and copy into given buffer variables
void rmcDecode(char* source, size_t srcLen, float *longit, float *lat, bool *validFix, int *time, int *date) {
  /*
  example message:
  $GPRMC,123519,A,4807.038,N,01131.000,E,0.022,269.131,230394,,,A,C*
  name   time  fix   long      lat      speed  sat angle date        always ends with *
  */

  Serial.println("source");
  Serial.println(source);
  char* fields[10]; // only concerned with first 10 fields
  int field = 0;
  int letter = 0;
  char temp[10];
  int i = 0;
  for (i; i<srcLen;i++) {
    if (source[i] == ',' || letter >= 10) {
      if (field < 10) {
        // store string of field efficiently
        char* fieldptr = (char*) calloc(sizeof(temp) + 1, sizeof(char));
        strcpy(fieldptr, temp);
        fields[field] = fieldptr;
        Serial.println();
        Serial.print("set field "); Serial.println(field);
        Serial.println(fields[field]);
      }
      Serial.print(" (");
      Serial.print(temp);
      Serial.print(") ");
      Serial.print(field);
      Serial.println();
      field++;
      letter = 0;
      memset(temp, 0, 10 * sizeof(char)); // clear buffer
      continue;
    }
    Serial.print(source[i]);
    temp[letter] = source[i];
    letter++;
  }
  Serial.print("last idx: "); Serial.println(i);
  for (int p = 0; p < 10; p++) {
    Serial.println(fields[p]);
  }
  
  // parse: 

  // time+date
  *time = atoi(fields[1]);
  *date = atoi(fields[9]);
  Serial.println(*time);
  Serial.println(*date);
  

  // position data
  float longitBuf = (float)atof(fields[3]);
  *longit = (float)floor(longitBuf/100.0) + fmodf(longitBuf, 100.0)/60; // DDMM.MMM --> DD.DDDD
  if (fields[4][0] == 'S') { // south
    *longit = *longit * -1;
  }

  float latBuf = (float)atof(fields[5]);
  *lat = (float)floor(latBuf/100.0) + fmodf(latBuf, 100.0)/60; // DDDMM.MMM --> DDD.DDDD
  if (fields[6][0] == 'W') { // west
    *lat = *lat * -1;
  }

  *validFix = (fields[2][0] == 'A');
  for (int p = 0; p < 10; p++) {
    free(&fields[p]);
  }
}

// ~~~~~~~~~~~~~~~~~~~~~~~~ GPS LOOP ~~~~~~~~~~~~~~~~~~~~~~~~~~~
// to get RMC sentence:
// 1. clear buffer
// 2. wait for gps data
// 3. slowly read until full sentence is formed
// 4. parse
// 5. finish or try again

// checks if NMEA sentence is RMC (Recommended Minimum Coordinates) aka GPS data
bool isRMC(char * sent) {
  char msgType[6];
  for (int i = 0; i < 6; i++) {
    msgType[i] = sent[i];
  }
  return !strcmp(msgType, "$GNRMC");
}

void parseAndStore() {
  
  // allocate how many seconds to attempting to find RMC?
  int timeoutSec = 5; // value below 5 not recommended for GPS updates at 1Hz
  int timeoutChars = timeoutSec*9600/5;


  // 1.
  while(GPSSerial.available()) {
    GPSSerial.read();
  }

  // 2.
  while(!GPSSerial.available());
  // 3.
  int i = 0;
  int j = 0;
  bool done = false;
  // only copy found sentence to permanent buffer if it's an rmc (no timeout)
  bool rmcValid = false;
  char sentence[128];
  Serial.println("begin gps parse");
  while(!done) {
    delay(5);
    if (GPSSerial.available()) {
      char c = GPSSerial.read();
      switch (c) {
        case '$':
          // restart message from beginning
          j = 0;
          for (j; j < 128; j++) {
             sentence[j] = '\0';
           }
        case '*':
          done = true;
          rmcValid = true;
          // 4.
          if (!isRMC(sentence)) {
            // continue to read from serial
            delay(25);
            done = false;
            rmcValid = false;
            j = 0;
          }
        default:
          if (j < 128) {
            sentence[j] = c;
            j++;
          } else {
            Serial.println("overflow");
            j = 0;
          }
      }
      i++;
      if (i > timeoutChars) {
        done = true;
        Serial.println();
        Serial.print("search timeout reached (");
        Serial.print(timeoutChars);
        Serial.println(") chars");
      }
    } else {
      done = true;
      Serial.println();
      Serial.print("ended early at index: "); Serial.println(i);
    }
  }
  
  if (rmcValid) {
    Serial.println();
    Serial.print("found: ");
    Serial.println(sentence);
    configFile = fatfs.open(CONFIG_FILE, FILE_WRITE | O_CREAT);
    configFile.write(sentence);
    configFile.close();
    reboot();
  }
  Serial.println("DONE!");
  delay(500);
  
}