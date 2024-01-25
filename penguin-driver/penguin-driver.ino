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

Adafruit_BMP3XX bmp;
Adafruit_NeoPixel rgb(1, 8);  //for status updates during testing

/*          PROGRAM OPTIONS          */
float SEALEVELPRESSURE_HPA = 1019.6; 
#define USE_TEST_MESSAGE false // still read gps serial, but parse and transmit test message instead
#define VERBOSE_MODE false // describe code execution in detail, good for isolating crash points
#define WAIT_FOR_SERIAL false // do not start the code unless the serial port is connected (computer)


void verboseMessage(const char* msg) {
  #if VERBOSE_MODE
  Serial.println(msg);
  #endif
}

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
  Serial.println();
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
  if (WAIT_FOR_SERIAL) {
    while (!Serial) delay(1);
  }
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
  File32 eraseFlag = fatfs.open("noErase.txt", FILE_WRITE);
  bool truncate = false;
  if (!eraseFlag) {
    truncate = true;
    eraseFlag = fatfs.open("noErase.txt", FILE_WRITE | O_CREAT);
  }
  eraseFlag.close();

  Serial.print("formatting csv for data recording... ");
  csvFile = 
  truncate ? 
  fatfs.open("bmp390.csv", FILE_WRITE | O_CREAT | O_TRUNC) :
  fatfs.open("bmp390.csv", FILE_WRITE | O_CREAT);
  if (csvFile) {
    if (truncate) {
      csvFile.println("sensor data: ");
      csvFile.println("\"date\",\"time\",\"altitude (m)\",\"pressure (hPa)\",\"temperature (°C)\",\"longitude (°)\",\"latitude (°)\",\"fix valid\"");
      Serial.println("data file first line written");
    } else {
      Serial.println("did not print first line");
    }
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
  rf95.setTxPower(23, false);
  Serial.print("power set to ");
  Serial.print(23);
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
  verboseMessage("kill check");

  if (!bmp.performReading()) {
    Serial.println("reading error");
    delay(10);
    return;
  }
  verboseMessage("bmp read");

  // collect data from BMP390 sensor array
  float altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  float pressure = bmp.pressure / 100.0;
  float temperature = bmp.temperature;
  if (!altitude || !pressure || !temperature) {
    Serial.println("data measurement failure!");
  }
  verboseMessage("bmp valid");

  // recover and parse gps
  float longitude; // decimal degrees
  float latitude; // decimal degrees
  bool validFix;
  int time; // in format 123456 HHMMSS
  int date; // in format 123456 MMDDYY

  configFile = fatfs.open(CONFIG_FILE, FILE_READ);
  verboseMessage("opened config");

  if (!configFile) {
    Serial.println("error, failed to read gps data file");
  } else {
    // parse raw nmea sentence
    char rmcSentence[128];
    if (USE_TEST_MESSAGE) {
      strcpy(rmcSentence, "$GPRMC,123519,A,4807.038,N,01131.000,E,0.022,269.131,230394,,,A,C*");
      verboseMessage("gps input replaced by test message");
    } else {
      configFile.readBytes(rmcSentence, sizeof(rmcSentence));
      verboseMessage("rmc get");
    }
    rmcDecode(rmcSentence, sizeof(rmcSentence), &longitude, &latitude, &validFix, &time, &date); // extract data and put into variables
    verboseMessage("rmc decoded:");
    // print everything in one line
    // timing
    char foo[10];
    sprintf(foo, "%f", longitude);
    verboseMessage(foo);
    sprintf(foo, "%f", latitude);
    verboseMessage(foo);
    Serial.println(validFix);
    Serial.println(time);
    Serial.println(date);
    short month = floor(date % 10000 /100); 
    short day = floor(date/10000);
    short year = date % 100;
    short hour = floor(time/10000); 
    short min = floor(time % 10000 /100); 
    short sec = time % 100;
    char timedate[17];
    addZerosPrint(month); Serial.print("/"); addZerosPrint(day); Serial.print("/"); addZerosPrint(year); Serial.print(" ");
    addZerosPrint(hour); Serial.print(":"); addZerosPrint(min); Serial.print(":"); addZerosPrint(sec);

    // posn
    Serial.print(" long: "); Serial.print(longitude, 4);
    Serial.print(" lat: "); Serial.print(latitude, 4);
    Serial.print(" fix: "); Serial.println(validFix ? "valid" : "invalid");
  }
  verboseMessage("config closed "); 
  verboseMessage(configFile.close() ? "1" : "0");

  Serial.print("Altitude: "); Serial.println(altitude);
  Serial.print("Pressure: "); Serial.println(pressure);
  Serial.print("Temperature: "); Serial.println(temperature);

  csvFile = fatfs.open("bmp390.csv", FILE_WRITE | O_CREAT);
  // record data to local file
  if (csvFile) {
    csvFile.print("\"");
    addZerosPrintToFile((int)floor(date % 10000 /100)); csvFile.print('/'); 
    addZerosPrintToFile((int)floor(date/10000)); csvFile.print('/');
    addZerosPrintToFile(date % 100);
    csvFile.print("\",\"");
    addZerosPrintToFile((int)floor(time/10000)); csvFile.print(':');
    addZerosPrintToFile((int)floor(time % 10000 /100)); csvFile.print(':'); 
    addZerosPrintToFile((int)(time % 100));
    csvFile.print("\","); 
    csvFile.print(altitude);
    csvFile.print(",");
    csvFile.print(pressure);
    csvFile.print(",");
    csvFile.print(temperature);
    csvFile.print(",");
    csvFile.print(longitude, 4);
    csvFile.print(",");
    csvFile.print(latitude, 4);
    csvFile.print(",");
    csvFile.println(validFix);
  } else {
    Serial.println("failed to write data to file");
  }
  csvFile.close();


  uint8_t statusMessage[] = "data incoming";
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
  float longitude = dataSet[3];
  float latitude = dataSet[4];
  bool validFix = (bool)(int)dataSet[5];
  int time = dataSet[6];
  int date = dataSet[7];
  bool negativeAlt = alt < 0;
  bool negativeLong = longitude < 0;
  bool negativeLat = latitude < 0;

  // each float needs to be broken up into integer sections: 1234.56 -> [12], [34], [56]

  // since we already know if values are negative, take abs to get pure digits and avoid negative overflow on unsigned ints
  alt = fabs(alt);
  longitude = fabs(longitude);
  latitude = fabs(latitude);

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
    lat2 = (uint8_t)(floor(latitude - lat1*100)); //bb
    lat3 = (uint8_t)(floor(100*(latitude - lat1*100 - lat2))); //cc
    lat4 = (uint8_t)((int)(10000*latitude) % 100); //dd
  }

  // xxyyzz
  uint8_t hour = floor(time/10000);
  uint8_t min = floor(time % 10000 /100);
  uint8_t sec = time % 100;
  // xxyyzz
  uint8_t day = floor(date/10000);
  uint8_t month = floor(date % 10000 /100);
  uint8_t year = date % 100;

  // alt: -220.29
  // press: 1027.43
  // temp: 28.74
  // $GPRMC,123519,A,4807.038,N,01131.000,E,0.022,269.131,230394,,,A,C*
  //alt long lat
  //+/- +/- +/-    alt       press    temp      long        lat      fix    time      date     end
  // 1 | 0 | 0 | 2 20 29 | 10 27 43 | 28 73 | 48 11 73 | 0 11 51 66 | 1 | 12 35 19 | 3 23 94 | 0

  uint8_t encodedPackets[] = 
  {(uint8_t)negativeAlt, (uint8_t)negativeLong, (uint8_t)negativeLat, alt1, alt2, alt3, pressure1, pressure2, pressure3, temp1, temp2, 
  long1, long2, long3, lat1, lat2, lat3, lat4, (uint8_t)validFix,
  hour, min, sec, month, day, year, 0};
  
  Serial.print("encoded message to send (");
  Serial.print(sizeof(encodedPackets));
  Serial.println(" bytes): ");
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

  verboseMessage("before decode");
  char fields[10][10]; // only concerned with first 10 fields
  int field = 0;
  int letter = 0;
  char temp[10];
  int i = 0;
  for (i; i<srcLen;i++) {
    #if VERBOSE_MODE
    Serial.print('_');
    Serial.print(source[i]);
    #endif

    if (source[i] == ',' || letter >= 10) {
      if (field < 10) {
        // store string of field efficiently
        size_t len = strlen(temp);
        #if VERBOSE_MODE
        Serial.print("  ["); Serial.print(len); Serial.println("]");
        #endif
        // char anonField[len];
        // strcpy(anonField, temp);
        // verboseMessage(anonField);
        strcpy(fields[field], temp);
        field++;
        letter = 0;
        memset(temp, 0, 10 * sizeof(char)); // clear buffer
        continue;
      } else {
        break;
      }
    }
    temp[letter] = source[i];
    letter++;
  }
  verboseMessage("field parse");
  
  // parse: 

  // time+date
  *time = atoi(fields[1]);
  *date = atoi(fields[9]);
  

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
}

// normalize nums like "9" and "4" to "09", "04", etc. (for time+date)
void addZerosPrint(int num) {
  if (abs(num) < 10) {
    Serial.print("0");
  } 
  Serial.print(num);
}

void addZerosPrintToFile(int num) {
  if (abs(num) < 10) {
    csvFile.print("0");
  } 
  csvFile.print(num);
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
  int timeoutSec = 3; // value below 5 not recommended for GPS updates at 1Hz
  int timeoutChars = timeoutSec*960/5;

  verboseMessage("start parse method");
  delay(10);
  // 1.
  while(GPSSerial.available()) {
    GPSSerial.read();
  }
  verboseMessage("cleared buffer");

  // 2.
  int countdown = 2000;
  while(!GPSSerial.available()) {
    delay(1);
    countdown--;
    if (countdown < 0) {
      Serial.println("no signal from gps, are you sure GPS TX is connected to pin A2?");
      reboot();
    }
  }
  // 3.
  int i = 0;
  int j = 0;
  bool done = false;
  // only copy found sentence to permanent buffer if it's an rmc (no timeout)
  bool rmcValid = false;
  char sentence[128];
  verboseMessage("begin gps parse");
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
    // write new gps data to file for radio to transmit
    configFile = fatfs.open(CONFIG_FILE, FILE_WRITE | O_CREAT);
    configFile.write(sentence);
    configFile.close();
    reboot();
  }
  Serial.println("DONE!");
  delay(500);
  
}