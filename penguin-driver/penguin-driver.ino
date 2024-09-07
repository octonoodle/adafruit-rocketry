// this program collects telemetry for a model rocket and transmits the data to a ground-based server
// written for the Adafruit Feather M0 Express or Adafruit Feather ESP32-S2
// designed to run with companion sketch penguin-ground-server.ino

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

#define GPSSerial Serial1

#define RFM95_RST 11  // "A"
#define RFM95_CS 10   // "B"
#define RFM95_INT 6   // "D"
#define RF95_FREQ 915.0
RH_RF95 rf95(RFM95_CS, RFM95_INT);

Adafruit_BMP3XX bmp;
Adafruit_NeoPixel rgb(1, 8);  //for status updates during testing

/*          PROGRAM OPTIONS          */
float SEALEVELPRESSURE_HPA = 1013.1; 
int dbm = 23; // power of radio (can set from 5 to 23)
int sampleRate = 10; // non-gps sample rate in Hz (theoretical)

#define USE_TEST_MESSAGE false // still read gps serial, but parse and transmit test message instead
#define VERBOSE_MODE false // describe code execution in detail, good for isolating crash points
#define WAIT_FOR_SERIAL false // do not start the code unless the serial port is connected (computer)
#define TOGGLE_ALL_MESSAGES true // if disabled, do not print any messages to Serial. used to increase speed.
#define NO_GPS true // disable GPS when enabled, requires definition of sampling frequency
/*                                  */

void verboseMessage(const char* msg) {
  #if VERBOSE_MODE
  Serial.println(msg);
  #endif
}

void fastPrintln(const char* msg) {
  #if TOGGLE_ALL_MESSAGES
  Serial.println(msg);
  #endif
}
void fastPrintln() {
  fastPrintln("");
}

void fastPrint(const char* msg) {
  #if TOGGLE_ALL_MESSAGES
  Serial.print(msg);
  #endif
}

void localEnterData(float longitude = 360.0, float latitude = 360.0, bool validFix = false, int time = -1, int date = -1);


void setup() {
  serialInit();
  bmpInit();
  flashInit();
  rf95Init();
  gpsInit();
}

void loop() {
  #if !NO_GPS
  parseAndStore(); // gps data collection
  #endif
  transmitData(); // bmp data collection, local data recording, data transmission
}

// ~~~~~~~~~~~~ SETUP ~~~~~~~~~~~~~~
void serialInit() {
  Serial.begin(115200);
  if (WAIT_FOR_SERIAL) {
    while (!Serial) delay(1);
  }
  delay(100);
  for (int i = 0; i < 20; i++) {
    fastPrint("/");
  }
  fastPrint("\n\n");
  fastPrintln("Welcome to CAI Rocketry Software");
  fastPrintln("Driver for Penguin-class electronics payload");
  delay(500);
  fastPrintln();
  for (int i = 0; i < 20; i++) {
    fastPrint("/");
  }
  fastPrint("\n\n");
}
void bmpInit() {
  fastPrintln("Initializing Barometer/Altimiter/Thermometer...");

  String input;
  input = "D";
  // fastPrintln("use default sea level definition or set at current pressure? d/c");
  // while (!Serial.available()) {
  //   delay(10);
  // }
  // input = Serial.readString();
  if (bmp.begin_I2C()) {
    fastPrintln("valid BMP3 sensor found");
    if (input.equals("c")) {
      fastPrint("setting to sensor value... ");
      delay(10);
      if (!bmp.performReading()) {
        fastPrintln("failed");
      } else {
        SEALEVELPRESSURE_HPA = (bmp.pressure);
        fastPrintln("success");
      }
    }
  } else {
    fastPrintln("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }
}
void flashInit() {
  fastPrint("Configuring flash memory... ");
  if (!flash.begin()) {
    fastPrintln("failed");
    while (1);
  }
  fastPrintln("success");

  // First call begin to mount the filesystem.  Check that it returns true
  // to make sure the filesystem was mounted.
  if (!fatfs.begin(&flash)) {
    fastPrintln("Error, failed to mount newly formatted filesystem");
    fastPrintln("Was the flash chip formatted with the fatfs_format example?");
    while (1)
      ;
  }
  fastPrintln("mounted filesystem");
  File32 eraseFlag = fatfs.open("noErase.txt", FILE_READ);
  bool truncate = false;
  if (!eraseFlag) {
    truncate = true;
    eraseFlag = fatfs.open("noErase.txt", FILE_WRITE | O_CREAT);
  }
  eraseFlag.close();

  fastPrint("formatting csv for data recording... ");
  csvFile = 
  truncate ? 
  fatfs.open("bmp390.csv", FILE_WRITE | O_CREAT | O_TRUNC) :
  fatfs.open("bmp390.csv", FILE_WRITE | O_CREAT);
  if (csvFile) {
    if (truncate) {
      csvFile.println("sensor data: ");
      csvFile.println("\"date\",\"time\",\"altitude (m)\",\"pressure (hPa)\",\"temperature (°C)\",\"longitude (°)\",\"latitude (°)\",\"fix valid\"");
      fastPrintln("data file first line written");
    } else {
      fastPrintln("did not print first line");
    }
  } else {
    fastPrintln("failed to start write to data file!");
    while (1);
  }
  csvFile.close();
}

void rf95Init() {
  fastPrint("Radio Client Initializing...");
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if (!rf95.init()) {
    fastPrintln("failed");
    while (1);
  }
  fastPrintln("success");

  rf95.setFrequency(RF95_FREQ);
  #if TOGGLE_ALL_MESSAGES
  Serial.print("frequency setting: ");
  Serial.print(RF95_FREQ);
  Serial.println("MHz");
  #endif

  //Serial.println("enter power setting [5-23]:");
  //while (!Serial.available())
  //  ;
  //int i = 0;
  //char c;
  //char num[2];
  //while (Serial.available() && i < 2) {  //get user defined power input
   // c = Serial.read();
   // num[i] = c;
   // i++;
 // }
  //int power;
  //power = atoi(num);
  rf95.setTxPower(dbm, false);
  fastPrint("power set to ");
  char dbmStr[2];
  sprintf(dbmStr, "%d", dbm);
  fastPrint(dbmStr);
  fastPrintln(" dBm");

  fastPrintln("Radio configured");
}
void gpsInit() {
  fastPrint("Initializing GPS... ");
  GPSSerial.begin(9600);

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
  //   fastPrintln("failed");
  //   while(1);
  // }
  fastPrintln("success");
}

// ~~~~~~~~~~~~~~~~~~~~~~~~ LOCAL DATA STORAGE ~~~~~~~~~~~~~~~~~~~~~~~~~~~
// record data to csv file
// assumes BMP390 and flash memory have already been properly initialized
void localEnterData(float longitude, float latitude, bool validFix, int time, int date) { 
  // default arguments of 360.0 or -1 means copy previously entered values because there are no new ones
  // can ether call localEnterData() for just updated bmp, or localEnterData(long, lat, time, date) which updates every field

  if (!bmp.performReading()) {
    fastPrintln("bmp reading error");
    delay(10);
    return;
  }
  verboseMessage("bmp read");

  // collect data from BMP390 sensor array
  float altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  float pressure = bmp.pressure / 100.0;
  float temperature = bmp.temperature;
  if (!altitude || !pressure || !temperature) {
    fastPrintln("data measurement failure!");
  }
  verboseMessage("bmp valid");

  csvFile = fatfs.open("bmp390.csv", FILE_WRITE | O_CREAT);
  // record data to local file
  if (csvFile) {
    csvFile.print("\"");
    if (date > 0) {
      addZerosPrintToFile((int)floor(date % 10000 /100)); csvFile.print('/'); 
      addZerosPrintToFile((int)floor(date/10000)); csvFile.print('/');
      addZerosPrintToFile(date % 100);
    }
    csvFile.print("\",\"");
    if (time > 0) {
      addZerosPrintToFile((int)floor(time/10000)); csvFile.print(':');
      addZerosPrintToFile((int)floor(time % 10000 /100)); csvFile.print(':'); 
      addZerosPrintToFile((int)(time % 100));
    }
    csvFile.print("\","); 
    csvFile.print(altitude);
    csvFile.print(",");
    csvFile.print(pressure);
    csvFile.print(",");
    csvFile.print(temperature);
    csvFile.print(",");
    if (longitude != 360.0) {
      csvFile.print(longitude, 4);
    }
    csvFile.print(",");
    if (latitude != 360.0) {
      csvFile.print(latitude, 4);
    }
    csvFile.print(",");
    csvFile.println(validFix);
  } else {
    Serial.println("failed to write data to file");
  }
  csvFile.close();

}

// ~~~~~~~~~~~~~~~~~~~~~~~~ RADIO TRANSMISSION ~~~~~~~~~~~~~~~~~~~~~~~~~~~
void transmitData() {
  #if TOGGLE_ALL_MESSAGES
  if (Serial && Serial.available()) {
    fastPrintln("You are typing something");
    String input = Serial.readString();
    if (input.equals("kill")) {
      fastPrintln("terminating datalogging...");\
      while (1)
        ;
    } else {
      fastPrintln("input ignored");
    }
  }
  verboseMessage("kill check");
  #endif

  if (!bmp.performReading()) {
    fastPrintln("bmp reading error");
    delay(10);
    return;
  }
  verboseMessage("bmp read");

  // collect data from BMP390 sensor array
  float altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  float pressure = bmp.pressure / 100.0;
  float temperature = bmp.temperature;
  if (!altitude || !pressure || !temperature) {
    fastPrintln("data measurement failure!");
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
    fastPrintln("error, failed to read gps data file");
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

    #if TOGGLE_ALL_MESSAGES
    #if VERBOSE_MODE
    Serial.println(validFix);
    Serial.println(time);
    Serial.println(date);
    #endif
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
    #endif
  }
  verboseMessage("config closed "); 
  verboseMessage(configFile.close() ? "1" : "0");

  #if TOGGLE_ALL_MESSAGES
  Serial.print("Altitude: "); Serial.println(altitude);
  Serial.print("Pressure: "); Serial.println(pressure);
  Serial.print("Temperature: "); Serial.println(temperature);
  #endif

  localEnterData(longitude, latitude, validFix, time, date);

  uint8_t statusMessage[] = "data incoming";
  rf95.send(statusMessage, sizeof(statusMessage));
  fastPrint("sending request... ");
  rf95.waitPacketSent();

  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (rf95.waitAvailableTimeout(3000)) {
    if (rf95.recv(buf, &len)) {
      if (!strcmp((char *)buf, "ready for data")) {
          fastPrintln("handshake complete");
          float dataPacket[] = {altitude, pressure, temperature, longitude, latitude, (float)validFix, (float)time, (float)date};
          sendAndWait(dataPacket);
        }
      else {
        fastPrint("error, recieved message: ");
        fastPrintln((char *)buf);
      }
    } else {
      fastPrintln("ready check recv failed");
    }
  } else {
    fastPrintln("no response from server during ready check!");
  }

  //delay(1000/sampleRate);
}

// ~~~~~~~~~~~~~~~~~~~~~~~~ RADIO MESSAGE ENCODING ~~~~~~~~~~~~~~~~~~~~~~~~~~~
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
  uint8_t hourUTC = floor(time/10000);
  uint8_t hour = (hourUTC + 19) % 24; // adjust UTC to EST
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
  
  #if TOGGLE_ALL_MESSAGES
  Serial.print("encoded message to send (");
  Serial.print(sizeof(encodedPackets));
  Serial.println(" bytes): ");
  for (int i = 0; i < sizeof(encodedPackets); i++) {
    Serial.print(encodedPackets[i]);
    Serial.print(" ");
  }
  Serial.println();
  #endif
  
  rf95.send(encodedPackets, sizeof(encodedPackets));
  rf95.waitPacketSent();

  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (rf95.waitAvailableTimeout(500)) {
    if (rf95.recv(buf, &len)) {
      if (!strcmp((char *)buf, "data recieved!!")) { // END OF LOOP: TRANSMIT SUCCESS
        fastPrintln("transmit success");
        fatfs.remove(CONFIG_FILE);
        return;
      } else {
        fastPrint("unexpected reply: ");
        fastPrintln((char *)buf);
      }
    } else {
      fastPrintln("data recv failed");
    }
  } else {
    fastPrintln("no response from server during data transfer!");
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
  #if TOGGLE_ALL_MESSAGES
  if (abs(num) < 10) {
    Serial.print("0");
  } 
  Serial.print(num);
  #endif
}

void addZerosPrintToFile(int num) {
  if (abs(num) < 10) {
    csvFile.print("0");
  } 
  csvFile.print(num);
}
// ~~~~~~~~~~~~~~~~~~~~~~~~ GPS DATA PARSING ~~~~~~~~~~~~~~~~~~~~~~~~~~~
// to get RMC sentence:
// 1. clear buffer
// 2. wait for gps data (from peripheral chip)
// 3. slowly read until full sentence is formed
// 4. parse
// 5. finish or try again

// checks if NMEA sentence is RMC (Recommended Minimum Coordinates) aka GPS data
bool isRMC(char * sent) {
  char msgType[7];
  for (int i = 0; i < 6; i++) {
    msgType[i] = sent[i];
  }
  msgType[6] = '\0';
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
      fastPrintln("no signal from gps, are you sure GPS TX is connected to pin A2?");
      return;
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
            verboseMessage("found wrong NMEA sentence, looking again...");
            verboseMessage(sentence);
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
            fastPrintln("overflow");
            j = 0;
          }
      }
      i++;
      if (i > timeoutChars) {
        done = true;
        #if TOGGLE_ALL_MESSAGES
        Serial.println();
        Serial.print("search timeout reached (");
        Serial.print(timeoutChars);
        Serial.println(") chars");
        #endif
      }
    } else {
      done = true;
      #if TOGGLE_ALL_MESSAGES
      Serial.println();
      Serial.print("ended early at index: "); Serial.println(i);
      #endif
    }
  }
  
  if (rmcValid) {
    fastPrintln();
    fastPrint("found: ");
    fastPrintln(sentence);
    // write new gps data to file for radio to transmit
    configFile = fatfs.open(CONFIG_FILE, FILE_WRITE | O_CREAT);
    configFile.write(sentence);
    configFile.close();
  }
  fastPrintln("DONE!");
  delay(500);
  
}