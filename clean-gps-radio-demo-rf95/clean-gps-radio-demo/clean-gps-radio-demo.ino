#include <RH_RF95.h>
#include <SPI.h>

#ifdef ARDUINO_ARCH_RP2040  // probably the wrong macro for this
#define RFM95_RST 11        // "A"
#define RFM95_CS 10         // "B"
#define RFM95_INT 8         // "D"
// #define SCK 18
// #define MOSI 19
// #define MISO 20
// #define RX 1
// #define TX 0

#else
#define RFM95_RST 11  // "A"
#define RFM95_CS 10   // "B"
#define RFM95_INT 6   // "D"
#endif

#define RF95_FREQ 915.0
int dbm = 20;  // power of radio (can set from 5 to 23)
RH_RF95 rf95(RFM95_CS, RFM95_INT);


#define GPSSerial Serial1
#define GPS_BAUD_RATE 115200
#define NMEA_MAX_LENGTH 82
#define BIG_BOY_LENGTH 6 * NMEA_MAX_LENGTH // size of the large buffer

void setup() {
  Serial.begin(500000);
  // while (!Serial && millis() > 2000);
  //   ;
  Serial.println("gps beacon demo");
  rf95Init();
  gpsInit();
}

void loop() {
  char rmc[NMEA_MAX_LENGTH] = "";
  findRMC(rmc, NMEA_MAX_LENGTH);
  Serial.print("found rmc: ");
  Serial.println(rmc);

  // for the sending the radio thingy
  uint8_t rmcbetter[NMEA_MAX_LENGTH];
  int b = 0;
  while(rmc[b] != 0) rmcbetter[b] = (uint8_t)rmc[b++];
  
  uint8_t statusMessage[] = "data incoming";
  rf95.send(statusMessage, sizeof(statusMessage));
  Serial.println("sending request... ");
  rf95.waitPacketSent();

  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  Serial.println("awaiting response");
  if (rf95.waitAvailableTimeout(10000)) {
    if (rf95.recv(buf, &len)) {
      if (!strcmp((char *)buf, "ready for data")) {
          Serial.println("handshake complete");

          rf95.send(rmcbetter, strlen(rmc));
          rf95.waitPacketSent();

          uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
          uint8_t len = sizeof(buf);

          if (rf95.waitAvailableTimeout(5000)) {
            if (rf95.recv(buf, &len)) {
              if (!strcmp((char *)buf, "data recieved!!")) { // END OF LOOP: TRANSMIT SUCCESS
                Serial.println("transmit success");
                return;
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
  //char num[2];
  //while (Serial.available() && i < 2) {  //get user defined power input
  // c = Serial.read();
  // num[i] = c;
  // i++;
  // }
  //int power;
  //power = atoi(num);
  // configure modem for low datarate, ultra-long-range settings
  // Bw = 31.25 kHz, Sf = 4096chips/symbol
  rf95.setSignalBandwidth(250000);
  rf95.setSpreadingFactor(12);
  Serial.println("set bandwith to 31.25kHz, spreading factor 4096 chips/symbol (long range settings)");

  rf95.setTxPower(dbm, false);
  Serial.print("power set to ");
  char dbmStr[2];
  sprintf(dbmStr, "%d", dbm);
  Serial.print(dbmStr);
  Serial.println(" dBm");

  Serial.println("Radio configured");
}

int calc_checksum(const char* cmd) {
  int sum;
  for (int i = 0; i < strlen(cmd); i++) {
    sum = sum ^ cmd[i];
  }
  return sum;
}

void send_gps_command(const char *cmd) {
  // NMEA cmd format: $...*XX, where ... is the command and comma-separated arguments, and XX is the XOR checksum
  char formatted[NMEA_MAX_LENGTH] = "$"; 
  strcat(formatted, cmd);
  sprintf(formatted, "%s*%X", formatted, calc_checksum(cmd));
  // Serial.println(formatted);
  GPSSerial.println(formatted);
  delay(50); // let the gps chip process its instructions
}

void gpsInit() {
  Serial.print("Initializing GPS... ");
  // quite important to ensure RMC sentences are definitely captured
  // GPSSerial.setFIFOSize(BIG_BOY_LENGTH);

  GPSSerial.begin(9600);
  while(!GPSSerial) delay(10);

  // Change Ultimate GPS baud rate to 115200 in preparation for high frequency updates (10Hz)
  char command[64] = "";
  sprintf(command, "PMTK251,%d",GPS_BAUD_RATE);
  send_gps_command(command);  // config command for gps: set baudrate to GPS_BAUD_RATE (likely 115200)
  GPSSerial.end();
  delay(50);
  GPSSerial.begin(GPS_BAUD_RATE);
  while (!GPSSerial) delay(10);

  // further GPS configuration
  send_gps_command("PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0"); // select only RMC messages
  send_gps_command("PMTK220,100"); // set update frequency to 10Hz
  
  Serial.println("success");
}

// checks if NMEA sentence is RMC (Recommended Minimum Coordinates) aka GPS data
bool isRMC(char *sent) {
  char msgType[7];
  for (int i = 0; i < 6; i++) {
    msgType[i] = sent[i];
  }
  msgType[6] = '\0';
  return !strcmp(msgType, "$GNRMC");
}

// read the GPS Serial port until an RMC sentence is found
void findRMC(char *buf, size_t buf_len) {
  // Serial.println("finding rmc");

  // Serial.println("hi");
  // clear serial buffer
  while (GPSSerial.available()) {
    GPSSerial.read();
  }
// Serial.println("hi");
  while (!GPSSerial.available()) delay(10);
// Serial.println("hi");
  char nmea[NMEA_MAX_LENGTH] = "";
  int i = 0; // buffer index
  bool done = false;
  while (!done) {
    // Serial.println("hi");
    if (GPSSerial.available()) {
      char c = GPSSerial.read();
      // Serial.println("hey");
      if (c != '$') {
        continue; // skip until beginning of nmea sentence
        // Serial.println("bye");
      } else {
        i = 0; 
        memset(nmea, 0, NMEA_MAX_LENGTH); // clear buffer
        // Serial.println("bing");
        nmea[i] = c;
        while(nmea[i] != '*' && i < NMEA_MAX_LENGTH) {
          i++;
          c = GPSSerial.read();
          if (c == '$') {
            // wuh oh
            break;
          }
          nmea[i] = c;
          // delayMicroseconds((int)1000000/GPS_BAUD_RATE); // number of microsec taken per symbol
          while(!GPSSerial.available()) delayMicroseconds(1);
        }
        // Serial.println("bong");
        // finished. ran out of space or finished the sentence
        if (isRMC(nmea) && nmea[strlen(nmea) - 1] == '*') {
          memset(buf, 0, buf_len);
          memcpy(buf, nmea, buf_len);
          // Serial.println("woah");
          // Serial.println("found valid rmc");
          done = true;
        } else {
          Serial.print("did not find an rmc, got: "); Serial.println(nmea);
        }
      }
    } else {
      // Serial.println("poo");
      delay(5);
    }
  }
  // Serial.println("hi");
}