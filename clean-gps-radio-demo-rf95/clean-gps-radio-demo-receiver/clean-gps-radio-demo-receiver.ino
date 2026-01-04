// ground-based reciever to recover model rocket telemetry.
// written for the Adafruit Feather M0 microprocessor with RFM95 LoRa radio
// designed to run with companion sketch clean-gps-radio-demo.ino

#include <SPI.h>
#include <RH_RF95.h>

// Singleton instance of the radio driver
RH_RF95 rf95(8, 3);

int led = 13;

void setup() {
  pinMode(led, OUTPUT);
  Serial.begin(115200);
  while (!Serial);  // Wait for serial port to be available
  Serial.print("radio initializing... ");

  if (!rf95.init()) {
    Serial.println("init failed");
    while(1);
  }
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
  Serial.println("init success");
  rf95.setFrequency(915.0);
  // configure modem for low datarate, ultra-long-range settings
  // Bw = 31.25 kHz, Sf = 4096chips/symbol
  rf95.setSignalBandwidth(250000);
  rf95.setSpreadingFactor(12);


  rf95.setTxPower(20, false);
}

// normalize nums like "9" and "4" to "09", "04", etc. (for time+date)
void addZerosPrint(int num) {
  if (abs(num) < 10) {
    Serial.print("0");
  } 
  Serial.print(num);
}

void display(float alt, float press, float temp, float longit, float lat, bool validFix, int time, int date) {
  // time
  // HH MM SS DD MM YY
  int timings[] = {(int)floor(time/10000), (int)floor((time % 10000) / 100), time % 100, (int)floor(date/10000), (int)floor((date % 10000) / 100), date % 100, };

  addZerosPrint(timings[4]); Serial.print("/"); addZerosPrint(timings[3]); Serial.print("/"); addZerosPrint(timings[5]); Serial.print(" ");
  addZerosPrint((timings[0] - 4) % 24); Serial.print(":"); addZerosPrint(timings[1]); Serial.print(":"); addZerosPrint(timings[2]);

  // posn
  Serial.print(" long: "); 
  Serial.print(longit, 4);
  Serial.print(" lat: "); 
  Serial.print(lat, 4);
  Serial.print(" fix: "); Serial.println(validFix ? "valid" : "invalid");
  
  // bmp data
  // Serial.print("Altitude: "); Serial.println(alt);
  // Serial.print("Pressure: "); Serial.println(press);
  // Serial.print("Temperature: "); Serial.println(temp);
}

void rmcDecode(uint8_t *source, float alt, float press, float tempr, int src_len) {
  /*
  example message:
  $GPRMC,123519,A,4807.038,N,01131.000,E,0.022,269.131,230394,,,A,C*
  name   time  fix   long      lat      speed  sat angle date        always ends with *
  */

  // Serial.print("method source: "); 
  // for (int i = 0; i < src_len; i++) {
  //   Serial.print((char)(source[i]));
  // }
  // Serial.println();

  // verboseMessage("before decode");
  char fields[10][10]; // only concerned with first 10 fields
  int field = 0;
  int letter = 0;
  char temp[10];
  int i = 0;
  for (i; i < src_len; i++) {
    // #if VERBOSE_MODE
    // Serial.print('_');
    // Serial.print(source[i]);
    // #endif

    if (source[i] == ',' || letter >= 10) {
      if (field < 10) {
        // store string of field efficiently
        size_t len = strlen(temp);
        // #if VERBOSE_MODE
        // Serial.print("  ["); Serial.print(len); Serial.println("]");
        // #endif
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
  // verboseMessage("field parse");
  
  // parse: 

  // time+date
  int time = atoi(fields[1]);
  int date = atoi(fields[9]);
  

  // position data
  float longitBuf = (float)atof(fields[3]);
  float longit = (float)floor(longitBuf/100.0) + fmodf(longitBuf, 100.0)/60; // DDMM.MMM --> DD.DDDD
  if (fields[4][0] == 'S') { // south
    longit = longit * -1;
  }

  float latBuf = (float)atof(fields[5]);
  float lat = (float)floor(latBuf/100.0) + fmodf(latBuf, 100.0)/60; // DDDMM.MMM --> DDD.DDDD
  if (fields[6][0] == 'W') { // west
    lat = lat * -1;
  }
  bool validFix = (fields[2][0] == 'A');

  display(alt, press, tempr, longit, lat, validFix, time, date);
}

void loop() {
  if (Serial.available()) {
    Serial.println("You are typing something");
    String input = Serial.readString();
    if (input.equals("kill\n")) {
      Serial.println("terminating datalogging...");
      //rgb.setPixelColor(0, 0, 0, 255);
      //rgb.show();
      while (1)
        ;
    } else {
      Serial.println("input ignored");
    }
  }

  if (rf95.waitAvailableTimeout(10000)) {
    // Serial.println("Message request recieved, decoding...");
    // Should be a message for us now
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    //Serial.println(RH_RF95_MAX_MESSAGE_LEN);
    uint8_t len = sizeof(buf);
    if (rf95.recv(buf, &len)) {
      digitalWrite(led, HIGH);
      // RH_RF95::printBuffer("request: ", buf, len);
      // Serial.print("got request: ");
      // Serial.println((char *)buf);
      if (!strcmp((char*)buf, "data incoming")) {
        // Send a reply
        uint8_t okSignal[] = "ready for data";
        rf95.send(okSignal, sizeof(okSignal));
        rf95.waitPacketSent();
        // Serial.println("replied to handshake...");
        digitalWrite(led, LOW);

        // wait for real data
        if (rf95.waitAvailableTimeout(10000)) {
          memset(buf, 0, RH_RF95_MAX_MESSAGE_LEN);// clear buffer
          
          len = sizeof(buf);
          if(rf95.recv(buf, &len)) {
            digitalWrite(led, HIGH);
            // Serial.print("successfully recieved message (");
            // Serial.print(len);
            // Serial.println(" bytes):");

            for (int i = 0; i < len; i++) {
              Serial.print((char)(buf[i]));
            }
            Serial.println();
            // Serial.println();

            // respond to client
            uint8_t newMessage[] = "data recieved!!";
            rf95.send(newMessage, sizeof(newMessage));
            rf95.waitPacketSent();
            // Serial.println("acknowldeged data recieve success");
            // Serial.println();
            

            rmcDecode(buf, -1.0, -1.0, -1.0, len);
            Serial.print("message RSSI: ");
            Serial.println(rf95.lastRssi());
            Serial.println();

          } else {
            Serial.println("data recv failed");
          }
          digitalWrite(led, LOW);
        } else {
          Serial.println("did not recieve data after handshake");
        }
      } else {
        Serial.println("malformed handshake");
      }
    } else {
      Serial.println("request recv failed");
    }
  } else {
    Serial.println("no data recieved, listening again for 10s");
  }
}