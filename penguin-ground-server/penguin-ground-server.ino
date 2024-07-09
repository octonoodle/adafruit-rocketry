// rf95_server.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messageing server
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95  if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example rf95_client
// Tested with Anarduino MiniWirelessLoRa, Rocket Scream Mini Ultra Pro with
// the RFM95W, Adafruit Feather M0 with RFM95

#include <SPI.h>
#include <RH_RF95.h>

// Singleton instance of the radio driver
//RH_RF95 rf95;
//RH_RF95 rf95(5, 2); // Rocket Scream Mini Ultra Pro with the RFM95W
RH_RF95 rf95(8, 3); // Adafruit Feather M0 with RFM95

// Need this on Arduino Zero with SerialUSB port (eg RocketScream Mini Ultra Pro)
//#define Serial SerialUSB

int led = 13;

//#define RFM95_RST 11  // "A"
//#define RFM95_CS 10   // "B"
//#define RFM95_INT 6   // "D"
//RH_RF95 rf95(RFM95_CS, RFM95_INT);



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

  // You can change the modulation parameters with eg
  // rf95.setModemConfig(RH_RF95::Bw500Cr45Sf128);

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 2 to 20 dBm:
  rf95.setTxPower(20, false);
  // If you are using Modtronix inAir4 or inAir9, or any other module which uses the
  // transmitter RFO pins and not the PA_BOOST pins
  // then you can configure the power transmitter power for 0 to 15 dBm and with useRFO true.
  // Failure to do that will result in extremely low transmit powers.
  //  driver.setTxPower(14, true);
}

void unpack(uint8_t buffer[RH_RF95_MAX_MESSAGE_LEN], float *alt, float *press, float *temp, float *longit, float *lat, bool *validFix, int *timings) {
  // encoded input message format: 
  //+/- +/- +/-    alt         press       temp        long           lat      fix    time            date     end
  // 1 | 0 | 0 | 2 20 . 29 | 10 27 . 43 | 28 . 73 | 48 . 11 73 | 0 11 . 51 66 | 1 | 12 : 35 : 19 | 3 / 23 / 94 | 0
  // 0   1   2   3  4    5    6  7    8   9    10   11   12 13  14 15   16 17   18  19   20   21  22   23   24 
  // buffer index /\

  *alt = ((bool)buffer[0] ? -1 : 1) * (100*buffer[3] + buffer[4] + (buffer[5]/100.0));
  *press = 100*buffer[6] + buffer[7] + (buffer[8]/100.0);
  *temp = buffer[9] + (buffer[10]/100.0);
  *longit = ((bool)buffer[1] ? -1 : 1) * (buffer[11] + buffer[12]/100.0 + buffer[13]/10000.0);
  *lat = ((bool)buffer[2] ? -1 : 1) * (100*buffer[14] + buffer[15] + buffer[16]/100.0 + buffer[17]/10000.0);
  *validFix = buffer[18];
  for (int i = 0; i < 6; i++) {
    timings[i] = buffer[i+19];
  }
}

// normalize nums like "9" and "4" to "09", "04", etc. (for time+date)
void addZerosPrint(int num) {
  if (abs(num) < 10) {
    Serial.print("0");
  } 
  Serial.print(num);
}

void display(float alt, float press, float temp, float longit, float lat, bool validFix, int *timings) {
  // time
  addZerosPrint(timings[3]); Serial.print("/"); addZerosPrint(timings[4]); Serial.print("/"); addZerosPrint(timings[5]); Serial.print(" ");
  addZerosPrint(timings[0]); Serial.print(":"); addZerosPrint(timings[1]); Serial.print(":"); addZerosPrint(timings[2]);

  // posn
  Serial.print(" long: "); 
  Serial.print(longit, 4);
  Serial.print(" lat: "); 
  Serial.print(lat, 4);
  Serial.print(" fix: "); Serial.println(validFix ? "valid" : "invalid");
  
  // bmp data
  Serial.print("Altitude: "); Serial.println(alt);
  Serial.print("Pressure: "); Serial.println(press);
  Serial.print("Temperature: "); Serial.println(temp);
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
    Serial.println("Message request recieved, decoding...");
    // Should be a message for us now
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    //Serial.println(RH_RF95_MAX_MESSAGE_LEN);
    uint8_t len = sizeof(buf);
    if (rf95.recv(buf, &len)) {
      digitalWrite(led, HIGH);
      RH_RF95::printBuffer("request: ", buf, len);
      Serial.print("got request: ");
      Serial.println((char *)buf);
      if (!strcmp((char*)buf, "data incoming")) {
        // Send a reply
        uint8_t okSignal[] = "ready for data";
        rf95.send(okSignal, sizeof(okSignal));
        rf95.waitPacketSent();
        Serial.println("replied to handshake...");
        digitalWrite(led, LOW);

        // wait for real data
        if (rf95.waitAvailableTimeout(5000)) {
          for(int i; i < RH_RF95_MAX_MESSAGE_LEN; i++) {
            buf[i] = 0; // clear buffer
          }
          len = sizeof(buf);
          if(rf95.recv(buf, &len)) {
            digitalWrite(led, HIGH);
            Serial.print("successfully recieved message (");
            Serial.print(len);
            Serial.println(" bytes):");

            for (int i = 0; i < len; i++) {
              Serial.print(buf[i]);
            Serial.print(", ");
            }
            Serial.println();
            Serial.println();

            // respond to client
            uint8_t newMessage[] = "data recieved!!";
            rf95.send(newMessage, sizeof(newMessage));
            rf95.waitPacketSent();
            Serial.println("acknowldeged data recieve success");
            Serial.println();
            
            // parse recieved data
            float alt;
            float press;
            float temp;
            float longitude; // decimal degrees
            float latitude; // decimal degrees
            bool validFix;
            int timings[6] = {0}; // in format 12 34 56   12 34 56
            //                            HHMMSS     MMDDYY
            unpack(buf, &alt, &press, &temp, &longitude, &latitude, &validFix, timings);
            display(alt, press, temp, longitude, latitude, validFix, timings);

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
