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
#define SERIAL_MSG false // enable info messages

//#define RFM95_RST 11  // "A"
//#define RFM95_CS 10   // "B"
//#define RFM95_INT 6   // "D"
//RH_RF95 rf95(RFM95_CS, RFM95_INT);

void fastPrint(const char* msg) {
  #if SERIAL_MSG
  Serial.print(msg);
  #endif
}

void fastPrintln(const char* msg) {
  #if SERIAL_MSG
  Serial.println(msg);
  #endif
}

void setup() {
  pinMode(led, OUTPUT);
  #if SERIAL_MSG
  Serial.begin(115200);
  while (!Serial);  // Wait for serial port to be available
  Serial.print("radio initializing... ");
  #endif

  if (!rf95.init())
    fastPrintln("init failed");
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
  fastPrintln("init success");
  rf95.setFrequency(915.0);

  // You can change the modulation parameters with eg
  // rf95.setModemConfig(RH_RF95::Bw500Cr45Sf128);

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 2 to 20 dBm:
  //rf95.setTxPower(20, false);
  // If you are using Modtronix inAir4 or inAir9, or any other module which uses the
  // transmitter RFO pins and not the PA_BOOST pins
  // then you can configure the power transmitter power for 0 to 15 dBm and with useRFO true.
  // Failure to do that will result in extremely low transmit powers.
  //  driver.setTxPower(14, true);
}

void unpack(float target[3], uint8_t buffer[RH_RF95_MAX_MESSAGE_LEN]) {
  #if SERIAL_MSG
  for (int i = 0; i < 8; i++) {
    Serial.print(buffer[i]);
    Serial.print(", ");
  }
  Serial.println();
  #endif
  float sum = 100*buffer[0] + buffer[1] + (buffer[2]/100.0);
  target[0] = sum;
  sum = 100*buffer[3] + buffer[4] + (buffer[5]/100.0);
  target[1] = sum;
  sum = buffer[6] + (buffer[7]/100.0);
  target[2] = sum;
}

void loop() {
  #if SERIAL_MSG
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
  #endif

  if (rf95.waitAvailableTimeout(10000)) {
    fastPrintln("Message request recieved, decoding...");
    // Should be a message for us now
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf95.recv(buf, &len)) {
      digitalWrite(led, HIGH);
      #if SERIAL_MSG
      RH_RF95::printBuffer("request: ", buf, len);
      Serial.print("got request: ");
      Serial.println((char *)buf);
      #endif

      // Send a reply
      uint8_t okSignal[] = "ready for data";
      rf95.send(okSignal, sizeof(okSignal));
      rf95.waitPacketSent();
      fastPrintln("replying to handshake...");
      digitalWrite(led, LOW);

      // wait for real data
      while(!rf95.available());

      if(rf95.recv(buf, &len)) {
        digitalWrite(led, HIGH);
        fastPrintln("successfully recieved data: ");
        float unpackedData[3] = {0, 0, 0};
        unpack(unpackedData, buf);

        for (int i = 0; i < 3; i++) {
          Serial.println(unpackedData[i]);
        }
        Serial.println();

        uint8_t newMessage[] = "data recieved!!";
        rf95.send(newMessage, sizeof(newMessage));
        rf95.waitPacketSent();
        fastPrintln("acknowldeged data recieve success");
      } else {
        fastPrintln("data recv failed");
      }
      digitalWrite(led, LOW);
    } else {
      fastPrintln("request recv failed");
    }
  } else {
    Serial.println("no data recieved, listening again for 10s");
  }
}
