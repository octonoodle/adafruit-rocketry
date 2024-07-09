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
#define TOGGLE_ALL_MESSAGES false // if disabled, do not print any messages to Serial. used to increase speed.

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

// important flag: this controls whether the chip is running
// in radio transmit mode (1) or gps data collection mode (0)
bool TRANSMIT_MODE;

void setup() {
  serialInit();
  bmpInit();
  rf95Init();
  transmitData();
}

void loop() {
  // this will terminate with a reboot once data is successfully transmitted
  transmitData();
}

// reboot the chip (same as pressing RST button)
void reboot() {
  fastPrintln();
  fastPrintln("REBOOTING...");
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

  //fastPrintln("enter power setting [5-23]:");
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
  fastPrint("power set to ");
  char dbm[2];
  sprintf(dbm, "%d", 42);
  fastPrint(dbm);
  fastPrintln(" dBm");

  fastPrintln("Radio configured");
}

// ~~~~~~~~~~~~~~~~~~~~~~~~ RADIO TRANSMISSION ~~~~~~~~~~~~~~~~~~~~~~~~~~~
void transmitData() {

  if (!bmp.performReading()) {
    fastPrintln("reading error");
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

  #if TOGGLE_ALL_MESSAGES
  Serial.print("Altitude: "); Serial.println(altitude);
  Serial.print("Pressure: "); Serial.println(pressure);
  Serial.print("Temperature: "); Serial.println(temperature);
  #endif

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
          float dataPacket[] = {altitude, pressure, temperature};
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

  delay(1000);
}

//xxxx.xx,yyyy.yy,zz.zz
void sendAndWait(float dataSet[3]) {
  // xxyy.zz
  float alt = dataSet[0];
  float pressure = dataSet[1];
  float temp = dataSet[2];

  // each float needs to be broken up into integer sections: 1234.56 -> [12], [34], [56]

  // since we already know if values are negative, take abs to get pure digits and avoid negative overflow on unsigned ints
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

  uint8_t encodedPackets[] = 
  {alt1, alt2, alt3, pressure1, pressure2, pressure3, temp1, temp2};
  
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
      if (!strcmp((char *)buf, "data recieved!!")) { // END CONDITION: TRANSMIT SUCCESS AND REBOOT
        fastPrintln("transmit success");
        //fatfs.remove(CONFIG_FILE);
        //reboot();
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