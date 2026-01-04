/*
Program for Homemade Chute Release
Copyright (c) Auren Amster, October 2025

control code for a chute release to be used in model rocketry

v1: Jun '25
v2: Oct '25 (update flash library, add rp2040 support, update bmp581 library)
*/

/*/ program options /*/
// pins
#define SOLENOID_PIN 10  // release mechanism
#define BMP_INTERRUPT_PIN 12      // data ready interrupt

// velocity and acceleration calculation
#define ACCEL_ARRAY_LENGTH 10     // how many positions to hold for finding acceleration
#define VELOCITY_SAMPLE_LENGTH 3  // how many positions at each end of array to use for v1 and v0
// ex: array length 10, sample length 3:
// [* * *] * * * * [* * *]
// v1=x[2]-x[0]   v0=x[9]-x[7]
// a = v1 - v0 / arr_time[8] - arr_time[1]
#define ACCEL_LIFTOFF_THRESHOLD_M_S2 40.0

#define CHECK_DEPLOY_THRESHOLD_CT 20  // how many times to check alt before being sure to deploy

// constants
#define SEALEVELPRESSURE_HPA 1009.0
#define DATA_FILE "data.csv"

// debug etc
#define WAIT_FOR_SERIAL true
#define ENABLE_DATALOGGING true
#define BMP3_USE_INTERRUPT_PIN false  // timer vs hardware interrupt
// sampling bmp data at 100 Hz. Putting BMP into 200 Hz sampling mode
/*///////////////////*/

#if defined(ADAFRUIT_ITSYBITSY_M0)
#define USING_TIMER_TC3 true
#define USING_TIMER_TC4 false  // Not to use with Servo library
#define USING_TIMER_TC5 false
#define USING_TIMER_TCC false
#define USING_TIMER_TCC1 false
#define USING_TIMER_TCC2 false
#define SELECTED_TIMER TIMER_TC3
#elif defined(ESP32)
#define SELECTED_TIMER 1  // use ESP timer1
#elif defined(ARDUINO_ARCH_RP2040)
#define SELECTED_TIMER 0 // use pseudo-hardware timer 0
#endif

#define TIMER_INTERVAL_MS 10  //
#define TIMER_INTERRUPT_DEBUG 0
#define _TIMERINTERRUPT_LOGLEVEL_ 1

// libraries
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP5xx.h"
#if ENABLE_DATALOGGING
#include <SPI.h>
#include "SdFat_Adafruit_Fork.h"
#include <Adafruit_SPIFlash.h>
#endif
#include <Wire.h>
#include "TimerInterrupt_Generic.h"
#include "ISR_Timer_Generic.h"


// global variables
Adafruit_BMP5xx bmp;
#if ENABLE_DATALOGGING
#include "flash_config.h"
Adafruit_SPIFlash flash(&flashTransport);
FatVolume fatfs;
File32 data_file;
#endif

#if defined(ESP32)
ESP32Timer ITimer(SELECTED_TIMER);
#elif defined(_SAMD21_)
SAMDTimer ITimer(SELECTED_TIMER);
#elif defined(ARDUINO_ARCH_RP2040)
RPI_PICO_Timer ITimer(SELECTED_TIMER);
#else
#error device unsupported for timer library!
#endif
ISR_Timer My_ISR_Timer;

// global variables and interrupt functions
bool launched = false;
bool deployed = false;
bool fresh_data = false;
volatile bool get_data = true;
volatile int secs = 0;
volatile int millisecs = 0;

float last_new_data_time;  // timestamp of latest data in seconds
float last_altitude;
float past_positions[ACCEL_ARRAY_LENGTH];
float past_position_timestamps[ACCEL_ARRAY_LENGTH];
float current_accel = 0;  // calculated acceleration

float deploy_altitude;
float ground_altitude;
int temp_deploy_ct = 0;  // times deploy was triggered so far

#ifdef ESP32
bool IRAM_ATTR run_isr(void* timerNo) {
#elif ARDUINO_ARCH_RP2040
bool run_isr(struct repeating_timer *t) {
	(void) t;
#else
void run_isr() {
#endif

  My_ISR_Timer.run();
#if defined(ESP32) || defined(ARDUINO_ARCH_RP2040)
  return 1;
#endif
}

void bmp_sample() {
  get_data = true;
  // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  // Serial.println("hi");
}

void second_ticker() {
  secs++;
  millisecs = 0;
}

void millisecond_ticker() {
  millisecs++;
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SOLENOID_PIN, OUTPUT);
  pinMode(BMP_INTERRUPT_PIN, INPUT_PULLUP);
  delay(10);
  digitalWrite(SOLENOID_PIN, LOW);

  Serial.begin(500000);
#if WAIT_FOR_SERIAL
  while (!Serial) {
    delay(1);
  }
#else
  delay(10);
#endif
  Serial.println("*** welcome to chute release ***\n");

#if ENABLE_DATALOGGING
  flash_init();
  maybe_recover_data();
  wipe_file();
#endif

  bmp_init();
  initialize_positions();
  select_deploy_altitude();

  // interrupts
  ITimer.attachInterruptInterval(1000, run_isr);

#if BMP3_USE_INTERRUPT_PIN
  attachInterrupt(digitalPinToInterrupt(BMP_INTERRUPT_PIN), bmp_sample, RISING);
#else
  Serial.print("configuring timer-based sampling interrupt...");
  if (My_ISR_Timer.setInterval(TIMER_INTERVAL_MS, bmp_sample) == 0) {  //ITimer.attachInterruptInterval_MS(TIMER_INTERVAL_MS, bmp_sample)) {
    Serial.println("success");
  } else {
    Serial.println("failed");
    while (1)
      ;
  }
#endif

  // timing interrupts
  // ITimer.attachInterruptInterval_MS(1000, second_ticker);
  // ITimer.attachInterruptInterval_MS(1, millisecond_ticker);
  My_ISR_Timer.setInterval(1000, second_ticker);
  My_ISR_Timer.setInterval(1, millisecond_ticker);
}

void loop() {
  if (get_data && !fresh_data) {  // avoid get_data loop from too-fast interrupt
    bmp.performReading();
    // Serial.println("bmp");
    last_new_data_time = secs + 0.001 * millisecs;
    fresh_data = true;
    get_data = false;
    last_altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  } else {
    if (launched) {
      if (deployed) {
        if (last_altitude > ground_altitude + 10) {
          Serial.println("i've deployed");
#if ENABLE_DATALOGGING
          log_data_csv();
#endif
        } else {
          Serial.println("i've landed");
          delay(200);
        }
      } else {
        if (fresh_data) {
          // update_positions();
          check_deploy();

#if ENABLE_DATALOGGING
          log_data_csv();
#endif

          fresh_data = false;
        }
      }

    } else {
      if (fresh_data) {
        update_positions();
        check_liftoff();
        // Serial.println(bmp.readAltitude(SEALEVELPRESSURE_HPA));
        fresh_data = false;
      } else {
        // Serial.println("no data yet.....");
      }
    }
  }
  delayMicroseconds(100);  // for stability
  // delay(10);
}

void bmp_init() {
  // Serial.print("manually configuring bmp390...");
  // Wire.begin();
  // Wire.beginTransmission(BMP3XX_DEFAULT_ADDRESS);
  // Wire.write(0x19); // INT_CTRL register (to read from)
  // Wire.endTransmission();

  // Wire.requestFrom(BMP3XX_DEFAULT_ADDRESS, 1); // get one byte (INT_CTRL value) from sensor
  // while(!Wire.available());
  // uint8_t prev_val = Wire.read();
  // Serial.println(prev_val, HEX);

  // Wire.beginTransmission(BMP3XX_DEFAULT_ADDRESS);
  // Wire.write(0x19); // INT_CTRL register
  // Wire.write((prev_val | 0b01000000) & ~(0b00011000) ); // enable DATA_READY interrupt, disable other modes
  // Wire.endTransmission();
  // Wire.end();
  // Serial.println("success");


  Serial.print("initializing bmp581...");
  if (!bmp.begin(BMP5XX_ALTERNATIVE_ADDRESS, &Wire)) {
    Serial.println("failed");
    while (1)
      ;
  }
  Serial.println("success");

  Serial.print("configuring built-in options...");
  if (!bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_8X)
      || !bmp.setOutputDataRate(BMP5XX_ODR_240_HZ)
      || !bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3)) {  // data smoothing
    Serial.println("failed");
    while (1)
      ;
  }
  Serial.println("success");
}

#if ENABLE_DATALOGGING
void flash_init() {
  Serial.print("Configuring flash memory... ");
  if (!flash.begin()) {
    Serial.println("failed");
    while (1)
      ;
  }
  Serial.println("success");

  Serial.print("mounting filesystem...");
  if (!fatfs.begin(&flash)) {
    Serial.println("failed");
    while (1)
      ;
  }
  Serial.println("success");
}

void wipe_file() {
  Serial.println("opening data file...");
  data_file = fatfs.open(DATA_FILE, FILE_WRITE | O_CREAT | O_TRUNC);
  if (data_file) {
    data_file.println("\"seconds\",\"millisec\",\"altitude (m)\",\"pressure (hPa)\",\"temperature (°C)\"");
    data_file.close();
    Serial.println("success");
  } else {
    Serial.println("failed");
  }
}
#endif

// populate position memory to properly initialize it
void initialize_positions() {
  Serial.print("initialize position array...");
  delay(10);
  if (!bmp.performReading()) {
    Serial.println("failed (1)");
    while (1)
      ;
  }
  delay(10);
  if (!bmp.performReading()) {
    Serial.println("failed (2)");
    while (1)
      ;
  }
  float base_alt = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  for (int i = 0; i < ACCEL_ARRAY_LENGTH; i++) {
    past_positions[i] = base_alt;
    past_position_timestamps[i] = -1 * i;  // avoid delta-t of 0
  }
  last_altitude = base_alt;
  ground_altitude = base_alt;

  Serial.print("success (");
  Serial.print(base_alt);
  Serial.println(" m)");
}

void select_deploy_altitude() {
  char alt_str[32];
  Serial.print("what altitude would you like to deploy the parachute at? ");
  while (!Serial.available()) {
    delay(10);
  }
  Serial.readBytesUntil(0, alt_str, sizeof(alt_str) / sizeof(char));
  deploy_altitude = atof(alt_str);
  Serial.println(deploy_altitude);
}

// main loop / interrupt functions
void update_positions() {
  for (int i = ACCEL_ARRAY_LENGTH - 1; i > 0; i--) {
    past_positions[i] = past_positions[i - 1];
    past_position_timestamps[i] = past_position_timestamps[i - 1];
  }

  past_positions[0] = last_altitude;
  past_position_timestamps[0] = last_new_data_time;
  Serial.print("[");
  Serial.print(past_positions[0]);
  Serial.print(",");
  Serial.print(past_position_timestamps[0], 3);
  Serial.println("]");
}

bool hit_upwards = false;    // have we gotten to deploy alt the first time
bool hit_downwards = false;  // have we gotten to deploy alt the second time'
int start_delay_sec;
void check_deploy() {  // are we up yet?
  if (!hit_upwards) {
    hit_upwards = (last_altitude > deploy_altitude);
    Serial.println("before up");
    if (hit_upwards) {
      start_delay_sec = secs;  // begin deploy delay
    }
  } else {  // are we down yet?
    // delay deployment checking for 2 - 3 seconds
    if (secs < start_delay_sec + 3) {
      return;
    }

    if (!hit_downwards) {
      hit_downwards = (last_altitude < deploy_altitude);
      temp_deploy_ct = 0;
      Serial.println("up and coming down");
    } else {
      if (temp_deploy_ct > CHECK_DEPLOY_THRESHOLD_CT) {
        // pulse solenoid on to release parachute !!!!!!!!!!!
        digitalWrite(SOLENOID_PIN, HIGH);
        delay(1000);
        digitalWrite(SOLENOID_PIN, LOW);
        deployed = true;
      } else {
        temp_deploy_ct++;
        Serial.print("deploy count at: "); Serial.println(temp_deploy_ct);
      }
    }
  }
}

void check_liftoff() {
  // Serial.println("are we there yet");
  // a = (v1 - v0) / dt
  // v = (x1 - x0) / dt
  float v0 =
    (past_positions[ACCEL_ARRAY_LENGTH - 1] - past_positions[ACCEL_ARRAY_LENGTH - VELOCITY_SAMPLE_LENGTH]) / (past_position_timestamps[ACCEL_ARRAY_LENGTH - 1] - past_position_timestamps[ACCEL_ARRAY_LENGTH - VELOCITY_SAMPLE_LENGTH]);

  float v1 =
    (past_positions[VELOCITY_SAMPLE_LENGTH - 1] - past_positions[0]) / (past_position_timestamps[VELOCITY_SAMPLE_LENGTH - 1] - past_position_timestamps[0]);

  float a = (v1 - v0) / ((past_position_timestamps[ACCEL_ARRAY_LENGTH - 1] + past_position_timestamps[ACCEL_ARRAY_LENGTH - VELOCITY_SAMPLE_LENGTH] - (past_position_timestamps[VELOCITY_SAMPLE_LENGTH - 1] + past_position_timestamps[0])) / 2);
  // time "at" v: center of time interval = avg time: (t1 - t0) / 2
  Serial.print("accel: ");
  Serial.print(a, 3);
  Serial.println(" m/s^2");

  if (abs(a) > ACCEL_LIFTOFF_THRESHOLD_M_S2) {
    Serial.println("lifting off!");
    launched = true;
  }
}

#if ENABLE_DATALOGGING
void log_data_csv() {
  data_file = fatfs.open(DATA_FILE, FILE_WRITE | O_CREAT);
  if (data_file) {
    data_file.print(secs);
    data_file.print(",");
    data_file.print(millisecs);
    data_file.print(",");
    data_file.print(last_altitude);
    data_file.print(",");
    data_file.print(bmp.pressure / 100);
    data_file.print(","); /* convert Pa to hPa */
    data_file.print(bmp.temperature);
    data_file.println();
  } else {
    Serial.println("error, could not write to file");
  }
  data_file.close();
}

void maybe_recover_data() {
  char buffer[50];
  memset(buffer, 0, sizeof(buffer));
  Serial.print("do you want to stop the program and recover data? (y/n): ");
  while (!Serial.available())
    ;
  int i = 0;
  while (Serial.available()) {
    buffer[i] = Serial.read();
    i++;
  }
  Serial.println(buffer);

  if (strcmp(buffer, "y")) {  // not equals y
    return;
  } else {
    while (!Serial.available()) delayMicroseconds(10);
    // Open the file for reading and check that it was successfully opened.
    // The FILE_READ mode will open the file for reading.
    File32 readingFile = fatfs.open(DATA_FILE, FILE_READ);
    if (readingFile) {
      // File was opened, now print out data character by character until at the
      // end of the file.
      //Serial.println("Opened file, printing contents below:");
      while (readingFile.available()) {
        // Use the read function to read the next character.
        // You can alternatively use other functions like readUntil, readString, etc.
        // See the fatfs_full_usage example for more details.
        char c = readingFile.read();
        Serial.print(c);
      }
    } else {
      Serial.print("Failed to open file \"");
      Serial.print(DATA_FILE);
      Serial.print("\" !! Does it exist?");
    }

    while (1) delay(10);
  }
}
#endif