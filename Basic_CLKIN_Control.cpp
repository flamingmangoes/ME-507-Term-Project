/** @file bedagkar.cpp
 *  This file contains code used to read the Hall sensor in an ESP32 board and return 
 *  the magnetic field value in mT.
 * @author Neil Bedagkar, with comment format copied from John Ridgely's "pleasedontcrash.cpp"
 *  and calibration assistance from ChatGPT's GPT-5 mini model included in Github Copilot.
 */

#include <Arduino.h>
#include <SPI.h>
#include "PrintStream.h"
SPIClass vspi(VSPI);


// Pin and variable definitions
float freq;
const unsigned long SAMPLE_MS = 100;  // sample window (100 ms)
const int PIN_SCLK  = 18;   // VSPI SCLK
const int PIN_MISO  = 19;   // VSPI MISO  (SDATAO)
const int PIN_MOSI  = 23;   // VSPI MOSI  (SDATAI)
const int PIN_SCS   = 5;    // DRV8308 SCS, ACTIVE HIGH
const int PIN_EN    = 27;    // DRV8308 ENABLE, SET HIGH TO ENABLE
const int PIN_CLKIN = 17;   // DRV8308 CLKIN (from LEDC PWM)
const int PIN_DIR   = 14;   // Optional DIR (set as you like)
const int PIN_FGOUT = 33;    // FGOUT Hall sense output that CLKIN signal matches
const int PIN_FAULTn = 34;   // FAULTn output from DRV8308
const int PIN_LOCKn = 35;   // LOCKn output from DRV8308
const int ledChannel = 0;
const int resolution = 8; // 8-bit resolution


// Helper code for SCS set for communication
inline void scs_begin() 
{ 
  digitalWrite(PIN_SCS, HIGH); 
}

inline void scs_end()   
{ 
  digitalWrite(PIN_SCS, LOW);  
}


// Helper code for ENABLE pin for communication
inline void enable()
{ 
  digitalWrite(PIN_EN, HIGH); 
}

inline void disable()
{ 
  digitalWrite(PIN_EN, LOW); 
}


// Initialize SCS and Enable pins, set SCS and Enable inactive 
void init_pins()
{
  pinMode(PIN_SCS, OUTPUT);
  scs_end(); // SCS inactive
  pinMode(PIN_EN, OUTPUT);
  disable(); // ENABLE inactive
  pinMode(PIN_FGOUT, INPUT);
}


// 24-bit DRV frame to write on MOSI: [R/W:1][Addr:7][Data:16], MSB first
void drv_write(uint8_t addr7, uint16_t data16) 
{
  vspi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0)); // 1 MHz, Mode 0
  scs_begin();
  vspi.transfer((0u << 7) | (addr7 & 0x7F));
  vspi.transfer((uint8_t)(data16 >> 8));
  vspi.transfer((uint8_t)(data16 & 0xFF));
  scs_end();
  vspi.endTransaction();
  delayMicroseconds(1); // meet SCS timing
}


// 24-bit DRV frame to read from MISO: [R/W:1][Addr:7][Data:16], MSB first
uint16_t drv_read(uint8_t addr7) 
{
  vspi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  scs_begin();
  vspi.transfer((1u << 7) | (addr7 & 0x7F));
  uint8_t msb = vspi.transfer(0x00);
  uint8_t lsb = vspi.transfer(0x00);
  scs_end();
  vspi.endTransaction();
  return (uint16_t(msb) << 8) | lsb;
}


// Build Register 0x00 value: bits [7:6]=PWMF, [5:4]=SPDMODE, [3:2]=FGSEL
// This takes register values that you want to set at the 0x00 address and returns a 16 bit integer that you send to that address
uint16_t reg00_make(uint8_t spdmode, uint8_t pwmf, uint8_t fgsel) {
  return ((uint16_t)(pwmf & 3) << 6) | ((uint16_t)(spdmode & 3) << 4) | ((uint16_t)(fgsel & 3) << 2);
}

// Calculate PWM frequency from desired RPM
float PWM_SETUP(int RPM)
{ 
  // Set up LEDC to output a clock of frequency freq on PIN_CLKIN
  float freq = (RPM / 15); // Calculate frequency in Hz from RPM
  return freq;
}

// Command to set frequency on LEDC channel 0
void COMMAND_FREQ(float freq)
{
  // ledcWriteTone expects an integer frequency (Hz). Cast to uint32_t to be explicit.
  ledcWriteTone(ledChannel, (uint32_t)freq); // automatically outputs on channel 0 (PIN_CLKIN as configured above) at a 50% duty cycle
}


// Read FGOUT pin and compute frequency and RPM every SAMPLE_MS milliseconds
void readAndPrintRPM() 
{
  static int lastState = LOW;
  static unsigned long edgeCount = 0;
  static unsigned long lastSample = millis();

  // 1. Detect rising edges
  int currentState = digitalRead(PIN_FGOUT);
  if (currentState == HIGH && lastState == LOW) {
    edgeCount++;
  }
  lastState = currentState;

  // 2. Every SAMPLE_MS, compute and print frequency/RPM
  unsigned long now = millis();
  if (now - lastSample >= SAMPLE_MS) {
    float dt = (now - lastSample) / 1000.0;  // seconds
    lastSample = now;

    float freq = edgeCount / dt;             // Hz
    float rpm  = (15 * freq);    // mechanical RPM
    Serial.printf("FGOUT freq = %.1f Hz,  RPM = %.1f\n", freq, rpm);

    edgeCount = 0;  // reset count for next window
  }
}


void setup() 
{
  Serial.begin(115200);
  delay(10000); // 10 secs for the user to open Serial Monitor!
  Serial << "Hello! This begins the serial reader." << endl;
  vspi.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, -1); 
  init_pins();

  // Program SPDMODE=00 (Clock Frequency), PWMF=01 (~50kHz), FGSEL=00 (Hall-U)
  uint16_t r00 = reg00_make(/*spdmode=*/0b00, /*pwmf=*/0b01, /*fgsel=*/0b00);
  drv_write(0x00, r00);

  enable(); // Enable the DRV8308
  delay(2); // Wait for DRV8308 to power up

  // Configure LEDC: compute frequency, setup channel, attach pin, then start tone
  freq = PWM_SETUP(150);
  ledcSetup(ledChannel, (uint32_t)freq, resolution); // configure timer for the channel
  ledcAttachPin(PIN_CLKIN, ledChannel);
  COMMAND_FREQ(freq);

}

void loop() 
{
  readAndPrintRPM();
  delay(10);
}

