/** @file Driver.cpp
 *  This file contains a class written for the Texas Instruments DRV8308 motor driver chip.
 *  This class sets pins on the ESP32 which interact with respective pins on the driver,
 *  initializes the driver with default settings and gains, 
 *  and defines methods to control, command, and read the driver. 
 * 
 *  This code was written mostly by Neil Bedagkar and Bricen Rigby, with help from 
 *  Claude Sonnet 4.5 to write the ISR handling code.
*/

#include <Arduino.h>
#include <SPI.h>
#include "Driver.h"
#include "Shares.h"
#include <PrintStream.h>



// Create a SPI class object
SPIClass vspi(VSPI);



// A queue which uses an ISR to trigger the readActual task and calculate the motor speed by reading the square wave 
// frequency on the FGOUT pin
extern Queue<uint32_t> edge_time;



// Static instance pointer initialization
Driver* Driver::_instance = nullptr;


/** @brief Constructor for the Driver class
 * 
 *  @details This function initializes the DRV8308, setting each of the pins for the command and 
 *  default gains tuned for mediocre performance at all speeds 
 *  and during both idle and transient states.
 */
Driver::Driver(void)
{
    data16 = 0;
    msb = 0;
    lsb = 0;
    PIN_SCLK = 18;
    PIN_MISO = 19;
    PIN_MOSI = 23;
    PIN_SCS = 5;
    PIN_EN = 13;
    PIN_CLKIN = 14;
    PIN_FGOUT = 25;
    PIN_FAULTn = 26;
    PIN_LOCKn = 27;
    PIN_RESET = 15;
    PIN_BRAKE = 12;
    PIN_DIR = 16;
    FILK1 = 127;
    FILK2 = 507;
    COMPK1 = 100;
    COMPK2 = 100;
    _lastEdgeTime = 0;
    _instance = this;
}

/** @brief Constructor for the Driver class
 * 
 *  @details This function initializes the DRV8308, setting each of the pins for the command 
 *  setting custom gain coefficients. An accompanying MATLAB script is included to calculate 
 *  these gain coefficients using equations detailed in the DRV8308 datasheet.
 * 
 *  @param FILK1_ setting the coefficient for the filter pole
 *  @param FILK2_ setting the coefficient for the filter zero
 *  @param COMPK1_ setting the coefficient for the compensator pole
 *  @param COMPK2_ setting the coefficient for the compensator zero
 */
Driver::Driver(uint8_t FILK1_, uint8_t FILK2_, uint8_t COMPK1_, uint8_t COMPK2_)
{
    data16 = 0;
    msb = 0;
    lsb = 0;
    PIN_SCLK = 18;
    PIN_MISO = 19;
    PIN_MOSI = 23;
    PIN_SCS = 5;
    PIN_EN = 13;
    PIN_CLKIN = 14;
    PIN_FGOUT = 25;
    PIN_FAULTn = 26;
    PIN_LOCKn = 27;
    PIN_RESET = 15;
    PIN_BRAKE = 12;
    PIN_DIR = 16;
    FILK1 = FILK1_;
    FILK2 = FILK2_;
    COMPK1 = COMPK1_;
    COMPK2 = COMPK2_;
    _lastEdgeTime = 0;
    _instance = this;
}



/** @brief A function which initializes the DRV8308
 * 
 *  @details This function sets each pin as an input or output, 
 *  attaches an interrupt to FGOUT so it can read square wave rising edges,
 *  enables the DRV8308, 
 *  unbrakes it from any previous operation,
 *  initializes the direction to be forward,
 *  begins SPI communication,
 *  writes initial gains to the DRV8308 chip,
 *  and sets up the CLKIN pin to output a square wave to output 50% duty cycle.
 * 
 *  @bug The unbrake() does not actually work, sometimes you have to manually
 *  spin the motor before it starts taking commands again.
 */
void Driver::begin() 
{
    // Pin input/output definitions
    pinMode(PIN_SCS, OUTPUT);
    pinMode(PIN_EN, OUTPUT);
    pinMode(PIN_FGOUT, INPUT);
    pinMode(PIN_FAULTn, INPUT);
    pinMode(PIN_LOCKn, INPUT);
    pinMode(PIN_CLKIN, OUTPUT);         
    pinMode(PIN_RESET, OUTPUT);
    digitalWrite(PIN_RESET, LOW); // Keep RESET low for nominal operation
    pinMode(PIN_BRAKE, OUTPUT);   
    pinMode(PIN_DIR, OUTPUT);

    // Attach interrupt on fgout pin (rising edge only)
    attachInterrupt(digitalPinToInterrupt(PIN_FGOUT), ISR_wrapper, RISING);

    // Initialize the driver
    enable();  // Enable the driver
    unbrake(); // Release BRAKE
    set_dir(LOW); // Set direction forward

    // SPI setup
    // PIN_SCS is set to -1 because we control that manually using scs_begin() and scs_end()
    // SPI defaults to ACTIVE LOW but the DRV8308 is ACTIVE HIGH so we control it manually
    vspi.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, -1);  // VSPI pins

    // Initial register programming
    drv_write(0x00, 0x2000);
    Serial.print("0x00 expect 0x2000, got 0x"); Serial.println(drv_read(0x00), HEX);

    // set MOD120 to 3970 as per DRV8308EVM users guide
    drv_write(0x03, 0b00000111110000010);
    Serial.print("0x03 expect 0xF82, got 0x"); Serial.println(drv_read(0x03), HEX);

    // set AUTOGAIN to 1
    drv_write(0x04, 0x0200);
    Serial.print("0x04 expect 0x0200, got 0x"); Serial.println(drv_read(0x04), HEX);

    // set SPDGAIN to 2048 and intclk to 000
    drv_write(0x05, 0x0800);
    Serial.print("0x05 expect 0x0800, got 0x"); Serial.println(drv_read(0x05), HEX);

    // set FILK1
    drv_write(0x06, FILK1);
    Serial.print("0x06 expect 0x007F, got 0x"); Serial.println(drv_read(0x06), HEX);

    // set FILK2
    drv_write(0x07, FILK2);
    Serial.print("0x07 expect 0x01FB, got 0x"); Serial.println(drv_read(0x07), HEX);

    // set COMPK1
    drv_write(0x08, COMPK1);
    Serial.print("0x08 expect 0x007F, got 0x"); Serial.println(drv_read(0x08), HEX);

    // set COMPK2
    // AUTOADV is here, setting to zero for now
    drv_write(0x09, COMPK2);
    Serial.print("0x09 expect 0x22A3, got 0x"); Serial.println(drv_read(0x09), HEX);

    // set LOOPGAIN to 512
    drv_write(0x0A, 0x00000200);
    Serial.print("0x0A expect 0x0200, got 0x"); Serial.println(drv_read(0x0A), HEX);

    // set SPEED to 1280
    drv_write(0x0B, 0x0500);
    Serial.print("0x0B expect 0x0500, got 0x"); Serial.println(drv_read(0x0B), HEX);

    // Setup CLKIN for square wave output
    // The internal control loop in the DRV8308 matches the frequency input on CLKIN 
    // to the motor electrical frequency output on FGOUT
    ledcSetup(0, 100, 8); // channel 0, 20 kHz, 8-bit resolution
    ledcAttachPin(PIN_CLKIN, 0); // attach PIN_CLKIN to
}



/** @brief A function which writes a register on the DRV8308
 * 
 *  @details This function takes an 8-bit address and a 16-bit message, 
 *  and uses SPI to write a register on the DRV8308. This code was written
 *  with the help of ChatGPT 5.1.
 * 
 *  @param addr7 the 7-bit address register. The first bit is always zero 
 *  to signify a WRITE operation to the DRV8308 chip.
 * 
 *  @param message the 16-bit number that is being written to the DRV8308.
 *  Registers are defined on the DRV8308 datasheet.
 */
void Driver::drv_write(uint8_t addr7, uint16_t message) 
{
    vspi.beginTransaction(SPISettings(10000, MSBFIRST, SPI_MODE0)); // 1 MHz, Mode 0
    scs_begin();
    delayMicroseconds(1); // Setup time for SCS
    vspi.transfer((0u << 7) | (addr7 & 0x7F));
    vspi.transfer((uint8_t)(message >> 8));
    vspi.transfer((uint8_t)(message & 0xFF));
    delayMicroseconds(1); // Hold time for data
    scs_end();
    vspi.endTransaction();
    delayMicroseconds(5); // Recovery time between transactions
}



/** @brief A function which reads a register on the DRV8308
 *  
 *  @details This function takes an 8-bit address and uses SPI to read 
 *  that register on the DRV8308. It returns the 16-bit value of that 
 *  register. This was developed primarily to debug SPI communication 
 *  and was kept because of its usefulness, and just in case the SPI
 *  demons decide to haunt us again. This code was written with the
 *  help of ChatGPT 5.1.
 * 
 *  @param addr7 the 7-bit address register. The first bit is always one
 *  to signify a READ operation to the DRV8308 chip.
 * 
 *  @return the 16 bit value of the register on the DRV8308.
 */
uint16_t Driver::drv_read(uint8_t addr7)  
{
    vspi.beginTransaction(SPISettings(10000, MSBFIRST, SPI_MODE0)); // Same mode as write
    scs_begin();
    delayMicroseconds(1); // Setup time for SCS
    vspi.transfer((1u << 7) | (addr7 & 0x7F));
    msb = vspi.transfer(0x00);
    lsb = vspi.transfer(0x00);
    delayMicroseconds(1); // Hold time for data
    scs_end();
    vspi.endTransaction();
    delayMicroseconds(5); // Recovery time
    return (uint16_t(msb) << 8) | lsb;
}



/** @brief A function which commands a square wave to the CLKIN pin
 * 
 *  @details This function writes a square wave of 50% duty cycle to the CLKIN pin
 *  at the desired electrical frequency of the motor. From an equation on the 
 *  DRV8308 datasheet, the electrical frequency is equal to RPM / 15. 
 * 
 *  @param SPEED_CMD The desired rpm speed of the motor.
 */
void Driver::cmd_speed_PWM(float SPEED_CMD)
{
    double freq = SPEED_CMD / 15;
    ledcWriteTone(0, freq); // Set the frequency of the PWM signal on channel 0

}


// ============ Interrupt-based edge detection ============


/** @brief An ISR wrapper function which calls a different 
 *  function once the ISR is triggered.
 */
void Driver::ISR_wrapper() 
{
    if (_instance) {
        _instance->handleISR();
    }
}



/** @brief an ISR handler which puts the timestamp at which
 *  the function is called into the edge_time queue.
 */
void Driver::handleISR() 
{
    // Very fast: capture timestamp and set flag
    _lastEdgeTime = micros();

    edge_time.put(_lastEdgeTime);
}

