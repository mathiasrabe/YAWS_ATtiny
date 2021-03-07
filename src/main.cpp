/**************************************************************************************************************************
 * 
 * *** Pin Assignment ***
 * 
 *              _________
 *       Reset o 1     8 o VCC
 *  SHDN / PB3 o 2     7 o PB2 / SCL
 *  5VEN / PB4 o 3     6 o PB1
 *         GND o 4     5 o PB0 / SDA
 *              ---------
 * 
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files.
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 **************************************************************************************************************************/

#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <EEPROM.h>
#include <TinyWireS.h>

#define I2C_SLAVE_ADDRESS 0x4 // the 7-bit I2C address
// The default buffer size, though we cannot actually affect it by defining it in the sketch
#ifndef TWI_RX_BUFFER_SIZE
  #define TWI_RX_BUFFER_SIZE ( 16 )
#endif


// variable to save the content of the ADC for later
volatile byte saveADCSRA;
// save the battery voltage
volatile uint16_t vcc = 0;
// enum variables for i2c_regs
enum i2c_reg_names {
    STATUS = 0,
    SLEEP = 1,
    VCC_LO = 2,
    VCC_HI = 3,
    V1VREF_LO = 4,
    V1VREF_HI = 5,
    VCCMIN_LO = 6,
    VCCMIN_HI = 7,
    VCCMAX_LO = 8,
    VCCMAX_HI = 9,
    VCCHYST_LO = 10,
    VCCHYST_HI = 11
};
// The "registers" we expose to I2C
volatile uint8_t i2c_regs[] = {
    [STATUS] = 0x0, // status register, the first bit is 1 if VCC was measured; the second bit shows if the EEPROM is conditioned
    [SLEEP] = 0x0, // sleep register, !=0 means that the ATtiny should disable 3.3V and going to sleep for 8sec times this register
    [VCC_LO] = 0x0, // low byte of VCC
    [VCC_HI] = 0x0, // high byte of VCC
    [V1VREF_LO] = 0x0, // low byte of V1VREF
    [V1VREF_HI] = 0x0, // high byte of V1VREF
    [VCCMIN_LO] = 0x0, // low byte of VCCMIN
    [VCCMIN_HI] = 0x0, // high byte of VCCMIN
    [VCCMAX_LO] = 0x0, // low byte of VCCMAX
    [VCCMAX_HI] = 0x0, // high byte of VCCMAX
    [VCCHYST_LO] = 0x0, // low byte of VCCHYST
    [VCCHYST_HI] = 0x0, // high byte of VCCHYST
};
const byte reg_size = sizeof(i2c_regs);
// Tracks the current register pointer position
volatile byte reg_position;
// true if the VCC needs to be converted with the ADC, will be set by Timer1
volatile bool convertVCC = true;
// true if a conversion was started and there might be a result
volatile bool readVCC = true;

// EEPROM struct to save some variables - can be changed by I2C
struct EEPROMVar {
  uint8_t conditioned = 3;  // will show us if the EEPROM was conditioned and the values are reliable
                            // 1:conditioned - 2:conditioned but has not been saved yet - 3:not conditioned
  uint16_t v1v_ref = 1100;  // mV - actual voltage of internal 1V1 reference - should be measured and changed
  uint16_t vcc_min = 3700;  // mV - minimum voltage of VCC power supply
  uint16_t vcc_max = 4200;  // mV - maximum voltage of VCC power supply - if exceeded, we will disabke battery charging
  uint16_t vcc_hyst = 4000;  // mV - if below this level we will enable battery charging again
};
EEPROMVar eepromVar;

void resetWatchDog() {
  MCUSR = 0;
  WDTCR = bit ( WDCE ) | bit ( WDE ) | bit ( WDIF );  // allow changes, disable reset, clear existing interrupt
  WDTCR = bit ( WDIE ) | bit ( WDP3 )| bit ( WDP0 );  // set WDIE ( Interrupt only, no Reset ) and 8 second TimeOut
                                                     
  wdt_reset ();  // reset WDog to parameters
}

void resetI2CRegs() {  // set VCC registers and ADC bit to 0
  i2c_regs[STATUS] &= ~bit(0); // just reset the first ADC bit
  i2c_regs[VCC_LO] = 0x0;
  i2c_regs[VCC_HI] = 0x0;
}

void readEEPROM() {
  EEPROM.get(0, eepromVar);
  if (eepromVar.conditioned == 1) {
    i2c_regs[STATUS] |= bit(1); // set the i2c register bit for conditioned chip
  } else {
    // The EEPROM is a maiden, load standard settings to RAM again
    EEPROMVar initialVar;
    eepromVar = initialVar;
  }
  // copy the variables from EEPROM to the i2c regs
  i2c_regs[V1VREF_LO] = lowByte(eepromVar.v1v_ref);
  i2c_regs[V1VREF_HI] = highByte(eepromVar.v1v_ref);
  i2c_regs[VCCMIN_LO] = lowByte(eepromVar.vcc_min);
  i2c_regs[VCCMIN_HI] = highByte(eepromVar.vcc_min);
  i2c_regs[VCCMAX_LO] = lowByte(eepromVar.vcc_max);
  i2c_regs[VCCMAX_HI] = highByte(eepromVar.vcc_max);
  i2c_regs[VCCHYST_LO] = lowByte(eepromVar.vcc_hyst);
  i2c_regs[VCCHYST_HI] = highByte(eepromVar.vcc_hyst);
}

void writeEEPROM() {
  // write to EEPROM if we received new data
  if (eepromVar.conditioned == 2) {
    eepromVar.conditioned = 1;
    eepromVar.v1v_ref = (i2c_regs[V1VREF_HI] << 8) |  i2c_regs[V1VREF_LO];
    eepromVar.vcc_min = (i2c_regs[VCCMIN_HI] << 8) |  i2c_regs[VCCMIN_LO];
    eepromVar.vcc_max = (i2c_regs[VCCMAX_HI] << 8) |  i2c_regs[VCCMAX_LO];
    eepromVar.vcc_hyst = (i2c_regs[VCCHYST_HI] << 8) |  i2c_regs[VCCHYST_LO];
    EEPROM.put(0, eepromVar);
  }
}

void activeSleepNow() {
  // write data to EEPROM if necessary
  writeEEPROM();
  // consume some power to reduce battery voltage
  while (i2c_regs[SLEEP] != 0) {
    digitalWrite( PB3, LOW );  // disable 3.3V
    delay(8000);  // ms
    i2c_regs[SLEEP]--;  // decrease the sleep register (WDog firing counter).
  }
}

void sleepNow() {
  digitalWrite( PB3, LOW );               // disable 3.3V
  convertVCC = true;                      // the voltage needs to be measured when we wake up
  saveADCSRA = ADCSRA;                    // save the state of the ADC.
  ADCSRA = 0;                             // turn off the ADC

  writeEEPROM();                          // write data to EEPROM if necessary

  power_all_disable ();                   // turn power off to ADC, TIMER 1 and 2, Serial Interface
  
  noInterrupts();                         // turn off interrupts as a precaution
  resetI2CRegs();                         // reset all I2C registers, except the sleep register
  resetWatchDog ();                       // reset the WatchDog before beddy bies
  sleep_enable();                         // allows the system to be commanded to sleep
  interrupts();                           // turn on interrupts
  
  sleep_cpu();                            // send the system to sleep, night night!

  sleep_disable();                        // after ISR fires, return to here and disable sleep
  power_all_enable();                     // turn on power to ADC, TIMER1 and 2, Serial Interface
  
  ADCSRA = saveADCSRA;                    // turn on and restore the ADC
}

void VCCstartConversion() {
  //reads internal 1V1 reference against VCC
  ADCSRA |= _BV(ADSC); // Convert
}

bool VCCconversionInProgress() {
  return bit_is_set(ADCSRA, ADSC);
}

uint16_t getVCC() {
  uint8_t low = ADCL;
  uint16_t val = (ADCH << 8) | low;
  return ((uint32_t)1024 * eepromVar.v1v_ref) / val;
}

/**
 * This is called for each read request we receive, never put more than one byte of data (with TinyWireS.send) to the 
 * send-buffer when using this callback
 */
void requestEvent()
{
  TinyWireS.send(i2c_regs[reg_position]);
  // Increment the reg position on each read, and loop back to zero
  reg_position++;
  if (reg_position >= reg_size)
  {
    reg_position = 0;
  }
}

/**
 * The I2C data received -handler
 *
 * This needs to complete before the next incoming transaction (start, data, restart/stop) on the bus does
 * so be quick, set flags for long running tasks to be called from the mainloop instead of running them directly,
 */
void receiveEvent(uint8_t howMany)
{
  if (howMany < 1) {
    // Sanity-check
    return;
  }
  if (howMany > TWI_RX_BUFFER_SIZE) {
    // Also insane number
    return;
  }

  reg_position = TinyWireS.receive();
  howMany--;
  if (!howMany) {
    // This write was only to set the buffer for next read
    return;
  }
  while(howMany--) {
    // I2C master is just allowed to write some registers
    if (reg_position == SLEEP) {
      i2c_regs[reg_position] = TinyWireS.receive();
    } else if ((reg_position >= V1VREF_LO) && (reg_position <= VCCHYST_HI)) {
      eepromVar.conditioned = 2;
      i2c_regs[reg_position] = TinyWireS.receive();
    }
    reg_position++;
    if (reg_position >= reg_size)
    {
      reg_position = 0;
    }
  }
}

void setup() {
  //resetWatchDog();  // do this first in case WDog fires
  //wdt_disable();

  noInterrupts();

  pinMode( PB3, OUTPUT );  // SHDN to enable 3.3V
  pinMode( PB4, OUTPUT );  // 5VEN to enable battery charging
  digitalWrite( PB4, HIGH );  // enable battery charging until we have a measurement

  // configure the ADC to read internal 1V1 reference against VCC
  #if defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny44__)
    ADMUX = _BV(MUX5) | _BV(MUX0); // For ATtiny84
  #elif defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny45__)
    ADMUX = _BV(MUX3) | _BV(MUX2); // For ATtiny85/45
  #elif defined(__AVR_ATmega1284P__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);  // For ATmega1284
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);  // For ATmega328
  #endif

  // set sleep mode Power Down
  set_sleep_mode ( SLEEP_MODE_PWR_DOWN );

  // read EEPROM
  readEEPROM();

  // set Timer1 to trigger the ADC conversion every 500ms
  // 243 = 8,000,000 / (16384 * 2) - 1
  TCCR1 = 0;
  TCCR1 |= _BV(CTC1);  // enable CTC-Mode
  TCCR1 |= _BV(CS10) | _BV(CS11) | _BV(CS12) | _BV(CS13);  // prescaler to 16384
  TCNT1 = 0;  // delete timer counter register
  OCR1C = 243;  // set timer compare register
  TIMSK |= _BV(OCIE1A);  // activate Timer1

  // enable I2C stuff
  //USICR |= _BV(USIWM0); // Clock stretching?
  TinyWireS.begin(I2C_SLAVE_ADDRESS);
  TinyWireS.onReceive(receiveEvent);
  TinyWireS.onRequest(requestEvent);

  interrupts();
}

void loop() {
  if (i2c_regs[SLEEP] != 0) {
    // go to sleep if the sleep register is not zero
    if (vcc > eepromVar.vcc_max) {
      activeSleepNow();  // consume some power to reduce battery voltage
    } else {
      sleepNow();  // enter sleep mode
    }
    return;
  }
  if (convertVCC) {
    VCCstartConversion();
    convertVCC = false;  // we will convert again when Timer1 says so
    readVCC = true;  // VCC needs to be read after conversion
  }
  if (!VCCconversionInProgress() && readVCC) {
    uint16_t vcc_buf = getVCC();
    // check if the value is plausible
    if ((vcc_buf > 1800) && (vcc_buf < 5500)) {
      vcc = vcc_buf;
    } else {
      convertVCC = true;
      goto i2c_state_check;
    }
    readVCC = false;  // we don't need to read it again if there was no convertion

    if ( vcc <= eepromVar.vcc_hyst ) {
      // enable battery charging if voltage is too low
      digitalWrite( PB4, HIGH );
    } else if ( vcc >= eepromVar.vcc_max ) {
      // disable battery charging if voltage is too high
      digitalWrite( PB4, LOW );
    }

    if ( vcc < eepromVar.vcc_min ) {
      // if the voltage is to low
      sleepNow();  // then set up and enter sleep mode
      return;
    } else {
      digitalWrite( PB3, HIGH );  // enable 3.3V

      // write the voltage to the I2C registers
      i2c_regs[VCC_LO] = lowByte(vcc);
      i2c_regs[VCC_HI] = highByte(vcc);
      i2c_regs[STATUS] |= bit(0);  // we have a measurement - set the first bit of the STATUS register
    }
  }
  i2c_state_check:
  //TinyWireS_stop_check();
  TinyWireS.stateCheck();
}

ISR( WDT_vect ) {  // will enter this if watch dog is timed out
  wdt_disable();  // until next time....
  if (i2c_regs[SLEEP] != 0) {
    i2c_regs[SLEEP]--;  // decrease the sleep register (WDog firing counter).
  }
} 

ISR( TIMER1_COMPA_vect ) {  // will enter this if Timer1 is timed out
  convertVCC = true;
}
