/**************************************************************************************************************************
 * 
 * *** Pin Assignment ***
 * 
 *              _________
 *       Reset o 1     8 o VCC
 *  SHDN / PB3 o 2     7 o PB2 / SCL
 *         PB4 o 3     6 o PB1
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
#include <TinyWireS.h>

#define VCCMIN 3700 // mV - minimum voltage of VCC power supply
#define V1V1REF 1117 // mV - actual voltage of internal 1V1 reference - should be measured and changed TODO: make this adjustable via I2C
#define I2C_SLAVE_ADDRESS 0x4 // the 7-bit I2C address
// The default buffer size, though we cannot actually affect it by defining it in the sketch
#ifndef TWI_RX_BUFFER_SIZE
  #define TWI_RX_BUFFER_SIZE ( 16 )
#endif


byte saveADCSRA;  // variable to save the content of the ADC for later
// The "registers" we expose to I2C
volatile uint8_t i2c_regs[] = {
    0x0, // status register, shows how many VCC measurements were made - only the last one will be stored in register 3 and 4
    0x0, // sleep register, !=0 means that the ATtiny should disable 3.3V and going to sleep for 8sec times this register
    0x0, // low byte of VCC
    0x0, // high byte of VCC
};
const byte reg_size = sizeof(i2c_regs);
// Tracks the current register pointer position
volatile byte reg_position;
// true if the VCC needs to be converted with the ADC, will be set by Timer1
volatile bool convertVCC = true;
// true if a conversion was started and there might be a result
volatile bool readVCC = true;


void resetWatchDog() {
  MCUSR = 0;
  WDTCR = bit ( WDCE ) | bit ( WDE ) | bit ( WDIF );  // allow changes, disable reset, clear existing interrupt
  WDTCR = bit ( WDIE ) | bit ( WDP3 )| bit ( WDP0 );  // set WDIE ( Interrupt only, no Reset ) and 8 second TimeOut
                                                     
  wdt_reset ();  // reset WDog to parameters
}

void resetI2CRegs() {  // set all registers to 0, except the sleep register
  for (uint8_t i=0; i < reg_size; i++) {
    if (i == 1) continue;  // sleep register should bot be overwritten
    i2c_regs[i] = 0x0;
  }
}

void sleepNow() {
  digitalWrite( PB3, LOW );               // disable 3.3V
  convertVCC = true;                      // the voltage needs to be measured when we wake up
  saveADCSRA = ADCSRA;                    // save the state of the ADC.
  ADCSRA = 0;                             // turn off the ADC
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
  return ((uint32_t)1024 * V1V1REF) / val;
}

/**
 * This is called for each read request we receive, never put more than one byte of data (with TinyWireS.send) to the 
 * send-buffer when using this callback
 */
void requestEvent()
{
  digitalWrite( PB1, HIGH );
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
    // I2C master is just allowed to write the second register
    if (reg_position == 1) {
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
  // delay( 2 );

  // set sleep mode Power Down
  set_sleep_mode ( SLEEP_MODE_PWR_DOWN );

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
  if ( i2c_regs[1] != 0 ) {
    // go to sleep if the sleep register is not zero
    //blink(1);
    sleepNow();  // then set up and enter sleep mode
    return;
  }
  if (convertVCC) {
    VCCstartConversion();
    convertVCC = false;  // we will convert again when Timer1 says so
    readVCC = true;  // VCC needs to be read after conversion
  }
  if (!VCCconversionInProgress() && readVCC) {
    uint16_t vcc = getVCC();
    readVCC = false;  // we don't need to read it again if there was no convertion

    if ( vcc < VCCMIN ) {
      // if the voltage is to low
      //blink(2);
      sleepNow();  // then set up and enter sleep mode
      return;
    } else {
      //blink(3);
      digitalWrite( PB3, HIGH );  // enable 3.3V

      // write the voltage to the I2C registers
      i2c_regs[2] = lowByte(vcc);
      i2c_regs[3] = highByte(vcc);
      i2c_regs[0]++;
    }
  }
  //TinyWireS_stop_check();
  TinyWireS.stateCheck();
}

ISR( WDT_vect ) {  // will enter this if watch dog is timed out
  wdt_disable();  // until next time....
  if (i2c_regs[1] != 0) {
    i2c_regs[1]--;  // decrease the sleep register (WDog firing counter).
  }
} 

ISR( TIMER1_COMPA_vect ) {  // will enter this if Timer1 is timed out
  convertVCC = true;
}
