// EE-201 Arduometer Checkout:  Voltage-Current-Power Meter
// R.B.Darling, Alvin Cao
// Revision R1:  11/2022
// This code deploys to the EE-201 custom Arduometer PCB
// The target MCU is an Microchip/Atmel ATmega328P, configured to behave like an Arduino Uno
// The Arduometer PCB is designed to accept a Sparkfun FTDI Basic USB-UART bridge and to be programmed directly through the Arduino IDE
// Use a USB A to mini-B cable to connect the FTDI Basic to a PC running the Arduino IDE
// Select Board = 'Arduino Uno' if using the Arduino IDE, and use the Arduino bootloader to program through UART Tx/Rx lines
// More adventurous users can also program the Arduometer using the ICSP port and Atmel Studio which will also provide debug capability
// The ICSP is also used to burn the Arduino bootloader into the ATmega328P, which is blank when installed
// Clock is set up for 16.000 MHz using the on-board crystal
// The code controls the SPI bus to the MAX7219 8-digit 7-segment LED driver

// The Voltage-Current-Power Meter implements a digital meter that can be used to measure DC voltages and currents.  
// The voltage is measured across the red and black binding posts, captured on analog channel 0.  
// The current is measured through the green and black binding posts, captured on analog channel 1.  
// A reference voltage of +2.50V is established for the two measurements and is captured on analog channel 2.
// The battery voltage (divided by 2) is available on analog channel 3.  
// Referencing the measurements to the +2.50V level allows for full 4-quadrant measurement of voltage and current.  
// The product of the voltage and current can be computed to give the power flow through a port.  
// The range of the voltage measurement is +/- 25 Volts.  
// The range of the current measurement is +/- 2.5 Amperes.  
// The voltage and current are each sampled every 100 ms, and the display is updated at that same rate.  
// The ADC on the ATmega328P is only 10 bits (1024 values), and it takes about 100 us for each conversion.  

// MCU pin assignments
#define RX 0 // UART RX on digital pin 0
#define TX 1 // UART TX on digital pin 1
#define ISRMKR 2 // ISR timing marker on digital pin 2 (PD2) for diagnostics (oscilloscope timing/trigger)
#define DIAG3 3 // diagnostic GPIO pin (PD3)
#define DIAG4 4 // diagnostic GPIO pin (PD4)
#define DIAG5 5 // diagnostic GPIO pin (PD5)
#define DIAG6 6 // diagnostic GPIO pin (PD6)
#define DIAG7 7 // diagnostic GPIO pin (PD7)
#define PB0 8 // pushbutton PB0 on digital pin 8, normally pulled HI
#define PB1 9 // pushbutton PB1 on digital pin 9, normally pulled HI
#define CS 10 // chip select
#define MOSI 11 // SPI bus MOSI
#define MISO 12 // SPI bus MISO
#define SCLK 13 // SPI bus SCLK
#define ANA0 A0 // ADC channel 0, voltage across red and black binding posts, divided by 10.0
#define ANA1 A1 // ADC channel 1, current through green into black binding posts, multiplied by 1.00 Ohm
#define ANA2 A2 // ADC channel 2, +2.50V reference
#define ANA3 A3 // ADC channel 3, VBATT/2 to measure the battery voltage

// globals
char pbtns = 0; // pushbutton read variable
unsigned int counterPB0 = 0; // millisecond duration counter for pushbutton PB0 pushed alone
unsigned int counterPB1 = 0; // millisecond duration counter for pushbutton PB1 pushed alone
unsigned int counterPB01 = 0; // millisecond duration counter for both pushbuttons held together
unsigned int counterMeas = 0; // millisecond duration counter to time measurement period of 100 ms
bool measure = false; // flag to initiate measurement
enum dispState // display states
{
  sVolts = 0, // show voltage
  sAmps = 1, // show amperage
  sWatts = 2, // show wattage
  sRef = 3, // show reference
  sBatt = 4 // show battery voltage
};
enum dispState showUD = sVolts; // start with upper display showing voltage
enum dispState showLD = sAmps; // start with lower display showing amperage
int V0 = 0; // converted measurement from ANA0
int V1 = 0; // converted measurement from ANA1
int V2 = 0; // converted measurement from ANA2
int V3 = 0; // converted measurement from ANA3
int adcVolts = 0; // measured voltage relative to reference to give polarity
int adcAmps = 0; // measured current relative to reference to give polarity
int calVolts = 0; // calibrated voltage magnitude
int calAmps = 0; // calibrated current magnitude
int calWatts = 0; // calibrated power magnitude
int calRef = 0; // calibrated reference magnitude
int calBatt = 0; // calibrated battery magnitude
int magUD = 0; // magnitude for upper display
int magLD = 0; // magnitude for lower display
bool negUD = false; // negative flag for upper display
bool negLD = false; // negative flag for lower display
// Code B font used in MAX7219:  
// bit 7 = decimal point
// bits 6..4 = X, don't care
// bits 3..0 = 0,1,2,3,4,5,6,7,8,9,-,E,H,L,P,blank
// display digits:  start with each blanked
char D0 = 0x0F; // digit 0, MSD of upper display, initialize to 0xF = blank
char D1 = 0x0F; // digit 1
char D2 = 0x0F; // digit 2
char D3 = 0x0F; // digit 3, LSD of upper display
char D4 = 0x0F; // digit 4, MSD of lower display
char D5 = 0x0F; // digit 5
char D6 = 0x0F; // digit 6
char D7 = 0x0F; // digit 7, LSD of lower display
// display is arranged like this:  D0 D1 D2 D3 <-- upper display
//                                 D4 D5 D6 D7 <-- lower display
//                                   PB0 PB1   <-- pushbuttons

// SPI transfer function
char spi_transfer(volatile char data)
{
  SPDR = data; // load data and start transmission
  while(!(SPSR & (1<<SPIF))) // wait for end of transmission
  {
    // do nothing; SCLK should run at 4 MHz, transmit of one byte should take 2 us.  
  }
  return SPDR; // return the received byte - will be all zeroes for the MAX7219 which does not have a MISO line
}

// 16b load function for MAX7219 display driver
// technically not SPI, but SPI protocol will work with MAX7219
// CS does not need to be LOW to clock data into MAX7219, but do it anyway to frame the data
void display_load(char dataH, char dataL) 
{
  digitalWrite(CS, LOW); // pull CS LOW to start clocking in data
  spi_transfer(dataH); // clock in high byte for address
  spi_transfer(dataL); // clock in low byte for data
  digitalWrite(CS, HIGH); // bring CS HIGH to load data
}

// update upper display with current digit values
void update_upper_display()
{
  display_load(0x01,D0); // load digit 0
  display_load(0x02,D1); // load digit 1
  display_load(0x03,D2); // load digit 2
  display_load(0x04,D3); // load digit 3
}

// update lower display with current digit values
void update_lower_display()
{
  display_load(0x05,D4); // load digit 4
  display_load(0x06,D5); // load digit 5
  display_load(0x07,D6); // load digit 6
  display_load(0x08,D7); // load digit 7
}

// initialization function for MAX7219 display driver
void display_init() 
{
  display_load(0x09, 0xFF); // decode mode register: code B decode for digits 0-7
  display_load(0x0A, 0x03); // intensity register: 31/32 duty cycle; keep low to minimize ADC interference
  display_load(0x0B, 0x07); // scan limit register: scan digits 0-7
  display_load(0x0C, 0x01); // shutdown mode register: normal operation
  display_load(0x0F, 0x00); // display test register: normal operation
  D0 = 0x0F; // blank
  D1 = 0x0F; // blank
  D2 = 0x0F; // blank
  D3 = 0x00; // zero
  update_upper_display();
  D4 = 0x0F; // blank
  D5 = 0x0F; // blank
  D6 = 0x0F; // blank
  D7 = 0x00; // zero
  update_lower_display();
}

// pushbutton PB0 action:  change upper display state
// pushbutton PB0 cycles through 3 display choices:  Volts, Amps, or Watts
void actionPB0()
{
  switch(showUD) // cyclic state transition matrix
  {
    case sVolts:
      showUD = sAmps;
      break;
    case sAmps:
      showUD = sWatts;
      break;
    case sWatts:
      showUD = sVolts;
      break;
  }
}

// pushbutton PB1 action:  change lower display state
// pushbutton PB1 cycles through 3 display choices:  Volts, Amps, or Watts
void actionPB1()
{
  switch(showLD) // cyclic state transition matrix
  {
    case sVolts:
      showLD = sAmps;
      break;
    case sAmps:
      showLD = sWatts;
      break;
    case sWatts:
      showLD = sVolts;
      break;
  }
}

// pushbutton PB0 and PB1 together action:  nothing for this sketch; future expansion
void actionPB01()
{
  
}

// required one-time function for Arduino IDE
void setup() 
{
  cli(); // stop interrupts so that setup does not hang up
  // setup pushbuttons
  pinMode(PB0, INPUT); // sets the digital pin as input
  pinMode(PB1, INPUT); // sets the digital pin as input
  // setup ISR timing marker
  pinMode(ISRMKR, OUTPUT); // sets the digital pin as output
  digitalWrite(ISRMKR, LOW); // start with ISRMRKR = LO
  // setup SPI bus
  pinMode(CS, OUTPUT); // sets the digital pin to output
  pinMode(MOSI, OUTPUT); // sets the digitl pin to output
  pinMode(MISO, INPUT); // sets the digital pin to input
  pinMode(SCLK, OUTPUT); // sets the digital pin to output
  digitalWrite(CS, HIGH); // start with CS = HI to disable SPI transactions
  digitalWrite(MOSI, LOW); // start with MOSI = LO
  digitalWrite(SCLK, LOW); // start with SCLK = LO for its idle state
  SPCR = (1<<SPE) | (1<<MSTR); // setup SPI control register:  enable SPI, msb first, master mode, CPOL=0, CPHA=0, SCLK = fosc/4
  // setup timer0 to trigger interrupt at 1 kHz rate (1 ms period)
  TCCR0A = 0; // clear timer0 control register A
  TCCR0B = 0; // clear timer0 control register B
  TCNT0 = 0; // initialize counter to zero
  OCR0A = 249; // set outpout compare match to (16 MHz)/(1 kHz * 64) - 1 = 249
  TCCR0A |= (1<<WGM01); // turn on CTC mode
  TCCR0B |= (1<<CS01) | (1<<CS00); // use prescaler /64 output
  TIMSK0 |= (1<<OCIE0A); // enable timer0 compare A interrupt, OCF0A flag set on interrupt, cleared by HW when ISR executes
  sei(); // allow interrupts
  display_init(); // initialize the display, put a 0 in digits 3 (upper) and 7 (lower)
  Serial.begin(115200); // initialize serial output to the console at 115200 baud
}

// ISR for timer0 compare A interrupt
ISR(TIMER0_COMPA_vect)
{
  digitalWrite(ISRMKR, HIGH); // set ISR timing marker HI (oscilloscope diagnostic pin)
  // pushbutton service - provides debounce and hold down times, action initiated upon release
  pbtns = PINB & 0x3; // read pushbuttons PB0 and PB1, which also happen to be bits 0 and 1 of PORTB
  switch(pbtns)
  {
    case 0: // both pushbuttons depressed
      counterPB01++;
      break;
    case 1: // only PB1 depressed
      counterPB1++;
      break;
    case 2: // only PB0 depressed
      counterPB0++;
      break;
    case 3: // neither pushbutton depressed, check for any action to perform upon release
      if(counterPB01 > 1000) // both PB0 and PB1 held down together for more than 1.0 sec
      {
        actionPB01(); // action for both PB0 and PB1 together
      }
      else 
      {
        if(counterPB0 > 50) // PB0 held down for more than 0.05 sec for debounce
        {
          actionPB0(); // action for PB0
        }
        if(counterPB1 > 50) // PB1 held down for more than 0.05 sec for debounce
        {
           actionPB1(); // action for PB1
        }
      }
      counterPB0 = 0; // clear all of the pushbutton duration counters
      counterPB1 = 0;
      counterPB01 = 0;
      break;
  }
  // measurement service - sets measurement flag every 100 ms - update rate for meter function
  counterMeas++; 
  if(counterMeas >= 100)
  {
    counterMeas = 0; // reset counter
    measure = true; // set flag to measure analog channels, cleared by measure function
  }
  digitalWrite(ISRMKR, LOW); // set ISR timing marker LO
}

// required loop function for Arduino IDE
void loop() 
{
  if(measure)
  {
    // sample inputs
    V0 = analogRead(ANA0); // read and convert voltage measurement
    V1 = analogRead(ANA1); // read and convert current measurement
    V2 = analogRead(ANA2); // read and convert reference voltage
    V3 = analogRead(ANA3); // read and convert battery voltage
    // scale and calibrate
    adcVolts = V0 - V2; // subtract reference to establish polarity
    adcAmps = V1 - V2; // subtract reference to establish polarity
    calVolts = abs(adcVolts) / 2; // calibration of voltage magnitude to divider and gain
    calAmps = abs(adcAmps) / 2; // calibration of amperage magnitude to sense resistor and gain
    calWatts = (calVolts / 10) * (calAmps / 10); // calibration of wattage magnitude
    calRef = abs(V2); // calibration of reference voltage magnitude; needs scaling factor
    calBatt = abs(V3); // calibration of battery voltage magnitude; needs scaling factor
    // select upper display value
    switch(showUD)
    {
      case sVolts:
        magUD = calVolts;
        if(adcVolts < 0) negUD = true;
        else negUD = false;
        break;
      case sAmps:
        magUD = calAmps;
        if(adcAmps < 0) negUD = true;
        else negUD = false;
        break;
      case sWatts:
        magUD = calWatts;
        if(((adcVolts < 0) && (adcAmps > 0)) || ((adcVolts > 0) && (adcAmps < 0))) negUD = true;
        else negUD = false;
        break;
    }
    // update upper display
    D3 = magUD % 10;
    D2 = (magUD / 10) % 10;
    D1 = (magUD / 100) % 10;
    if(negUD)
      D0 = 0x0A; // light minus sign
    else
      D0 = 0x0F; // blank
    if((showUD == sVolts) || (showUD == sWatts)) D2 |= 0x80;
    if(showUD == sAmps) D1 |= 0x80;
    if(showUD == sWatts) D0 |= 0x80; // extra DP to indicate power from amperage
    update_upper_display();
    // select lower display value
    switch(showLD)
    {
      case sVolts:
        magLD = calVolts;
        if(adcVolts < 0) negLD = true;
        else negLD = false;
        break;
      case sAmps:
        magLD = calAmps;
        if(adcAmps < 0) negLD = true;
        else negLD = false;
        break;
      case sWatts:
        magLD = calWatts;
        if(((adcVolts < 0) && (adcAmps > 0)) || ((adcVolts > 0) && (adcAmps < 0))) negLD = true;
        else negLD = false;
        break;
    }
    // update lower display
    D7 = magLD % 10;
    D6 = (magLD / 10) % 10;
    D5 = (magLD / 100) % 10;
    if(negLD < 0)
      D4 = 0x0A; // light minus sign
    else
      D4 = 0x0F; // blank
    if((showLD == sVolts) || (showLD == sWatts)) D6 |= 0x80;
    if(showLD == sAmps) D5 |= 0x80;
    if(showLD == sWatts) D4 |= 0x80; // extra DP to indicate power from amperage
    update_lower_display();
    // finish off measurement routine
    measure = false; // clear measure flag
  }
}
