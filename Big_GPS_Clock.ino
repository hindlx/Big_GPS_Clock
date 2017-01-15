/*
 * Big_GPS_Clock.ino
 * 2017-01-15
 * 
 *      1          a
 *    -----      -----        -----      -----     --   --
 *   |6    |2   |f    |b     |     |    |     |   |  | |  |
 *   |     |    |     |  0   |     |    |     |    --   --
 *    --7--      --g--        -----      -----    |  | |  |
 *   |5    |3   |e    |c     |     |    |     |    --   --
 *   |     |    |     |  0   |     |    |     |  
 *    -----      -----        -----      -----  
 *      4          d
 *      
 * Col  5          4            3          2        1    0
 * Dig  H          H     c      M          M        S    S
 * 
 */


// Dependensis / Librarys
// #include "avr/wdt.h"

#include "LedControl.h"
LedControl lc=LedControl(10,12,11,1);
/* pin 12 is connected to the CS (CS)(LOAD)
 * pin 11 is connected to the CLK (CLK)
 * pin 10 is connected to LOAD(DIN)
 */

unsigned  int secs  = 0;
unsigned  int mins  = 0;
unsigned  int hours = 0;


unsigned long Now_time;              // Time test
unsigned long old_time;              // Time test
unsigned int brightness = 1;         // LED brightness
const int PIN_butt_bright = 2;       // Pushbutton brightness upp
const int PIN_LED_bright  = 3;       // LED brightness upp 
const int PIN_LED_blink   = 4;       // LED sec blink 
const int PIN_LED_4       = 5;       // LED spare 
const int PIN_butt_setHH  = 6;       // Pushbutton hours upp
const int PIN_butt_setMM  = 7;       // Pushbutton minuters upp
unsigned int bright_pressed=0;
unsigned int HH_pressed=0;
unsigned int MM_pressed=0;

void setup() {
  Serial.begin(9600);
  lc.shutdown(0,false);            // Bring MAX72XX out off power-saving mode
  lc.setIntensity(0,brightness);   // Set the brightness to startvalue
  lc.clearDisplay(0);              // Clear the display 
  pinMode(PIN_butt_bright, INPUT); // Button for brightness adjustment
  pinMode(PIN_butt_setHH, INPUT);  // Button for hours adjustment                                           // Mabye remove everything about HH adjustment when GPS
  pinMode(PIN_butt_setMM, INPUT);  // Button for minutes adjustment                                         // Remove everything about MM adjustment when GPS
  pinMode(PIN_LED_bright, OUTPUT); // LED for visualisation of brighness adjustment
  pinMode(PIN_LED_blink, OUTPUT);  // LED for visualisation of seconds
  pinMode(PIN_LED_4, OUTPUT);      // LED for visualisation spare
//  check_disp();                    // Lights all led i disply for visual check of broken LEDs
//  write_helo();                    // Write msg to display
//  wdt_enable(WDTO_8S);             // Start Arduino Watchdog and set Threshold value to 8 sec
}

void loop() { 
  int devices=1;
  WriteDisp(secs, 1);
  WriteDisp(mins ,3);
  WriteDisp(hours,5);

do {
  Now_time=micros();                                  // 
  if ((Now_time - old_time)> 997879) {
//    wdt_reset();                                      // Reset the watchdog otherwise the arduino restarts
    checkBTN_brightness();
    checkBTN_setHH();
    checkBTN_setMM();
    secs++;                                           // 
    blink_led();
    if (secs > 59) { 
      secs=0; 
      mins++;
      if (mins > 59) { 
        mins=0;
        secs=0; 
        hours++; 
        if (hours > 23) { 
          secs=0; 
          mins=0; 
          hours=0; 
        }
      }
    }
  WriteDisp(secs, 1);
  WriteDisp(mins ,3);
  WriteDisp(hours,5);
  Now_time = micros();                                // 
  Serial.println(micros() - old_time);                // 
  old_time = Now_time;                                // 
  }
}
  while(1); {
}}


/* Subroutines: 
 * ------------------------------------------------------------------------------ 
 * ------------------------------------------------------------------------------ 
 * ------------------------------------------------------------------------------ */

void blink_led() {
  if ( (secs & 0x01) == 0) { digitalWrite(PIN_LED_blink, HIGH); }
  else                     { digitalWrite(PIN_LED_blink, LOW); }
}

/*
 * Write value XX to display
 * disp=1 > ss 
 * disp=3 > mm
 * disp=5 > hh
 */ 
void WriteDisp (int value, int disp) {
  lc.setDigit(0,disp,value/10,false);
  lc.setDigit(0,disp-1,value%10,false);
}

/*
 * Increase MM of the display every 2 seconds when holding down MM button
 */
void checkBTN_setMM() {
  if (MM_pressed > 1) {
    mins++;
    if (mins > 59) {
      mins=0;
    }
    WriteDisp(mins,3);
    MM_pressed=0;
    digitalWrite(PIN_LED_bright, HIGH);
    delay(100);
    digitalWrite(PIN_LED_bright, LOW);
  }
  if (digitalRead(PIN_butt_setMM) == HIGH) {
    MM_pressed++;
  }
}


/*
 * Increase HH of the display every 2 seconds when holding down HH button
 */
void checkBTN_setHH() {
  if (HH_pressed > 1) {
    hours++;
    if (hours > 23) {
      hours=0;
    }
    WriteDisp(hours,5);
    HH_pressed=0;
    digitalWrite(PIN_LED_bright, HIGH);
    delay(100);
    digitalWrite(PIN_LED_bright, LOW);
  }
  if (digitalRead(PIN_butt_setHH) == HIGH) {
    HH_pressed++;
  }
}

/*
 * Increase brightness of the display every 2 seconds when holding down brightness button
 * until max value, then brightness goes low
 */
void checkBTN_brightness() {
  if (bright_pressed > 1) {
    brightness++;
    if (brightness > 15) {                      // MAX brightness value
      brightness=0;                             // MIN brightness value
    }
    lc.setIntensity(0,brightness);              // Set new brightness value
    bright_pressed=0;
    digitalWrite(PIN_LED_bright, HIGH);
    delay(100);
    digitalWrite(PIN_LED_bright, LOW);
  }
  if (digitalRead(PIN_butt_bright) == HIGH) {
    bright_pressed++;
  }
}

/*
 * Write blinking "helo" on display over HH and MM
 * Used as an welcome message during initialization
 */
void write_helo() { 
lc.clearDisplay(0); // 
for (int a=0; a<4; a++) {
   lc.setChar(0,5,'h',false);
   lc.setChar(0,4,'e',false);
   lc.setChar(0,3,'l',false);
   lc.setChar(0,2,'0',false);
   delay(500);
   lc.setChar(0,2,' ',false); // пустота
   lc.setChar(0,3,' ',false);
   lc.setChar(0,4,' ',false);
   lc.setChar(0,5,' ',false);
   delay(500);
   lc.clearDisplay(0);
   }
}

/*
 * Lights upp every led i display
 * Used as an delay during initialization
 */
void check_disp(){

  lc.clearDisplay(0);
  for (int y=5; y>-1; y--) {
    lc.setLed(0,y,6,true);
    lc.setLed(1,y,6,true);
    delay(150);
    lc.setLed(0,y,5,true);
    lc.setLed(1,y,5,true);
    delay(150);
  }
  for (int y=5; y>-1; y--) {
    lc.setLed(0,y,2,true);
    lc.setLed(1,y,2,true);
    delay(150);
    lc.setLed(0,y,3,true);
    lc.setLed(1,y,3,true);
    delay(150);
  }
  for (int y=5; y>-1; y--) {
    lc.setLed(0,y,1,true);
    lc.setLed(1,y,1,true);
    delay(150);
    //lc.setLed(0,y,1,false);
    lc.setLed(0,y,7,true);
    lc.setLed(1,y,7,true);
    delay(150);
    //lc.setLed(0,y,7,false);
    lc.setLed(0,y,4,true);
    lc.setLed(1,y,4,true);
    delay(150);
    //lc.setLed(0,y,4,false);
  }
  delay(2000);
  lc.clearDisplay(0);
}
