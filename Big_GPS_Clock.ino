/*
 * Big_GPS_Clock.ino
 * by Lars-Johan Lindh
 * Yet another GPS syncronised clock, built on Ardunino mini and u-blox GPS module.
 * Use MAX7219 one 7-segment display for time and optional display for status
 * messages.
 *
 * DP (digit 3 and 4) for colon "blink".
 * Use DP (digit 1) for indication of many > 3 satelites in sight, means very high time accuracy.
 *
 * GPS u-blox 8, software version 14.
 * Set TZ for timezone adjustments against UTC-time. 
 * DST calculations are for Sweden, last sunday in mars and october.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
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
#define Version   "170328"        // Softvare version / date
#define displays  1               // # of displays
#define DEBUG     0               // Prints time and date to serial port when DEBUG = 1

const int     TZ  = 1;            // GPS time = UTC, set TZ according to time zone
int           DST = 0;            // Daylight Saving Time start variable set
float         LightSensorValue;   // LDR-value

// Dependensis / Librarys
#include "avr/wdt.h"              // Built in watchdog

#include <LedControl.h>
LedControl lc = LedControl(12, 11, 13, displays);
 /* pin 11 is connected to the CLK (CLK)
  * pin 12 is connected to LOAD(DIN)
  * pin 13 is connected to the CS (CS)(LOAD)
  */
#include <Time.h>                 // Use Time lib for time adjusting
#include <AltSoftSerial.h>        // Connect the GPS RX/TX to arduino mini pins 8 and 9, default AltSoftSerial pin
AltSoftSerial serial ;

time_t prevDisplay = 0;           // When the digital clock was displayed last

const char UBLOX_INIT[] PROGMEM = {
  // Disable NMEA http://www.iforce2d.net/
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x24, // GxGGA off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B, // GxGLL off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32, // GxGSA off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39, // GxGSV off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40, // GxRMC off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47, // GxVTG off

  // Disable UBX http://www.iforce2d.net/
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0xDC, //NAV-PVT off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0xB9, //NAV-POSLLH off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0xC0, //NAV-STATUS off

  // Enable UBX http://www.iforce2d.net/
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x18, 0xE1, //NAV-PVT on
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x01,0x00,0x00,0x00,0x00,0x13,0xBE, //NAV-POSLLH on
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x01,0x00,0x00,0x00,0x00,0x14,0xC5, //NAV-STATUS on

  // Rate http://www.iforce2d.net/
  //0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12, //(10Hz)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A, //(5Hz)
  0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xE8, 0x03, 0x01, 0x00, 0x01, 0x00, 0x01, 0x39, //(1Hz)
};

const unsigned char UBX_HEADER[] = { 0xB5, 0x62 };

struct NAV_PVT {                  // u-Blox 8 version 14
  unsigned char  cls;
  unsigned char  id;
  unsigned short len;
  unsigned long  iTOW;            // GPS time of week of the navigation epoch (ms)

  unsigned short year;            // Year (UTC)
  unsigned char  month;           // Month, range 1..12 (UTC)
  unsigned char  day;             // Day of month, range 1..31 (UTC)
  unsigned char  hour;            // Hour of day, range 0..23 (UTC)
  unsigned char  minute;          // Minute of hour, range 0..59 (UTC)
  unsigned char  second;          // Seconds of minute, range 0..60 (UTC)
  char           valid;           // Validity Flags (see graphic below)
  unsigned long  tAcc;            // Time accuracy estimate (UTC) (ns)
  long           nano;            // Fraction of second, range -1e9 .. 1e9 (UTC) (ns)
  unsigned char  fixType;         // GNSSfix Type, range 0..5
  char           flags;           // Fix Status Flags
  char           flags2;          // Fix Status Flags
  unsigned char  numSV;           // Number of satellites used in Nav Solution

  long           lon;             // Longitude (deg)
  long           lat;             // Latitude (deg)
  long           height;          // Height above Ellipsoid (mm)
  long           hMSL;            // Height above mean sea level (mm)
  unsigned long  hAcc;            // Horizontal Accuracy Estimate (mm)
  unsigned long  vAcc;            // Vertical Accuracy Estimate (mm)

  long           velN;            // NED north velocity (mm/s)
  long           velE;            // NED east velocity (mm/s)
  long           velD;            // NED down velocity (mm/s)
  long           gSpeed;          // Ground Speed (2-D) (mm/s)
  long           heading;         // Heading of motion 2-D (deg)
  unsigned long  sAcc;            // Speed Accuracy Estimate
  unsigned long  headingAcc;      // Heading Accuracy Estimate
  unsigned short pDOP;            // Position dilution of precision
  unsigned long  reserved1;       // Reserved byte 1-4
  unsigned short reserved1_2;     // Reserved byte 5-6
  long           headingVeh;      // Heading of vehicle
  unsigned long  reserved2;       // Reserved
};

NAV_PVT pvt;

void setup() {
  Serial.begin(9600);
  delay(100);
  Serial.println(" ");
  Serial.println("Booting GPS clock ");
  Serial.print("Software version: 20");
  Serial.println(Version);
  for (int z = 0; z < displays; z++) {
    lc.shutdown(z, false);        // Bring MAX72XX out off power-saving mode
    lc.setIntensity(z, 5);        // Set the brightness to startvalue
    lc.clearDisplay(z);           // Clear the display
    delay(100);
  }
  /* start softSerial and send configuration data in UBX protocol
   * see http://www.iforce2d.net/ */
  Serial.print(" Configuration of GPS ...");
  serial.begin(9600);
  for (int i = 0; i < sizeof(UBLOX_INIT); i++) {
    serial.write( pgm_read_byte(UBLOX_INIT + i) );
    Serial.print(".");
    delay(5);
  }
  Serial.println(".");
  Serial.println("GPS ready");
  write_msg(3);                   // Lights all led in disply for visual check of LEDs
  write_msg(2);                   // Write software version
//  write_msg(4);                   // Lights all led in disply for visual check of LEDs
//  write_msg(1);                   // Write HELO to display
  wdt_enable(WDTO_8S);            // Start Arduino Watchdog and set Threshold value
  Serial.println(" Arduino watchdog activated!!");
}
/* Main loop
 * 
 */
void loop() {
  if ( processGPS() ) {
    setTime(pvt.hour, pvt.minute, pvt.second, pvt.day, pvt.month, pvt.year);
    adjustTime((TZ + DST) * SECS_PER_HOUR);
    if (timeStatus() != timeNotSet) {
      if (now() != prevDisplay) {                  // update the display only if the time has changed
        prevDisplay = now();
        digitalClockDisplay();
        DST = calcDST();                           // Check DST, Yes one sec after
      }
      wdt_reset();                                 // Reset the watchdog otherwise the arduino restarts
    }
  }
}

/* Calculate when summertime in sweden
 * PARAMS:
 * RETURN: Return DST set to 0 or 1 
 */
int calcDST() {
  if (month() > 3 and month() < 10) {              // Summer months 4,5,6,7,8,9
    return 1;
  }
  if ( month() == 3 ) {
    if ( ( (day() * 24 * 60 * 60UL) + (hour() * 60 * 60UL) + (minute() * 60UL) + second()) < (NthDate(year(),  3, 0, 1) * 24 * 60 * 60UL + 7200UL) ) { // Spring DST ON
      return 0;
    }
    else  return 1;
  }
  if ( month() == 10 ) {
    if ( ( (day() * 24 * 60 * 60UL) + ((hour() - DST) * 60 * 60UL) + (minute() * 60UL) + second()) < (NthDate(year(), 10, 0, 1) * 24 * 60 * 60UL + 7200UL) )  { // Autumn DST OFF
      return 1;
    }
    else  return 0;
  }
  return 0;                                        // Winter months 1,2,11,12
}

/* Write hour, minute and second to display #1
 * GPS-fix value, Day Of Week and light sensor value to display #2
 * Blink : if correct time othervise ether ' or . is light
 * PARAMS: -
 * RETURN: -
 */
void digitalClockDisplay() {
  WriteDisp(second(), 0, 1);
  if ( bitRead(pvt.valid, 0) == 1 ) {              // Check if received time from GPS is okay, then show time
    WriteDisp(minute(), 0, 3);
    WriteDisp(hour(), 0, 5);
  }
  else {
    write_msg(0);                                  // write Err to display to indicate GPS time lost
  }
  if ( bitRead(pvt.valid, 2) == 1 ) {              // Check if valid time is received from GPS
    if ( (pvt.second & 0x01) == 0) {
      lc.setLed(0, 3, 0, false);                   // Use DP for Blink ' ' off every 2 second
      lc.setLed(0, 4, 0, false);
    }
    else {
      lc.setLed(0, 3, 0, true);                    // Use DP for Blink ':' on every 2 second
      lc.setLed(0, 4, 0, true);
    }
  }
  else {
    if ( (pvt.second & 0x01) == 0) {
      lc.setLed(0, 3, 0, false);                   // Use DP for Blink '.'  every 2 second
      lc.setLed(0, 4, 0, true);
    }
    else {
      lc.setLed(0, 3, 0, true);                    // Use DP for Blink '''  every 2 second
      lc.setLed(0, 4, 0, false);
    }
  }
  // Check ambient light and increase light on display if brighter ambient light
  LightSensorValue = (LightSensorValue * .8  + (0.2) * constrain(map(analogRead(A0), 650, 1023 , 0 , 19), 2 , 16)); // (int) 15.999999999999999 = 15
  for (int z = 0; z < displays; z++) {
    lc.setIntensity(z, LightSensorValue);          // Set new brightness value display 1
    delay(100);
  }

  // write some stuff to the secondary display if present
  WriteDisp(pvt.fixType, 1, 7);                    // Write GPS fix value to display 2
  if (pvt.numSV < 3 ) {                            // If satelit count > 3 then the time is very correct
    lc.setLed(0, 1, 0, true);                      // Use DP led for idication of almost correct time
  }
  else {
    lc.setLed(0, 1, 0, false);                     //  DP led OFF for idication of realy correct time
  }
  WriteDisp(WeekDay(year(), month(), day()), 1, 4);// Write weekday number sun=0, sat=6
  WriteDisp(LightSensorValue, 1, 1);               // Write brightnes value

  if ( DEBUG ) {                                   // display output on serial port if debug is set
    Serial.print(hour());
    Serial.print(":");
    printDigits(minute());
    Serial.print(":");
    printDigits(second());
    Serial.print("  ");
    Serial.print(year());
    Serial.print("-");
    printDigits(month());
    Serial.print("-");
    printDigits(day());
    Serial.print("  DST=");
    if ( DST ) {
      Serial.print("Summer");
    }
    else {
      Serial.print("Winter");
    }
    Serial.print("  GPS-fix: ");
    Serial.print(pvt.fixType);
    Serial.print("  SatInSight: ");
    Serial.print(pvt.numSV);
    Serial.print("  TimeAccuracy: ");
    Serial.print(pvt.tAcc);
    Serial.print("ns  WeekDay: ");
    Serial.print( WeekDay(year(), month(), day()) );
    Serial.print("  LightSensor: ");
    Serial.print(LightSensorValue);
    Serial.print(" ValidFlag: ");
    Serial.print(bitRead(pvt.valid, 2));
    Serial.print(bitRead(pvt.valid, 1));
    Serial.print(bitRead(pvt.valid, 0));
    if ( bitRead(pvt.valid, 2) == 1 ) {
      Serial.println(" Time Is Valid: ");
    }
    else if ( bitRead(pvt.valid, 1) == 1 ) {
      Serial.println(" Time Is Okay: ");
    }
    else {
      Serial.println(" Time Error : ");
    }
  }
}

/* Write 2-digit value to 7-segment display
 * PARAMS: value to display, display number, digit number
 * RETURN: - 
 */
void WriteDisp (int value, int disp, int dig) {
  if ( disp == 0 )if ( dig == 5 )if ( value < 10) {
        lc.setDigit(disp, dig - 1, value, false);
        lc.setChar(disp, dig, ' ', false);
        return;
      }
  lc.setDigit(disp, dig, value / 10, false);
  lc.setDigit(disp, dig - 1, value % 10, false);
}

/* Calculate Day of Week from the date with Sakamoto's Algorithm
 * https://en.wikipedia.org/wiki/Determination_of_the_day_of_the_week#Sakamoto.27s_algorithm
 * PARAMS: year, month, day
 * RETURN: Day of week, Sun=0, Sat=6 
 */
int WeekDay(int y, int m, int d) {
  static int t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
  y -= m < 3;
  return (y + y / 4 - y / 100 + y / 400 + t[m - 1] + d) % 7;
}

/* Calculate Nth last day in month where weekday is DOW in month where it is 31 days.
 * This module calculates DST date for Sweden.
 * Summertime from last sunday in mars 02:00 to last sunday in october 03:00
 * https://www.timeanddate.com/time/change/sweden/stockholm
 * PARAMS: year, month, day of week, Nth occurence of that day from last day in month
 * RETURN: date 
 */
int NthDate(int y, int m, int DOW, int NthWeek) {
  char Date = 31;
  char firstDOW = WeekDay(y, m, Date);
  while (firstDOW != DOW) {
    firstDOW = (firstDOW - 1) % 7;
    Date--;
  }
  return Date;
}

/* Prints a leding zero to singular digits on Serial
 * PARAMS: Digit to print to serial
 * RETURN: - 
 */
void printDigits(int digits) {
  // utility function for digital clock display: prints preceding colon and leading 0
  if (digits < 10) {
    Serial.print('0');
  }
  Serial.print(digits);
}

/* Calculate checksum of received GPS reading
 * PARAMS: Reading fråm GPS
 * RETURN: -  
 */
void calcChecksum(unsigned char* CK) {
  memset(CK, 0, 2);
  for (int i = 0; i < (int)sizeof(NAV_PVT); i++) {
    CK[0] += ((unsigned char*)(&pvt))[i];
    CK[1] += CK[0];
  }
}

/* Read data from GPS
 * PARAMS:
 * RETURN: Reading OK or NotOK  
 */
bool processGPS() {
  static int fpos = 0;
  static unsigned char checksum[2];
  const int payloadSize = sizeof(NAV_PVT);

  while ( serial.available() ) {
    byte c = serial.read();
    if ( fpos < 2 ) {
      if ( c == UBX_HEADER[fpos] )
        fpos++;
      else
        fpos = 0;
    }
    else {
      if ( (fpos - 2) < payloadSize )
        ((unsigned char*)(&pvt))[fpos - 2] = c;
      fpos++;
      if ( fpos == (payloadSize + 2) ) {
        calcChecksum(checksum);
      }
      else if ( fpos == (payloadSize + 3) ) {
        if ( c != checksum[0] )
          fpos = 0;
      }
      else if ( fpos == (payloadSize + 4) ) {
        fpos = 0;
        if ( c == checksum[1] ) {
          return true;
        }
      }
      else if ( fpos > (payloadSize + 4) ) {
        fpos = 0;
      }
    }
  }
  return false;
}

/* Write messages on display
 * PARAMS: message number
 * RETURN: -  
 */
void write_msg( int msg_type) {
  switch (msg_type) {
    case 0:
      /* Write "Err " on display(0) over HH and MM
       * Used as an error message if GPS time lost 
       */
      lc.setChar(0, 5, 'e', false);
      lc.setLed(0, 4, 5, true);
      lc.setLed(0, 4, 7, true);
      lc.setLed(0, 3, 5, true);
      lc.setLed(0, 3, 7, true);
      lc.setChar(0, 2, ' ', false);
      break;
    case 1:
      /* Write blinking "helo" on display(0) over HH and MM
       * Used as an welcome message during initialization 
       */
      lc.clearDisplay(0);
      for (int a = 0; a < 4; a++) {
        lc.setChar(0, 5, 'h', false);
        lc.setChar(0, 4, 'e', false);
        lc.setChar(0, 3, 'l', false);
        lc.setChar(0, 2, '0', false);
        delay(500);
        lc.clearDisplay(0);
        delay(500);
      }
      break;
    case 2:
      /* Write software Version on display(0)
       * Message during initialization
       */
      lc.clearDisplay(0);
      for (int i=0; i < 6; i++) {
        lc.setChar(0, i, Version[5-i], false);
      }
      delay(2000);
      lc.clearDisplay(0);
      break;
    case 3:
      /* Lights upp every led and DP in display(0,1) to check
       * display during initialization 
       */
      lc.clearDisplay(0);
      lc.clearDisplay(1);
      for (int y = 5; y > -1; y--) {
        lc.setLed(0, y, 0, true);
        delay(150);
      }
      for (int y = 5; y > -1; y--) {
        lc.setLed(0, y, 6, true);
        lc.setLed(1, y, 6, true);
        delay(150);
        lc.setLed(0, y, 5, true);
        lc.setLed(1, y, 5, true);
        delay(150);
      }
      for (int y = 5; y > -1; y--) {
        lc.setLed(0, y, 2, true);
        lc.setLed(1, y, 2, true);
        delay(150);
        lc.setLed(0, y, 3, true);
        lc.setLed(1, y, 3, true);
        delay(150);
      }
      for (int y = 5; y > -1; y--) {
        lc.setLed(0, y, 1, true);
        lc.setLed(1, y, 1, true);
        delay(150);
        lc.setLed(0, y, 7, true);
        lc.setLed(1, y, 7, true);
        delay(150);
        lc.setLed(0, y, 4, true);
        lc.setLed(1, y, 4, true);
        delay(150);
      }
      delay(500);
      lc.clearDisplay(0);
      lc.clearDisplay(1);
      break;
    case 4:

      /* Lights upp every led in display(0,1) to check
       * display during initialization 
       */
      lc.clearDisplay(0);
      lc.clearDisplay(1);
      for (int y = 0; y < 6; y++) {
        lc.setLed(0, y, 2, true);
        lc.setLed(1, y, 2, true);
        delay(150);
        lc.setLed(0, y, 3, true);
        lc.setLed(1, y, 3, true);
        delay(150);
        lc.setLed(0, y, 1, true);
        lc.setLed(1, y, 1, true);
        delay(150);
        lc.setLed(0, y, 7, true);
        lc.setLed(1, y, 7, true);
        delay(150);
        lc.setLed(0, y, 4, true);
        lc.setLed(1, y, 4, true);
        delay(150);
        lc.setLed(0, y, 6, true);
        lc.setLed(1, y, 6, true);
        delay(150);
        lc.setLed(0, y, 5, true);
        lc.setLed(1, y, 5, true);
        delay(150);
      }
      delay(500);
      lc.clearDisplay(0);
      lc.clearDisplay(1);
      break;
  }
}
