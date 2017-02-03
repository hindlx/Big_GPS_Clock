/*
 * Big_GPS_Clock.ino
 * 2017-02-03 Version 1.01
 * Lars-Johan Lindh
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
#include "avr/wdt.h"         // Built in watchdog

#include <LedControl.h>
LedControl lc=LedControl(10,12,11,2);
/* pin 11 is connected to the CS (CS)(LOAD)
 * pin 12 is connected to the CLK (CLK)
 * pin 10 is connected to LOAD(DIN)
 */

#include <Time.h>
#include <AltSoftSerial.h>
// Connect the GPS RX/TX to arduino pins 8(RX) and 9(TX)
AltSoftSerial serial ;

const int     TZ  = 1;           // GPS time = UTC/GMT set TZ according to time zone
int           DST = 0;           // Daylight Saving Time start variable set
int           LightSensorValue;  // LDR-value
int           DEBUG = 1;         // Prints time and date to serial port when DEBUG = 1

//
time_t prevDisplay = 0;          // When the digital clock was displayed last

const char UBLOX_INIT[] PROGMEM = {
  // Disable NMEA http://www.iforce2d.net/
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x24, // GxGGA off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x2B, // GxGLL off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x02,0x00,0x00,0x00,0x00,0x00,0x01,0x02,0x32, // GxGSA off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x03,0x00,0x00,0x00,0x00,0x00,0x01,0x03,0x39, // GxGSV off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x04,0x40, // GxRMC off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x05,0x00,0x00,0x00,0x00,0x00,0x01,0x05,0x47, // GxVTG off

  // Disable UBX http://www.iforce2d.net/
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x17,0xDC, //NAV-PVT off
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0xB9, //NAV-POSLLH off
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x13,0xC0, //NAV-STATUS off

  // Enable UBX http://www.iforce2d.net/
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x18,0xE1, //NAV-PVT on
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x01,0x00,0x00,0x00,0x00,0x13,0xBE, //NAV-POSLLH on
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x01,0x00,0x00,0x00,0x00,0x14,0xC5, //NAV-STATUS on

  // Rate http://www.iforce2d.net/
  //0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12, //(10Hz)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A, //(5Hz)
  0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39, //(1Hz)
};

const unsigned char UBX_HEADER[] = { 0xB5, 0x62 };

struct NAV_PVT {
  unsigned char cls;
  unsigned char id;
  unsigned short len;
  unsigned long iTOW;          // GPS time of week of the navigation epoch (ms)
  
  unsigned short year;         // Year (UTC) 
  unsigned char month;         // Month, range 1..12 (UTC)
  unsigned char day;           // Day of month, range 1..31 (UTC)
  unsigned char hour;          // Hour of day, range 0..23 (UTC)
  unsigned char minute;        // Minute of hour, range 0..59 (UTC)
  unsigned char second;        // Seconds of minute, range 0..60 (UTC)
  char valid;                  // Validity Flags (see graphic below)
  unsigned long tAcc;          // Time accuracy estimate (UTC) (ns)
  long nano;                   // Fraction of second, range -1e9 .. 1e9 (UTC) (ns)
  unsigned char fixType;       // GNSSfix Type, range 0..5
  char flags;                  // Fix Status Flags
  char flags2;                 // Fix Status Flags
  unsigned char reserved1;     // reserved
  unsigned char numSV;         // Number of satellites used in Nav Solution
  
  long lon;                    // Longitude (deg)
  long lat;                    // Latitude (deg)
  long height;                 // Height above Ellipsoid (mm)
  long hMSL;                   // Height above mean sea level (mm)
  unsigned long hAcc;          // Horizontal Accuracy Estimate (mm)
  unsigned long vAcc;          // Vertical Accuracy Estimate (mm)
  
  long velN;                   // NED north velocity (mm/s)
  long velE;                   // NED east velocity (mm/s)
  long velD;                   // NED down velocity (mm/s)
  long gSpeed;                 // Ground Speed (2-D) (mm/s)
  long heading;                // Heading of motion 2-D (deg)
  unsigned long sAcc;          // Speed Accuracy Estimate
  unsigned long headingAcc;    // Heading Accuracy Estimate
  unsigned short pDOP;         // Position dilution of precision
  short reserved2;             // Reserved
  long headingVeh;             // Heading of vehicle
  unsigned long reserved3;     // Reserved
};

NAV_PVT pvt;

void setup() {
  Serial.begin(9600);
  lc.shutdown(0,false);            // Bring MAX72XX out off power-saving mode
  lc.setIntensity(0,0);            // Set the brightness to startvalue
  lc.clearDisplay(0);              // Clear the display 
  lc.shutdown(1,false);            // Bring MAX72XX out off power-saving mode
  lc.setIntensity(1,0);            // Set the brightness to startvalue
  lc.clearDisplay(1);              // Clear the display 
  Serial.print("Configuration of GPS ...");
  /* start softSerial and send configuration data in UBX protocol
   * see http://www.iforce2d.net/
   */
  serial.begin(9600);
  for(int i = 0; i < sizeof(UBLOX_INIT); i++) {                        
    serial.write( pgm_read_byte(UBLOX_INIT+i) );
    Serial.print(".");
    delay(5);  }
    Serial.println(".");
    Serial.println("GPS ready");
    delay(100);  
    check_disp();                    // Lights all led i disply for visual check of broken LEDs
    write_helo();                    // Write msg to display
    wdt_enable(WDTO_8S);             // Start Arduino Watchdog and set Threshold value to 8 sec
}

void loop() {
  while (serial.available()) {
    processGPS();
    setTime(pvt.hour, pvt.minute, pvt.second, pvt.day, pvt.month, pvt.year);
    adjustTime((TZ+DST) * SECS_PER_HOUR);  }
  if (timeStatus()!= timeNotSet) {
    if (now() != prevDisplay) { //update the display only if the time has changed
      prevDisplay = now();
      if (pvt.fixType > -1 and pvt.fixType < 6)
      digitalClockDisplay();  
      DST = calcDST(); }   // Check DST
  }
  wdt_reset();                                      // Reset the watchdog otherwise the arduino restarts
}

/*
 * Calculate when summertime
 * PARAMS: 
 * RETURN: Return DST set to 0 or 1
 */
int calcDST(){
  if (month() > 3 and month() < 10){                                                                           // Summer month 4,5,6,7,8,9 
//    Serial.print(" Summertime "); 
    return 1; } 
  if ( month() == 3 ) {
    if ( ((day()*24*60*60+hour()*60*60+minute()*60+second()) < ((NthDate(year(), 3, 0, 1))*24*60*60+7200)) ) { // Spring DST ON
//      Serial.print(" Wintertime mars "); 
      return 0; } 
    else {
//      Serial.print(" Summertime mars"); 
      return 1; }
  }
  if ( month() == 10 ) {
    if ( ((day()*24*60*60+hour()*60*60+minute()*60+second()) < ((NthDate(year(), 10, 0, 1))*24*60*60+7200)) ) { // Autumn DST OFF
//      Serial.print(" Summertime october "); 
      return 1; } 
    else {
//      Serial.print("Wintertime october"); 
      return 0; }
  }
//    Serial.print(" Wintertime  "); 
    return 0;                                                                                                   // Winter month 1,2,,11,12
}

/*
 * Write hour, minute and second to display #1
 * GPS-fix value, Day Of Week and light sensor value to display #2 
 * PARAMS: -
 * RETURN: -
 */
void digitalClockDisplay(){
  WriteDisp(second(),0,1);
  WriteDisp(minute(),0,3);
  WriteDisp(hour(),0,5);
  WriteDisp(pvt.fixType,1,7);
  if ( (pvt.second & 0x01) == 0) { 
    lc.setLed(0,0,0,false);    // Blink ':' every second
    lc.setLed(0,4,0,false);  } // Blink ':' every second
   else { 
    lc.setLed(0,0,0,true); 
    lc.setLed(0,4,0,true); }
  LightSensorValue = (LightSensorValue*.8  + (0.2)*constrain(map(analogRead(A1), 550, 1000 ,0 ,19),0 ,19));
  lc.setIntensity(0,LightSensorValue);              // Set new brightness value
  lc.setIntensity(1,LightSensorValue);              // Set new brightness value
  WriteDisp(LightSensorValue,1,1);
  WriteDisp(DoW(year(), month(), day()),1,4);

  // digital clock display the time on serial port if DEBUG=1
  if (DEBUG == 1 ){
    Serial.print(hour());
    Serial.print(":");
    printDigits(minute());
    Serial.print(":");
    printDigits(second());
    Serial.print(" ");
    Serial.print(year()); 
    Serial.print("-");
    printDigits(month());
    Serial.print("-");
    printDigits(day());
    Serial.println(" "); }
}

/*
 * Write data to 7-segment display
 * PARAMS: value to display, display number, digit number 
 * RETURN: -
 */
void WriteDisp (int value, int disp, int dig) {
  if ( disp == 0 )if ( dig == 5 )if ( value < 10) { lc.setDigit(disp,dig-1,value,false); lc.setChar(disp,dig,' ',false); return;}
  lc.setDigit(disp,dig,value/10,false);
  lc.setDigit(disp,dig-1,value%10,false);
}

/*
 * Calculate Day of Week from the date with Sakamoto's Algorithm
 * https://en.wikipedia.org/wiki/Determination_of_the_day_of_the_week#Sakamoto.27s_algorithm
 * PARAMS: year, month, day
 * RETURN: Day of week, Sun=0, Sat=6
 */
int DoW(int y, int m, int d) {
  static int t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
  y -= m < 3; 
  return (y + y/4 - y/100 + y/400 + t[m-1] + d) % 7; 
}

/*
 * Calculate Nth last day in month where weekday is DOW in month where it is 31 days.
 * PARAMS: year, month, day of week, Nth occurence of that day from last day in month
 * RETURN: date
 * 
 */
int NthDate(int y, int m, int DOW, int NthWeek){
  char Date = 31;
  char firstDOW = DoW(y,m,Date);
  while (firstDOW != DOW){
    firstDOW = (firstDOW-1)%7;
    Date--;  }
  return Date;
}

/*
 * Prints a leding zero to singular digits
 * PARAMS: Digit to print to serial
 * RETURN: -
 */
void printDigits(int digits) {
  // utility function for digital clock display: prints preceding colon and leading 0
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

/*
 * Calculate checksum of received GPS reading
 * PARAMS: Reading fråm GPS
 * RETURN: -
 */
void calcChecksum(unsigned char* CK) {
  memset(CK, 0, 2);
  for (int i = 0; i < (int)sizeof(NAV_PVT); i++) {
    CK[0] += ((unsigned char*)(&pvt))[i];
    CK[1] += CK[0];  }
}

/*
 * Read data frpm GPS
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
        fpos = 0; }
    else {      
      if ( (fpos-2) < payloadSize )
        ((unsigned char*)(&pvt))[fpos-2] = c;
      fpos++;
      if ( fpos == (payloadSize+2) ) {
        calcChecksum(checksum);
      }
      else if ( fpos == (payloadSize+3) ) {
        if ( c != checksum[0] )
          fpos = 0;  }
      else if ( fpos == (payloadSize+4) ) {
        fpos = 0;
        if ( c == checksum[1] ) {
          return true; }
      }
      else if ( fpos > (payloadSize+4) ) {
        fpos = 0;  }
    }
  }
  return false;
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
   lc.setChar(0,2,' ',false);
   lc.setChar(0,3,' ',false);
   lc.setChar(0,4,' ',false);
   lc.setChar(0,5,' ',false);
   delay(500);
   lc.clearDisplay(0);
   }
}

/*
 * Lights upp every led i display to check
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
  delay(500);
  lc.clearDisplay(0);
  lc.clearDisplay(1);
}
