/* =====================================
// Title : Fibonacciklok
// Author: Ed Nieuwenhuys
// V001 Stripped version from Fibonacci_Colour_Clock_SK6812_WS2812_1284-328-V003
// V002 Optimized software. Working version for Arduino Nano
// V003 Development
// V004 Development 
// V005 In Fibonacciklok No 5&6
// V006 Derived from VierkantbuisklokV005. Rotary added, Keypad wirh 7 pins and for One-wire keypad. Menu changed
// V007 Release version in Fibonacci-klok No7
// V008 Optimized HC-12 receiver code from: https://forum.arduino.cc/index.php?topic=396450.0
// V009 Changed time input. Only 6 digits allowed in format hhmmss. T124500 for time entry in menu added
// V010 Added coding for the stick clock that uses no PCB
// V011 Changed palettes P0-P9 and colours in it. Corrected BT pinnumbers. corrected initialisation: int Previous_LDR_read = 51 --> 512;
// V012 Palette stored in EEPROM. Darkgray in palette choices added
// V013 Removed Secpulse and simplified Brightness calculation. Cleaned coding
// V014 12-edge clock, Added MAX7219, Added time MAX7219 On/Off
// V015 Normal, Extreme, Ultimate clock in Fibonacci display added. Time display LEDs On/Off added


// Turn on or off the compilation of the used modules by the #defines. 
// A // before the #define turns compilation off 
// The time is presented in the coloured LEDs that are grouped in the {1,1,2,3,5, .. compartments.
// One can use as many LEDs in a compartment as needed but this had to be noted in the program
// Change the number of LEDs around line 116: const byte NUM_LEDS  = 12; 
// Also update the LED positions in  the function setPixel()  
   =====================================*/

//--------------------------------------------
// ARDUINO Definition of installed modules
//--------------------------------------------
                 // ***   Define one type of LED strip 
//#define LED2812                // Use  RGB LED strip WS2812
#define LED6812                  // Use RGBW LED strip SK6812
const byte NUM_LEDS =  12;       //  How many leds in  strip?  --->  Update LED positions in  setPixel() (at +/- line 920) TOO!  
// ---------------  Define one type of clock
#define STICKCLOCK               // If wires of clock and LEDs are directly connected to Arduino Nano
//#define PCBCLOCK               // If Fibonacci clock is made on PCB. PCB uses other connections, see below

// ---------------  Define if a module is present or uncomment it by adding // before a #define 
#define ROTARYMOD                // Use rotary encoder
//#define KEYPAD3x4              // Use a 3x4 keypad with 7 wires
//#define ONEWIREKEYPAD          // Use a 3x4 keypad with one wire
//#define HC12MOD                // Use HC12 time transreceiver Long Range Wireless Communication Module
#define BLUETOOTHMOD
#define MOD_DS3231               // DS3231 RTC module installed
#define MAX7219                  // MAX7219 display

char VERSION[] = "V015";         // Version of this coding

//--------------------------------------------
// ARDUINO Includes defines and initialisations
// All libraries can be installed from the Arduino IDE
// Tools --> Manage libraries
//--------------------------------------------
                     #if defined(KEYPAD3x4) || defined(ONEWIREKEYPAD)
#define KEYPAD
                     #endif
                     #ifdef ROTARYMOD
#include <Encoder.h>             // http://www.pjrc.com/teensy/td_libs_Encoder.html
                     #endif ROTARYMOD
#include <Wire.h>                // Default Arduino library
#include <RTClib.h>              // https://github.com/adafruit/RTClib 
#include <EEPROM.h>              // Default Arduino library To store data in EEPROM
#include <TimeLib.h>             // Arduino library Time by Michael Margolis   http://playground.arduino.cc/Code/Time/   
#include <Adafruit_NeoPixel.h>   // https://github.com/adafruit/Adafruit_NeoPixel   for LED strip WS2812 or SK6812
                     #ifdef KEYPAD
#include "Adafruit_Keypad.h"     // https://github.com/adafruit/Adafruit_Keypad//                     
                     #endif KEYPAD 
                     #ifdef BLUETOOTHMOD
#include <SoftwareSerial.h>        // Arduino library for Bluetooth communication
                     #endif BLUETOOTHMOD    
                     #ifdef HC12MOD
#include <SoftwareSerial.h>       // Arduino library For HC12 or Bluetooth
                     #endif HC12MOD  
                     #ifdef MAX7219
#include <LedControl.h> 
                      #endif MAX7219 
//--------------------------------------------
// PIN Assigments
//-------------------------------------------- 
enum DigitalPinAssignments {
                    #ifdef STICKCLOCK
 HC_12TX      = 0,                // HC-12 TX Pin
 HC_12RX      = 1,                // HC-12 RX Pin    
 EmptyD00     = 0,                // EmptyD00
 EmptyD01     = 1,                // EmptyD01 
 EncoderPow   = 2,                // give power to Encoder
 clearButton  = 3,                // switch (labeled SW on decoder)
 encoderPinA  = 4,                // right (labeled DT on decoder)
 encoderPinB  = 5,                // left (labeled CLK on decoder)
 BT_TX        = 6,                // Bluetooth TX 
 BT_RX        = 7,                // Bluetooth RX
 LED_PIN      = 8,                // Pin to control colour SK6812 WS2812 LEDs 
 HeartbeatLED = 9,                // EmptyD09
 MAX7219CLK   = 10,               // MAX7219CLK
 MAX7219CS    = 11,               // MAX7219CS 
 MAX7219DataIn= 12,               // MAX7219DataIn 
                     #endif STICKCLOCK
                     #ifdef PCBCLOCK 
 HC_12TX      = 0,                // HC-12 TX Pin 
 HC_12RX      = 1,                // HC-12 RX Pin 
// BT_TX        = 0,                // Bluetooth TX 
// BT_RX        = 1,                // Bluetooth RX
 EmptyD02     = 2,                // EmptyD02
 encoderPinA  = 3,                // right (labeled DT on Rotary decoder)
 clearButton  = 4,                // switch (labeled SW on Rotary decoder)
 LED_PIN      = 5,                // Pin to control colour SK6812 WS2812 LEDs
 BT_TX        = 6,                // Bluetooth TX 
 BT_RX        = 7,                // Bluetooth RX
 encoderPinB  = 8,                // left (labeled CLK on Rotary decoder)
 EmptyD09     = 9,                // EmptyD09 
 MAX7219CLK   = 10,               // MAX7219CLK
 MAX7219CS    = 11,               // MAX7219CS 
 MAX7219DataIn= 12,               // MAX7219DataIn
//                   #ifdef KEYPAD3x4
 KEYPAD1      = 6,                // Used for 3*4 KEYPAD      
 KEYPAD2      = 7,                // Used for 3*4 KEYPAD
 KEYPAD3      = 8,                // Used for 3*4 KEYPAD
 KEYPAD4      = 9,                // Used for 3*4 KEYPAD
 KEYPAD5      = 10,               // Used for 3*4 KEYPAD
 KEYPAD6      = 11,               // Used for 3*4 KEYPAD
 KEYPAD7      = 12,               // Used for 3*4 KEYPAD
//                   #endif KEYPAD3x4
                     #endif PCBCLOCK
 secondsPin   = 13};              // if set to 13 led will blink on board     SCK
                                   // Analogue hardware constants ----
enum AnaloguePinAssignments {
 EmptyA0      = 0,                // EmptyA0
 EmptyA1      = 1,                // EmptyA1 
 PhotoCellPin = 2,                // LDR pin
 OneWirePin   = 3,                // OneWire Keypad
 SDA_pin      = 4,                // SDA pin
 SCL_pin      = 5,                // SCL pin
 EmptyA6      = 6,                // EmptyA6
 EmptyA7      = 7};               // EmptyA7
//--------------------------------------------
// HC-12 Long Range Wireless Communication Module
//--------------------------------------------
                        #ifdef HC12MOD
SoftwareSerial HC12(HC_12TX, HC_12RX); // HC-12 TX Pin, HC-12 RX Pin
//const byte numChars = 32;
//char receivedChars[numChars];
//bool newData = false;
                         #endif HC12MOD
                         #ifdef BLUETOOTHMOD
SoftwareSerial Bluetooth(BT_RX, BT_TX);                               // BT_RX <=> TXD on BT module, BT_TX <=> RXD on BT module
String  BluetoothString;
                         #endif BLUETOOTHMOD                       
//--------------------------------------------
// LED
//--------------------------------------------
bool  LEDsAreOff               = false;                               // If true LEDs are off except time display
byte  TurnOffLEDsAtHH          = 0;                                   // Turn off the LEDs at ..hh
byte  TurnOnLEDsAtHH           = 0;                                   // Turn on  the LEDs at ..hh
const byte BRIGHTNESS          = 10;                                  // BRIGHTNESS 0 - 255
int   Previous_LDR_read        = 512;                                 // Previous light sensor value
int   ToggleEdsoft             = 1;                                   // Flash LEDs every hour  
                         #ifdef LED6812    
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRBW + NEO_KHZ800);  //NEO_RGBW
                         #endif LED6812  
                         #ifdef LED2812
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);  //NEO_RGB NEO_GRB
                         #endif LED2812
//--------------------------------------------
// COLOURS
//--------------------------------------------   

#define CLOCK_PIXELS    5                                            // Number of cells in clock = 5 (1,1,2,3,5)
#define TOTAL_PALETTES 10
#define DISPLAY_PALETTE 1                                            // Number of palette to start with
byte DisplayPalette = DISPLAY_PALETTE;                               // Palette to display 1 = Mondriaan
byte NormalExtremeUltimate = 0;                                      // 0 = Normal, 1 = Extreme, 2 = Ultimate display of colours                
bool FiboChrono =  true;                                             // true = Fibonacci, false = chrono clock display
byte bits[CLOCK_PIXELS+1];                                           // Stores the hours=1 and minutes = 2 to set in LEDsetTime(byte hours, byte minutes)
byte BitSet[CLOCK_PIXELS+1];                                         // For calculation of the bits to set

                         #ifdef LED2812
const uint32_t white  = 0xFFFFFF;                                    // R, G and B on together gives white light
const uint32_t lgray  = 0x666666;
const uint32_t gray   = 0x333333;
const uint32_t dgray  = 0x222222;                 
                         #endif LED2812
                         #ifdef LED6812    
const uint32_t white  = 0xFF000000;                                  // The SK6812 LED has a white LED that is pure white
const uint32_t lgray  = 0x66000000;
const uint32_t gray   = 0x33000000;
const uint32_t dgray  = 0x22000000;
                         #endif LED6812  
const uint32_t black  = 0x000000;
const uint32_t red    = 0xFF0000;
const uint32_t orange = 0xFF7600;
const uint32_t yellow = 0xFFDD00;
const uint32_t dyellow= 0xFFAA00;
const uint32_t apple  = 0x80FF00;
const uint32_t brown  = 0xC65F00;
const uint32_t green  = 0x00FF00;
const uint32_t grass  = 0x00FF80;
const uint32_t sky    = 0x00FFFF;
const uint32_t marine = 0x0080FF;
const uint32_t blue   = 0x0000FF;
const uint32_t pink   = 0xFF0080;
const uint32_t purple = 0xFF00FF;
const uint32_t colors[TOTAL_PALETTES][5] = 
    {//off   hours   minutes both;
    { white, red     , green   , blue  , green },  // #0 RGB  
    { white, red     , yellow  , blue  , green },  // #1 Mondriaan
    { gray,  red     , dyellow , blue  , black },  // #2 Mondriaan1 
    { white, apple   , green   , grass , blue  },  // #3 Greens
    { white, red     , grass   , purple, green },  // #4 Pastel
    { white, orange  , green   , marine, blue  },  // #5 Modern
    { white, sky     , purple  , blue  , green },  // #6 Cold
    { white, red     , yellow  , orange, green },  // #7 Warm
    { white, brown   , grass   , sky   , green },  // #8 Earth
    { white, red     , dyellow , blue  , green }} ;// #9 Mondriaan 2
            
//--------------------------------------------
// KY-040 ROTARY
//-------------------------------------------- 
                          #ifdef ROTARYMOD                         
Encoder myEnc(encoderPinA, encoderPinB);                            // Use digital pin  for encoder
                          #endif ROTARYMOD      
long     Looptime          = 0;
byte     RotaryPress       = 0;                                     // Keeps track displaychoice and how often the rotary is pressed.
uint32_t RotaryPressTimer  = 0;
byte     NoofRotaryPressed = 0;

//--------------------------------------------
// LDR PHOTOCELL
//--------------------------------------------
byte     LightReducer     = 60;                                     // Factor to dim ledintensity with. Between 0.1 and 1 in steps of 0.05
byte     LowerBrightness  = 5;                                      // Lower limit of Brightness ( 0 - 255)
byte     TestLDR          = 0;                                      // If true LDR inf0 is printed every second in serial monitor
int      OutPhotocell;                                              // Stores reading of photocell;
int      MinPhotocell     = 999;                                    // Stores minimum reading of photocell;
int      MaxPhotocell     = 1;                                      // Stores maximum reading of photocell;

//--------------------------------------------
// CLOCK
//--------------------------------------------                                 
#define  MAXTEXT 50                                                 // Maximum characters for text printing
static   unsigned long msTick;                                      // The number of millisecond ticks since we last incremented the second counter
int      count; 
int      Delaytime            = 200;
byte     ChangeTime           = false;
byte     ChangeLightIntensity = false;
byte     Demo                 = 0;
byte     Toggle_HetWasIsUit   = 0;                                  // Turn off HetIsWas after 10 sec
byte     Is                   = true;                               // Toggle of displaying Is or Was
byte     Isecond, Iminute, Ihour, Iday, Imonth, Iyear; 
byte     lastminute = 0, lasthour = 0, sayhour = 0;
char     sptext[MAXTEXT+2];                                         // for common print use
//--------------------------------------------
// DS3231 CLOCK MODULE
//--------------------------------------------
#define DS3231_I2C_ADDRESS          0x68
#define DS3231_TEMPERATURE_MSB      0x11
#define DS3231_TEMPERATURE_LSB      0x12
        #ifdef MOD_DS3231
RTC_DS3231 RTCklok;
        #else if
RTC_Millis RTCklok;   
        #endif
DateTime Inow;

//--------------------------------------------
// MAX7219 display
//--------------------------------------------
                     
                      #ifdef MAX7219
//const byte NUMBEROF7219DISPLAYS = 1;                       // 1 or 2. Second chained with first one
// pin 10 is connected to the MAX7219CLK 
// pin 11 is connected to MAX7219CS 
// pin 12 is connected to the MAX7219DataIn 
 LedControl lc= LedControl(MAX7219DataIn,MAX7219CLK,MAX7219CS,1); //NUMBEROF7219DISPLAYS);
                       #endif MAX7219 
                       #ifdef KEYPAD
//--------------------------------------------
// KEYPAD 
//--------------------------------------------
String        KeypadString;
unsigned long KeyLooptime;
const byte    ROWS = 4; 
const byte    COLS = 3; 
byte          rowPins[ROWS] = {KEYPAD7,KEYPAD6,KEYPAD5,KEYPAD4}; 
byte          colPins[COLS] = {KEYPAD3,KEYPAD2,KEYPAD1};
   // Wire under * to pin 6. Wire under * to pin 12
char   hexaKeys[ROWS][COLS] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'} };
Adafruit_Keypad Keypad3x4 = Adafruit_Keypad( makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS); 
                        #endif KEYPAD
byte   KeyInputactivated = false;                                 

//--------------------------------------------
// Menu
//--------------------------------------------  
const byte MenuItems = 14;                   // sentences, rows, in menu
const char menu[MenuItems][MAXTEXT] PROGMEM = {
 "12-segment Fibonacci-klok",
 "D D14042020 is date 14 April 2020",
 "E Normal, Extreme or Ultimate mode",
 "F Fibonacci or Chrono display",
 "L (L5) Min light intensity (1-255)",
 "M (M90)Max light intensity (1-250)",
 "N (N2208)Turn On/OFF LEDs between Nhhhh",
 "P (P1) to select a palette (0-9)",
 "I  for this info",
 "R  Reset to default settings",
 "T  T031500 is quarter past three am",
 "W  Test LDR reading every second",
 "X (X50) Demo mode. ms delay (0-9999)",
 "Ed Nieuwenhuys August 2020" };
 
// --------------------------------------------------------------------------
// End Definitions                                                    
// --------------------------------------------------------------------------
//--------------------------------------------
// Version
//-------------------------------------------- 
void SWversion(void) 
{ 
 PrintLine(40);
 for (int i = 0; i < MenuItems; i++)   {strcpy_P(sptext, menu[i]);                     Tekstprintln(sptext);  }
 PrintLine(52);
 sprintf(sptext,"Brightness Min: %3d bits  Max: %3d%%",LowerBrightness, LightReducer); Tekstprintln(sptext);
 sprintf(sptext,"  LDR read Min: %d bits  Max: %3d bits",MinPhotocell, MaxPhotocell);  Tekstprintln(sptext);
 sprintf(sptext,"Number of LEDs: %d   Palette: %d", NUM_LEDS, DisplayPalette);         Tekstprintln(sptext); 
 sprintf(sptext," LEDs off from: %0.2d - %0.2d",TurnOffLEDsAtHH, TurnOnLEDsAtHH);      Tekstprintln(sptext);
 sprintf(sptext,"Version: %s",VERSION);                                                Tekstprintln(sptext); 
 PrintLine(40);
 GetTijd(1);
}

void PrintLine(byte Lengte)
{
 for (int n=0; n<Lengte; n++) {Serial.print(F("_"));} Serial.println(); 
}
//--------------------------------------------
// ARDUINO Loop
//--------------------------------------------
void loop(void)
{
 SerialCheck();
 Heartbeat();
 if(Demo)  Demomode();
 else
  { 
   EverySecondCheck();
   if(!KeyInputactivated) EveryMinuteUpdate();                      // If keyboard input then do not update display
                              #ifdef ROTARYMOD      
   RotaryEncoderCheck(); 
                              #endif ROTARYMOD 
  }
}  
//--------------------------------------------
// ARDUINO Setup initialise the hardware  
//--------------------------------------------
void setup()         // initialise the hardware // initialize the appropriate pins as outputs:
{
 Serial.begin(9600);                                                // Setup the serial port to 9600 baud 
 while (!Serial);                                                   // Wait for serial port to connect. Needed for native USB port only
 Tekstprintln("\n*********\nSerial started"); 
 pinMode(secondsPin,   OUTPUT );
 pinMode(HeartbeatLED, OUTPUT );  
                          #ifdef ROTARYMOD
 pinMode(encoderPinA,  INPUT_PULLUP);
 pinMode(encoderPinB,  INPUT_PULLUP);  
 pinMode(clearButton,  INPUT_PULLUP);
                          #ifdef STICKCLOCK
 pinMode(EncoderPow,   OUTPUT );
 digitalWrite(EncoderPow,HIGH);                                     // Provide the rotary encoder with power
                          #endif STICKCLOCK
 Tekstprintln("Rotary encoder enabled");
                          #endif ROTARYMOD
                          #ifdef KEYPAD3x4
 Keypad3x4.begin();                                                 // Start the 3x4 keypad
 Tekstprintln("4*3 keypad 7 wires enabled");
                          #endif KEYPAD3x4
                          #ifdef ONEWIREKEYPAD  
 Tekstprintln("4*3 keypad one wire enabled");                       // The one wire keypad is enabled
                          #endif ONEWIREKEYPAD
                          #ifdef MOD_DS3231
 Tekstprintln("RTC DS3231 enabled");
                          #else
 Tekstprintln("Internal clock enabled");                          
                          #endif MOD_DS3231
 strip.begin();                                                     // Start communication to LED strip
 strip.setBrightness(BRIGHTNESS);                                   // Set brightness of LEDs
 ShowLeds();  
                          #ifdef LED6812    
 Tekstprintln("LEDs SK6812 enabled");
                         #endif LED6812  
                         #ifdef LED2812
 Tekstprintln("LEDs WS2812 enabled");
                         #endif LED2812
                         #ifdef BLUETOOTHMOD 
                         #if defined ARDUINO_SAMD_MKRWIFI1010       // || defined ARDUINO_AVR_NANO_EVERY
 Serial1.begin(9600);                                               // Bluetooth connected to Serial1
 Tekstprintln("Bluetooth MKR enabled"); 
                         #else if
 Bluetooth.begin(9600); 
 Tekstprintln("Bluetooth enabled"); 
                         #endif ARDUINO_SAMD_MKRWIFI1010
                         #endif BLUETOOTHMOD
                         #ifdef MAX7219                     
 InitialyseMAX7219();
 sprintf(sptext, "12 34 56");  PrintStringToSegDisplay(sptext);         
 Tekstprintln("MAX7219 enabled");     
 lc.clearDisplay(0);
                         #endif MAX7219 
                         #ifdef HC12MOD
 HC12.begin(9600);                                                  // Serial port to HC12
 Tekstprintln("HC-12 time receiver enabled");
                         #endif HC12MOD 
 Wire.begin();                                                      // start the wire communication I2C for RTC module
                         #ifdef MOD_DS3231
 RTCklok.begin();                                                   // start the RTC-module
                         # else if 
 RTCklok.begin(DateTime(F(__DATE__), F(__TIME__)));                 // if no RTC module is installed use the ATMEGAchip clock
                         #endif MOD_DS3231
 DateTime now = RTCklok.now();
 DateTime compiled = DateTime(F(__DATE__), F(__TIME__));
 if (now.unixtime() < compiled.unixtime()) 
  {
   Serial.println(F("RTC is older than compile time! Updating"));  // following line sets the RTC to the date & time this sketch was compiled
   RTCklok.adjust(DateTime(F(__DATE__), F(__TIME__))); 
  }
// RTCklok.adjust(DateTime(10,04,20, 12, 00, 10 ));// F(__TIME__))); 
 if (EEPROMread(0) <3 || EEPROMread(0) > 200)  EEPROMwrite(0, LightReducer);     // Default intensity for this clock
 if (EEPROMread(1) <1 || EEPROMread(1) > 100)  EEPROMwrite(1, LowerBrightness);  // Default Lower Brightness for this clock
 if (EEPROMread(2) > 9)  EEPROMwrite(2, DISPLAY_PALETTE);                        // Default for this clock 
 if (EEPROMread(3) > 24) EEPROMwrite(3, 0);                         // Default for this clock 
 if (EEPROMread(4) > 24) EEPROMwrite(4, 0);                         // Default for this clock 
 if (EEPROMread(5) > 1)  EEPROMwrite(5, 0);                         // Default for this clock 
 LightReducer    = EEPROMread(0);                                   // Store EEPROMvalue is the work variable
 LowerBrightness = EEPROMread(1);                                   // Store EEPROMvalue is the work variable
 DisplayPalette  = EEPROMread(2);                                   // Store EEPROMvalue is the work variable
 TurnOffLEDsAtHH = EEPROMread(3);                                   // Store EEPROMvalue is the work variable
 TurnOnLEDsAtHH  = EEPROMread(4);                                   // Store EEPROMvalue is the work variable
 FiboChrono      = EEPROMread(5);                                   // Store EEPROMvalue is the work variable
 Looptime = millis();                                               // Used in KY-040 rotary
 msTick   = millis();                                               // Used in second loop 
 SWversion();                                                       // Display the version number of the software
 //Selftest();                                                      // Play the selftest
 GetTijd(0);                                                        // Get the time and print it to serial
 Displaytime();
 } 

//--------------------------------------------
// CLOCK Demo mode
//--------------------------------------------
void Demomode(void)
{
 if ( millis() - msTick >50)   digitalWrite(secondsPin,LOW);        // Turn OFF the second on pin 13
 if ( millis() - msTick >999)                                       // Flash the onboard Pin 13 Led so we know something is happening
 {    
  msTick = millis();                                                // second++; 
  digitalWrite(secondsPin,HIGH);                                    // turn ON the second on pin 13
  if (++Iminute >59) { Iminute = 0; Isecond = 0; Ihour++;}
  if (    Ihour >23)  Ihour = 0;
  DimLeds(false);
  Displaytime();
 }
}
//--------------------------------------------
// ARDUINO Reset to default settings
//--------------------------------------------
void Reset(void)
{
 LightReducer         = 80;                                        // Factor to dim ledintensity with. Between 0.1 and 1 in steps of 0.05
 EEPROMwrite(0, LightReducer);                                     // Default intensity for this clock
 LowerBrightness      = 10;                                        // Lower limit of Brightness ( 0 - 255)
 EEPROMwrite(1, LowerBrightness);                                  // Default Lower Brightness for this clock
 DisplayPalette       = DISPLAY_PALETTE; 
 EEPROMwrite(2, DISPLAY_PALETTE);
 MinPhotocell         = 1024;                                      // Stores minimum reading of photocell;
 MaxPhotocell         = 1;                                         // Stores maximum reading of photocell;
 TestLDR              = 0;                                         // if true LDR display is printed every second
 ChangeTime           = false;
 ChangeLightIntensity = false;
 Selftest();                                                       // Play the selftest
 GetTijd(0);                                                       // Get the time and store it in the proper variables
 SWversion();                                                      // Display the version number of the software
 Displaytime();
}
//--------------------------------------------
// CLOCK common print routines
//--------------------------------------------
void Tekstprint(char tekst[])
{
 Serial.print(tekst);    
                          #ifdef BLUETOOTHMOD   
                          #if defined(ARDUINO_SAMD_MKRWIFI1010)
 Serial1.print(tekst);  
                          #else
 Bluetooth.print(tekst);  
                          #endif  
                          #endif BLUETOOTHMOD
}

void Tekstprintln(char tekst[])
{
 Serial.println(tekst);    
                          #ifdef BLUETOOTHMOD
                          #if defined(ARDUINO_SAMD_MKRWIFI1010)
 Serial1.println(tekst);
                          #else
 Bluetooth.println(tekst);  
                          #endif defined(ARDUINO_SAMD_MKRWIFI1010)   
                          #endif BLUETOOTHMOD 
}
//--------------------------------------------
// CLOCK Update routine done every second
//--------------------------------------------
void EverySecondCheck(void)
{
 if ( millis() - msTick == 50)   digitalWrite(secondsPin,LOW);     // Turn OFF the second on pin 13
 if ( millis() - msTick >999)                                      // Flash the onboard Pin 13 Led so we know something is happening
  {    
   msTick = millis();                                              // second++; 
   digitalWrite(secondsPin,HIGH);                                  // Turn ON the second on pin 13
   GetTijd(0);                                                     // Synchronize time with RTC clock
                            #ifdef MAX7219
   PrintTimetoMAX7219();
                            #endif MAX7219 
   if(Isecond % 30 == 0)  DimLeds(true);                           // Test LED intensity control + seconds tick print every 30 seconds   
   else                   DimLeds(TestLDR);                        // Every second an intensitiy check and update from LDR reading 
  if (FiboChrono)                                                  // If Fibonacci series display
  {
    if(Iminute == 0 && Isecond <9)
    { 
     ToggleEdsoft = Isecond % 2;                                   // ToggleEdsoft bocomes 0 or 1 and turn on and off the first seconds at minute 0 the Edsoft light on pin 24
     Serial.println(ToggleEdsoft);
     Displaytime();                                                // --- dit moet iets laten knipperen
    }
  }
  else if(Isecond % 5 == 0)  Displaytime();                        // Chrono clock display Display seconds every 5 seconds
  }
 }

//--------------------------------------------
// CLOCK Update routine done every minute
//--------------------------------------------
 void EveryMinuteUpdate(void)
 {
 if (Iminute != lastminute)                                         // Show time every minute
  { 
   lastminute = Iminute;
   GetTijd(0);
   Displaytime();
  } 
 if (Ihour != lasthour) EveryHourUpdate(); 
 }

//--------------------------------------------
// CLOCK Update routine done every hour
//--------------------------------------------
void EveryHourUpdate(void)
{
 GetTijd(0);                                                         // Sometimes one of the LEDs is white because of wrong Ihour?? remove if in middle ring white LEDs are at wrong hour
 if(Ihour == TurnOffLEDsAtHH) LEDsAreOff = true;                     // is it time to turn off the LEDs?
 if(Ihour == TurnOnLEDsAtHH)  LEDsAreOff = false;                    // or on?
 lasthour = constrain(Ihour,0,23);

}
//--------------------------------------------
// CLOCK Heart beat in LED
//--------------------------------------------
void Heartbeat() 
{
 static byte hbval         = 128;    // Heartbeat initial intensity
 static byte hbdelta       = 10;     // Determines how fast heartbeat is
 static uint32_t last_time = 0;  
 unsigned long now = millis();
 if ((now - last_time) < 40)    return;
 last_time = now;
 if (hbval > 230 || hbval < 20 ) hbdelta = -hbdelta; 
 hbval += hbdelta;
 analogWrite(HeartbeatLED, hbval);
}
//--------------------------------------------
// CLOCK check for serial input
//--------------------------------------------
void SerialCheck(void)
{
 String   SerialString;
 char c = 0;
 while (Serial.available() && c!=13)
  { 
   delay(3);
   c = Serial.read();
   if (c>31 && c<128) SerialString += c;                            // allow input from Space - Del
  }
 if (SerialString.length()>0)     ReworkInputString(SerialString);  // Rework ReworkInputString();
 SerialString = "";
                           #ifdef KEYPAD3x4   
 Keypad3x4Check(); 
                           #endif KEYPAD3x4
                           #ifdef ONEWIREKEYPAD   
 OnewireKeypadCheck(); 
                           #endif ONEWIREKEYPAD
                           #ifdef HC12MOD
  HC12Check();
                           #endif HC12MOD 
                           #ifdef BLUETOOTHMOD   
 BluetoothCheck(); 
                           #endif BLUETOOTHMOD  
} 
                           #ifdef HC12MOD
//--------------------------------------------
// CLOCK check for HC-12 input
//--------------------------------------------                            
void HC12Check(void)
{
 static bool DataInBuffer = false;
 String      HC12String; 
 static long HC12StartTime = millis(); 
 char        c;
 HC12String.reserve(64);
 HC12String ="";
 HC12.listen();                       // When using two software serial ports, you have to switch ports by listen()ing on each one in turn.
 while (HC12.available()>0)           // If HC-12 has data
   {       
    c = HC12.read();     
//    if (c>31 && c<128)  
    HC12String += c;                  // Allow only input from Space - Del
//    else c = 0;
    delay(3);
   }
 DataInBuffer = false;
 if (HC12String.length()>0) 
   {
    Serial.print("Received HC-12: "); Serial.println(HC12String);
//     ReworkInputString(HC12String);
    DataInBuffer = false;  
    }                  
}                         
                           #endif HC12MOD
                           #ifdef HC12MODx
 //--------------------------------------------
// CLOCK check for HC12 477 MHz transceiver input
//--------------------------------------------
void HC12Check(void) 
 {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '@';
  char endMarker = ']';
  char rc;
 
  while (HC12.available() > 0 && newData == false) 
    {
     rc = HC12.read();
     if (recvInProgress == true) 
        {
        if (rc != endMarker) 
           {
           receivedChars[ndx] = rc;
           ndx++;
           if (ndx >= numChars) { ndx = numChars - 1; }
           }
            else
            {
             receivedChars[ndx] = '\0'; // terminate the string
             recvInProgress = false;
             ndx = 0;
             newData = true;
             Serial.println(receivedChars);
             ReworkInputString(receivedChars); 
            }
        }
        else if (rc == startMarker) {recvInProgress = true;}
     }
}
                           #endif HC12MODx
                           #ifdef BLUETOOTHMOD
//--------------------------------------------
// CLOCK check for Bluetooth input
//--------------------------------------------                           
void BluetoothCheck(void)
{ 
 Bluetooth.listen();  //  When using two software serial ports, you have to switch ports by listen()ing on each one in turn.
 while (Bluetooth.available()) 
  {
   delay(3); 
                           #if defined(ARDUINO_SAMD_MKRWIFI1010)
   char c = Serial1.read();
                           #else if
   char c = Bluetooth.read();
                           #endif   
   Serial.print(c);
   if (c>31 && c<128) BluetoothString += c;
   else c = 0;     // delete a CR
  }
 if (BluetoothString.length()>0) ReworkInputString(BluetoothString); // Rework ReworkInputString();
 BluetoothString = "";
}
                           #endif BLUETOOTHMOD  
                           #ifdef ONEWIREKEYPAD
 //--------------------------------------------
// CLOCK check for Onewire Keypad input
//--------------------------------------------
void OnewireKeypadCheck(void)
{
 byte keyvalue;
 char Key;
 int sensorValue = analogRead(OneWirePin); // read the value from the sensor:
 switch(sensorValue)
  {
    case   0 ... 100:  keyvalue = 13; break;   // noise
    case 101 ... 132:  keyvalue = 12; Key = '*'; break;   // * 
    case 133 ... 154:  keyvalue =  0; Key = '0'; break;   // 0 
    case 155 ... 216:  keyvalue = 11; Key = '#'; break;   // # 
    case 217 ... 281:  keyvalue =  7; Key = '7'; break;   // 7 
    case 282 ... 318:  keyvalue =  4; Key = '4'; break;   // 4 
    case 319 ... 349:  keyvalue =  1; Key = '1'; break;   // 1 
    case 350 ... 390:  keyvalue =  8; Key = '8'; break;   // 8 
    case 391 ... 463:  keyvalue =  5; Key = '5'; break;   // 5 
    case 464 ... 519:  keyvalue =  2; Key = '2'; break;   // 2 
    case 520 ... 619:  keyvalue =  9; Key = '9'; break;   // 9 
    case 620 ... 848:  keyvalue =  6; Key = '6'; break;   // 6 
    case 849 ... 1023: keyvalue =  3; Key = '3'; break;   // 3
  }
 if(keyvalue<13) { Serial.println(Key); delay(300); }
  if (Key == 12)   // *                  // Pressing a * activates the keyboard input. 
   { 
    KeyInputactivated = true;
    KeyLooptime = millis();
    KeypadString ="";
    ColorLeds("",0,NUM_LEDS-1,0x00FF00);                            // Turn all LEDs green
    ShowLeds();                                                     // Push data in LED strip to commit the changes
    Serial.println(F("Key entry activated"));
   }
 if (KeyInputactivated && (Key>=0 && Key<10))
   {
    delay(20); 
    KeypadString += Key;                                            // Digit keys 0 - 9
    ColorLeds("",0,Key-48,0xFF0000);                                // Turn all LEDs red
    ShowLeds();                                                     // Push data in LED strip to commit the changes
    Serial.println(KeypadString);
   }
 if (KeypadString.length()>5)                                       // If six numbers are entered rework this to a time hhmmss
   {       
   if(KeypadString=="999999")
     { 
      KeypadString = "";   
      Reset();
      Serial.println(F("Clock settings resetted"));   
     }
    else 
     {      
      ReworkInputString(KeypadString);                              // Rework ReworkInputString();
      KeypadString = "";
      Serial.println(F("Time changed"));
     }    
   }
 if ( KeyInputactivated && ((millis() - KeyLooptime) > 30000) ) 
   {  
    KeyInputactivated = false;                                      // Stop data entry after 30 seconds. This avoids unintended entry 
    KeypadString ="";
    Serial.println(F("Keyboard entry stopped"));
  }
}
                           #endif ONEWIREKEYPAD
                           #ifdef KEYPAD3x4
 //--------------------------------------------
// CLOCK check for Keypad input
//--------------------------------------------                           
void Keypad3x4Check(void)
{ 
  char Key = 0;
  Keypad3x4.tick(); 
  while(Keypad3x4.available())
   {
    keypadEvent e = Keypad3x4.read();  
    if(e.bit.EVENT == KEY_JUST_PRESSED)
     {
 //     Serial.println(" pressed");  
      delay(20);
      }  
    else  if(e.bit.EVENT == KEY_JUST_RELEASED) 
     {
      Key = (char) e.bit.KEY;
//      Serial.print(Key);  Serial.println(" released");
     Keypad3x4.clear();
     delay(20);
    }
   }

 if (Key == 42)   // *                                              // Pressing * activates the keyboard input. 
   { 
    KeyInputactivated = true;
    KeyLooptime = millis();
    KeypadString = "";
    ColorLeds("",0,NUM_LEDS-1,0x00FF00);                            // Turn all LEDs green
    ShowLeds();                                                     // Push data in LED strip to commit the changes
    Serial.println(F("Key entry activated"));
   }
 if (KeyInputactivated && (Key>47 && Key<58))
   {
    delay(20); 
    KeypadString += Key;                                            // Digit keys 0 - 9
    ColorLeds("",0,Key-48,0xFF0000);                                // Turn all LEDs red
    ShowLeds();                                                     // Push data in LED strip to commit the changes
    Serial.println(KeypadString);
   }
 if(KeypadString.length()>5)                                       // if six numbers are entered rework this to a time hhmmss
   {  
   if(KeypadString=="999999")
     { 
      KeypadString = "";   
      Reset();
      Serial.println(F("Clock setting resetted"));   
     }
    else 
     {      
      ReworkInputString(KeypadString);                              // Rework ReworkInputString();
      KeypadString = "";
      Serial.println(F("Time changed"));
     }
   }
  if (Key == 35)   // #                                             // Pressing # changes palettes. 
   { 
    KeypadString ="";
    DisplayPalette++;
    DisplayPalette = constrain(DisplayPalette,0,9);
    Displaytime();
   }
   
 if ( KeyInputactivated && ((millis() - KeyLooptime) > 30000) )     // Stop keyboard entry after 30 seconds
   {  
    KeyInputactivated = false;                                      // Stop data entry after 30 seconds. This avoids unintended entry 
    KeypadString ="";
    Serial.println(F("Keyboard entry stopped"));
  }
} 
                           #endif KEYPAD3x4                           
//------------------------ KY-040 rotary encoder ------------------------- 
//--------------------------------------------
// KY-040 ROTARY check if the rotary is moving
//--------------------------------------------
                           #ifdef ROTARYMOD
void RotaryEncoderCheck(void)
{
 long encoderPos = myEnc.read();
 if ( (unsigned long) (millis() - RotaryPressTimer) > 60000)        // 60 sec after shaft is pressed time of light intensity can not be changed 
   {
    if (ChangeTime || ChangeLightIntensity)                         
      {
        Tekstprintln("<-- Changing time is over -->");
        NoofRotaryPressed = 0;
      }
    ChangeTime            = false;
    ChangeLightIntensity  = false;
   }  
 if (ChangeTime || ChangeLightIntensity)                            // If shaft is pressed time of light intensity can be changed
   {
    if ( encoderPos && ( (millis() - Looptime) > 250))              // If rotary turned avoid debounce within 0.25 sec
     {   
     Serial.print(F("----> Index:"));   Serial.println(encoderPos);
     if (encoderPos >0)                                             // Increase  MINUTES of light intensity
       {     
        if (ChangeLightIntensity)  { WriteLightReducer(5); }        // If time < 60 sec then adjust light intensity factor
        if (ChangeTime) 
          {
           if (NoofRotaryPressed == 1)                              // Change hours
              {
               if( ++Ihour >23) { Ihour = 0; }
              }      
           if (NoofRotaryPressed == 2)                              // Change minutes
              { 
               Isecond = 0;
               if( ++Iminute >59) { Iminute = 0; if( ++Ihour >23) { Ihour = 0; } }   
              }
           } 
        }    
      if (encoderPos <0)                                            // Increase the HOURS
       {
       if (ChangeLightIntensity)   { WriteLightReducer(-5); }       // If time < 60 sec then adjust light intensity factor
       if (ChangeTime)     
          {
           if (NoofRotaryPressed == 1)                              // Change hours
            {
             if( Ihour-- ==0) { Ihour = 23; }
            }      
           if (NoofRotaryPressed == 2)                              // Change minutes
            { 
             Isecond = 0;
             if( Iminute-- == 0) { Iminute = 59; if( Ihour-- == 0) { Ihour = 23; } }   
            }
          }          
        } 
      SetRTCTime();  
      Print_RTC_tijd();
      myEnc.write(0);                                               // Set encoder pos back to 0
      Looptime = millis();       
     }                                                
   }
 if (digitalRead(clearButton) == LOW )                              // Set the time by pressing rotary button
   { 
    delay(200);
    ChangeTime            = false;
    ChangeLightIntensity  = false;
    RotaryPressTimer      = millis();                               // Record the time the shaft was pressed.
    if(++NoofRotaryPressed >10 ) NoofRotaryPressed = 0;
    switch (NoofRotaryPressed)                                      // No of times the rotary is pressed
      {
       case 1:  ChangeTime = true; 
                ColorLeds("",0,NUM_LEDS-1,red);
                ShowLeds(); 
                delay(1000);                           
                                                       break; // Change the hours RED 
       case 2:  ChangeTime = true;
                ColorLeds("",0,NUM_LEDS-1,yellow);
                ShowLeds(); 
                delay(1000);
                                                       break; // Change the minutes YELLOW        
       case 3:  ChangeLightIntensity = true;
                ColorLeds("",0,NUM_LEDS-1,white);
                ShowLeds();
                delay(1000);
                                                       break; // Change intensity  
       case 4:  DisplayPalette = 1;                    break;
       case 5:  DisplayPalette = 2;                    break;        
       case 6:  DisplayPalette = 3;                    break;
       case 7:  DisplayPalette = 4;                    break;
       case 8:  DisplayPalette = 5;                    break;
       case 9:  DisplayPalette = 6;                    break;    
       case 10: DisplayPalette = 7;                    break;
       case 11: DisplayPalette = 8;                    break;
       case 12: DisplayPalette = 9;                    break;
       case 13: DisplayPalette = 0;                    break;
       case 14:
       case 15:
       case 16:
       case 17:
       case 18:
       case 19:                                       break;                     
       default: NoofRotaryPressed = 0;     Reset();   break;                         
      }
    Serial.print(F("NoofRotaryPressed: "));   Serial.println(NoofRotaryPressed);   
    myEnc.write(0);
    WritePalette(DisplayPalette);
    Looptime = millis();     
    Displaytime(); 
                           #ifdef MAX7219
    PrintTimetoMAX7219();
                           #endif MAX7219  
   }
  myEnc.write(0);
 }
                           #endif ROTARYMOD
//--------------------------------------------
// CLOCK Self test sequence
//--------------------------------------------
void Selftest(void)
{
  Play_Lights();     
}
// -------------------------- End Selftest

//--------------------------------------------
// FIBONACCI Set the color and strip based on the time
//--------------------------------------------
void LEDsetTime(byte hours, byte minutes,byte seconds)
{ 
 hours %=12;                                                       // keep the hours between 0 and 12
 for(int i=0; i<CLOCK_PIXELS; i++) { bits[i] = 0; BitSet[i] = 0; } // clear all bits  
 MakeFibonacciList(hours);
 for(int i=0; i<CLOCK_PIXELS; i++) if(BitSet[i]) bits[i] +=1;      // if hour must be lit add 1
 for(int i=0; i<CLOCK_PIXELS; i++)    BitSet[i] = 0;               // clear  bits  
 MakeFibonacciList(minutes/5);                                     // block is 5 minutes  
  for(int i=0; i<CLOCK_PIXELS; i++)  
 {
   if( BitSet[i]) bits[i] +=2;    
   setPixel(i, colors[DisplayPalette][bits[i]]);  
// Serial.println(colors[DisplayPalette][bits[i]],HEX);
  }
 MakeFibonacciList(seconds/5);                                     // block is 5 minutes           
 for(int i=0; i<CLOCK_PIXELS; i++)  
 {
   if(BitSet[i] && NormalExtremeUltimate==1) 
          setPixel(i, colors[DisplayPalette][5]);                    // if seconds use color [5]
   if(BitSet[i] && NormalExtremeUltimate==2) 
          setPixel(i, (colors[DisplayPalette][5] / 2 + colors[DisplayPalette][5] / 2) );                    // if seconds use average (hour/min colour + second color) / 2
  }

}

//--------------------------------------------
// FIBONACCI Calculate the proper Fibonacci-numbers (Pixels)
//--------------------------------------------
void MakeFibonacciList(byte Value)
{
 byte CalcValue = 0;
 byte pos = CLOCK_PIXELS;
 while (Value != CalcValue  )
  {
   byte Fibonaccireeks[] = {1,1,2,3,5,0};             // set up Fibonacci array with 6 numbers.
   for(int i=0; i<=CLOCK_PIXELS; i++) BitSet[i] = 0;  // Clear all bits. NB CLOCK_PIXELS is noof cells / strips in the clock
   CalcValue = 0;
   while ( (Value != CalcValue) &&  CalcValue <=  Value)   
    {
     do { pos = random(CLOCK_PIXELS); } while(Fibonaccireeks[pos] == 0 );   
     CalcValue += Fibonaccireeks[pos];
     BitSet[pos] = 1;                                 // Set pos in array for valid number    
     Fibonaccireeks[pos] = 0;                         // Number taken from array 
    }
  }
}

//--------------------------------------------
// FIBONACCI Calculate the proper chronological numbers (Pixels)
//--------------------------------------------
void MakeChronoList(byte hours, byte minutes,byte seconds)
{
 hours %=12;                                                       // keep the hours between 0 and 12
 byte Secsegment = seconds / 5;
 byte Minsegment = minutes / 5;
 byte Bit;
 for(int i=0; i<12; i++)
  {
   Bit = 0;
   if(i < hours)        Bit+= 1;                                  // If hours use the second colour
   if(i < Minsegment)   Bit+= 2;                                  // If minute use the third colour. If hours was set the fourth colour is displayed 
   if(i == Secsegment)  Bit = 4;                                  // is second use the fifth colour to display     
   ColorLed(i,colors[DisplayPalette][Bit]); 
  }

}
//--------------------------------------------
// FIBONACCI Turn on the right pixels and colours for 24 hour 
//--------------------------------------------
void setPixel(byte pixel, uint32_t kleur)
{
 
 switch(pixel)  
  {
   case 0:      ColorLeds("", 0, 0,kleur); break;
   case 1:      ColorLeds("", 1, 1,kleur); break;
   case 2:      ColorLeds("", 2, 3,kleur); break;
   case 3:      ColorLeds("", 4, 6,kleur); break;
   case 4:      ColorLeds("", 7,11,kleur); break;
  }
 
/*  switch(pixel)   // for 32 LEDs, 4 strips of 8 LEDs
   {
    case 0:      ColorLeds("", 2, 3,kleur);                            break;
    case 1:      ColorLeds("",12,13,kleur);                            break;
    case 2:      ColorLeds("", 0, 1,kleur); ColorLeds("",14,15,kleur); break;
    case 3:      ColorLeds("",16,19,kleur); ColorLeds("",28,31,kleur); break;
    case 4:      ColorLeds("", 4,11,kleur); ColorLeds("",20,27,kleur); break;
   }

  switch(pixel)  
  {

 // for case clock  
  case 0:      ColorLeds("", 0, 0,kleur); break;
  case 1:      ColorLeds("", 1, 1,kleur); break;
  case 2:      ColorLeds("", 2, 3,kleur); break;
  case 3:      ColorLeds("", 4, 7,kleur); break;
  case 4:      ColorLeds("", 8,13,kleur); break;
 
 }
*/
}

//--------------------------- Time functions --------------------------
//--------------------------------------------
// CLOCK Self test sequence
//--------------------------------------------
void Displaytime(void)
{     
  if (FiboChrono) LEDsetTime(Ihour , Iminute, Isecond);              // Fibonacci series display   
  else            MakeChronoList(Ihour , Iminute, Isecond);          // Chrono display
}

//--------------------------------------------
// DS3231 Get time from DS3231
//--------------------------------------------
void GetTijd(byte printit)
{
 Inow =    RTCklok.now();
 Ihour =   Inow.hour();
 Iminute = Inow.minute();
 Isecond = Inow.second();
// if (Ihour > 24) { Ihour = random(12)+1; Iminute = random(60)+1; Isecond = 30;}  // set a time if time module is absent or defect
 if (printit)  Print_RTC_tijd(); 
}

//--------------------------------------------
// DS3231 utility function prints time to serial
//--------------------------------------------
void Print_RTC_tijd(void)
{
 sprintf(sptext,"%0.2d:%0.2d:%0.2d %0.2d-%0.2d-%0.4d",Inow.hour(),Inow.minute(),Inow.second(),Inow.day(),Inow.month(),Inow.year());
 Tekstprintln(sptext);
}
//--------------------------------------------
// CLOCK utility function prints time to serial
//--------------------------------------------
void Print_tijd(void)
{
 sprintf(sptext,"%0.2d:%0.2d:%0.2d",Ihour,Iminute,Isecond);
 Tekstprintln(sptext);
}
                     
//--------------------------------------------
// DS3231 Set time in module and print it
//--------------------------------------------
void SetRTCTime(void)
{ 
 Ihour   = constrain(Ihour  , 0,24);
 Iminute = constrain(Iminute, 0,59); 
 Isecond = constrain(Isecond, 0,59); 
 RTCklok.adjust(DateTime(Inow.year(), Inow.month(), Inow.day(), Ihour, Iminute, Isecond));
 GetTijd(1);                                     // Synchronize time with RTC clock
 Displaytime();
// Print_tijd();
}
//--------------------------------------------
// DS3231 Get temperature from module
//--------------------------------------------
int get3231Temp(void)
{
 byte tMSB, tLSB;
 int  temp3231;
 
  Wire.beginTransmission(DS3231_I2C_ADDRESS);    // Temp registers (11h-12h) get updated automatically every 64s
  Wire.write(0x11);
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 2);
 
  if(Wire.available()) 
  {
    tMSB = Wire.read();                          // 2's complement int portion
    tLSB = Wire.read();                          // fraction portion 
    temp3231 = (tMSB & B01111111);               // do 2's math on Tmsb
    temp3231 += ( (tLSB >> 6) * 0.25 ) + 0.5;    // only care about bits 7 & 8 and add 0.5 to round off to integer   
  }
  else   {temp3231 = -273; }  
  return (temp3231);
}

                     #ifdef MAX7219
//--------------------------------------------
//  MAX7219 Print time to MAX7219
//--------------------------------------------
void PrintTimetoMAX7219(void)
{
 sprintf(sptext,"%0.2d_%0.2d_%0.2d",Ihour,Iminute,Isecond);
 PrintStringToSegDisplay(sptext); 
}
//--------------------------------------------
//  MAX7219 Print in MAX7219 with 16 digits
//--------------------------------------------
void PrintStringToSegDisplay(char *text)
{
if(LEDsAreOff) {lc.shutdown(0,false); lc.clearDisplay(0); return; }
if(TurnOffLEDsAtHH!=TurnOnLEDsAtHH)
    if(Ihour >= TurnOffLEDsAtHH && Ihour<24)
      if(Ihour>=0 && Ihour<TurnOnLEDsAtHH) 
               {lc.shutdown(0,false); lc.clearDisplay(0); return;  }  
 for (int n = 0; n<16;n++)
    {
     int d = text[n] - '0';  
     lc.setDigit(n/8, 7-n%8, d ,false);
     }
}
//--------------------------------------------
//  MAX7219 Initialyse MAX7219 with two units
//--------------------------------------------
void InitialyseMAX7219(void)
{
 lc.shutdown(0,false); 
 lc.setIntensity(0,0);                                               // Set the brightness 0 - 15
 lc.clearDisplay(0);                                                 // and clear the display
// lc.shutdown(1,false);   
// lc.setIntensity(1,0);                                               // Set the brightness 0 - 15
// lc.clearDisplay(1);                                                 // and clear the display
}
                     #endif MAX7219 
// --------------------Light functions -----------------------------------
//--------------------------------------------
//  LED Set color for LED
//--------------------------------------------
void ColorLeds(char* Tekst, int FirstLed, int LastLed, uint32_t RGBWColor)
{   
 strip.fill(RGBWColor, FirstLed, ++LastLed - FirstLed );
 //for (int n = FirstLed; n <= LastLed; n++)  strip.setPixelColor(n,RGBWColor  );
 //  Serial.println(RGBWColor,HEX); 
 if (strlen(Tekst) > 0 ){sprintf(sptext,"%s ",Tekst); Tekstprint(sptext); }   // Print the Tekst  
}
//--------------------------------------------
//  LED Set color for one LED
//--------------------------------------------
void ColorLed(int Lednr, uint32_t RGBWColor)
{   
 strip.fill(RGBWColor, Lednr, 1 );
}
//--------------------------------------------
//  LED Push data in LED strip to commit the changes
//--------------------------------------------
void ShowLeds(void)
{
 strip.show();
}
//--------------------------------------------
//  LED Set brighness of LEDs
//--------------------------------------------  
void SetBrightnessLeds(byte Bright)
{
 strip.setBrightness(Bright); 
 ShowLeds();
}

//--------------------------------------------
//  LED function to make RGB color
//-------------------------------------------- 
uint32_t FuncCRGB(uint32_t Red, uint32_t Green, uint32_t Blue)
{
return (256*256 * Red + 256 * Green + Blue);
}

//--------------------------------------------
//  LED Dim the leds measured by the LDR and print values
//--------------------------------------------
void DimLeds(byte print) 
{                                                                                                       
  int LDR_read = (4 * Previous_LDR_read + analogRead(PhotoCellPin)) / 5;               // Read lightsensor 
  int Temp;
  Previous_LDR_read = LDR_read;
  OutPhotocell = (int)((LightReducer * sqrt(63.5*LDR_read))/100);                      // Linear --> hyperbolic with sqrt
  MinPhotocell = MinPhotocell > LDR_read ? LDR_read : MinPhotocell;
  MaxPhotocell = MaxPhotocell < LDR_read ? LDR_read : MaxPhotocell;
  byte BrightnessCalcFromLDR = (byte)constrain(OutPhotocell, LowerBrightness, 255);               // filter out of strange results 
  if(print)
  {
   sprintf(sptext,"Sensor:%3ld%",(long)(analogRead(PhotoCellPin))); Tekstprint(sptext);
   sprintf(sptext," Min:%3ld%",(long)(MinPhotocell));               Tekstprint(sptext);
   sprintf(sptext," Max:%3ld%",(long)(MaxPhotocell));               Tekstprint(sptext);
   sprintf(sptext," Out:%3ld",(long)(OutPhotocell));                Tekstprint(sptext);
   sprintf(sptext,"=%2ld%%",(long)(BrightnessCalcFromLDR/2.55));    Tekstprint(sptext);
   sprintf(sptext," Temp:%2ldC ",(long) get3231Temp());             Tekstprint(sptext);
   Print_tijd(); 
  }
 if(LEDsAreOff) BrightnessCalcFromLDR = 0;
 SetBrightnessLeds(BrightnessCalcFromLDR);  
}

//--------------------------------------------
//  LED Turn On en Off the LED's
//--------------------------------------------
void Play_Lights()
{
  for(int j=0; j<5; j++)  WhiteOverRainbow(50, 50, 1 );
}

//--------------------------------------------
//  LED In- or decrease light intensity value
//--------------------------------------------
void WriteLightReducer(float amount)
{
 LightReducer += amount; 
 WriteLightReducerEeprom(LightReducer);
}

//--------------------------------------------
//  LED Write light intensity to EEPROM
//--------------------------------------------
void WriteLightReducerEeprom(int value)
{
 LightReducer = constrain(value,1,255);;                     // Range between 1 and 255 (1 byte)
 EEPROM.write(0, LightReducer);                              // Store the value in permanent EEPROM memory at address 0
 sprintf(sptext,"Max brightness: %3d%%",LightReducer);
 Tekstprintln(sptext);
}

//--------------------------------------------
//  LED Write lowest allowable light intensity to EEPROM
//--------------------------------------------
void WriteLowerBrightness(int value)
{
 LowerBrightness = constrain(value,1,255);             // Range between 1 and 255 (1 byte)
 EEPROM.write(1, LowerBrightness);                                  // Default Lower Brightness for this clock
 sprintf(sptext,"Lower brightness: %3d bits",LowerBrightness);
 Tekstprintln(sptext);
}
//--------------------------------------------
//  LED Write Palette to EEPROM
//--------------------------------------------
void WritePalette(int value)
{
 EEPROM.write(2, value);                                  // Default Lower Brightness for this clock
 sprintf(sptext,"Palette:%d",value);
 Tekstprintln(sptext);
}
//--------------------------------------------
//  LED function to make RGBW color
//-------------------------------------------- 
uint32_t FuncCRGBW( uint32_t Red, uint32_t Green, uint32_t Blue, uint32_t White)
{ 
 return ( (White<<24) + (Red << 16) + (Green << 8) + Blue );
}
//--------------------------------------------
//  LED Wheel
//  Input a value 0 to 255 to get a color value.
//  The colours are a transition r - g - b - back to r.
//--------------------------------------------

uint32_t Wheel(byte WheelPos) 
{
 WheelPos = 255 - WheelPos;
 if(WheelPos < 85)   { return FuncCRGBW( 255 - WheelPos * 3, 0, WheelPos * 3, 0);  }
 if(WheelPos < 170)  { WheelPos -= 85;  return FuncCRGBW( 0,  WheelPos * 3, 255 - WheelPos * 3, 0); }
 WheelPos -= 170;      
 return FuncCRGBW(WheelPos * 3, 255 - WheelPos * 3, 0, 0);
}

//--------------------------------------------
//  LED RainbowCycle
//--------------------------------------------
// Slightly different, this makes the rainbow equally distributed throughout
void RainbowCycle(uint8_t wait) 
{
  uint16_t i, j;
  for(j=0; j<256 * 5; j++)          // 5 cycles of all colors on wheel
   { 
    for(i=0; i< NUM_LEDS; i++) 
     {
 //   strip.setPixelColor(i, Wheel(((i * 256 / NUM_LEDS) + j) & 255));
      ColorLeds("",i,i,Wheel(((i * 256 / NUM_LEDS) + j) & 255));
     }
   ShowLeds();
   delay(wait);
  }
}

//--------------------------------------------
//  LED WhiteOverRainbow
//--------------------------------------------
void WhiteOverRainbow(uint8_t wait, uint8_t whiteSpeed, uint8_t whiteLength ) 
{
  if(whiteLength >= NUM_LEDS) whiteLength = NUM_LEDS - 1;
  int head = whiteLength - 1;
  int tail = 0;
  int loops = 1;
  int loopNum = 0;
  static unsigned long lastTime = 0;
  while(true)
  {
    for(int j=0; j<256; j++) 
     {
      for(uint16_t i=0; i<NUM_LEDS; i++) 
       {
        if((i >= tail && i <= head) || (tail > head && i >= tail) || (tail > head && i <= head) )
              ColorLeds("",i,i,white);     // White
        else  ColorLeds("",i,i,Wheel(((i * 256 / NUM_LEDS) + j) & 255));
       }
      if(millis() - lastTime > whiteSpeed) 
       {
        head++;        tail++;
        if(head == NUM_LEDS) loopNum++;
        lastTime = millis();
      }
      if(loopNum == loops) return;
      head %= NUM_LEDS;
      tail %= NUM_LEDS;
      ShowLeds();
      delay(wait);
    }
  }  // end while
}
//--------------------------------------------
//  LED Write light intensity to EEPROM
//--------------------------------------------
void WriteLightReducerEeprom(byte waarde)
{
 LightReducer = constrain (waarde, 0 , 255);  // May not be larger than 255
 EEPROMwrite(0, LightReducer);              // Store the value (0-250) in permanent EEPROM memory at address 0
 sprintf(sptext,"Max brightness: %3d%%",LightReducer); 
 Tekstprintln(sptext);
}

//--------------------------------------------
//  Write to EEPROM
//--------------------------------------------
void EEPROMwrite(byte pos, byte waarde)
{ 
  EEPROM.write(pos, waarde);     
                     #if defined(ARDUINO_SAMD_MKRWIFI1010)
  EEPROM.commit();
                     #endif 
}
//--------------------------------------------
//  Read  EEPROM
//--------------------------------------------
byte EEPROMread(byte pos)  
{ 
  return EEPROM.read(pos);
}
//--------------------------------------------
//  LED Write lowest allowable light intensity to EEPROM
//--------------------------------------------
void WriteLowerBrightness(byte waarde)
{
 LowerBrightness = constrain (waarde, 0 , 150);                    // Range between 1 and 100
 EEPROMwrite(1, LowerBrightness);                                    // Default Lower Brightness for this clock
 sprintf(sptext,"Lower brightness: %3d bits",LowerBrightness);
 Tekstprintln(sptext);
}
// --------------------End Light functions 

//--------------------------------------------
//  CLOCK Input from Bluetooth or Serial
//--------------------------------------------
void ReworkInputString(String InputString)
{
 String temp;
 int ff;
 Serial.println(InputString);
 InputString.trim();
 if (InputString.length()>10) return;
 if (InputString[0] > 64 )
  {
  int val = InputString[0];
  switch (val)
   {
    case 'D':
    case 'd':  
            if (InputString.length() == 9 )
             {
              int Jaar;
              temp   = InputString.substring(1,3);     Iday = (byte) temp.toInt(); 
              temp   = InputString.substring(3,5);   Imonth = (byte) temp.toInt(); 
              temp   = InputString.substring(5,9);     Jaar =  temp.toInt(); 
              Iday   = constrain(Iday  , 0, 31);
              Imonth = constrain(Imonth, 0, 12); 
              Jaar   = constrain(Jaar , 1000, 9999); 
              RTCklok.adjust(DateTime(Jaar, Imonth, Iday, Inow.hour(), Inow.minute(), Inow.second()));
              sprintf(sptext,"%0.2d:%0.2d:%0.2d %0.2d-%0.2d-%0.4d",Inow.hour(),Inow.minute(),Inow.second(),Iday,Imonth,Jaar);
              Tekstprintln(sptext);
             }
             else Tekstprintln("**** Length fault. Enter ddmmyyyy ****");
            break;    
    case 'F':
    case 'f':
             if(InputString.length() == 1)
               {
                FiboChrono = !FiboChrono;
                EEPROMwrite(5, FiboChrono);   // Write the value in EEPROM
                sprintf(sptext,"Display is %s", FiboChrono?"Fibonacci":"Chrono" );
                Tekstprintln(sptext); 
               }
             break;
    case 'L':                                 // Lowest value for Brightness
    case 'l':    
             if (InputString.length() >4) break;
             temp = InputString.substring(1);
             ff = temp.toInt();
             WriteLowerBrightness(ff);
             break;                       
            
    case 'M':                                 // Factor ( 0% - 255%) to multiply brighness (0 - 255) with 
    case 'm': 
             if (InputString.length() >4) break; 
             temp = InputString.substring(1);
             ff = temp.toInt();            
             WriteLightReducerEeprom(ff);
             break;
    case 'N':
    case 'n':
             if (InputString.length() == 1 )
              {
                TurnOffLEDsAtHH = 0;
                TurnOnLEDsAtHH  = 0;
              }
             if (InputString.length() == 5 )
              {
               temp = InputString.substring(1,3);   TurnOffLEDsAtHH = (byte) temp.toInt(); 
               temp = InputString.substring(3,5);   TurnOnLEDsAtHH  = (byte) temp.toInt(); 
              }
             TurnOffLEDsAtHH = constrain(TurnOffLEDsAtHH, 0, 23);
             TurnOnLEDsAtHH  = constrain(TurnOnLEDsAtHH,  0, 23); 
             EEPROMwrite(3, TurnOffLEDsAtHH);                        // Write the value in EEPROM
             EEPROMwrite(4, TurnOnLEDsAtHH);                         // Write the value in EEPROM
             sprintf(sptext,"LEDs are OFF between %2d:00 and %2d:00", TurnOffLEDsAtHH,TurnOnLEDsAtHH );             
             Tekstprintln(sptext); 
             break;
    case 'O':
    case 'o':
             if(InputString.length() == 1)
               {
                LEDsAreOff = !LEDsAreOff;
                sprintf(sptext,"LEDs are %s", LEDsAreOff?"OFF":"ON" );
                Tekstprintln(sptext); 
               }
             break;
    case 'P':
    case 'p':
             if (InputString.length() != 2) break;
             temp = InputString.substring(1);
             DisplayPalette = temp.toInt();
             EEPROMwrite(2, DisplayPalette);
             sprintf(sptext,"Colour scheme change to palette: %d",DisplayPalette);  Tekstprintln(sptext);
             break;
    case 'I':
    case 'i':
             if (InputString.length() >1) break;   
             SWversion();
             break;    
    case 'R':
    case 'r':
             if (InputString.length() >1) break;   
             Reset();                                                // Reset to default values
             sprintf(sptext,"Reset performed");  Tekstprintln(sptext);
             break;
    case 'T':
    case 't':

             if(InputString.length() >= 7)  // T125500
              {
               temp = InputString.substring(1,3);   
               if(temp.toInt() <24) Ihour = temp.toInt();
               else break; 
               temp = InputString.substring(3,5);   
               if(temp.toInt() <60) Iminute = temp.toInt();
               else break;                
               temp = InputString.substring(5,7);   
               if(temp.toInt() <60) Isecond = temp.toInt();
               else break;
               SetRTCTime();
              }
              break;         
    case 'W':
    case 'w':
             if (InputString.length() >1) break;   
             TestLDR = 1 - TestLDR;                                   // If TestLDR = 1 LDR reading is printed every second instead every 30s
             sprintf(sptext,"TestLDR every second: %s",TestLDR? "On" : "Off");  Tekstprintln(sptext);
             break;
    case 'X':
    case 'x':    
            Demo = 1 - Demo;                                         // Toggle Demo mode
           // Play_Lights();
            GetTijd(0);  
            Displaytime();
            break;
    default:
            break;
   }
   Displaytime();
   InputString = "";
 }
 else if (InputString.length() == 6 )                                // For compatibility
 {
  temp = InputString.substring(0,2);   
  if(temp.toInt() <24) Ihour = temp.toInt(); 
  temp = InputString.substring(2,4);   
  if(temp.toInt() <60) Iminute = temp.toInt(); 
  temp = InputString.substring(4,6);   
  if(temp.toInt() <60) Isecond = temp.toInt(); 
  SetRTCTime();

 }
 InputString = "";
 temp = "";
}
