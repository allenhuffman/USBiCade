/*-----------------------------------------------------------------------------

USB iCade Converter
by Allen C. Huffman (alsplace@pobox.com)

Monitor digital inputs, then emit a USB keyboard character mapped to an iCade
button depending on the pin status. The character will be the "hold" character
for pin connected (N.O. button push) and "release" character for pin
disconnected (N.O. button released). 

Pin 13 is reserved for blinking the onboard LED as a heartbeat "we are alive"
indicator.

This software was written to allow a Teensy 2.0 to interface between arcade
buttons and an iPad via USB and Camera Connector Kit.

iCade/iCade Core button layout and numbering:
 1   (5) (7) (9) (E1)
3+4
 2   (6) (8) (0) (E2)
 
iCade Jr./iCade Mobile button layout and numbering:


 1      (7)     front   (E1)   (E2)
3+4  (5)   (8)  butons: (9)    (0)
 2      (6)
 
 
iCade Jr./iCade Mini Button Layout:

 
2013-02-21 0.0 allenh - Initial version, based on my icade_teensy code.

-----------------------------------------------------------------------------*/
#define VERSION "0.0"
//#define LED_OFF

//#include <EEPROM.h>
//#include <avr/wdt.h>
// Header files, taken from USBHIDJoystick example.
#include <avr/pgmspace.h>

#include <avrpins.h>
#include <max3421e.h>
#include <usbhost.h>
#include <usb_ch9.h>
#include <Usb.h>
#include <usbhub.h>
#include <avr/pgmspace.h>
#include <address.h>
#include <hid.h>
#include <hiduniversal.h>

#include "hidjoystickrptparser.h"

#include <printhex.h>
#include <message.h>
#include <hexdump.h>
#include <parsetools.h>

// Define some C++ stuff.
USB                                             Usb;
USBHub                                          Hub(&Usb);
HIDUniversal                                    Hid(&Usb);
JoystickEvents                                  JoyEvents;
JoystickReportParser                            Joy(&JoyEvents);

/*
iCade keyboard mappings.
See developer doc at: http://www.ionaudio.com/products/details/icade

   WE     YT UF IM OG
AQ<-->DC
   XZ     HR JN KP LV
*/

/*
  The following values will be used as digital inputs
  for each specific iCade function.
*/
// We will be treating joystick as a 16-bit value.
// Extra Buttons:
#define SELECT_USB  (1<<8)  // Z1:0x0100 - Select
#define L3_USB      (1<<9)  // Z1:0x0200 - L3
#define R3_USB      (1<<10) // Z1:0x0400 - R3
#define START_USB   (1<<11) // Z1:0x0800 - Start
// Joystick:
#define UP_USB      (1<<12) // Z1:0x1000 - Up
#define RIGHT_USB   (1<<13) // Z1:0x2000 - Right
#define DOWN_USB    (1<<14) // Z1:0x4000 - Down
#define LEFT_USB    (1<<15) // Z1:0x8000 - Left
// Grey/Front Buttons:
#define BTN1_USB    (1<<0)  // Z2:0x0001 - L2
#define BTN2_USB    (1<<1)  // Z2:0x0002 - R2
#define BTN3_USB    (1<<2)  // Z2:0x0004 - L1
#define BTN4_USB    (1<<3)  // Z2:0x0008 - R1
// Primary Buttons:
#define BTN5_USB    (1<<4)  // Z2:0x0010 - "Triangle"
#define BTN6_USB    (1<<5)  // Z2:0x0020 - "Circle"
#define BTN7_USB    (1<<6)  // Z2:0x0040 - "X"
#define BTN8_USB    (1<<7)  // Z2:0x0080 - "Square"

// Redefine some buttons for Playstation type layout:
#define TRI_BTN     BTN5_USB 
#define CIR_BTN     BTN6_USB
#define X_BTN       BTN7_USB
#define SQR_BTN     BTN8_USB
#define L1_BTN      BTN3_USB
#define R1_BTN      BTN4_USB
#define L2_BTN      BTN1_USB
#define R2_BTN      BTN2_USB
/*
  The following keys are the iCade sequence (hold, release) for each
  function. Send "W" to indicate UP, and "E" when UP is released.
*/
#define UP_KEYS    "we" // 1
#define DOWN_KEYS  "xz" // 2
#define LEFT_KEYS  "aq" // 3
#define RIGHT_KEYS "dc" // 4

#define BTN5_KEYS  "yt" // 5
#define BTN6_KEYS  "hr" // 6
#define BTN7_KEYS  "uf" // 7
#define BTN8_KEYS  "jn" // 8

#define BTN9_KEYS  "im" // 9
#define BTN0_KEYS  "kp" // 0 (10?)
#define BTNE1_KEYS "og" // E1
#define BTNE2_KEYS "lv" // E2

#define USB_BTN_COUNT  12   // 12 buttons.

/*---------------------------------------------------------------------------*/

// The order in the following three arrays must match.

// Each of these items is a 16-bit value, where the bits represent the 12
// iCade buttons.

// iCade/iCade Code - joystick and 4x2 button layout.
// This mapping is for arcade fighter style controllers that have the 8
// buttons laid out like the original iCade unit, with two rows four
// buttons, with the left and right two being "grey" and the center 2x2
// being the colored buttons (like the Playtech Pro Arcade Fighting Stick).
//
// http://www.amazon.com/Playtech-Arcade-Fighting-Stick-Playstation-3/dp/B004DFDVWW/ref=pd_cp_vg_0
//
//  U   L1 Sq Tr R1  map   1  (5) (7) (9) (E1)
// L+R  L2 X  O  R2  to:  3+4 (6) (8) (0) (E2) 
//  D                      2 
unsigned int myPins[USB_BTN_COUNT] =
  {UP_USB, DOWN_USB, LEFT_USB, RIGHT_USB,
   /*5*/L1_BTN,  /*6*/L2_BTN,  /*7*/SQR_BTN, /*8*/X_BTN,
   /*9*/TRI_BTN, /*0*/CIR_BTN, /*E1*/R1_BTN, /*E2*/R2_BTN};

// iCade Jr. - joystick and 4 front buttons, and 4 rear panel buttons.
// iCade Mobile - joypad and 4 pad buttons, and 4 front buttons.
// These two are mapped the same way, with the pimary four buttons matching
// the left 2x2 buttons on the original iCade. The rear/front buttons
// are the right 2x2 buttons. This mapping is for a Playstation style
// gamepad:
//
//  U           Tr    map   1    (7)   (9)  (0)
// L+R  Sl St Sq  Ci  to:  3+4 (5) (8) (E1) (E2) 
//  D           X           2    (6)
#if PLAYSTATION_MAPPING
unsigned int myPins[USB_BTN_COUNT] =
  {UP_USB, DOWN_USB, LEFT_USB, RIGHT_USB,
  /*5*/SQR_BTN,   /*6*/X_BTN,    /*7*/TRI_BTN,   /*8*/CIR_BTN,
  /*9*/BTN5_USB,  /*0*/BTN6_USB, /*E1*/BTN7_USB, /*E2*/BTN8_USB};
#endif

// Each of these items is a two character string, with the first character
// being the iCade "button down" key, and the second being the "button up"
// key.
char iCadeKeymap[][USB_BTN_COUNT] =
  {UP_KEYS, DOWN_KEYS, LEFT_KEYS, RIGHT_KEYS,
  BTN5_KEYS, BTN6_KEYS, BTN7_KEYS, BTN8_KEYS,
  BTN9_KEYS, BTN0_KEYS, BTNE1_KEYS, BTNE2_KEYS};

// This is just used for printing out debug info to the console.
char iCadeDesc[][USB_BTN_COUNT] =
  {"Up", "Down", "Left", "Right",
  "Btn5", "Btn6", "Btn7", "Btn8",
  "Btn9", "Btn0", "BtnE1", "BtnE2"};

/* We want a very short debounce delay for an arcade controller. */
#define DEBOUNCE_MS    10 // 100ms (1/10th second)

#define LED_PIN        13    // 11 on Teensy 2.0
#define LEDBLINK_MS    1000

/*---------------------------------------------------------------------------*/

/* For I/O pin status and debounce. */
unsigned int  buttonStatus[USB_BTN_COUNT];         // Last set PIN mode.
unsigned long digitalDebounceTime[USB_BTN_COUNT];   // Debounce time.
//unsigned long digitalCounter[USB_BTN_COUNT];      // Times button pressed.
unsigned int  digitalDebounceRate = DEBOUNCE_MS; // Debounce rate.

/* For the blinking LED (heartbeat). */
unsigned int  ledStatus = LOW;             // Last set LED mode.
unsigned long ledBlinkTime = 0;            // LED blink time.
unsigned int  ledBlinkRate = LEDBLINK_MS;  // LED blink rate.

unsigned int pinsOn = 0;

/*---------------------------------------------------------------------------*/

void setup()
{
  // Just in case it was left on...
  //wdt_disable();

  // Initialize the serial port.
  Serial.begin(9600);

  // Docs say this isn't necessary for Uno.
  while(!Serial) { }

  showHeader();
  
  // Initialize watchdog timer for 2 seconds.
  //wdt_enable(WDTO_4S);

  // LOW POWER MODE!
  // Pins default to INPUT mode. To save power, turn them all to OUTPUT
  // initially, so only those being used will be turn on. See:
  // http://www.pjrc.com/teensy/low_power.html
  for (int thisPin=0; thisPin < USB_BTN_COUNT; thisPin++ )
  {
    pinMode(thisPin, OUTPUT);
  }

  // Disable Unused Peripherals
  ADCSRA = 0;

#ifdef DI
  // Initialize the pins and digitalPin array.
  for (int thisPin=0; thisPin < USB_BTN_COUNT; thisPin++ )
  {
    // Set pin to be digital input using pullup resistor.
    //pinMode(myPins[thisPin], INPUT_PULLUP);
    // Set the current initial pin status.
    buttonStatus[thisPin] = 0;
    // Clear debounce time.
    digitalDebounceTime[thisPin] = 0;
    //digitalCounter[thisPin] = 0;
  }
#endif

  // Set LED pin to output, since it has an LED we can use.
  pinMode(LED_PIN, OUTPUT);
  
  // USB initialization stuff.
  if (Usb.Init() == -1)
      Serial.println("OSC did not start.");
      
  delay( 200 );

  if (!Hid.SetReportParser(0, &Joy))
      ErrorMessage<uint8_t>(PSTR("SetReportParser"), 1  ); 

  Keyboard.begin();    

  Serial.println("Ready.");
} // End of setup()

/*---------------------------------------------------------------------------*/

void loop()
{
  // Tell the watchdog timer we are still alive.
  //wdt_reset();
  
  // Handle USB
  Usb.Task();

#ifndef LED_OFF
  // LED blinking heartbeat. Yes, we are alive.
  if ( (long)(millis()-ledBlinkTime) >= 0 )
  {
    // Toggle LED.
    if (ledStatus==LOW)  // If LED is LOW...
    {
      ledStatus = HIGH;  // ...make it HIGH.
    } else {
      ledStatus = LOW;   // ...else, make it LOW.
    }
    // Set LED pin status.
    if (pinsOn==0) digitalWrite(LED_PIN, ledStatus);
    // Reset "next time to toggle" time.
    ledBlinkTime = millis()+ledBlinkRate;
  }
#endif

  // Check for serial data.
  if (Serial.available() > 0) {
    // If data ready, read a byte.
    int incomingByte = Serial.read();
    // Parse the byte we read.
    switch(incomingByte)
    {
      case '?':
        showStatus();
        break;
      default:
        break;
    }
  }
} // End of loop();

void handleJoystick(unsigned int buttonMask)
{
  /*-------------------------------------------------------------------------*/
  // Loop through each Digital Input pin.
  for (int thisPin=0; thisPin < USB_BTN_COUNT; thisPin++ )
  {
    // Read the pin's current status.
    unsigned int status = (buttonMask & myPins[thisPin]);
    
    // In pin status has changed from our last toggle...
    if (status != buttonStatus[thisPin])
    {
      // Remember when it changed, starting debounce mode.
      // If not currently in debounce mode,
      if (digitalDebounceTime[thisPin]==0)
      {
        // Set when we can accept this as valid (debounce is considered
        // done if the time gets to this point with the status still the same).
        digitalDebounceTime[thisPin] = millis()+digitalDebounceRate;
      }

      // Check to see if we are in debounce detect mode.
      if (digitalDebounceTime[thisPin]>0)
      {
        // Yes we are. Have we delayed long enough yet?
        //if ( (long)(millis()-digitalDebounceTime[thisPin]) >= 0 )
        {
            // Yes, so consider it switched.
            // If pin is Active LOW,
            if (status!=0)
            {
              // Emit BUTTON PRESSED string.
              Serial.print(iCadeDesc[thisPin]);
              Serial.print(" pressed  (sending ");
              Serial.print(iCadeKeymap[thisPin][0]);
              Serial.println(" to iCade).");
              Keyboard.print(iCadeKeymap[thisPin][0]);
              //digitalCounter[thisPin]++;
              pinsOn++;
#ifndef LED_OFF
              digitalWrite(LED_PIN, HIGH);
#endif
            } else {
              // Emit BUTTON RELEASED string.
              Serial.print(iCadeDesc[thisPin]);
              Serial.print(" released (sending ");
              Serial.print(iCadeKeymap[thisPin][1]);
              Serial.println(" to iCade).");
              Keyboard.print(iCadeKeymap[thisPin][1]);
              if (pinsOn>0) pinsOn--;
              if (pinsOn==0) digitalWrite(LED_PIN, LOW);
            }
            // Remember current (last set) status for this pin.
            buttonStatus[thisPin] = status;
            // Reset debounce time (disable, not looking any more).
            digitalDebounceTime[thisPin] = 0;
        } // End of if ( (long)(millis()-digitalDebounceTime[thisPin]) >= 0 )
        
      } // End of if (digitalDebounceTime[thisPin]>0)
    }
    else // No change? Flag no change.
    {
      // If we were debouncing, we are no longer debouncing.
      digitalDebounceTime[thisPin] = 0;
    }
  } // End of (int thisPin=0; thisPin < USB_BTN_COUNT; thisPin++ )
}
/*---------------------------------------------------------------------------*/

void showHeader()
{
  int i;
  // Emit some startup stuff to the serial port.
  Serial.print("iCadeTeensy ");
  Serial.print(VERSION);
  Serial.println(" by Allen C. Huffman (alsplace@pobox.com)");
  Serial.print(USB_BTN_COUNT);
  Serial.print(" DI Pins (");
  for (i=0; i<USB_BTN_COUNT; i++)
  {
    Serial.print(myPins[i]);
    Serial.print("=");
    Serial.print(iCadeDesc[i]);
    Serial.print(" ");
  }
  Serial.print("), ");
  Serial.print(digitalDebounceRate);
  Serial.println("ms Debounce.");
}

/*---------------------------------------------------------------------------*/

void showStatus()
{
  showDigitalInputStatus();
}

/*---------------------------------------------------------------------------*/

void showDigitalInputStatus()
{
  Serial.print("DI: ");

  for (int thisPin=0; thisPin < USB_BTN_COUNT; thisPin++ )
  {
    // Read the pin's current status.
    Serial.print(iCadeDesc[thisPin]);
    Serial.print("=");
    Serial.print(digitalRead(myPins[thisPin]));
    Serial.print(" ");
    //Serial.print(" (");
    //Serial.print(digitalCounter[thisPin]);
    //Serial.print(") ");
  }
  Serial.println("");
}

/*---------------------------------------------------------------------------*/

// End of file.

