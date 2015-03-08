/*
 *  Remote Systems Control With Wireless Ad-Hoc Radio Network And RFID.
 *
 *  This sketch implements the firmware of the identification node.
 *
 *  Copyright (C) 2010  Efstathios Chatzikyriakidis (contact@efxa.org)
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *  In order to compile the sketch you will need to hack the Streaming library.
 *
 *  Find the Streaming.h header and replace:
 *
 *    enum _EndLineCode { endl };
 *
 *  With the following:
 *
 *    enum _EndLineCode { eol };
 *
 *  Note:
 *
 *    Need to do this replacement because there is conflict
 *    between the 'endl' of the SdFat lib and the 'endl' of
 *    the Streaming lib.
 */

/*
 * include external libraries headers.
 */

// library for creating software serial ports.
#include <SoftwareSerial.h>

// library for using LCDs with shift registers.
#include <ShiftRegLCD.h>

// library for using output streaming operators.
#include <Streaming.h>

// library for handling IR remote control sensors.
#include <IRremote.h>

// library for handling EEPROM memory of Arduino.
#include <EEPROM.h>

// library for performing non-local jumps.
#include <setjmp.h>

// library for using SD storage card devices.
#include <SdFat.h>

/*
 * include internal libraries headers.
 */

// library for generic handling of Arduino EEPROM.
#include "eeprom_generic.h"

/*
 * define preproccessor constants.
 */

// set true / false for debug support.
#define DEBUG_STATE true

// define controllers if debug state is on.
#if defined (DEBUG_STATE) && true == DEBUG_STATE
  // for hardware serial verbose printing.
  #define SERIAL_ENABLED
#endif

/*
 * define function-like macros.
 */

// macro for performing a "kernel" panic state and 
// printing an error from the Arduino flash memory.
#define error(s) systemError (PSTR (s))

/*
 * define Arduino I/O PINS constants.
 */

const byte LCD_SR_DTA_PIN = 7;  // the PIN number for the LCD's shift register data.
const byte LCD_SR_CLK_PIN = 9;  // the PIN number for the LCD's shift register clock.

const byte RFID_RLINE_PIN = 4;  // the PIN number for the RFID reader serial RX line.
const byte RFID_TLINE_PIN = 5;  // the PIN number for the RFID reader serial TX line (dumb).
const byte RMIR_RLINE_PIN = 6;  // the PIN number for the RMIR sensor RX line.

const byte LED_STATUS_PIN = 3;  // the PIN number for the external status led.
const byte BUT_STATES_PIN = 2;  // the PIN number for the external state button.

const byte DEFAULT_CS_PIN = 10; // the PIN number for the default hardware chip select.

const byte XBEE_TLINE_PIN = 5;  // the PIN number for the XBEE reader serial TX line.
const byte XBEE_RLINE_PIN = 8;  // the PIN number for the XBEE reader serial RX line.

/*
 * define Arduino I/O IRQ constants.
 */

// the IRQ number for the external state button.
const byte BUT_STATES_IRQ = 0;

/*
 * define Arduino I/O serial port constants.
 */

#if defined (SERIAL_ENABLED)
// hardware serial port baud rate.
const unsigned long USB_BAUD_RATE = 9600;
#endif

// RFID software serial port baud rate.
const unsigned long RFID_BAUD_RATE = 9600;

// XBEE software serial port baud rate.
const unsigned long XBEE_BAUD_RATE = 9600;

/*
 * define FSM-related variables.
 */

// define FSM state function type definition name.
typedef void (*FSMStateFunctionT) ();

// array of FSM states functions' pointers.
const FSMStateFunctionT FSMStatesFunctions[][2] = {
  { idleP          , idleV          }, // FSM state for doing nothing - idling.
  { RFIDHandleTagP , RFIDHandleTagV }, // FSM state for handling an RFID tag.
  { RFIDLearnTagP  , RFIDLearnTagV  }, // FSM state for learning an RFID tag.
  { RMIRLearnKeysP , RMIRLearnKeysV }  // FSM state for learning RMIR keys.
};

// calculate the number of total FSM states in the array.
const byte FSM_STATES_NUM =
  sizeof (FSMStatesFunctions) / (sizeof (const FSMStateFunctionT) * 2);

// current FSM state number (zero-first, index-based).
volatile byte FSMCurrentState = 0;

/*
 * define LCD-related variables.
 */

// number of total LCD lines.
const byte LCD_LINES_NUM = 2;

// object to control the LCD with a shift register.
ShiftRegLCD SYS_LCD (LCD_SR_DTA_PIN, LCD_SR_CLK_PIN, TWO_WIRE, LCD_LINES_NUM);

/*
 * define XBEE-related variables.
 */

// object to control the XBEE software serial port.
SoftwareSerial SYS_XBEE = SoftwareSerial (XBEE_RLINE_PIN, XBEE_TLINE_PIN);

// define XBEE data separator token.
const char XBEEDataSeparator = '!';

/*
 * define RFID-related variables.
 */

// object to control the RFID software serial port.
SoftwareSerial SYS_RFID = SoftwareSerial (RFID_RLINE_PIN, RFID_TLINE_PIN);

// RFID knowledge filename (8.3 filename format).
const char RFID_KNOWLEDGE_FILENAME[] = "rfids.txt";

// the code size (digits) of the RFID tags.
const byte RFID_CODE_SIZE = 10;

// temporary RFID code (+1 for string termination).
char RFIDCode[RFID_CODE_SIZE + 1] = { '\0' };

/*
 * define RMIR-related variables.
 */

// object to control the RMIR sensor.
IRrecv SYS_RMIR (RMIR_RLINE_PIN);

// object to handle decoded RMIR codes.
decode_results RMIRCodesDecoded;

// define bounce time for RMIR codes.
const word RMIR_CODE_DURATION = 1000;

// size of the RMIR codes arrays.
const byte RMIR_ARRAY_SIZE = 60;

// array for the current RMIR codes.
uint32_t RMIRCurrentCodes[RMIR_ARRAY_SIZE];

// array for the new RMIR codes.
uint32_t RMIRNewCodes[RMIR_ARRAY_SIZE];

// number of the real RMIR codes.
byte RMIRCodesNumber = 0;

// temporary RMIR code index.
int8_t RMIRCodeIndex = -1;

/*
 * define microSD-related variables.
 */

Sd2Card SDCard;    // object to control the microSD card of the microSD shield.
SdVolume SDVolume; // object to control the volume of the microSD card.
SdFile SDRoot;     // object to control the root directory of the microSD card.
SdFile RFIDFile;   // object to control the RFID knowledge file of the microSD card.

/*
 * define general variables.
 */

// information to restore calling environment.
jmp_buf bufEnv;

// define bounce duration time for buttons.
const word BUTTON_KEY_DURATION = 500;

// millis count to debounce pressed buttons.
volatile unsigned long buttonKeyBounceTime = 0;

/*
 * define ISR-related functions.
 */

// ISR function for the external state button IRQ.
void
stateButtonISR ()
{
  // get the time since board began running the current program.
  unsigned long time = millis ();

  // check both if there was an overflow in millis() (back to zero again)
  // and ignore presses intervals less than the bounce duration time (ms).

  if ((time < buttonKeyBounceTime) ||
     ((time - buttonKeyBounceTime) > BUTTON_KEY_DURATION)) {
    // change the current FSM state to the next one (cyclic behaviour).
    FSMCurrentState = ++FSMCurrentState % FSM_STATES_NUM;

    // flush any possible data coming from RFID reader.
    StreamFlush(SYS_RFID);

    // flush any possible data coming from RMIR sensor (hack).
    RMIRFlush ();

    // clear the LCD display.
    SYS_LCD.clear ();

    // print a message to LCD according to the FSM.
    FSMStatesFunctions[FSMCurrentState][1] ();

    // set whatever bounce time in ms is appropriate.
    buttonKeyBounceTime = millis ();

    // go to the main loop.
    longjmp (bufEnv, 0);
  }
}

// try to attach ISRs for external interrupts.
void
attachInterruptsISRs ()
{
  // attach ISR for the IRQ (for state button presses).
  attachInterrupt (BUT_STATES_IRQ, stateButtonISR, RISING);
}

// try to detach ISRs for external interrupts.
void
detachInterruptsISRs ()
{
  // detach ISR for the IRQ (for state button presses).
  detachInterrupt (BUT_STATES_IRQ);
}

/*
 * define RMIR-related functions.
 */

// try to check if an RMIR code exists.
int8_t
RMIRCodeExists ()
{
  // get the type of the RMIR code.
  int16_t cType = RMIRCodesDecoded.decode_type;

  // if the RMIR code type is not unknown.
  if (cType != UNKNOWN) {
    // get the value of the RMIR code.
    uint32_t cValue = RMIRCodesDecoded.value;

    // if the type of the RMIR code is RC5/6.
    if (cType == RC5 || cType == RC6) {
      // get the length (in bits) of the RMIR code.
      int16_t cLength = RMIRCodesDecoded.bits;

      // calculate the toggle value of the RMIR code.
      uint32_t tValue = cValue ^ (1 << (cLength - 1));

      // search the RMIR codes array.
      for (byte i = 0; i < RMIRCodesNumber; i++) {
        // if the RMIR code or the toggle value of it exists.
        if (RMIRCurrentCodes[i] == cValue ||
            RMIRCurrentCodes[i] == tValue)
          // return the index of it.
          return i;
      }
    }
    else
      // search the RMIR codes array.
      for (byte i = 0; i < RMIRCodesNumber; i++)
        // if the RMIR code exists.
        if (RMIRCurrentCodes[i] == cValue)
          // return the index of it.
          return i;
  }

  // RMIR code does not exist.
  return -1;
}

// try to get an RMIR code index.
int8_t
getRMIRCodeIndex ()
{
  // the index of the RMIR code.
  int8_t index;

  do {
    // until an RMIR code is received.
    while (!SYS_RMIR.decode (&RMIRCodesDecoded));

    // disable all external interrupts.
    detachInterruptsISRs ();

    // try to get the index of the RMIR code.
    index = RMIRCodeExists ();

    // now accept the next RMIR code.
    SYS_RMIR.resume ();

    // enable all external interrupts.
    attachInterruptsISRs ();
  }
  // while the RMIR code does not exist.
  while (index < 0);

  // return the index of the RMIR code.
  return index;
}

// try to debounce the RMIR sensor.
void
debounceRMIR ()
{
  // light the status led.
  digitalWrite (LED_STATUS_PIN, HIGH);

  // wait some time.
  delay (RMIR_CODE_DURATION);

  // dark the status led.
  digitalWrite (LED_STATUS_PIN, LOW);
}

// try to flush any possible data coming from RMIR sensor (hack).
void
RMIRFlush ()
{
  // init / start the RMIR sensor.
  SYS_RMIR.enableIRIn ();
}

// try to learn RMIR keys (printing LCD information).
void
RMIRLearnKeysV ()
{
  SYS_LCD << F ("3) Learning RMIR");
}

// try to learn RMIR keys.
void
RMIRLearnKeysP ()
{
  byte i = 0;           // RMIR codes array index.
  uint32_t oValue = 0;  // previous RMIR code read.
  bool learning = true; // learning process status.

  // while there are unlearned RMIR codes.
  while (i < RMIR_ARRAY_SIZE && true == learning) {
    // if RMIR code is received.
    if (SYS_RMIR.decode (&RMIRCodesDecoded)) {
      // disable all external interrupts.
      detachInterruptsISRs ();

      // get the type of the RMIR code.
      int16_t cType = RMIRCodesDecoded.decode_type;

      // if the RMIR code type is not unknown.
      if (cType != UNKNOWN) {
        // get the value of the RMIR code.
        uint32_t cValue = RMIRCodesDecoded.value;

        // if this is the first RMIR code.
        if (0 == i) {
          // print information to LCD & hardware serial.

          #if defined (SERIAL_ENABLED)
          Serial << F ("Learning Codes") << eol;
          #endif

          SYS_LCD.clear ();
          SYS_LCD << F (" Learning Codes ");

          // check if the first RMIR code is
          // the previous initialized value.
          if (cValue == oValue)
            // change the previous RMIR code value.
            oValue = ~cValue;
        }

        // if the type of the RMIR code is RC5/6.
        if (cType == RC5 || cType == RC6) {
          // get the length (in bits) of the RMIR code.
          int16_t cLength = RMIRCodesDecoded.bits;

          // toggle value of the previous RMIR code.
          uint32_t tValue = oValue ^ (1 << (cLength - 1));

          // if the current RMIR code is not simular to
          // the previous RMIR code (and toggle value).
          if (cValue != oValue && cValue != tValue) {
            // store the RMIR code.
            RMIRNewCodes[i++] = cValue;

            // refresh the previous RMIR code.
            oValue = cValue;

            // debounce RMIR sensor.
            debounceRMIR ();
          }
          else
            learning = false; // stop learning process.
        }
        else {
          // if the RMIR code is different from the previous RMIR code.
          if (cValue != oValue) {
            // store the RMIR code.
            RMIRNewCodes[i++] = cValue;

            // refresh the previous RMIR code.
            oValue = cValue;

            // debounce RMIR sensor.
            debounceRMIR ();
          }
          else
            learning = false; // stop learning process.
        }
      }

      // now accept the next RMIR code.
      SYS_RMIR.resume ();

      // enable all external interrupts.
      attachInterruptsISRs ();
    }
  }

  // disable all external interrupts.
  detachInterruptsISRs ();

  // print information to LCD & hardware serial.

  #if defined (SERIAL_ENABLED)
  Serial << F ("RMIR Codes Saved") << eol;
  #endif

  SYS_LCD.setCursor (0, 1);
  SYS_LCD << F ("RMIR Codes Saved");

  // store the number of RMIR codes read.
  RMIRCodesNumber = i;

  // save the RMIR codes to EEPROM of Arduino.
  saveRMIRCodes ();

  // load RMIR codes from EEPROM of Arduino.
  loadRMIRCodes ();

  // flush any possible data coming from RMIR sensor (hack).
  RMIRFlush ();

  // enable all external interrupts.
  attachInterruptsISRs ();
}

// load RMIR codes from EEPROM of Arduino.
void
loadRMIRCodes ()
{
  // read number of real RMIR codes from EEPROM.
  EEPROMGenericRead (0, RMIRCodesNumber);

  // read RMIR codes from EEPROM to current array.
  EEPROMGenericRead (1, RMIRCurrentCodes);
}

// save RMIR codes to EEPROM of Arduino.
void
saveRMIRCodes ()
{
  // write number of real RMIR codes to EEPROM.
  EEPROMGenericWrite (0, RMIRCodesNumber);

  // write RMIR codes from new array to EEPROM.
  EEPROMGenericWrite (1, RMIRNewCodes);
}

/*
 * define RFID-related functions.
 */

// try to handle / parse an RFID tag.
bool
RFIDTagHandled (bool verbose = true)
{
  byte value = 0;       // temporary data received from RFID reader.
  byte code[6];         // code + checksum data of RFID tag received.
  byte checksum = 0;    // checksum data of RFID tag received.
  byte bytesRead = 0;   // number of received data from RFID reader.
  byte tempByte = 0;    // temporary value used for checksum calculation.
  bool handled = false; // flag indicating if an RFID tag was handled.

  // if there are any data coming from the RFID reader.
  if (SYS_RFID.available () > 0) {
    // disable all external interrupts.
    detachInterruptsISRs ();

    // check for the STX header (0x02 ASCII value).
    if (0x02 == (value = SYS_RFID.read ())) {
      // read the RFID 10-digit code & the 2 digit checksum.
      while (bytesRead < (RFID_CODE_SIZE + 2)) {
        // if there are any data coming from the RFID reader.
        if (SYS_RFID.available () > 0) {
          // get a byte from the RFID reader.
          value = SYS_RFID.read ();

          // check for ETX | STX | CR | LF.
          if ((0x0D == value) ||
              (0x0A == value) ||
              (0x03 == value) ||
              (0x02 == value)) {
            // stop reading - there is an error.
            break;
          }

          // store the RFID code digits to an array.
          if (bytesRead < RFID_CODE_SIZE)
            RFIDCode[bytesRead] = value;

          // convert hex tag ID.
          if ((value >= '0') && (value <= '9'))
            value = value - '0';
          else if ((value >= 'A') && (value <= 'F'))
            value = 10 + value - 'A';

          // every two hex-digits, add byte to code.
          if (bytesRead & 1 == 1) {
            // make some space for this hex-digit by shifting
            // the previous hex-digit with 4 bits to the left.
            code[bytesRead >> 1] = (value | (tempByte << 4));

            if (bytesRead >> 1 != 5)
              // if we're at the checksum byte, calculate the checksum (XOR).
              checksum ^= code[bytesRead >> 1];
          }
          else
            tempByte = value;

          // ready to read next digit.
          bytesRead++;
        }
      }

      // handle the RFID 10-digit code & the 2 digit checksum data.
      if (bytesRead == (RFID_CODE_SIZE + 2)) {
        // if the caller wants output information.
        if (verbose) {
          // print information to LCD & hardware serial.

          #if defined (SERIAL_ENABLED)
          Serial << F ("RFID: ") << RFIDCode << F (", CSUM: ") << _HEX (code[5]) << (code[5] == checksum ? F (" (PASS)") : F (" (ERROR)")) << eol;
          #endif

          SYS_LCD.clear ();
          SYS_LCD << F ("RFID: ") << RFIDCode;
          SYS_LCD.setCursor (0, 1);
          SYS_LCD << F ("CSUM: ") << _HEX (code[5]) << (code[5] == checksum ? F (" (PASS)") : F (" (ERROR)"));
        }

        // check if the RFID code is correct.
        if (code[5] == checksum)
          // set that the tag was handled.
          handled = true;
      }
    }

    // enable all external interrupts.
    attachInterruptsISRs ();
  }

  return handled;
}

// try to learn an RFID tag (printing LCD information).
void
RFIDLearnTagV ()
{
  SYS_LCD << F ("2) Learning RFID");
}

// try to learn an RFID tag.
void
RFIDLearnTagP ()
{
  // if an RFID tag was handled.
  if (RFIDTagHandled ()) {
    // flush any possible data coming from RMIR sensor (hack).
    RMIRFlush ();

    // try to get an RMIR code index.
    RMIRCodeIndex = getRMIRCodeIndex ();

    // if the RFID code was stored.
    if (RFIDCodeStored ()) {
      // print information to LCD & hardware serial.

      #if defined (SERIAL_ENABLED)
      Serial << F ("RFID: ") << RFIDCode << F (", IDEV: ") << RMIRCodeIndex << eol;
      #endif

      SYS_LCD.clear ();
      SYS_LCD << F ("RFID: ") << RFIDCode;
      SYS_LCD.setCursor (0, 1);
      SYS_LCD << F ("IDEV: ") << RMIRCodeIndex;
    }

    // flush any possible data coming from RFID reader.
    StreamFlush(SYS_RFID);
  }
}

// try to store an RFID code to the knowledge file.
bool
RFIDCodeStored ()
{
  // disable all external interrupts.
  detachInterruptsISRs ();

  // flag indicating if the RFID code was stored.
  bool stored = false;

  // try to open the knowledge file for appending.
  if (!RFIDFile.open (&SDRoot, RFID_KNOWLEDGE_FILENAME, O_CREAT | O_APPEND | O_WRITE))
    error ("RFIDFile.open ()");
  else {
    // print information to hardware serial.

    #if defined (SERIAL_ENABLED)
    Serial << F ("Appending: ") << RFID_KNOWLEDGE_FILENAME << eol;
    #endif

    // clear write error.
    RFIDFile.writeError = false;

    // write data to the file.
    RFIDFile.print (RFIDCode);
    RFIDFile.print (F (" "));

    if (RMIRCodeIndex < 10)
     RFIDFile.print (F ("0"));

    RFIDFile.println (RMIRCodeIndex);

    // check if there was a write error.
    if (RFIDFile.writeError)
      error ("RFIDFile.write ()");

    // try to close the file.
    if (!RFIDFile.close ())
      error("RFIDFile.close ()");

    // set that the code was stored.
    stored = true;
  }

  // enable all external interrupts.
  attachInterruptsISRs ();

  return stored;
}

// try to get the device number of an RFID code.
bool
getRFIDCodeDevice (int8_t & iDev)
{
  // disable all external interrupts.
  detachInterruptsISRs ();

  const byte bufSize = 13; // the size of the data buffer.
  byte bytesRead = 0;      // the number of received data.

  // the buffer of received data (+1 for string termination).
  char bufData[bufSize + 1] = { '\0' };

  // assume that the RFID code does not exist.
  iDev = -1;

  // try to open the RFID knowledge file for reading.
  if (!RFIDFile.open (&SDRoot, RFID_KNOWLEDGE_FILENAME, O_READ))
    return false; // file cannot open or does not exist.
  else {
    // keep reading records from the RFID knowledge file.
    while (true) {
      // try to read a record from the file.
      bytesRead = RFIDFile.read (bufData, bufSize);

      // if we reached EOF break the loop.
      if (0 == bytesRead) break;

      // if there was a read error.
      if (bytesRead != bufSize)
        error ("RFIDFile.read ()");

      // check if the RFID code is the one in the current record.
      if (strncmp (RFIDCode, bufData, RFID_CODE_SIZE) == 0)
        // get the device number of the RFID code.
        iDev = atoi (bufData + RFID_CODE_SIZE + 1);

      // try to ignore `endl' characters.

      RFIDFile.read (); // '\r'.
      RFIDFile.read (); // '\n'.
    }

    // try to close the file.
    if (!RFIDFile.close ())
      error ("RFIDFile.close ()");
  }

  // enable all external interrupts.
  attachInterruptsISRs ();

  // the RFID knowledge file exists.
  return true;
}

// try to handle an RFID tag (printing LCD information).
void
RFIDHandleTagV ()
{
  SYS_LCD << F ("1) Handling RFID");
}

// try to handle an RFID tag.
void
RFIDHandleTagP ()
{
  // the device number of the RFID code.
  int8_t iDev;

  // if an RFID tag was handled (work silently).
  if (RFIDTagHandled (false)) {
    // clear the LCD display.
    SYS_LCD.clear ();

    if (getRFIDCodeDevice (iDev)) {
      // if the RFID code exists (checking the device number).
      if (iDev >= 0) {
        // print information to LCD & hardware serial.

        #if defined (SERIAL_ENABLED)
        Serial << F ("RFID: ") << RFIDCode << F (", IDEV: ") << iDev << eol;
        #endif

        SYS_LCD << F ("RFID: ") << RFIDCode;
        SYS_LCD.setCursor (0, 1);
        SYS_LCD << F ("IDEV: ") << iDev;

        // try to send the device number with
        // a separator to remote network node.
        SYS_XBEE << iDev << XBEEDataSeparator;
      }
      else {
        // print information to LCD & hardware serial.

        #if defined (SERIAL_ENABLED)
        Serial << F ("Wrong RFID Tag") << eol;
        #endif

        SYS_LCD << F (" Wrong RFID Tag ");
      }
    }
    else {
      // print information to LCD & hardware serial.

      #if defined (SERIAL_ENABLED)
      Serial << F ("No RFID File") << eol;
      #endif

      SYS_LCD << F ("  No RFID File  ");
    }
  }
}

// do nothing - idling function (printing LCD information).
void
idleV ()
{
  SYS_LCD << F ("Wi-Remote & RFID");
  SYS_LCD.setCursor (0, 1);
  SYS_LCD << F ("V1.0 (GNU / GPL)");
}

// do nothing - idling function.
void idleP () {}

/*
 * define utilities functions.
 */

// try to print to hardware serial a
// string from Arduino flash memory.
void
printFString (PGM_P str)
{
  #if defined (SERIAL_ENABLED)

  // read the characters of the string from flash memory.
  for (byte c; c = pgm_read_byte (str); str++)
    // print the current character to hardware serial.
    Serial.write(c);

  #endif
}

// try to perform a "kernel" panic where the system needs reset.
void
systemError (PGM_P str)
{
  // disable all external interrupts.
  detachInterruptsISRs ();

  // clear the LCD display.
  SYS_LCD.clear ();

  // print error message to LCD.
  SYS_LCD << F ("  System Error  ");

  #if defined (SERIAL_ENABLED)
  // print error message to hardware serial.
  Serial << F ("System Error: ");
  printFString (str);
  Serial << eol;

  // if there was an error with microSD.
  if (SDCard.errorCode ())
    // print microSD error message to hardware serial.
    Serial << F ("MicroSD Error: (") << _HEX (SDCard.errorCode ()) << F (", ") << _HEX (SDCard.errorData ()) << F (")") << eol;
  #endif

  // system needs hardware reset.
  while (true);

  // code never reaches here.
}

// stream flush function.
void
StreamFlush (Stream & stream)
{
  // perform old behaviour of flush (before Arduino 1.0).
  while (stream.available ()) {
    stream.read ();
  }
}

/*
 * define setup & loop functions.
 */

// startup point entry (runs once).
void
setup ()
{
  #if defined (SERIAL_ENABLED)
  // set hardware serial port data rate.
  Serial.begin (USB_BAUD_RATE);
  #endif

  // set RFID serial port data rate.
  SYS_RFID.begin (RFID_BAUD_RATE);

  // set XBEE serial port data rate.
  SYS_XBEE.begin (XBEE_BAUD_RATE);

  // set state button PIN as input.
  pinMode (BUT_STATES_PIN, INPUT);

  // set status led PIN as output.
  pinMode (LED_STATUS_PIN, OUTPUT);

  // set default chip select PIN as output.
  pinMode (DEFAULT_CS_PIN, OUTPUT);

  // start RMIR sensor.
  SYS_RMIR.enableIRIn ();

  // initialize the SD card at SPI_HALF_SPEED to avoid bus errors
  // with breadboards. use SPI_FULL_SPEED for better performance.
  if (!SDCard.init (SPI_HALF_SPEED))
    error ("SDCard.init ()");

  // initialize a FAT volume.
  if (!SDVolume.init (&SDCard))
    error ("SDVolume.init ()");

  // open the root directory.
  if (!SDRoot.openRoot (&SDVolume))
    error ("SDRoot.openRoot ()");

  // load RMIR codes from EEPROM of Arduino.
  loadRMIRCodes ();

  // display application information.
  idleV ();

  // enable all external ISR.
  attachInterruptsISRs ();
}

// loop the main sketch.
void
loop ()
{
  // save the environment of the calling function.
  setjmp (bufEnv);

  // perform a state action according to the FSM.
  FSMStatesFunctions[FSMCurrentState][0] ();
}
