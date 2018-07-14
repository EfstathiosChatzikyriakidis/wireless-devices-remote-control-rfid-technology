/*
 *  Remote Systems Control With Wireless Ad-Hoc Radio Network And RFID.
 *
 *  This sketch removes the RFID knowledge of the identification node.
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

// library for using output streaming operators.
#include <Streaming.h>

// library for using SD storage card devices.
#include <SdFat.h>

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

// the default hardware chip select PIN number.
const byte DEFAULT_CS_PIN = 10;

/*
 * define Arduino I/O serial port constants.
 */

#if defined (SERIAL_ENABLED)
// hardware serial port baud rate.
const unsigned long USB_BAUD_RATE = 9600;
#endif

/*
 * define microSD-related variables.
 */

Sd2Card SDCard;    // object to control the microSD card of the microSD shield.
SdVolume SDVolume; // object to control the volume of the microSD card.
SdFile SDRoot;     // object to control the root directory of the microSD card.
SdFile RFIDFile;   // object to control the RFID knowledge file of the microSD card.

/*
 * define RFID-related variables.
 */

// RFID knowledge filename (8.3 filename format).
const char RFID_KNOWLEDGE_FILENAME[] = "rfids.txt";

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
  for (uint8_t c; c = pgm_read_byte (str); str++)
    // print the current character to hardware serial.
    Serial.write(c);

  #endif
}

// try to perform a "kernel" panic where the system needs reset.
void
systemError (PGM_P str)
{
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

  // set the default chip select PIN as output.
  pinMode (DEFAULT_CS_PIN, OUTPUT);

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

  // open the RFID knowledge file.
  if (!RFIDFile.open (&SDRoot, RFID_KNOWLEDGE_FILENAME, O_WRITE))
    error ("RFIDFile.open ()");
 
  // remove the RFID knowledge file.
  if (!RFIDFile.remove ())
    error ("RFIDFile.remove ()");

  #if defined (SERIAL_ENABLED)
  // print remove information to hardware serial.
  Serial << F ("RFID knowledge removed.") << eol;
  #endif
}

// loop the main sketch.
void
loop ()
{
  // do nothing - idling function.
}
