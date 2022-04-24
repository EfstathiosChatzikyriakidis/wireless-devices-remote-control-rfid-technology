/*
 *  Remote Systems Control With Wireless Ad-Hoc Radio Network And RFID.
 *
 *  This sketch implements the firmware of the control node.
 *
 *  Copyright (C) 2010  Efstathios Chatzikyriakidis (stathis.chatzikyriakidis@gmail.com)
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
 * include external libraries headers.
 */

// library for creating software serial ports.
#include <SoftwareSerial.h>

/*
 * define Arduino I/O PINS constants.
 */

const byte XBEE_TLINE_PIN = 2; // the PIN number for the XBEE reader serial TX line.
const byte XBEE_RLINE_PIN = 3; // the PIN number for the XBEE reader serial RX line.

// start Arduino I/O PINS (Board: Duemilanove).

// array for the digital I/O standard logic PINS.
const byte IODigitalLogicPINS[] = {
  4, 5, 6, 7, 8, 9, 10, 11, 12, 13 // ignore: 0/1 (Serial), 2/3 (XBEE).
};

// end Arduino I/O PINS (Board: Duemilanove).

// number of the digital I/O standard logic PINS.
const byte IO_DIGITAL_LOGIC_PINS_NUM =
  sizeof (IODigitalLogicPINS) / sizeof (const byte);

/*
 * define Arduino I/O serial port constants.
 */

// XBEE software serial port baud rate.
const unsigned long XBEE_BAUD_RATE = 9600;

/*
 * define XBEE-related variables.
 */

// object to control the XBEE software serial port.
SoftwareSerial SYS_XBEE = SoftwareSerial (XBEE_RLINE_PIN, XBEE_TLINE_PIN);

// define XBEE data separator token.
const char XBEEDataSeparator = '!';

/*
 * define XBEE-related functions.
 */

// try to handle / parse a decimal number from XBEE.
int8_t
getXBEEDecimalNumber () {
  int8_t value = -1; // the decimal number fetched from XBEE.
  char ch;           // the temporary character read from XBEE.

  // if there are any data coming from the XBEE.
  if (SYS_XBEE.available () > 0) {
    // keep reading characters until
    // the separator token is found.
    do {
      // fetch a new character from XBEE.
      ch = SYS_XBEE.read ();

      // if the character is a decimal digit.
      if (ch >= '0' && ch <= '9') {
        // if the first decimal digit is fetched now.
        if (value == -1) value = 0;

        // recalculate the value of the decimal number.
        value = value * 10 + ch - '0';
      }
    } while (ch != XBEEDataSeparator);
  }

  // return the decimal number.
  return value;
}

/*
 * define setup & loop functions.
 */

// startup point entry (runs once).
void
setup ()
{
  // set XBEE serial port data rate.
  SYS_XBEE.begin (XBEE_BAUD_RATE);

  // set digital standard logic PINS as output.
  for (byte p = 0; p < IO_DIGITAL_LOGIC_PINS_NUM; ++p)
    pinMode (IODigitalLogicPINS[p], OUTPUT);
}

// loop the main sketch.
void
loop ()
{
  // try to get a device number from the remote network node.
  int8_t idev = getXBEEDecimalNumber ();

  // if the device number is supported in the system.
  if (idev >= 0 && idev < IO_DIGITAL_LOGIC_PINS_NUM) {
    // try to get the value of the appropriate pin of the device.
    byte value = digitalRead (IODigitalLogicPINS[idev]);

    // write on the pin of the device the inversed value.
    digitalWrite (IODigitalLogicPINS[idev], !value);
  }
}
