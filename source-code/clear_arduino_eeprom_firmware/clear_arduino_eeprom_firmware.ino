/*
 *  Remote Systems Control With Wireless Ad-Hoc Radio Network And RFID.
 *
 *  This sketch clears the EEPROM memory of an Arduino.
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
 * include external libraries headers.
 */

// library for handling EEPROM memory of Arduino.
#include <EEPROM.h>

/*
 * define Arduino I/O PINS constants.
 */

// the PIN number for the status led.
const byte LED_STATUS_PIN = 13;

/*
 * define Arduino EEPROM constants.
 */

// the capacity (in bytes) of the EEPROM.
const word EEPROM_SIZE = 1024;

/*
 * define setup & loop functions.
 */

// startup point entry (runs once).
void
setup ()
{
  // set the status led PIN as output.
  pinMode (LED_STATUS_PIN, OUTPUT);

  // dark the status led.
  digitalWrite (LED_STATUS_PIN, LOW);

  // clear the EEPROM memory.
  for (word i = 0; i < EEPROM_SIZE; i++)
    EEPROM.write (i, 0);

  // light the status led.
  digitalWrite (LED_STATUS_PIN, HIGH);
}

// loop the main sketch.
void
loop ()
{
  // do nothing - idling function.
}
