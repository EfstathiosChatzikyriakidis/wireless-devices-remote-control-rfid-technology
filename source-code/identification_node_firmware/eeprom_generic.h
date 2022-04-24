/*
 *  Remote Systems Control With Wireless Ad-Hoc Radio Network And RFID.
 *
 *  This file implements generic tools for handling the Arduino EEPROM.
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

#if !defined (EEPROM_GENERIC_H)
#define EEPROM_GENERIC_H

/*
 * include external libraries headers.
 */

// include Arduino basic header.
#include <Arduino.h>

/*
 * define Arduino EEPROM read / write functions.
 */

// write to Arduino EEPROM a data structure.
template <class T> int
EEPROMGenericWrite (int addr, const T & value)
{
  const byte *p = (const byte *) (const void *) &value;

  int i;
  for (i = 0; i < sizeof (value); i++)
    EEPROM.write (addr++, *p++);

  return i;
}

// read from Arduino EEPROM a data structure.
template <class T> int
EEPROMGenericRead (int addr, T & value)
{
  byte *p = (byte *) (void *) &value;

  int i;
  for (i = 0; i < sizeof (value); i++)
    *p++ = EEPROM.read (addr++);

  return i;
}

#endif // EEPROM_GENERIC_H
