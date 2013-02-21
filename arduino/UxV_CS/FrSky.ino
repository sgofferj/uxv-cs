//
// Library for decoding FrSky telemetry packages
//
//      Copyright (c) 2013 Stefan Gofferje. All rights reserved.
//
//      This library is free software; you can redistribute it and/or
//      modify it under the terms of the GNU Lesser General Public
//      License as published by the Free Software Foundation; either
//      version 2.1 of the License, or (at your option) any later
//      version.
//
//      This library is distributed in the hope that it will be
//      useful, but WITHOUT ANY WARRANTY; without even the implied
//      warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//      PURPOSE.  See the GNU Lesser General Public License for more
//      details.
//
//      You should have received a copy of the GNU Lesser General
//      Public License along with this library; if not, write to the
//      Free Software Foundation, Inc., 51 Franklin St, Fifth Floor,
//      Boston, MA 02110-1301 USA
//

#define FRSKY_MSG_LENGTH 9                // Frame length without leading and trailing 0x7E
#define FRSKY_MSG_ID_VOLTAGE_LINK 0xFE    // A1 and A2 voltage, up- and downlink RSSI

void FRSKY_handle_message(uint8_t* msg) {
  switch (msg[0]) {
    case FRSKY_MSG_ID_VOLTAGE_LINK:
    {
      frsky_rx_a1 = msg[1];
      frsky_rx_a2 = msg[2];
      frsky_link_up = msg[3];
      frsky_link_dn = msg[4];
    } 
  }
}

uint8_t FRSKY_parse_char(uint8_t c,uint8_t* msg) {
  static uint8_t bufferIndex; 
  static boolean msgBegin;
  uint8_t result = 0;
  if (c == 0x7E)  {
    if (bufferIndex == FRSKY_MSG_LENGTH) {
      bufferIndex = 0;
      result=1;
      msgBegin=false;
    }
    else if ( (bufferIndex > 0) && (bufferIndex < FRSKY_MSG_LENGTH) ) {
      bufferIndex = 0;
      msgBegin=false;
    }
    else {
      msgBegin = true;
    }
  }
  if ( (msgBegin) && (c != 0x7E) ) {
    msg[bufferIndex] = c;
    bufferIndex++;
  }
  return result;  
}
