//
// Library for controlling Electronic Assembly eDIPTFT displays
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

#include <Arduino.h>
#include <EDIPTFT.h>

EDIPTFT::EDIPTFT(int port, int smallprotocol) {
  _port = port;
  _smallprotocol = smallprotocol;
}

void EDIPTFT::sendData(char* data, int len) {
  if (_smallprotocol > 0) {
  }
  else {
    unsigned char i;
    for(i=0; i < len; i++) {
      switch (_port) {
	case 0 : {
	  Serial.write(data[i]);
	  break;
	}
	case 1 : {
	  Serial1.write(data[i]);
	  break;
	}
	case 2 : {
	  Serial2.write(data[i]);
	  break;
	}
	case 3 : {
	  Serial3.write(data[i]);
	  break;
	}
      }
    }
  }
}

void EDIPTFT::sendSmall(char* data, int len) {
  unsigned char i, bcc;
  Serial3.write(0x11);
  bcc = 0x11;
  Serial3.write(len);
  bcc = bcc + len;
  for(i=0; i < len; i++) {
    Serial3.write(data[i]);
    bcc = bcc + data[i];
  }
  Serial3.write(bcc);
}

void EDIPTFT::clear() {
  char command [] = {12};
  sendData(command,1);
}

void EDIPTFT::invert() {
  char command [] = {
    27,'D','I'
  };
  sendData(command,3);
}

void EDIPTFT::setDisplayColor(char fg, char bg) {
  char command [] = {
    27,'F','D',fg,bg
  };
  sendData(command,5);
}

void EDIPTFT::fillDisplayColor(char bg) {
  char command [] = {
    27,'D','F',bg
  };
  sendData(command,4);
}

void EDIPTFT::terminalOn(boolean on) {
  if (on) {
    char command [] = {27,'T','E'};
    sendData(command,3);
  }
  else {
    char command [] = {27,'T','A'};
    sendData(command,3);
  }
}

void EDIPTFT::cursor(boolean on) {
  if (on) {
    char command [] = {27,'T','C',1};
    sendData(command,4);
  }
  else {
    char command [] = {27,'T','C',0};
    sendData(command,4);
  }
}

void EDIPTFT::setCursor(char col, char row) {
  char command [] = {27,'T','P',col,row};
  sendData(command,5);
}

void EDIPTFT::defineBargraph(char dir, char no, int x1, int y1, int x2, int y2, byte sv, byte ev, char type) {
  char command [] = {
    27,'B',dir,no,
    lowByte(x1),highByte(x1),lowByte(y1),highByte(y1),
    lowByte(x2),highByte(x2),lowByte(y2),highByte(y2),
    char(sv),
    char(ev),
    type
  };
  sendData(command,15);
}

void EDIPTFT::updateBargraph(char no, char val) {
  char command [] = {
    27,'B','A',no,val
  };
  sendData(command,5);
}

void EDIPTFT::setBargraphColor(char no, char fg, char bg, char fr) {
  char command [] = {
    27,'F','B',no,fg,bg,fr
  };
  sendData(command,7);
}

void EDIPTFT::linkBargraphLight(char no) {
  char command [] = {
    27,'Y','B',no
  };
  sendData(command,4);
}  

void EDIPTFT::makeBargraphTouch(char no) {
  char command [] = {
    27,'A','B',no
  };
  sendData(command,4);
}  

void EDIPTFT::deleteBargraph(char no,char n1) {
  char command [] = {
    27,'B','D',no,n1
  };
  sendData(command,5);
}

void EDIPTFT::setLineColor(char fg, char bg) {
  char command [] = {
    27,'F','G',fg,bg
  };
  sendData(command,5);
}

void EDIPTFT::setTextColor(char fg, char bg) {
  char command [] = {
    27,'F','Z',fg,bg
  };
  sendData(command,5);
}

void EDIPTFT::setTextFont(char font) {
  char command [] = {
    27,'Z','F',font
  };
  sendData(command,4);
}

void EDIPTFT::setTextAngle(char angle) {
  // 0 = 0°, 1 = 90°, 2 = 180°, 3 = 270°
  char command [] = {
    27,'Z','W',angle
  };
  sendData(command,4);
}

void EDIPTFT::drawText(int x1, int y1, char justification,String text) {
  byte len = text.length();
  byte i;
  char helper [len+8];
  char command [] = {
    27,'Z',justification,
    lowByte(x1),highByte(x1),lowByte(y1),highByte(y1),
  };
  for (i=0;i<7;i++) helper[i] = command[i];
  for (i=0;i<len;i++) helper[i+7] = text[i];
  helper[len+7] = 0;
  sendData(helper,len+8);
  delay(5);
}

void EDIPTFT::drawLine(int x1, int y1, int x2, int y2) {
  char command [] = {
    27,'G','D',
    lowByte(x1),highByte(x1),lowByte(y1),highByte(y1),
    lowByte(x2),highByte(x2),lowByte(y2),highByte(y2),
  };
  sendData(command,11);
}

void EDIPTFT::drawRect(int x1, int y1, int x2, int y2) {
  char command [] = {
    27,'G','R',
    lowByte(x1),highByte(x1),lowByte(y1),highByte(y1),
    lowByte(x2),highByte(x2),lowByte(y2),highByte(y2),
  };
  sendData(command,11);
}

void EDIPTFT::drawRectf(int x1, int y1, int x2, int y2, char color) {
  char command [] = {
    27,'R','F',
    lowByte(x1),highByte(x1),lowByte(y1),highByte(y1),
    lowByte(x2),highByte(x2),lowByte(y2),highByte(y2),
    color
  };
  sendData(command,12);
}

void EDIPTFT::defineTouchKey(int x1, int y1, int x2, int y2, char down, char up, String text) {
  byte len = text.length();
  byte i;
  char helper [len+14];
  char command [] = {
    27,'A','T',
    lowByte(x1),highByte(x1),lowByte(y1),highByte(y1),
    lowByte(x2),highByte(x2),lowByte(y2),highByte(y2),
    down,up
  };
  for (i=0;i<13;i++) helper[i] = command[i];
  for (i=0;i<len;i++) helper[i+13] = text[i];
  helper[len+13] = 0;
  sendData(helper,len+14);
}

void EDIPTFT::setTouchkeyColors(char n1, char n2, char n3, char s1, char s2, char s3) {
  char command [] = {
    27,'F','E',n1,n2,n3,s1,s2,s3
  };
  sendData(command,9);
}

void EDIPTFT::setTouchkeyFont(char font) {
  char command [] = {
    27,'A','F',font
  };
  sendData(command,4);
}

void EDIPTFT::setTouchkeyLabelColors(char nf,char sf) {
  char command [] = {
    27,'F','A',nf,sf
  };
  sendData(command,5);
}

void EDIPTFT::removeTouchArea(char code,char n1) {
  char command [] = {
    27,'A','L',code,n1
  };
  sendData(command,5);
}




