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

#ifndef EDIPTFT_h
#define EDIPTFT_h

#include <Arduino.h>

class EDIPTFT {
  public:
    EDIPTFT(int port, int smallprotocol);
    void sendData(char* data, int len);
    void clear();
    void invert();
    void setDisplayColor(char fg, char bg);
    void fillDisplayColor(char bg);
    void terminalOn(boolean on);
    void cursor(boolean on);
    void setCursor(char col, char row);
    void defineBargraph(char dir, char no, int x1, int y1, int x2, int y2, byte sv, byte ev, char type);
    void updateBargraph(char no, char val);
    void setBargraphColor(char no, char fg, char bg, char fr);
    void makeBargraphTouch(char no);
    void linkBargraphLight(char no);
    void deleteBargraph(char no,char n1);
    void setTextColor(char fg, char bg);
    void setTextFont(char font);
    void setTextAngle(char angle);
    void drawText(int x1, int y1, char justification,String text);
    void setLineColor(char fg, char bg);
    void drawLine(int x1, int y1, int x2, int y2);
    void drawRect(int x1, int y1, int x2, int y2);
    void drawRectf(int x1, int y1, int x2, int y2, char color);
    void defineTouchKey(int x1, int y1, int x2, int y2, char down, char up, String text);
    void setTouchkeyColors(char n1, char n2, char n3, char s1, char s2, char s3);
    void setTouchkeyFont(char font);
    void setTouchkeyLabelColors(char nf,char sf);
    void removeTouchArea(char code,char n1);
  private:
    int _port;
    int _smallprotocol;
    void sendSmall(char* data, int len);
};
#endif