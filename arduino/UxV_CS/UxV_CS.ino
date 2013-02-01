#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <GCS_MAVLink.h>
#include <EDIPTFT.h>

#define BLACK 1
#define BLUE 2
#define RED 3
#define GREEN 4
#define PURPLE 5
#define CYAN 6
#define YELLOW 7
#define WHITE 8
#define DARKGREY 9
#define ORANGE 10
#define LILA 11
#define DARKPURPLE 12
#define MINT 13
#define GRASSGREEN 14
#define LIGHTBLUE 15
#define LIGHTGREY 16

#define SMALLPROTOCOL false

String GPSFIX[] = {"NO FIX","NO FIX","2D FIX","3D FIX"};
String modeName[] = {"","Overview","Mode 2","Mode 3","Mode 4","Mode 5","Mode 6","Setup"};

FastSerialPort0(Serial);
FastSerialPort1(Serial1);
FastSerialPort2(Serial2);
FastSerialPort3(Serial3);
EDIPTFT ea(3,0);

#define DIST_CONV 1.0   // For distance display in meters choose 0.0, for feet 3.2808

#define toRad(x) (x*PI)/180.0
#define toDeg(x) (x*180.0)/PI

unsigned long hb_timer;
unsigned long timer;

#define SERVO_MAX 2600 //Range of servos pitch and roll
#define SERVO_MIN 400
#define TEST_PAN 0 //Test pan min and max (for calibration)
#define TEST_SOUTH 0 //test the south (just poing to south)
#define TEST_TILT 0 //test the tilt max and min (for calibration)

unsigned long Latitude_Home=0;
unsigned long Longitud_Home=0;
unsigned long Altitude_Home = 0;
unsigned long Distance_Home=0;
unsigned long Distance3D_Home=0;
int Angle_Home=0;
int Constrain_Angle_Home = 0;
unsigned long Bearing_Home=0;
unsigned long SvBearingHome = 0;

float offset = 0;
// flight data
float pitch=0;
float roll=0;
float yaw=0;
unsigned long altitude=0;
unsigned long longitude=0;
unsigned long latitude=0;
unsigned int velocity=0;
int numSats=0;
float battery=0;
int currentSMode=0;
int currentNMode=0;
int gpsfix=0;
int beat=0;
int connstat=0;
unsigned long mav_utime=0;

int GCS_MODE = 0;

void setup() {
  Serial.begin(57600);
  Serial.println(freeRam());
  Serial1.begin(57600);
  Serial3.begin(115200);
  delay(250);
  ea.clear();
  ea.clear();
  ea.clear();
  delay(250);
  ea.cursor(false);
  drawSplash();
  drawButtons();
}

void loop() {
  gcs_update();
  switch (GCS_MODE) {
    case 1 : {
      drawUAVdata();
      break;
    }
  }
  drawStatusbar();
  
  if (Serial3.available() >0) {
    char data = Serial3.read();
    if (data == char(27)) {
      parseSerial();
    }
  }
  Serial.println(freeRam());
  
}

void parseSerial() {
  char data;
  if (Serial3.available() >0) {
    data = Serial3.read();
    if (data == 'A') {
      data = Serial3.read();
      data = Serial3.read();
      setMode((int)data);
    }
  }
}

void setMode(int mode) {
  switch (GCS_MODE) {
    case 7 : {
      ea.deleteBargraph(1,1);
      break;
    }
  }
  GCS_MODE = mode;
  ea.clear();
  delay(5);

  drawButtons();

  switch (mode) {
    case 7 : {
      drawSetup();
      break;
    }
  }

}    

void drawStatusbar() {
  ea.setTextFont(4);
  ea.setTextColor(WHITE,BLACK);
  ea.drawText(240,4,'C',modeName[GCS_MODE]);
  if (connstat == 1) ea.setTextColor(GREEN,BLACK);
  else ea.setTextColor(RED,BLACK);
  ea.drawText(420,4,'C',"MAVL");
  switch (gpsfix) {
    case 0 : {
      ea.setTextColor(RED,BLACK);
      break;
    }
    case 1 : {
      ea.setTextColor(RED,BLACK);
      break;
    }
    case 2 : {
      ea.setTextColor(YELLOW,BLACK);
      break;
    }
    case 3 : {
      ea.setTextColor(GREEN,BLACK);
      break;
    }
  }    
  ea.drawText(479,4,'R',GPSFIX[gpsfix]);
  ea.setLineColor(WHITE,1);
  ea.drawLine(0,18,480,18);
}

void drawButtons() {
  ea.removeTouchArea(0,1);
  ea.setTouchkeyLabelColors(BLACK,BLACK);
  ea.setTouchkeyFont(5);
  if (GCS_MODE==1) ea.setTouchkeyColors(MINT,BLACK,MINT,YELLOW,BLACK,YELLOW);
  else ea.setTouchkeyColors(WHITE,BLACK,WHITE,YELLOW,BLACK,YELLOW);
  ea.defineTouchKey(  0,248, 60,272,0,1,"OVRV");
  if (GCS_MODE==2) ea.setTouchkeyColors(MINT,BLACK,MINT,YELLOW,BLACK,YELLOW);
  else ea.setTouchkeyColors(WHITE,BLACK,WHITE,YELLOW,BLACK,YELLOW);
  ea.defineTouchKey( 70,248,130,272,0,2,"MOD2");
  if (GCS_MODE==3) ea.setTouchkeyColors(MINT,BLACK,MINT,YELLOW,BLACK,YELLOW);
  else ea.setTouchkeyColors(WHITE,BLACK,WHITE,YELLOW,BLACK,YELLOW);
  ea.defineTouchKey(140,248,200,272,0,3,"MOD3");
  if (GCS_MODE==4) ea.setTouchkeyColors(MINT,BLACK,MINT,YELLOW,BLACK,YELLOW);
  else ea.setTouchkeyColors(WHITE,BLACK,WHITE,YELLOW,BLACK,YELLOW);
  ea.defineTouchKey(210,248,270,272,0,4,"MOD4");
  if (GCS_MODE==5) ea.setTouchkeyColors(MINT,BLACK,MINT,YELLOW,BLACK,YELLOW);
  else ea.setTouchkeyColors(WHITE,BLACK,WHITE,YELLOW,BLACK,YELLOW);
  ea.defineTouchKey(280,248,340,272,0,5,"MOD5");
  if (GCS_MODE==6) ea.setTouchkeyColors(MINT,BLACK,MINT,YELLOW,BLACK,YELLOW);
  else ea.setTouchkeyColors(WHITE,BLACK,WHITE,YELLOW,BLACK,YELLOW);
  ea.defineTouchKey(350,248,410,272,0,6,"MOD6");
  if (GCS_MODE==7) ea.setTouchkeyColors(MINT,BLACK,MINT,YELLOW,BLACK,YELLOW);
  else ea.setTouchkeyColors(WHITE,BLACK,WHITE,YELLOW,BLACK,YELLOW);
  ea.defineTouchKey(420,248,480,272,0,7,"Setup");
}  

void drawSetup() {
  ea.defineBargraph ('O',1,430,40,470,220,0,100,5);
  ea.setTextColor(WHITE,BLACK);
  ea.drawText(450,225,'C',"Light");
  ea.updateBargraph(1,66);
  ea.makeBargraphTouch(1);
  ea.linkBargraphLight(1);
}

void drawUAVdata() {
  ea.setTextFont(5);
  ea.setTextColor(YELLOW,BLACK);
  ea.drawText(0,24,'L',"LON: "+String(longitude));
  ea.drawText(0,44,'L',"LAT: "+String(latitude));
  ea.drawText(0,64,'L',"ALT: "+String(altitude/1000)+"m");
}

void drawSplash() {
  ea.setTextFont(6);
  ea.setTextColor(GRASSGREEN,BLACK);
  ea.drawText(240,50,'C',"UxV Control Station");
  ea.drawText(240,95,'C',"V0.1");
  ea.drawText(240,140,'C',"(C)2013 Stefan Gofferje");
  ea.drawText(240,185,'C',"Published under the Gnu GPL");
}
  
  
String leadingZero(int number) {
  String helper = "";
  if (number < 10) {
    helper = "0";
  }
  helper += String(number);
  return helper;
}

int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}



