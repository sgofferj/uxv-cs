#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <GCS_MAVLink.h>
#include <EDIPTFT.h>

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

String GPSFIX[] = {"NO FIX","NO FIX","2D FIX","3D FIX"};
String modeName[] = {"","Overview","Mode 2","Mode 3","Mode 4","Mode 5","Test","System"};

FastSerialPort0(Serial);
FastSerialPort1(Serial1);
FastSerialPort2(Serial2);
FastSerialPort3(Serial3);
EDIPTFT ea(3,1);

#define DIST_CONV 1.0   // For distance display in meters choose 0.0, for feet 3.2808

#define toRad(x) (x*PI)/180.0
#define toDeg(x) (x*180.0)/PI

unsigned long hb_timer;
unsigned long timer;

// data streams active and rates
#define MAV_DATA_STREAM_POSITION_ACTIVE 1
#define MAV_DATA_STREAM_RAW_SENSORS_ACTIVE 1
#define MAV_DATA_STREAM_EXTENDED_STATUS_ACTIVE 1
#define MAV_DATA_STREAM_RAW_CONTROLLER_ACTIVE 1
#define MAV_DATA_STREAM_EXTRA1_ACTIVE 1

// update rate is times per second (hz)
#define MAV_DATA_STREAM_POSITION_RATE 2
#define MAV_DATA_STREAM_RAW_SENSORS_RATE 1
#define MAV_DATA_STREAM_EXTENDED_STATUS_RATE 1
#define MAV_DATA_STREAM_RAW_CONTROLLER_RATE 1
#define MAV_DATA_STREAM_EXTRA1_RATE 5

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
uint8_t received_sysid=0;   ///< ID of heartbeat sender
uint8_t received_compid=0;  // component id of heartbeat sender

int GCS_MODE = 0;

void setup() {
  Serial.begin(57600);
  Serial1.begin(57600);
  Serial3.begin(115200);
  ea.smallProtoSelect(0);
  ea.smallProtoSelect(0);
  ea.clear();
  delay(250);
  ea.clear();
  delay(250);
  ea.clear();
  delay(250);
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
    case 6 : {
      drawTest();
      break;
    }
  }
  drawStatusbar();
  
  if (ea.datainBuffer() >0) parseSerial();
}

void parseSerial() {
  char i;
  char len = ea.datainBuffer();
  char data[len];
  ea.readBuffer(data);
  if (data[0] == 27) {
    if ((data[1] == 'A') && (data[2] == 1)) setMode((int)data[3]);
  }
}

void setMode(int mode) {
  switch (GCS_MODE) {
    case 6 : {
      destroyTest();
      break;
    }
    case 7 : {
      destroySystem();
      break;
    }
  }
  GCS_MODE = mode;
  ea.clear();

  drawButtons();

  switch (mode) {
    case 6 : {
      initTest();
      break;
    }
    case 7 : {
      initSystem();
      break;
    }
  }

}    

void drawStatusbar() {
  ea.setTextFont(4);
  ea.setTextColor(EA_WHITE,EA_BLACK);
  ea.drawText(240,4,'C',modeName[GCS_MODE]);
  if (connstat == 1) ea.setTextColor(EA_GREEN,EA_BLACK);
  else ea.setTextColor(EA_RED,EA_BLACK);
  ea.drawText(420,4,'C',"MAVL");
  switch (gpsfix) {
    case 0 : {
      ea.setTextColor(EA_RED,EA_BLACK);
      break;
    }
    case 1 : {
      ea.setTextColor(EA_RED,EA_BLACK);
      break;
    }
    case 2 : {
      ea.setTextColor(EA_YELLOW,EA_BLACK);
      break;
    }
    case 3 : {
      ea.setTextColor(EA_GREEN,EA_BLACK);
      break;
    }
  }    
  ea.drawText(479,4,'R',GPSFIX[gpsfix]);
  ea.setLineColor(EA_WHITE,1);
  ea.drawLine(0,18,480,18);
}

void drawButtons() {
  ea.removeTouchArea(0,1);
  ea.setTouchkeyLabelColors(EA_BLACK,EA_BLACK);
  ea.setTouchkeyFont(5);
  if (GCS_MODE==1) ea.setTouchkeyColors(EA_MINT,EA_BLACK,EA_MINT,EA_YELLOW,EA_BLACK,EA_YELLOW);
  else ea.setTouchkeyColors(EA_WHITE,EA_BLACK,EA_WHITE,EA_YELLOW,EA_BLACK,EA_YELLOW);
  ea.defineTouchKey(  0,248, 60,272,0,1,"OVRV");
  if (GCS_MODE==2) ea.setTouchkeyColors(EA_MINT,EA_BLACK,EA_MINT,EA_YELLOW,EA_BLACK,EA_YELLOW);
  else ea.setTouchkeyColors(EA_WHITE,EA_BLACK,EA_WHITE,EA_YELLOW,EA_BLACK,EA_YELLOW);
  ea.defineTouchKey( 70,248,130,272,0,2,"MOD2");
  if (GCS_MODE==3) ea.setTouchkeyColors(EA_MINT,EA_BLACK,EA_MINT,EA_YELLOW,EA_BLACK,EA_YELLOW);
  else ea.setTouchkeyColors(EA_WHITE,EA_BLACK,EA_WHITE,EA_YELLOW,EA_BLACK,EA_YELLOW);
  ea.defineTouchKey(140,248,200,272,0,3,"MOD3");
  if (GCS_MODE==4) ea.setTouchkeyColors(EA_MINT,EA_BLACK,EA_MINT,EA_YELLOW,EA_BLACK,EA_YELLOW);
  else ea.setTouchkeyColors(EA_WHITE,EA_BLACK,EA_WHITE,EA_YELLOW,EA_BLACK,EA_YELLOW);
  ea.defineTouchKey(210,248,270,272,0,4,"MOD4");
  if (GCS_MODE==5) ea.setTouchkeyColors(EA_MINT,EA_BLACK,EA_MINT,EA_YELLOW,EA_BLACK,EA_YELLOW);
  else ea.setTouchkeyColors(EA_WHITE,EA_BLACK,EA_WHITE,EA_YELLOW,EA_BLACK,EA_YELLOW);
  ea.defineTouchKey(280,248,340,272,0,5,"MOD5");
  if (GCS_MODE==6) ea.setTouchkeyColors(EA_MINT,EA_BLACK,EA_MINT,EA_YELLOW,EA_BLACK,EA_YELLOW);
  else ea.setTouchkeyColors(EA_WHITE,EA_BLACK,EA_WHITE,EA_YELLOW,EA_BLACK,EA_YELLOW);
  ea.defineTouchKey(350,248,410,272,0,6,"TEST");
  if (GCS_MODE==7) ea.setTouchkeyColors(EA_MINT,EA_BLACK,EA_MINT,EA_YELLOW,EA_BLACK,EA_YELLOW);
  else ea.setTouchkeyColors(EA_WHITE,EA_BLACK,EA_WHITE,EA_YELLOW,EA_BLACK,EA_YELLOW);
  ea.defineTouchKey(420,248,480,272,0,7,"SYST");
}  

void drawSplash() {
  ea.setTextFont(6);
  ea.setTextColor(EA_GRASSGREEN,EA_BLACK);
  ea.drawText(240,50,'C',"UxV Control Station");
  ea.drawText(240,95,'C',"V0.1");
  ea.drawText(240,140,'C',"(C)2013 Stefan Gofferje");
  ea.drawText(240,185,'C',"Published under the Gnu GPL");
}
  
void drawUAVdata() {
  ea.setTextFont(5);
  ea.setTextColor(EA_YELLOW,EA_BLACK);
  ea.drawText(0,24,'L',"LON: "+String(longitude));
  ea.drawText(0,44,'L',"LAT: "+String(latitude));
  ea.drawText(0,64,'L',"ALT: "+String(altitude/1000)+"m      ");
  start_feeds();
}

void initTest() {
  ea.defineInstrument(1,140,36,1,0,0,180);
}

void drawTest() {
  char hdg = (yaw+180)/2;
  ea.updateInstrument(1,yaw);
}

void destroyTest() {
  ea.deleteInstrument(1,1,1);
}

void initSystem() {
  ea.defineBargraph ('O',1,430,24,470,220,0,100,5);
  ea.setTextFont(5);
  ea.setTextColor(EA_WHITE,EA_BLACK);
  ea.drawText(450,225,'C',"Light");
  ea.updateBargraph(1,66);
  ea.makeBargraphTouch(1);
  ea.linkBargraphLight(1);
  ea.setTextFont(5);
  ea.setTextColor(EA_YELLOW,EA_BLACK);
  ea.drawText(0,24,'L',"Free RAM: "+String(freeRam())+" bytes");
}

void destroySystem() {
  ea.deleteBargraph(1,1);
}

  
String leadingZero(int number) {
  String helper = "";
  if (number < 10) {
    helper = "0";
  }
  helper += String(number);
  return helper;
}

int freeRam() {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}



