//#include <progmem.h>
#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <GCS_MAVLink.h>
#include <EDIPTFT.h>

#define _VERSION "0.4"

String GPSFIX[] = {" NOFIX "," NOFIX "," 2DFIX "," 3DFIX "};

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
#define MAV_DATA_STREAM_RAW_CONTROLLER_ACTIVE 0
#define MAV_DATA_STREAM_EXTRA1_ACTIVE 1
#define MAV_DATA_STREAM_EXTRA2_ACTIVE 1 //VFR_HUD

// update rate is times per second (hz)
#define MAV_DATA_STREAM_POSITION_RATE 2
#define MAV_DATA_STREAM_RAW_SENSORS_RATE 1
#define MAV_DATA_STREAM_EXTENDED_STATUS_RATE 2
#define MAV_DATA_STREAM_RAW_CONTROLLER_RATE 1
#define MAV_DATA_STREAM_EXTRA1_RATE 1
#define MAV_DATA_STREAM_EXTRA2_RATE 4

// Look and feel
#define UI_BUTTON_ACTIVE EA_MINT,EA_BLACK,EA_MINT,EA_YELLOW,EA_BLACK,EA_YELLOW
#define UI_BUTTON_INACTIVE EA_WHITE,EA_BLACK,EA_WHITE,EA_YELLOW,EA_BLACK,EA_YELLOW



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
int bmode=0;
int gpsfix=0;
int beat=0;
int status_mavlink=0;
int status_frsky=0;
int status_gps=0;
int heading=0;
unsigned long mav_utime=0;
uint8_t received_sysid=0;   ///< ID of heartbeat sender
uint8_t received_compid=0;  // component id of heartbeat sender

int GCS_MODE = 0;

void setup() {
  Serial.begin(57600);
  Serial1.begin(57600,256,16);
  Serial3.begin(115200);
  ea.smallProtoSelect(0);
  ea.smallProtoSelect(0);
  ea.clear();
  ea.cursor(false);
  drawSplash();
}

void loop() {
  if (gcs_update()) {
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
  }
  if ((status_mavlink == 1) && (!gcs_update())) drawStatusbar();
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
  ea.setTextFont(3);

  if (status_frsky == 1) ea.setTextColor(EA_BLACK,EA_GREEN);
  else ea.setTextColor(EA_BLACK,EA_RED);
  ea.drawText(384,3,'R'," FRSKY ");

  if (status_mavlink == 1) ea.setTextColor(EA_BLACK,EA_GREEN);
  else ea.setTextColor(EA_BLACK,EA_RED);
  ea.drawText(428,3,'R'," MAVL ");
  
  switch (gpsfix) {
    case 0 : {
      ea.setTextColor(EA_BLACK,EA_RED);
      break;
    }
    case 1 : {
      ea.setTextColor(EA_BLACK,EA_RED);
      break;
    }
    case 2 : {
      ea.setTextColor(EA_BLACK,EA_YELLOW);
      break;
    }
    case 3 : {
      ea.setTextColor(EA_BLACK,EA_GREEN);
      break;
    }
  }    
  ea.drawText(479,3,'R',GPSFIX[gpsfix]);

  if ((bmode & MAV_MODE_FLAG_SAFETY_ARMED) == MAV_MODE_FLAG_SAFETY_ARMED) {
    ea.setTextColor(EA_BLACK,EA_RED);
    ea.drawText(24,3,'C',"  ARM  ");
  }
  else {
    ea.setTextColor(EA_BLACK,EA_GREEN);
    ea.drawText(24,3,'C'," D-ARM ");
  }
  if ((bmode & MAV_MODE_FLAG_AUTO_ENABLED) == MAV_MODE_FLAG_AUTO_ENABLED) {
    ea.setTextColor(EA_BLACK,EA_RED);
    ea.drawText(72,3,'C'," AUTO ");
  }
  else if ((bmode & MAV_MODE_FLAG_GUIDED_ENABLED) == MAV_MODE_FLAG_GUIDED_ENABLED) {
    ea.setTextColor(EA_BLACK,EA_YELLOW);
    ea.drawText(72,3,'C'," GUID ");
  }
  else if ((bmode & MAV_MODE_FLAG_STABILIZE_ENABLED) == MAV_MODE_FLAG_STABILIZE_ENABLED) {
    ea.setTextColor(EA_BLACK,EA_LIGHTBLUE);
    ea.drawText(72,3,'C'," STAB ");
  }
  else {
    ea.setTextColor(EA_BLACK,EA_GREEN);
    ea.drawText(72,3,'C'," MANU ");
  }
  
  ea.setLineColor(EA_WHITE,1);
  ea.drawLine(0,18,480,18);
}

void drawButtons() {
  ea.removeTouchArea(0,1);
  ea.setTouchkeyLabelColors(EA_BLACK,EA_BLACK);
  ea.setTouchkeyFont(5);
  if (GCS_MODE==1) ea.setTouchkeyColors(UI_BUTTON_ACTIVE);
  else ea.setTouchkeyColors(UI_BUTTON_INACTIVE);
  ea.defineTouchKey(  0,248, 60,272,0,1,"OVRV");
  if (GCS_MODE==2) ea.setTouchkeyColors(UI_BUTTON_ACTIVE);
  else ea.setTouchkeyColors(UI_BUTTON_INACTIVE);
  ea.defineTouchKey( 70,248,130,272,0,2,"MOD2");
  if (GCS_MODE==3) ea.setTouchkeyColors(UI_BUTTON_ACTIVE);
  else ea.setTouchkeyColors(UI_BUTTON_INACTIVE);
  ea.defineTouchKey(140,248,200,272,0,3,"MOD3");
  if (GCS_MODE==4) ea.setTouchkeyColors(UI_BUTTON_ACTIVE);
  else ea.setTouchkeyColors(UI_BUTTON_INACTIVE);
  ea.defineTouchKey(210,248,270,272,0,4,"MOD4");
  if (GCS_MODE==5) ea.setTouchkeyColors(UI_BUTTON_ACTIVE);
  else ea.setTouchkeyColors(UI_BUTTON_INACTIVE);
  ea.defineTouchKey(280,248,340,272,0,5,"MOD5");
  if (GCS_MODE==6) ea.setTouchkeyColors(UI_BUTTON_ACTIVE);
  else ea.setTouchkeyColors(UI_BUTTON_INACTIVE);
  ea.defineTouchKey(350,248,410,272,0,6,"TEST");
  if (GCS_MODE==7) ea.setTouchkeyColors(UI_BUTTON_ACTIVE);
  else ea.setTouchkeyColors(UI_BUTTON_INACTIVE);
  ea.defineTouchKey(420,248,480,272,0,7,"SYST");
}  

void drawSplash() {
  ea.setTextFont(6);
  ea.setTextColor(EA_GRASSGREEN,EA_BLACK);
  ea.drawText(240,50,'C',"UxV Control Station");
  ea.drawText(240,95,'C',_VERSION);
  ea.drawText(240,140,'C',"(C)2013 Stefan Gofferje");
  ea.drawText(240,185,'C',"Published under the Gnu GPL");
  delay(2000);
  setMode(1);
}
  
void drawUAVdata() {
  ea.setTextFont(5);
  ea.setTextColor(EA_YELLOW,EA_BLACK);
  ea.drawText(0,24,'L',"LON: "+String(longitude));
  ea.drawText(0,44,'L',"LAT: "+String(latitude));
  ea.drawText(0,64,'L',"ALT: "+String(altitude/1000)+"m        ");
  start_feeds();
}

void initTest() {
  ea.defineInstrument(1,140,36,1,0,0,180);
}

void drawTest() {
  ea.updateInstrument(1,heading/2);
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



