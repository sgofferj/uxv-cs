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
#define PFD_SUPP_THICK 4


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
int32_t longitude=0;
int32_t latitude=0;
unsigned int ias=0;
unsigned int grs=0;
int vsi=0;
int numSats=0;
unsigned int vbat=0;
int bmode=0;
int gpsfix=0;
int beat=0;
int status_mavlink=0;
int status_frsky=0;
int status_gps=0;
int heading=0;
unsigned int cog=0;
unsigned long mav_utime=0;
uint8_t received_sysid=0;   ///< ID of heartbeat sender
uint8_t received_compid=0;  // component id of heartbeat sender

int GCS_MODE = 0;

void setup() {
  Serial.begin(57600);
  Serial1.begin(57600,256,16);
  Serial3.begin(115200);
  ea.smallProtoSelect(7);
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
    case 1 : {
      initUAVdata();
      break;
    }
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

void initUAVdata() {
  ea.setLineColor(EA_WHITE,0);
  ea.setLineThick(1,1);
  ea.drawLine(0,68,480,68);
  ea.drawLine(0,118,480,118);
  ea.drawLine(0,168,480,168);
  ea.drawLine(0,218,480,218);
  ea.drawLine(240,18,240,218);
  ea.drawLine(120,68,120,218);
  ea.drawLine(360,68,360,218);
  ea.setTextFont(9);
  ea.setTextColor(EA_WHITE,EA_BLACK);
  ea.drawText(10,20,'L',"LAT");
  ea.drawText(250,20,'L',"LON");
  ea.drawText(10,70,'L',"COG");
  ea.drawText(130,70,'L',"ALT");
  ea.drawText(250,70,'L',"IAS");
  ea.drawText(370,70,'L',"GS");
  ea.drawText(10,120,'L',"Vbatt");
}

void drawUAVdata() {
  String vbath=String(vbat);
  if (vbat/1000<10) vbath = vbath.substring(0,1)+"."+vbath.substring(2,4);
  else vbath = vbath.substring(0,2)+"."+vbath.substring(3,5);
  ea.setTextFont(10);
  ea.setTextColor(EA_WHITE,EA_BLACK);
  ea.drawText(120,36,'C',coordToString(latitude,'N'));
  ea.drawText(360,36,'C',coordToString(longitude,'E'));
  ea.drawText(100,86,'R'," "+String(cog/100));
  ea.drawText(220,86,'R'," "+String(altitude/1000)+"m");
  ea.drawText(340,86,'R'," "+String(ias));
  ea.drawText(460,86,'R'," "+String(grs));
  ea.drawText(100,136,'R'," "+vbath);
  start_feeds();
}

void initTest() {
  ea.defineInstrument(1,290,48,1,0,0,180);
  ea.defineInstrument(2,290,48,2,0,0,180);
//  ea.defineInstrument(3,290,48,3,0,0,180);
  ea.setTextFont(4);
  ea.setTextColor(EA_WHITE,EA_BLACK);
  ea.drawText(240, 42,'C',"IAS");
  ea.drawText(240, 92,'C',"GS");
  ea.drawText(240,142,'C',"VS");
  ea.drawText(240,192,'C',"ALT");
}

void drawTest() {
  drawPFD(100,136,pitch,roll);
  ea.updateInstrument(1,heading/2);
  ea.updateInstrument(2,heading/2);
//  ea.updateInstrument(3,182-(heading/2));
  ea.setTextFont(9);
  ea.setTextColor(EA_WHITE,EA_BLACK);
  ea.drawText(380,30,'C',leading2Zero(heading));
  ea.setTextFont(10);
  ea.setTextColor(EA_WHITE,EA_BLACK);
  ea.drawText(240, 55,'C',"  "+String(ias)+"  ");
  ea.drawText(240,105,'C',"  "+String(grs)+"  ");
  if (gpsfix >= 2) {
    ea.drawText(240,155,'C'," "+String(vsi)+" ");
    ea.drawText(240,205,'C',"  "+String(altitude/1000)+"  ");
  }
  else {
    ea.drawText(240,155,'C',"  ---  ");
    ea.drawText(240,205,'C',"  ---  ");
  }    
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

void drawPFD(int x, int y, int pitch, int roll) {
  static int oxs,oys,oxe,oye;
  static int oxss,oyss,oxes,oyes;
  int xs = x - (cos(toRad(roll))*90);
  int xe = x + (cos(toRad(roll))*90);
  int ys = (y + (sin(toRad(roll))*90))+pitch;
  int ye = (y - (sin(toRad(roll))*90))+pitch;
  int xss = x - (cos(toRad(roll))*20);
  int xes = x + (cos(toRad(roll))*20);
  int yss = (y + (sin(toRad(roll))*20))+pitch;
  int yes = (y - (sin(toRad(roll))*20))+pitch;
  ea.setLineColor(EA_BLACK,0);
  ea.setLineThick(PFD_SUPP_THICK,PFD_SUPP_THICK);
  ea.drawLine(oxss,oyss-15,oxes,oyes-15);
  ea.drawLine(oxss,oyss+15,oxes,oyes+15);
  ea.setLineThick(1,1);
  ea.drawLine(oxs,oys,oxe,oye);
  ea.setLineColor(EA_LIGHTBLUE,0);
  ea.setLineThick(PFD_SUPP_THICK,PFD_SUPP_THICK);
  if (abs(roll)<90) ea.drawLine(xss,yss-15,xes,yes-15);
  else ea.drawLine(xss,yss+15,xes,yes+15);
  ea.setLineColor(EA_ORANGE,0);
  if (abs(roll)<90) ea.drawLine(xss,yss+15,xes,yes+15);
  else ea.drawLine(xss,yss-15,xes,yes-15);
  ea.setLineColor(EA_WHITE,0);
  ea.setLineThick(1,1);
  ea.drawLine(xs,ys,xe,ye);
  ea.setLineColor(EA_GREEN,0);
  ea.drawLine(x-5,y,x+5,y);
  ea.drawLine(x,y-5,x,y+5);
  oxs=xs; oys=ys; oxe=xe; oye=ye;
  oxss=xss; oyss=yss; oxes=xes; oyes=yes;
}

String leadingZero(int number) {
  String helper = "";
  if (number < 10) {
    helper = "0";
  }
  helper += String(number);
  return helper;
}

String leading2Zero(int number) {
  String helper = "";
  if (number < 100) {
    helper = "0";
  }
  if (number < 10) {
    helper = "00";
  }
  helper += String(number);
  return helper;
}

String coordToString(int32_t coord, char direction) {
  String result = String(coord);
  if (coord > 0) {
    switch (direction) {
      case 'N' : {
        result = "N "+result;
        break;
      }
      case 'E' : {
        result = "E "+result;
        break;
      }
    }
  }
  if (coord < 0) {
    switch (direction) {
      case 'N' : {
        result = "S "+result.substring(1);
        break;
      }
      case 'E' : {
        result = "W "+result.substring(1);
        break;
      }
    }
    coord = -1*coord;
  }
  if (coord/1e7 < 10) result = result.substring(0,3)+"."+result.substring(4);
  else result = result.substring(0,4)+"."+result.substring(5);
  return result;
}

int freeRam() {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}



