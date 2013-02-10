//#include <progmem.h>
#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <GCS_MAVLink.h>

#include <EDIPTFT.h>

#define _VERSION "V0.46"

char *GPSFIX[] = {" NOFIX "," NOFIX "," 2DFIX "," 3DFIX "};

FastSerialPort0(Serial);
FastSerialPort1(Serial1);
FastSerialPort2(Serial2);
FastSerialPort3(Serial3);

EDIPTFT ea(3,1);

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

#define CUST_MODE_MANUAL 0
#define CUST_MODE_CIRCLE 1
#define CUST_MODE_STABILIZE 2
#define CUST_MODE_TRAINING 3
#define CUST_MODE_FLY_BY_WIRE_A 5
#define CUST_MODE_FLY_BY_WIRE_B 6
#define CUST_MODE_AUTO 10
#define CUST_MODE_RTL 11
#define CUST_MODE_LOITER 12
#define CUST_MODE_GUIDED 15
#define CUST_MODE_INITIALISING 16

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
long altitude=0;
int32_t longitude=0;
int32_t latitude=0;
unsigned int ias=0;
unsigned int grs=0;
int vsi=0;
int numSats=0;
unsigned int vbat=0;
int bmode=0;
int cmode=0;
int dmode=0; // 0=metric, 1=imperial land, 2=imperial nautic
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
//  drawSplash();
  
  ea.defineTouchKey(445,3,479,14,0,10," ");
  setMode(1);
}

void loop() {
  if (gcs_update()) {
    switch (GCS_MODE) {
      case 1 : {
        drawOVRV();
        break;
      }
      case 3 : {
        drawPFD();
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
    if ((data[1] == 'A') && (data[2] == 1)) {
      switch (data[3]) {
        case 1 : {
          setMode(1);
          break;
        }
        case 2 : {
          setMode(2);
          break;
        }
        case 3 : {
          setMode(3);
          break;
        }
        case 4 : {
          setMode(4);
          break;
        }
        case 5 : {
          setMode(5);
          break;
        }
        case 6 : {
          setMode(6);
          break;
        }
        case 7 : {
          setMode(7);
          break;
        }
        case 10 : {
          if (dmode < 2) dmode ++;
          else dmode = 0;
          break;
        }
      }
    }
  }
}

void setMode(int mode) {
  switch (GCS_MODE) {
    case 3 : {
      destroyPFD();
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
      initOVRV();
      break;
    }
    case 3 : {
      initPFD();
      break;
    }
    case 7 : {
      initSystem();
      break;
    }
  }
}    

void drawStatusbar() {
  char buf[8];
  ea.setTextFont(3);

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

  switch (cmode) {
    case CUST_MODE_MANUAL : {
      ea.setTextColor(EA_BLACK,EA_GREEN);
      ea.drawText(116,3,'C'," MANU ");
      break;;
    }
    case CUST_MODE_CIRCLE : {
      ea.setTextColor(EA_BLACK,EA_RED);
      ea.drawText(116,3,'C'," CIRC ");
      break;;
    }
    case CUST_MODE_STABILIZE : {
      ea.setTextColor(EA_BLACK,EA_LIGHTBLUE);
      ea.drawText(116,3,'C'," STAB ");
      break;;
    }
    case CUST_MODE_TRAINING : {
      ea.setTextColor(EA_BLACK,EA_PURPLE);
      ea.drawText(116,3,'C'," TRAI ");
      break;;
    }
    case CUST_MODE_FLY_BY_WIRE_A : {
      ea.setTextColor(EA_BLACK,EA_BLUE);
      ea.drawText(116,3,'C'," FBWA ");
      break;;
    }
    case CUST_MODE_FLY_BY_WIRE_B : {
      ea.setTextColor(EA_BLACK,EA_BLUE);
      ea.drawText(116,3,'C'," FBWB ");
      break;;
    }
    case CUST_MODE_RTL : {
      ea.setTextColor(EA_BLACK,EA_YELLOW);
      ea.drawText(116,3,'C'," RTL  ");
      break;;
    }
    case CUST_MODE_LOITER : {
      ea.setTextColor(EA_BLACK,EA_YELLOW);
      ea.drawText(116,3,'C'," LOIT ");
      break;;
    }
    case CUST_MODE_GUIDED : {
      ea.setTextColor(EA_BLACK,EA_YELLOW);
      ea.drawText(116,3,'C'," GUID ");
      break;;
    }
    case CUST_MODE_INITIALISING : {
      ea.setTextColor(EA_BLACK,EA_CYAN);
      ea.drawText(116,3,'C'," INIT ");
      break;;
    }
  }

  if (status_frsky == 1) ea.setTextColor(EA_BLACK,EA_GREEN);
  else ea.setTextColor(EA_BLACK,EA_RED);
  ea.drawText(317,3,'R'," FRSKY ");

  if (status_mavlink == 1) ea.setTextColor(EA_BLACK,EA_GREEN);
  else ea.setTextColor(EA_BLACK,EA_RED);
  ea.drawText(361,3,'R'," MAVL ");
  
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
  ea.drawText(412,3,'R',GPSFIX[gpsfix]);

  if (numSats < 5) ea.setTextColor(EA_BLACK,EA_RED);
  else if (numSats < 8) ea.setTextColor(EA_BLACK,EA_YELLOW);
  else if (numSats >= 8) ea.setTextColor(EA_BLACK,EA_GREEN);
  sprintf(buf," %02d ",numSats);
  ea.drawText(442,3,'R',buf);
  
  ea.setTextColor(EA_BLACK,EA_WHITE);
  switch (dmode) {
    case 0 : {
      ea.drawText(479,3,'R'," MET ");
      break;
    }
    case 1 : {
      ea.drawText(479,3,'R'," IMP ");
      break;
    }
    case 2 : {
      ea.drawText(479,3,'R'," NAU ");
      break;
    }
  }    

  ea.setLineColor(EA_WHITE,1);
  ea.drawLine(0,18,480,18);
}

void drawButtons() {
  ea.removeTouchArea(1,1);
  ea.removeTouchArea(2,1);
  ea.removeTouchArea(3,1);
  ea.removeTouchArea(4,1);
  ea.removeTouchArea(5,1);
  ea.removeTouchArea(6,1);
  ea.removeTouchArea(7,1);
  ea.setTouchkeyLabelColors(EA_BLACK,EA_BLACK);
  ea.setTouchkeyFont(5);
  if (GCS_MODE==1) ea.setTouchkeyColors(UI_BUTTON_ACTIVE);
  else ea.setTouchkeyColors(UI_BUTTON_INACTIVE);
  ea.defineTouchKey(  0,248, 60,272,0,1,"OVRV");
  if (GCS_MODE==2) ea.setTouchkeyColors(UI_BUTTON_ACTIVE);
  else ea.setTouchkeyColors(UI_BUTTON_INACTIVE);
  ea.defineTouchKey( 70,248,130,272,0,2,"FRSKY");
  if (GCS_MODE==3) ea.setTouchkeyColors(UI_BUTTON_ACTIVE);
  else ea.setTouchkeyColors(UI_BUTTON_INACTIVE);
  ea.defineTouchKey(140,248,200,272,0,3,"PFD");
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
}

void initOVRV() {
  start_feeds();
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

void drawOVRV() {
  char buf[16];
  ea.setTextFont(10);
  ea.setTextColor(EA_WHITE,EA_BLACK);
  if (gpsfix > 1) {
    dtostrf(latitude/1e7,2,6,buf);
    ea.drawText(125,36,'C',buf);
    dtostrf(longitude/1e7,2,6,buf);
    ea.drawText(365,36,'C',buf);
    sprintf(buf,"%03d",cog/100);
    ea.drawText(105,86,'R',buf);
    sprintf(buf,"%05d",altitude);
    if (altitude < 60000) ea.drawText(225,86,'R',buf);
    sprintf(buf,"%03d",grs);
    ea.drawText(465,86,'R',buf);
  }
  else {
    ea.drawText(125,36,'C',"    ---    ");
    ea.drawText(365,36,'C',"    ---    ");
    ea.drawText(105,86,'R',"   ---");
    ea.drawText(225,86,'R',"   ---");
    ea.drawText(465,86,'R',"   ---");
  }
  ea.setTextColor(EA_WHITE,EA_BLACK);
  sprintf(buf,"%03d",ias);
  ea.drawText(345,86,'R',buf);
  dtostrf(vbat/1000.0,2,2,buf);
  ea.drawText(105,136,'R',buf);
}

void initPFD() {
  ea.defineInstrument(1,290,48,1,0,0,180);
  ea.defineInstrument(2,290,48,2,0,0,180);
  ea.setLineThick(1,1);
  ea.drawLine(195,136,215,136);
  ea.defineBargraph('O',1,200,42,210,136,0,20,1);
  ea.defineBargraph('U',2,200,136,210,230,0,20,1);
//  ea.defineInstrument(3,290,48,3,0,0,180);
  ea.setTextFont(4);
  ea.setTextColor(EA_WHITE,EA_BLACK);
  ea.drawText(225, 42,'L',"IAS");
  ea.drawText(225, 92,'L',"GS");
  ea.drawText(225,192,'L',"ALT");
}

void drawPFD() {
  char buf[16];
  drawATTI(100,136,pitch,roll);
  ea.updateInstrument(1,heading/2+1);
  ea.updateInstrument(2,heading/2+1);
//  ea.updateInstrument(3,182-(heading/2));
  ea.setTextFont(9);
  ea.setTextColor(EA_WHITE,EA_BLACK);
  sprintf(buf,"%03d",heading);
  ea.drawText(380,30,'C',buf);
  ea.setTextFont(10);
  sprintf(buf,"%03d",ias);
  ea.drawText(270, 55,'R',buf);
  if (gpsfix > 1) {
    sprintf(buf,"%03d",grs);
    ea.drawText(270,105,'R',buf);
    sprintf(buf,"%05d",altitude);
    if (altitude/1000 < 60000) ea.drawText(302,205,'R',buf);
  }
  else {
    ea.drawText(270,105,'R',"---");
    ea.drawText(302,205,'R',"-----");
  }    
  if (vsi > 0) {
    ea.updateBargraph(1,vsi);
    ea.updateBargraph(2,0);
  }
  if (vsi < 0) {
    ea.updateBargraph(2,-1*vsi);
    ea.updateBargraph(1,0);
  }
  if (vsi == 0) {
    ea.updateBargraph(1,0);
    ea.updateBargraph(2,0);
  }
  
}

void destroyPFD() {
  ea.deleteInstrument(1,1,1);
  ea.deleteBargraph(1,1);
}

void initSystem() {
  char buf[16];
  ea.defineBargraph ('O',1,430,24,470,220,0,100,5);
  ea.setTextFont(5);
  ea.setTextColor(EA_WHITE,EA_BLACK);
  ea.drawText(450,225,'C',"Light");
  ea.updateBargraph(1,66);
  ea.makeBargraphTouch(1);
  ea.linkBargraphLight(1);
  ea.setTextFont(5);
  ea.setTextColor(EA_YELLOW,EA_BLACK);
  sprintf(buf,"Free RAM: %d bytes",freeRam());
  ea.drawText(0,24,'L',buf);
}

void destroySystem() {
  ea.deleteBargraph(1,1);
}

void drawATTI(int x, int y, int pitch, int roll) {
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
  ea.drawLine(oxss,oyss-45,oxes,oyes-45);
  ea.drawLine(oxss,oyss+45,oxes,oyes+45);
  ea.setLineThick(1,1);
  ea.drawLine(oxss,oyss-15,oxes,oyes-15);
  ea.drawLine(oxss,oyss+15,oxes,oyes+15);
  ea.drawLine(oxs,oys,oxe,oye);
  ea.setLineColor(EA_LIGHTBLUE,0);
  if (abs(roll)<90) {
    ea.setLineThick(1,1);
    ea.drawLine(xss,yss-15,xes,yes-15);
    ea.setLineThick(PFD_SUPP_THICK,PFD_SUPP_THICK);
    ea.drawLine(xss,yss-45,xes,yes-45);
  }
  else {
    ea.setLineThick(1,1);
    ea.drawLine(xss,yss+15,xes,yes+15);
    ea.setLineThick(PFD_SUPP_THICK,PFD_SUPP_THICK);
    ea.drawLine(xss,yss+45,xes,yes+45);
  }
  ea.setLineColor(EA_ORANGE,0);
  if (abs(roll)<90) {
    ea.setLineThick(1,1);
    ea.drawLine(xss,yss+15,xes,yes+15);
    ea.setLineThick(PFD_SUPP_THICK,PFD_SUPP_THICK);
    ea.drawLine(xss,yss+45,xes,yes+45);
  }
  else {
    ea.setLineThick(1,1);
    ea.drawLine(xss,yss-15,xes,yes-15);
    ea.setLineThick(PFD_SUPP_THICK,PFD_SUPP_THICK);
    ea.drawLine(xss,yss-45,xes,yes-45);
  }
  ea.setLineColor(EA_WHITE,0);
  ea.setLineThick(1,1);
  ea.drawLine(xs,ys,xe,ye);
  ea.setLineColor(EA_GREEN,0);
  ea.drawLine(x-5,y,x+5,y);
  ea.drawLine(x,y-5,x,y+5);
  oxs=xs; oys=ys; oxe=xe; oye=ye;
  oxss=xss; oyss=yss; oxes=xes; oyes=yes;
}

char *ftoa(char *a, double f, int precision)
{
  long p[] = {0,10,100,1000,10000,100000,1000000,10000000,100000000};
  
  char *ret = a;
  long heiltal = (long)f;
  itoa(heiltal, a, 10);
  while (*a != '\0') a++;
  *a++ = '.';
  long desimal = abs((long)((f - heiltal) * p[precision]));
  itoa(desimal, a, 10);
  return ret;
}

int freeRam() {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}



