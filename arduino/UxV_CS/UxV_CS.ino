//#include <progmem.h>
#include <EEPROM.h>
#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <GCS_MAVLink.h>

#include <EDIPTFT.h>

#define _VERSION "V0.52"

const char *GPSFIX[] = {" NOFIX "," NOFIX "," 2DFIX "," 3DFIX "}; // Labels for GPS-status in status bar
const char *TKLABEL[] = {"OVRV","FRSKY","PFD","MOD4","MOD5","TEST","SYST"}; // Touchkey labels

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

// APM custom modes in heartbeat package
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
#define UI_BUTTON_ACTIVE EA_MINT,EA_BLACK,EA_MINT,EA_YELLOW,EA_BLACK,EA_YELLOW // Colors of touchbutton when page selected
#define UI_BUTTON_INACTIVE EA_WHITE,EA_BLACK,EA_WHITE,EA_YELLOW,EA_BLACK,EA_YELLOW // Colors of touchbutton when page not selected
#define PFD_SUPP_THICK 4 // Thickness of +/- 45° pitch indicator lines in PFD attitude indicator (pixels)

// EEPROM storage
#define EE_BACKLIGHT 0 // Backlight level
#define EE_GCS_MODE 1  // GCS display mode = page which is displayed
#define EE_GCS_UNITS 2 // Display units, 0=metric, 1=imperial land, 2=imperial nautic

// Internal stuff
#define FAILCNT_MAX_FRSKY 20 // After how many cycles without receiving no FrSky packet do we assume lost connection?

// Stuff taken over from ArduStation - do we need this?
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
int beat=0;

// Flight data
int32_t longitude=0; // GPS lon
int32_t latitude=0;  // GPS lat
float pitch=0;       // Pitch angle
float roll=0;        // Roll angle
float yaw=0;         // Yaw angle
long altitude=0;     // Altitude ( --> Find out if GPS or baro!)
unsigned int ias=0;  // Indicated air speed
unsigned int grs=0;  // Ground speed
int heading=0;       // Heading (not course over ground!)
unsigned int cog=0;  // Course over ground (from GPS)
int vsi=0;           // Vertical speed

// Technical data
uint8_t received_sysid=0;   // ID of heartbeat sender
uint8_t received_compid=0;  // component id of heartbeat sender
int bmode=0;                // MAVLink basic mode
int cmode=0;                // MAVLink custom mode
int gpsfix=0;               // GPS fix status, 0 = no fix, 1 = dead reckoning, 2 = 2D-fix, 3 = 3D-fix
int numSats=0;              // Number of satellites used in position fix
unsigned int vbat=0;        // battery voltage
unsigned long mav_utime=0;  // ??
uint8_t frsky_rx_a1=0;      // FrSky receiver voltage
uint8_t frsky_rx_a2=0;      // FrSky receiver analog input 2
uint8_t frsky_link_up=0;    // FrSky link quality TX -> RX
uint8_t frsky_link_dn=0;    // FrSky link quality RX -> TX

// Ground station stuff
int status_mavlink=0; // Changes to 1 when a valid MAVLink package was received, 0 when no package or an invalid package was received
int status_frsky=0;   // Changes to 1 when a valid FrSky package was received, 0 when no package or an invalid package was received
int failcnt_frsky=0;  // Failure counter - counts the cycles in which no valid FrSky packet was received
int GCS_MODE=0;       // GCS display mode = page which is displayed
int GCS_UNITS=0;      // Display units, 0=metric, 1=imperial land, 2=imperial nautic

void setup() {
  Serial.begin(57600);
  Serial1.begin(57600,256,16);
  Serial2.begin(9600,256,16);
  Serial3.begin(115200,256,256);
  ea.smallProtoSelect(7);
  ea.clear();
  ea.cursor(false);
  
  GCS_MODE = EEPROM.read(EE_GCS_MODE);
  GCS_UNITS = EEPROM.read(EE_GCS_UNITS);
  
  drawSplash();
  delay(2000);
  
  ea.defineTouchKey(445,3,479,14,0,10," "); // define touchkey for unit change
  setMode(GCS_MODE);
}

void loop() {
  if (gcs_update()) {                  // Only update screen when a valid MAVLink package was received
    switch (GCS_MODE) {
      case 1 : { drawOVRV(); break; }
      case 2 : { drawFRSKY(); break; }
      case 3 : { drawPFD(); break; }
    }
    drawStatusbar();
  }
  if ((status_mavlink == 1) && (!gcs_update())) drawStatusbar(); // When no valid MAVLink package was received but the status still shows 1,
                                                                 // update status bar, so the indicator goes red.
  if (ea.datainBuffer() >0) parseSerial();                       // If the display has data for us, go and get it
}

void parseSerial() {
  char i;
  char len = ea.datainBuffer();
  char data[len];
  ea.readBuffer(data);
  if (data[0] == 27) {
    if ((data[1] == 'A') && (data[2] == 1)) {
      switch (data[3]) {
        case 1 : { setMode(1); break; }
        case 2 : { setMode(2); break; }
        case 3 : { setMode(3); break; }
        case 4 : { setMode(4); break; }
        case 5 : { setMode(5); break; }
        case 6 : { setMode(6); break; }
        case 7 : { setMode(7); break; }
        case 10 : {
          if (GCS_UNITS < 2) GCS_UNITS ++;
          else GCS_UNITS = 0;
          EEPROM.write(EE_GCS_UNITS,GCS_UNITS);
          break;
        }
      }
    }
  }
}

boolean gcs_update()
{
    boolean result = false;
    status_mavlink=0;
    status_frsky=0;
    mavlink_message_t msg;
    uint8_t frsky_msg [11];
    mavlink_status_t status;

    while (Serial1.available())
    {
      uint8_t c = Serial1.read();
      if(mavlink_parse_char(0, c, &msg, &status)) {
        gcs_handleMessage(&msg);
        status_mavlink = 1;
        result=true;
      }
    }
    if (!Serial2.available()) {
      if (failcnt_frsky < 32000) failcnt_frsky++;
      else failcnt_frsky = FAILCNT_MAX_FRSKY;
    }
    while (Serial2.available())
    {
      uint8_t c = Serial2.read();
      if (FRSKY_parse_char(c,frsky_msg)) {
        FRSKY_handle_message(frsky_msg);
        failcnt_frsky=0;
        status_frsky=1;
        result=true;
      }
      else {
        if (failcnt_frsky < 32000) failcnt_frsky++;
        else failcnt_frsky = FAILCNT_MAX_FRSKY;
      }
    }
    return result;
}

void setMode(int mode) {
  switch (GCS_MODE) {                     // First, check the old mode and destroy display objects if necessary
    case 3 : { destroyPFD(); break; }
    case 7 : { destroySystem(); break; }
  }
  
  GCS_MODE = mode;                        // Then, set the mode
  EEPROM.write(EE_GCS_MODE,GCS_MODE);     // write the mode to the EEPROM
  ea.clear();                             // clear the display

  drawButtons();                          // draw the touchkeys

  switch (mode) {                         // Finally, if the new mode has some display objects to initialize, do it
    case 1 : { initOVRV(); break; }
    case 2 : { initFRSKY(); break; }
    case 3 : { initPFD(); break; }
    case 7 : { initSystem(); break; }
  }
}    

void drawStatusbar() {
  char buf[8];
  ea.setTextFont(3);
  
  // Motor arm status
  if ((bmode & MAV_MODE_FLAG_SAFETY_ARMED) == MAV_MODE_FLAG_SAFETY_ARMED) {
    ea.setTextColor(EA_BLACK,EA_RED);
    ea.drawText(24,3,'C',"  ARM  ");
  }
  else {
    ea.setTextColor(EA_BLACK,EA_GREEN);
    ea.drawText(24,3,'C'," D-ARM ");
  }
  
  // MAVLink basic mode
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

  // MAVLink custom mode
  switch (cmode) {
    case CUST_MODE_MANUAL :        { ea.setTextColor(EA_BLACK,EA_GREEN);     ea.drawText(116,3,'C'," MANU "); break; }
    case CUST_MODE_CIRCLE :        { ea.setTextColor(EA_BLACK,EA_RED);       ea.drawText(116,3,'C'," CIRC "); break; }
    case CUST_MODE_STABILIZE :     { ea.setTextColor(EA_BLACK,EA_LIGHTBLUE); ea.drawText(116,3,'C'," STAB "); break; }
    case CUST_MODE_TRAINING :      { ea.setTextColor(EA_BLACK,EA_PURPLE);    ea.drawText(116,3,'C'," TRAI "); break; }
    case CUST_MODE_FLY_BY_WIRE_A : { ea.setTextColor(EA_BLACK,EA_BLUE);      ea.drawText(116,3,'C'," FBWA "); break; }
    case CUST_MODE_FLY_BY_WIRE_B : { ea.setTextColor(EA_BLACK,EA_BLUE);      ea.drawText(116,3,'C'," FBWB "); break; }
    case CUST_MODE_AUTO          : { ea.setTextColor(EA_BLACK,EA_RED);       ea.drawText(116,3,'C'," AUTO "); break; }
    case CUST_MODE_RTL :           { ea.setTextColor(EA_BLACK,EA_YELLOW);    ea.drawText(116,3,'C'," RTL  "); break; }
    case CUST_MODE_LOITER :        { ea.setTextColor(EA_BLACK,EA_YELLOW);    ea.drawText(116,3,'C'," LOIT "); break; }
    case CUST_MODE_GUIDED :        { ea.setTextColor(EA_BLACK,EA_YELLOW);    ea.drawText(116,3,'C'," GUID "); break; }
    case CUST_MODE_INITIALISING :  { ea.setTextColor(EA_BLACK,EA_CYAN);      ea.drawText(116,3,'C'," INIT "); break; }
  }

  if (failcnt_frsky < FAILCNT_MAX_FRSKY) {
    drawRSSIm(220,2,frsky_link_up);
    drawRSSI(243,2,frsky_link_dn);
  }
  else {
    drawRSSIm(220,2,0);
    drawRSSI(243,2,0);
  }

  // FrSky package indicator
  if (status_frsky == 1) ea.setTextColor(EA_BLACK,EA_GREEN);
  else ea.setTextColor(EA_BLACK,EA_RED);
  ea.drawText(317,3,'R'," FRSKY ");

  // MAVLink package indicator
  if (status_mavlink == 1) ea.setTextColor(EA_BLACK,EA_GREEN);
  else ea.setTextColor(EA_BLACK,EA_RED);
  ea.drawText(361,3,'R'," MAVL ");
  
  // GPS status indicator
  switch (gpsfix) {
    case 0 : { ea.setTextColor(EA_BLACK,EA_RED); break; }
    case 1 : { ea.setTextColor(EA_BLACK,EA_RED); break; }
    case 2 : { ea.setTextColor(EA_BLACK,EA_YELLOW); break; }
    case 3 : { ea.setTextColor(EA_BLACK,EA_GREEN); break; }
  }    
  ea.drawText(412,3,'R',(char*)GPSFIX[gpsfix]);

  // GPS number of satellites
  if (numSats < 5) ea.setTextColor(EA_BLACK,EA_RED);
  else if (numSats < 8) ea.setTextColor(EA_BLACK,EA_YELLOW);
  else if (numSats >= 8) ea.setTextColor(EA_BLACK,EA_GREEN);
  sprintf(buf," %02d ",numSats);
  ea.drawText(442,3,'R',buf);
  
  // Display units
  ea.setTextColor(EA_BLACK,EA_WHITE);
  switch (GCS_UNITS) {
    case 0 : { ea.drawText(479,3,'R'," MET "); break; }
    case 1 : { ea.drawText(479,3,'R'," IMP "); break; }
    case 2 : { ea.drawText(479,3,'R'," NAU "); break; }
  }    

  // Separator
  ea.setLineColor(EA_WHITE,1);
  ea.drawLine(0,18,480,18);
}

void drawButtons() {
  // First, destroy the old touchkeys - even is the display was cleared, the touchkey definitions are still valid
  ea.removeTouchArea(1,1);
  ea.removeTouchArea(2,1);
  ea.removeTouchArea(3,1);
  ea.removeTouchArea(4,1);
  ea.removeTouchArea(5,1);
  ea.removeTouchArea(6,1);
  ea.removeTouchArea(7,1);
  
  // Text color and font
  ea.setTouchkeyLabelColors(EA_BLACK,EA_BLACK);
  ea.setTouchkeyFont(5);
  
  // Define and draw new touchkeys
  if (GCS_MODE==1) ea.setTouchkeyColors(UI_BUTTON_ACTIVE);
  else ea.setTouchkeyColors(UI_BUTTON_INACTIVE);
  ea.defineTouchKey(  0,248, 60,272,0,1,(char*)TKLABEL[0]);
  if (GCS_MODE==2) ea.setTouchkeyColors(UI_BUTTON_ACTIVE);
  else ea.setTouchkeyColors(UI_BUTTON_INACTIVE);
  ea.defineTouchKey( 70,248,130,272,0,2,(char*)TKLABEL[1]);
  if (GCS_MODE==3) ea.setTouchkeyColors(UI_BUTTON_ACTIVE);
  else ea.setTouchkeyColors(UI_BUTTON_INACTIVE);
  ea.defineTouchKey(140,248,200,272,0,3,(char*)TKLABEL[2]);
  if (GCS_MODE==4) ea.setTouchkeyColors(UI_BUTTON_ACTIVE);
  else ea.setTouchkeyColors(UI_BUTTON_INACTIVE);
  ea.defineTouchKey(210,248,270,272,0,4,(char*)TKLABEL[3]);
  if (GCS_MODE==5) ea.setTouchkeyColors(UI_BUTTON_ACTIVE);
  else ea.setTouchkeyColors(UI_BUTTON_INACTIVE);
  ea.defineTouchKey(280,248,340,272,0,5,(char*)TKLABEL[4]);
  if (GCS_MODE==6) ea.setTouchkeyColors(UI_BUTTON_ACTIVE);
  else ea.setTouchkeyColors(UI_BUTTON_INACTIVE);
  ea.defineTouchKey(350,248,410,272,0,6,(char*)TKLABEL[5]);
  if (GCS_MODE==7) ea.setTouchkeyColors(UI_BUTTON_ACTIVE);
  else ea.setTouchkeyColors(UI_BUTTON_INACTIVE);
  ea.defineTouchKey(420,248,480,272,0,7,(char*)TKLABEL[6]);
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
  char buf2[16];
  ea.setTextFont(10);
  ea.setTextColor(EA_WHITE,EA_BLACK);
  if (gpsfix > 1) {
    if (latitude < 0) {
      dtostrf(-1*latitude/1e7,2,6,buf2);
      strcpy(buf,"S ");
      strcat(buf,buf2);
    }
    else {
      dtostrf(latitude/1e7,2,6,buf2);
      strcpy(buf,"N ");
      strcat(buf,buf2);
    }
    ea.drawText(125,36,'C',buf);
    if (longitude < 0) {
      dtostrf(-1*longitude/1e7,2,6,buf2);
      strcpy(buf,"W ");
      strcat(buf,buf2);
    }
    else {
      dtostrf(longitude/1e7,2,6,buf2);
      strcpy(buf,"E ");
      strcat(buf,buf2);
    }
    ea.drawText(365,36,'C',buf);
    sprintf(buf,"%3d",cog/100);
    ea.drawText(105,86,'R',buf);
    sprintf(buf,"%5d",altitude);
    if (altitude < 60000) ea.drawText(225,86,'R',buf);
    sprintf(buf,"%3d",grs);
    ea.drawText(465,86,'R',buf);
  }
  else {
    ea.drawText(125,36,'C'," --.------ ");
    ea.drawText(365,36,'C'," --.------ ");
    ea.drawText(105,86,'R',"   ---");
    ea.drawText(225,86,'R'," -----");
    ea.drawText(465,86,'R',"   ---");
  }
  ea.setTextColor(EA_WHITE,EA_BLACK);
  sprintf(buf,"%3d",ias);
  ea.drawText(345,86,'R',buf);
  
  dtostrf(vbat/1000.0,2,2,buf);
  ea.drawText(105,136,'R',buf);
}

void initFRSKY() {
  ea.setLineColor(EA_WHITE,0);
  ea.setLineThick(1,1);
  ea.drawLine(0,68,480,68);
  ea.drawLine(0,118,480,118);
  ea.drawLine(0,168,480,168);
  ea.drawLine(0,218,480,218);
  ea.drawLine(240,18,240,218);
  ea.drawLine(120,18,120,218);
  ea.drawLine(360,18,360,218);
  ea.setTextFont(9);
  ea.setTextColor(EA_WHITE,EA_BLACK);
  ea.drawText(10,20,'L',"RC RSSI");
  ea.drawText(130,20,'L',"TELEM RSSI");
  ea.drawText(250,20,'L',"RX V");
  ea.drawText(370,20,'L',"A2 V");
}

void drawFRSKY() {
  char buf[16];
  char buf2[16];
  ea.setTextFont(10);
  ea.setTextColor(EA_WHITE,EA_BLACK);
  if(failcnt_frsky < FAILCNT_MAX_FRSKY) {
    sprintf(buf,"%6d",frsky_link_up);
    ea.drawText(105,36,'R',buf);
    sprintf(buf,"%6d",frsky_link_dn/2);
    ea.drawText(225,36,'R',buf);
    dtostrf(frsky_rx_a1*0.0517647058824,6,2,buf);
    ea.drawText(345,36,'R',buf);
    dtostrf(frsky_rx_a2*0.0129411764706,6,2,buf);
    ea.drawText(465,36,'R',buf);
  }
  else {
    ea.drawText(105,36,'R',"---");
    ea.drawText(225,36,'R',"---");
    ea.drawText(345,36,'R',"------");
    ea.drawText(465,36,'R',"------");
  }
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
  char buf[8];
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
  char buf[32];
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

void drawRSSI (int x, int y, uint8_t rssi) {
  if (rssi >= 80) {
    ea.setLineThick(4,1);
    ea.setLineColor(EA_GREEN,0);
    ea.drawLine(x   ,y+12,x   ,y+11);
    ea.drawLine(x+4 ,y+12,x+4 ,y+9);
    ea.drawLine(x+8 ,y+12,x+8 ,y+6);
    ea.drawLine(x+12,y+12,x+12,y+3);
    ea.drawLine(x+16,y+12,x+16,y);
  }
  else if (rssi >= 60) {
    ea.setLineThick(4,1);
    ea.setLineColor(EA_GREEN,0);
    ea.drawLine(x   ,y+12,x   ,y+11);
    ea.drawLine(x+4 ,y+12,x+4 ,y+9);
    ea.drawLine(x+8 ,y+12,x+8 ,y+6);
    ea.drawLine(x+12,y+12,x+12,y+3);
    ea.setLineColor(EA_BLACK,0);
    ea.drawLine(x+16,y+12,x+16,y);
  }
  else if (rssi >= 40) {
    ea.setLineThick(4,1);
    ea.setLineColor(EA_YELLOW,0);
    ea.drawLine(x   ,y+12,x   ,y+11);
    ea.drawLine(x+4 ,y+12,x+4 ,y+9);
    ea.drawLine(x+8 ,y+12,x+8 ,y+6);
    ea.setLineColor(EA_BLACK,0);
    ea.drawLine(x+12,y+12,x+12,y+3);
    ea.drawLine(x+16,y+12,x+16,y);
  }
  else if (rssi >= 20) {
    ea.setLineThick(4,1);
    ea.setLineColor(EA_YELLOW,0);
    ea.drawLine(x   ,y+12,x   ,y+11);
    ea.drawLine(x+4 ,y+12,x+4 ,y+9);
    ea.setLineColor(EA_BLACK,0);
    ea.drawLine(x+8 ,y+12,x+8 ,y+6);
    ea.drawLine(x+12,y+12,x+12,y+3);
    ea.drawLine(x+16,y+12,x+16,y);
  }
  else if (rssi >= 0) {
    ea.setLineThick(4,1);
    ea.setLineColor(EA_RED,0);
    ea.drawLine(x   ,y+12,x   ,y+11);
    ea.setLineColor(EA_BLACK,0);
    ea.drawLine(x+4 ,y+12,x+4 ,y+9);
    ea.drawLine(x+8 ,y+12,x+8 ,y+6);
    ea.drawLine(x+12,y+12,x+12,y+3);
    ea.drawLine(x+16,y+12,x+16,y);
  }
}

void drawRSSIm (int x, int y, uint8_t rssi) {
  if (rssi >= 80) {
    ea.setLineThick(4,1);
    ea.setLineColor(EA_GREEN,0);
    ea.drawLine(x   ,y+12,x   ,y);
    ea.drawLine(x+4 ,y+12,x+4 ,y+3);
    ea.drawLine(x+8 ,y+12,x+8 ,y+6);
    ea.drawLine(x+12,y+12,x+12,y+9);
    ea.drawLine(x+16,y+12,x+16,y+11);
  }
  else if (rssi >= 60) {
    ea.setLineThick(4,1);
    ea.setLineColor(EA_BLACK,0);
    ea.drawLine(x   ,y+12,x   ,y);
    ea.setLineColor(EA_GREEN,0);
    ea.drawLine(x+4 ,y+12,x+4 ,y+3);
    ea.drawLine(x+8 ,y+12,x+8 ,y+6);
    ea.drawLine(x+12,y+12,x+12,y+9);
    ea.drawLine(x+16,y+12,x+16,y+11);
  }
  else if (rssi >= 40) {
    ea.setLineThick(4,1);
    ea.setLineColor(EA_BLACK,0);
    ea.drawLine(x   ,y+12,x   ,y);
    ea.drawLine(x+4 ,y+12,x+4 ,y+3);
    ea.setLineColor(EA_YELLOW,0);
    ea.drawLine(x+8 ,y+12,x+8 ,y+6);
    ea.drawLine(x+12,y+12,x+12,y+9);
    ea.drawLine(x+16,y+12,x+16,y+11);
  }
  else if (rssi >= 20) {
    ea.setLineThick(4,1);
    ea.setLineColor(EA_BLACK,0);
    ea.drawLine(x   ,y+12,x   ,y);
    ea.drawLine(x+4 ,y+12,x+4 ,y+3);
    ea.drawLine(x+8 ,y+12,x+8 ,y+6);
    ea.setLineColor(EA_YELLOW,0);
    ea.drawLine(x+12,y+12,x+12,y+9);
    ea.drawLine(x+16,y+12,x+16,y+11);
  }
  else if (rssi >= 0) {
    ea.setLineThick(4,1);
    ea.setLineColor(EA_BLACK,0);
    ea.drawLine(x   ,y+12,x   ,y);
    ea.drawLine(x+4 ,y+12,x+4 ,y+3);
    ea.drawLine(x+8 ,y+12,x+8 ,y+6);
    ea.drawLine(x+12,y+12,x+12,y+9);
    ea.setLineColor(EA_RED,0);
    ea.drawLine(x+16,y+12,x+16,y+11);
  }
}

int freeRam() {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}



