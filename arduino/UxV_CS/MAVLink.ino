
void gcs_handleMessage(mavlink_message_t* msg)
{
  switch (msg->msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
    {
      beat = 1;
      break;
    }
   case MAVLINK_MSG_ID_ATTITUDE:
    {
      // decode
      mavlink_attitude_t packet;
      mavlink_msg_attitude_decode(msg, &packet);
      pitch = toDeg(packet.pitch);
      yaw = toDeg(packet.yaw);
      roll = toDeg(packet.roll);
      break;
    }
    case MAVLINK_MSG_ID_GPS_RAW_INT:
    {
      // decode
      mavlink_gps_raw_int_t packet;
      mavlink_msg_gps_raw_int_decode(msg, &packet);
      latitude = packet.lat;
      longitude = packet.lon;
      velocity = packet.vel;
      altitude = packet.alt;
      gpsfix = packet.fix_type;
      mav_utime = packet.time_usec;
      break;
    }
    case MAVLINK_MSG_ID_GPS_STATUS:
    {
      mavlink_gps_status_t packet;
      mavlink_msg_gps_status_decode(msg, &packet);        
      numSats = packet.satellites_visible;
      break;
    }
    case MAVLINK_MSG_ID_RAW_PRESSURE:
    {
      // decode
      mavlink_raw_pressure_t packet;
      mavlink_msg_raw_pressure_decode(msg, &packet);
      break;
    }
    case MAVLINK_MSG_ID_SYS_STATUS:
    {

      mavlink_sys_status_t packet;
      mavlink_msg_sys_status_decode(msg, &packet);
      battery = packet.voltage_battery;
      break;
    }
  }
}

void gcs_update()
{
    connstat=0;
    // receive new packets
    mavlink_message_t msg;
    mavlink_status_t status;

    // process received bytes
    while(Serial1.available())
    {
        uint8_t c = Serial1.read();
        // Try to get a new message
        if(mavlink_parse_char(0, c, &msg, &status)) {
          gcs_handleMessage(&msg);
          connstat = 1;
        }
    }
}


