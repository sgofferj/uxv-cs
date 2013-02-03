
void gcs_handleMessage(mavlink_message_t* msg)
{
  switch (msg->msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
    {
      mavlink_heartbeat_t packet;
      mavlink_msg_heartbeat_decode(msg, &packet);
      beat = 1;
      if ((*msg).sysid != 0xff) { // do not process mission planner heartbeats if we have two receiver xbees
        received_sysid = (*msg).sysid; // save the sysid and compid of the received heartbeat for use in sending new messages
        received_compid = (*msg).compid;
      }
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

void send_message(mavlink_message_t* msg)
{
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
  for(uint16_t i = 0; i < len; i++)
  {
    Serial1.write(buf[i]);
  }
}

void start_feeds()
{
  mavlink_message_t msg;
  mavlink_msg_request_data_stream_pack(127, 0, &msg, received_sysid, received_compid, MAV_DATA_STREAM_RAW_SENSORS, MAV_DATA_STREAM_RAW_SENSORS_RATE, MAV_DATA_STREAM_RAW_SENSORS_ACTIVE);
  send_message(&msg);
  delay(10);
  mavlink_msg_request_data_stream_pack(127, 0, &msg, received_sysid, received_compid, MAV_DATA_STREAM_EXTRA1, MAV_DATA_STREAM_EXTRA1_RATE, MAV_DATA_STREAM_EXTRA1_ACTIVE);
  send_message(&msg);
  delay(10);
  mavlink_msg_request_data_stream_pack(127, 0, &msg, received_sysid, received_compid, MAV_DATA_STREAM_EXTENDED_STATUS, MAV_DATA_STREAM_EXTENDED_STATUS_RATE, MAV_DATA_STREAM_EXTENDED_STATUS_ACTIVE);
  send_message(&msg);
  delay(10);
  mavlink_msg_request_data_stream_pack(127, 0, &msg, received_sysid, received_compid, MAV_DATA_STREAM_RAW_CONTROLLER, MAV_DATA_STREAM_RAW_CONTROLLER_RATE, MAV_DATA_STREAM_RAW_CONTROLLER_ACTIVE);
  send_message(&msg);
  delay(10);
  mavlink_msg_request_data_stream_pack(127, 0, &msg, received_sysid, received_compid, MAV_DATA_STREAM_POSITION, MAV_DATA_STREAM_POSITION_RATE, MAV_DATA_STREAM_POSITION_ACTIVE);
  send_message(&msg);
}

void stop_feeds()
{
  mavlink_message_t msg1;
  mavlink_msg_request_data_stream_pack(127, 0, &msg1, received_sysid, received_compid, MAV_DATA_STREAM_ALL, 0, 0);
  send_message(&msg1);
  delay(500);
}
