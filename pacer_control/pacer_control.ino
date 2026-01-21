#
# Control scripts in a working state. Previously titled "YawAndForward.ino".
#
#include <HardwareSerial.h>
#include "Arduino.h"
#include "Libraries/common/mavlink.h"

HardwareSerial SerialMAV(2);  // Use UART2 (pins vary by board)

#define RXD2 9  // GPIO for RX (connect to Pixhawk TX), will be different for Camera vs DevBoard
#define TXD2 10
#define MAV_MODE_FLAG_SAFETY_ARMED 128

void setup() {
  Serial.begin(115200);                             // Set serial communication speed
  SerialMAV.begin(115200, SERIAL_8N1, RXD2, TXD2);  // MAVLink UART
  Serial.println("setup");
}

void loop() {
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];  //Declare buffer
  uint16_t len;                            //Declare len

  mavlink_message_t msg;
  mavlink_status_t status;
  mavlink_msg_request_data_stream_pack(
    1, 200,  // sysid, compid of ESP32
    &msg,
    1, 1,  // target_sys, target_comp (Pixhawk)
    10,    // stream ID (see below)
    5,     // messages per second
    1      // 1 = start, 0 = stop
  );

  len = mavlink_msg_to_send_buffer(buffer, &msg);  //pack request message to buffer
  // Send request message over serial
  for (int i = 0; i < len; i++) {
    SerialMAV.write(buffer[i]);
  }

  if (SerialMAV.available()) {  //check for mavlink serial data
    uint8_t c = SerialMAV.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      // Serial.println(msg.msgid);
      if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE) {
        mavlink_attitude_t att;
        mavlink_msg_attitude_decode(&msg, &att);

        float yaw_rad = att.yaw;
        float yaw_deg = yaw_rad * 180.0 / PI;
        if (yaw_deg < 0) yaw_deg += 360;  // Normalize to 0–360 if needed

        Serial.print("Yaw (deg): ");
        Serial.println(yaw_deg);
      }
    }
  }

  float vx = 2.0f;  // forward velocity (m/s)
  float vy = 0.0f;
  float vz = 0.0f;
  float yaw = radians(360);  // Desired yaw in radians (e.g., 90° = east)
  float yaw_rate = 0.0f;    // Optional if using fixed yaw

  uint16_t type_mask =
    (1 << 6) | (1 << 7) | (1 << 8) |  // Ignore acceleration
    (1 << 3) | (1 << 4) | (1 << 5);   // Ignore position

  // Don't ignore yaw (param 10), and DO ignore yaw_rate (param 11)
  type_mask &= ~(1 << 10);  // Enable yaw
  type_mask |= (1 << 11);   // Ignore yaw rate

  if (SerialMAV.available()) {  //check for mavlink serial data
    mavlink_msg_set_position_target_local_ned_pack(
      1, 200, &msg,
      millis(),  // Time boot (ms)
      1, 1,      // Target sys/comp
      MAV_FRAME_LOCAL_NED,
      type_mask,
      0, 0, 0,     // x, y, z position (ignored)
      vx, vy, vz,  // Velocity
      0, 0, 0,     // Acceleration (ignored)
      yaw, yaw_rate);


    unsigned long lastSend = 0;
    unsigned long interval = 500; // 10 Hz

    if (millis() - lastSend >= interval) {
    lastSend = millis();

    len = mavlink_msg_to_send_buffer(buffer, &msg);
    SerialMAV.write(buffer, len); // whole buffer, no loop
    }
    //len = mavlink_msg_to_send_buffer(buffer, &msg);  //pack move message to buffer
    // Send move message over serial
    // for (int i = 0; i < len; i++) {
    //   SerialMAV.write(buffer[i]);
    // }
  }
}
