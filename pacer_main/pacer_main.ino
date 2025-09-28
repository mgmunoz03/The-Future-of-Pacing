#include <HardwareSerial.h>
#include "Arduino.h"
#include "Libraries/common/mavlink.h"

#include <Arduino.h>
#include <MicroTFLite.h>
#include "esp_camera.h"
#include "lane_model_80.cc"  // Replace with your model array header

#include <esp_heap_caps.h>

#include <cmath>  // for std::cos and std::sqrt

HardwareSerial SerialMAV(2);  // Use UART2 (pins vary by board)

#define RXD2 3  // GPIO for RX (connect to Pixhawk TX), will be different for Camera vs DevBoard
#define TXD2 1
#define MAV_MODE_FLAG_SAFETY_ARMED 128

  unsigned long lastSend = 0;
  unsigned long interval = 100; // 10 Hz

  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];  //Declare buffer
  uint16_t len;                            //Declare len

// Model input/output size
#define IMG_WIDTH 72
#define IMG_HEIGHT 72
#define NUM_OUTPUTS 4

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22


constexpr int kTensorArenaSize = 500 * 1024;


// TensorArena memory for TFLM
// alignas(16) uint8_t tensor_arena[kTensorArenaSize];
uint8_t* tensor_arena;

// Resize 160x120 grayscale frame to 72x72
void resize_image(uint8_t* src, int sw, int sh, uint8_t* dst, int dw, int dh) {
  for (int y = 0; y < dh; y++) {
    for (int x = 0; x < dw; x++) {
      int sx = x * sw / dw;
      int sy = y * sh / dh;
      dst[y * dw + x] = src[sy * sw + sx];
    }
  }
}

float computeValue(float x1, float x3) {
    float numerator = abs(x1-x3);
    float denominator = std::sqrt(0.4 * 0.4 + (numerator) * (numerator));
    float degree = std::cos(numerator / denominator);
    float radian = (degree * PI)/180 ;
    return radian;
}

void setupCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QQVGA;
  config.fb_count = 1;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;  // avoids allocating 2 frame buffers

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed");
    while (true);
  }
}



void setup() {
  Serial.begin(115200);                             // Set serial communication speed
  SerialMAV.begin(115200, SERIAL_8N1, RXD2, TXD2);  // MAVLink UART
  Serial.println("setup");

  while (!Serial);

  Serial.println("Lane detection with MicroTFLite");
  Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
  Serial.printf("Largest free block: %d bytes\n", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));


  setupCamera();



  tensor_arena = (uint8_t*) heap_caps_malloc(kTensorArenaSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

  if (!tensor_arena) {
    Serial.println("Tensor arena allocation failed!");
    while (true);
  }
  // Initialize the model
  if (!ModelInit(lane_model_80, tensor_arena, kTensorArenaSize)) {
    Serial.println("Model initialization failed!");
    while (true);
  }
  Serial.println("Model initialization done.");

  ModelPrintMetadata();
  ModelPrintInputTensorDimensions();
  ModelPrintOutputTensorDimensions();

  sensor_t * s = esp_camera_sensor_get(); 
  s->set_special_effect(s, 2); //Gray Scale
  s->set_exposure_ctrl(s, 0); //GOOD - Disable auto exposure 
  s->set_aec_value(s, 50); //GOOD - Adjust exposure(higher = longer) ***Need to balance speed/blur with long exposure on black track; 500 relatively blurry; 800 more blurry and brighter
  s->set_agc_gain(s,1); //GOOD - Didn't see much difference; Increase gain for brightness
  s->set_aec2(s, 1); //Helps balance light/dark ares (helps with sunlight)

  Serial.println("Setup complete.");
}

void loop() {

  
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }
  // if (!fb || fb->format != PIXFORMAT_GRAYSCALE) {
  //   Serial.println("Camera capture failed or invalid format");
  //   return;
  // }

  uint8_t resized[IMG_WIDTH * IMG_HEIGHT];
  resize_image(fb->buf, fb->width, fb->height, resized, IMG_WIDTH, IMG_HEIGHT);

  // Copy normalized data to input tensor
  for (int i = 0; i < IMG_WIDTH * IMG_HEIGHT; i++) {
    ModelSetInput(resized[i] / 255.0f, i);
  }

  if (!ModelRunInference()) {
    Serial.println("Inference failed");
    esp_camera_fb_return(fb);
    return;
  }

  Serial.println("Predicted lane points:");
  // for (int i = 0; i < NUM_OUTPUTS; i++) {
  //   float result = ModelGetOutput(i);
  //   Serial.printf("x%d: %.4f\n", i + 1, result);
  //   double newRadianL = computeValue(x1, x3);
  //   double newRadianR
  // }
  float x1 = ModelGetOutput(0);
  float x2 = ModelGetOutput(1);
  float x3 = ModelGetOutput(2);
  float x4 = ModelGetOutput(3);
  float newRadianL = computeValue(x1, x3);
  float newRadianR = computeValue(x2, x4);
  float yawRate = newRadianL - newRadianR;


  esp_camera_fb_return(fb);
  delay(20);


  mavlink_message_t msg;
  mavlink_status_t status;
  // mavlink_msg_request_data_stream_pack(
  //   1, 200,  // sysid, compid of ESP32
  //   &msg,
  //   1, 1,  // target_sys, target_comp (Pixhawk)
  //   10,    // stream ID (see below)
  //   5,     // messages per second
  //   1      // 1 = start, 0 = stop
  // );

  // len = mavlink_msg_to_send_buffer(buffer, &msg);  //pack request message to buffer
  // // Send request message over serial
  // for (int i = 0; i < len; i++) {
  //   SerialMAV.write(buffer[i]);
  // }

  // if (SerialMAV.available()) {  //check for mavlink serial data
  //   uint8_t c = SerialMAV.read();
  //   if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
  //     // Serial.println(msg.msgid);
  //     if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE) {
  //       mavlink_attitude_t att;
  //       mavlink_msg_attitude_decode(&msg, &att);

  //       float yaw_rad = att.yaw;
  //       float yaw_deg = yaw_rad * 180.0 / PI;
  //       if (yaw_deg < 0) yaw_deg += 360;  // Normalize to 0–360 if needed

  //       Serial.print("Yaw (deg): ");
  //       Serial.println(yaw_deg);
  //     }
  //   }
  // }

  float vx = 2.0f;  // forward velocity (m/s)
  float vy = 0.0f;
  float vz = 0.0f;
  float yaw = radians(45);  // Desired yaw in radians (e.g., 90° = east)
  float yaw_rate = yawRate;    // Optional if using fixed yaw
  Serial.println(yawRate);

  // uint16_t type_mask =
  //   (1 << 6) | (1 << 7) | (1 << 8) |  // Ignore acceleration
  //   (1 << 3) | (1 << 4) | (1 << 5);   // Ignore position

  // // Don't ignore yaw (param 10), and DO ignore yaw_rate (param 11)
  // type_mask &= ~(1 << 10);  // Enable yaw
  // type_mask |= (1 << 11);   // Ignore yaw rate

//new type mask
  uint16_t type_mask =
  (1 << 0) | (1 << 1) | (1 << 2) |  // Ignore position
  (1 << 6) | (1 << 7) | (1 << 8) |  // Ignore acceleration
  (1 << 10);                        // Ignore yaw rate (we're setting yaw directly)

  //type_mask &= ~(1 << 10);
// yaw (bit 10) is 0 = enabled

    mavlink_msg_set_position_target_local_ned_pack(
      1, 200, &msg,
      millis(),  // Time boot (ms)
      1, 1,      // Target sys/comp
      MAV_FRAME_BODY_NED,
      type_mask,
      0, 0, 0,     // x, y, z position (ignored)
      vx, vy, vz,  // Velocity
      0, 0, 0,     // Acceleration (ignored)
      yaw, yaw_rate);


    if (millis() - lastSend >= interval) {
    lastSend = millis();

    Serial.println("sending pos");
    len = mavlink_msg_to_send_buffer(buffer, &msg);
    SerialMAV.write(buffer, len); // whole buffer, no loop
    }
    //len = mavlink_msg_to_send_buffer(buffer, &msg);  //pack move message to buffer
    // Send move message over serial
    // for (int i = 0; i < len; i++) {
    //   SerialMAV.write(buffer[i]);
    // }

}
