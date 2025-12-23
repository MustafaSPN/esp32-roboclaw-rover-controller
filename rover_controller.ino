#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <Adafruit_NeoPixel.h>
#include <RoboClaw.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>

// ==========================================================================
// --- 1. AYARLAR (KONFIGURASYON) ---
// ==========================================================================

// --- ROBOCLAW VE MOTOR AYARLARI ---
#define LEFT_MOTOR_IS_M1 false  // true: Sol M1, Sağ M2 | false: Sol M2, Sağ M1
#define RC_ADDRESS       0x80
#define RC_BAUDRATE      38400
#define RX2_PIN          16
#define TX2_PIN          17
#define CMD_VEL_TIMEOUT_MS 500
#define CONNECTED_CHECK_TIMEOUT_MS 1000
// --- ROBOT FİZİKSEL ÖZELLİKLERİ ---
#define WHEEL_DIA        0.192   // Tekerlek Çapı (Metre)
#define TRACK_WIDTH      0.495    // İki teker arası mesafe (Metre)
#define TICKS_PER_REV    751.8  // BİR TURDAKİ TICK SAYISI 145.6 1150 rpm motor için, 751.8 223 rpm motor için

// Otomatik Hesaplanan Değer (Buna dokunma)
const float TICKS_PER_METER = TICKS_PER_REV / (PI * WHEEL_DIA);

// --- NEOPIXEL AYARLARI ---
#define NEOPIXEL_PIN 48
#define NUM_PIXELS 1
Adafruit_NeoPixel pixels(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// ==========================================================================
// --- 2. MAKROLAR (ISTENEN RECONNECT YAPISI) ---
// ==========================================================================

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = millis();} \
  if (millis() - init > MS) { X; init = millis();} \
} while (0)\

// ==========================================================================
// --- 3. GLOBAL DEĞİŞKENLER ---
// ==========================================================================

RoboClaw roboclaw(&Serial2, 10000); // Timeout artırıldı

rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;

rcl_publisher_t odom_publisher;
rcl_subscription_t subscriber;

nav_msgs__msg__Odometry odom_msg;
geometry_msgs__msg__Twist twist_msg;

// Odometri Durumu
double x_pos = 0.0;
double y_pos = 0.0;
double theta = 0.0;

int32_t last_enc_left = 0;
int32_t last_enc_right = 0;

bool isMoving = false;

unsigned long last_cmd_vel_time = 0;
unsigned long last_connected_check_time = 0;


// Durum Makinesi
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// ==========================================================================
// --- 4. YARDIMCI FONKSİYONLAR ---
// ==========================================================================

void set_led(uint8_t r, uint8_t g, uint8_t b) {
  pixels.setPixelColor(0, pixels.Color(r, g, b));
  pixels.show();
}

void blink_red_neopixel() {
  static unsigned long last_blink = 0;
  static bool led_state = false;
  if (millis() - last_blink > 250) { 
    last_blink = millis();
    led_state = !led_state;
    if (led_state) set_led(255, 0, 0); 
    else set_led(0, 0, 0); 
  }
}

// ==========================================================================
// --- 5. CALLBACKLER VE ODOMETRİ ---
// ==========================================================================

void cmd_vel_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  last_cmd_vel_time = millis();

  float linear_x = msg->linear.x;
  float angular_z = msg->angular.z;

  float v_left = linear_x - (angular_z * TRACK_WIDTH / 2.0);
  float v_right = linear_x + (angular_z * TRACK_WIDTH / 2.0);

  int32_t qpps_left = (int32_t)(v_left * TICKS_PER_METER);
  int32_t qpps_right = (int32_t)(v_right * TICKS_PER_METER);

  // LEFT_MOTOR_IS_M1 ayarına göre motorları sür
  if(!isMoving){
    qpps_left *= 10;
    qpps_right *= 10;
  }
  if (LEFT_MOTOR_IS_M1) {
    roboclaw.SpeedM1M2(RC_ADDRESS, qpps_left, qpps_right);
  } else {
    roboclaw.SpeedM1M2(RC_ADDRESS, qpps_right, qpps_left);
  }
}

void publish_odometry_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) last_call_time;
  if (timer != NULL) {
    
    uint8_t status;
    bool valid_enc_m1, valid_enc_m2, valid_spd_m1, valid_spd_m2;

    // --- 1. SADECE ENCODERLARI OKU (EN KRİTİK VERİ) ---
    int32_t enc_m1 = roboclaw.ReadEncM1(RC_ADDRESS, &status, &valid_enc_m1);
    // Kısa bir nefes aldır (Serial buffer tıkanmasın)
    int32_t enc_m2 = roboclaw.ReadEncM2(RC_ADDRESS, &status, &valid_enc_m2);

    // Eğer Encoderlar geçerliyse, Hızları okumayı dene (Opsiyonel)
    int32_t spd_m1 = 0;
    int32_t spd_m2 = 0;
    
    if (valid_enc_m1 && valid_enc_m2) {
       // Encoderlar sağlamsa hızları da isteyelim
       spd_m1 = roboclaw.ReadSpeedM1(RC_ADDRESS, &status, &valid_spd_m1);
       spd_m2 = roboclaw.ReadSpeedM2(RC_ADDRESS, &status, &valid_spd_m2);
       
       // --- MAPLEME ---
       int32_t current_enc_left, current_enc_right;
       int32_t current_spd_left, current_spd_right;
       
       // Hız verisi gelmediyse (valid false ise) 0 kabul et, ama yayını durdurma!
       if(!valid_spd_m1) spd_m1 = 0; 
       if(!valid_spd_m2) spd_m2 = 0;
      
       if(spd_m1 !=0 || spd_m2 !=0){
        isMoving = true;
       }else{
        isMoving = false;
       }
       if (LEFT_MOTOR_IS_M1) {
          current_enc_left = enc_m1; current_enc_right = enc_m2;
          current_spd_left = spd_m1; current_spd_right = spd_m2;
       } else {
          current_enc_left = enc_m2; current_enc_right = enc_m1;
          current_spd_left = spd_m2; current_spd_right = spd_m1;
       }

       // --- HESAPLAMA ---
       int32_t delta_left = current_enc_left - last_enc_left;
       int32_t delta_right = current_enc_right - last_enc_right;

       // Noise Filter (Ani zıplama koruması)
       if (abs(delta_left) < 20000 && abs(delta_right) < 20000) {
         last_enc_left = current_enc_left;
         last_enc_right = current_enc_right;

         double d_l = (double)delta_left / TICKS_PER_METER;
         double d_r = (double)delta_right / TICKS_PER_METER;

         double d_center = (d_r + d_l) / 2.0;
         double d_theta = (d_r - d_l) / TRACK_WIDTH;

         if (d_center != 0) {
           double temp_theta = theta + d_theta / 2.0;
           x_pos += d_center * cos(temp_theta);
           y_pos += d_center * sin(temp_theta);
         }
         theta += d_theta;
         
         if (theta > PI) theta -= 2 * PI;
         else if (theta < -PI) theta += 2 * PI;

         // --- ZAMAN DAMGASI (OFFSET DÜZELTMELİ) ---
         int64_t time_ns = rmw_uros_epoch_nanos();
         // 600ms geri çekme hilesi (Senkronizasyon için)
         //if(time_ns > 600000000) time_ns -= 600000000; 
         //else time_ns = 0;

         odom_msg.header.stamp.sec = time_ns / 1000000000;
         odom_msg.header.stamp.nanosec = time_ns % 1000000000;

         odom_msg.header.frame_id.data = (char*)"odom";
         odom_msg.header.frame_id.size = 4;
         odom_msg.header.frame_id.capacity = 5;

         odom_msg.child_frame_id.data = (char*)"base_link";
         odom_msg.child_frame_id.size = 9;
         odom_msg.child_frame_id.capacity = 10;

         odom_msg.pose.pose.position.x = x_pos;
         odom_msg.pose.pose.position.y = y_pos;
         odom_msg.pose.pose.orientation.z = sin(theta / 2.0);
         odom_msg.pose.pose.orientation.w = cos(theta / 2.0);

         // Hız verisi her zaman basılır (Okunamazsa 0 gider, ama ODOM AKMAYA DEVAM EDER)
         double v_l_ms = (double)current_spd_left / TICKS_PER_METER;
         double v_r_ms = (double)current_spd_right / TICKS_PER_METER;
         
         odom_msg.twist.twist.linear.x = (v_r_ms + v_l_ms) / 2.0;
         odom_msg.twist.twist.linear.y = 0.0;
         odom_msg.twist.twist.angular.z = (v_r_ms - v_l_ms) / TRACK_WIDTH;
        // POSE: 9999 hack'ini kaldır. (Pose'u zaten EKF'de fuse etmeyeceksin.)
          odom_msg.pose.covariance[0]  = 0.0; // X
          odom_msg.pose.covariance[7]  = 0.0; // Y
          odom_msg.pose.covariance[14] = 0.0; // Z
          odom_msg.pose.covariance[21] = 0.0; // Roll
          odom_msg.pose.covariance[28] = 0.0; // Pitch
          odom_msg.pose.covariance[35] = 0.0; // Yaw

          // TWIST: sadece güvendiklerin küçük; vy'yi küçük yapma (fuse etmeyeceğiz)
          odom_msg.twist.covariance[0]  = 0.0025; // vx
          odom_msg.twist.covariance[7]  = 9999.0; // vy  (0 basıyoruz ama ÖLÇMÜYORUZ)
          odom_msg.twist.covariance[35] = 0.0025; // vyaw

          // Ölçmediklerimiz
          odom_msg.twist.covariance[14] = 9999.0; // vz
          odom_msg.twist.covariance[21] = 9999.0; // vroll
          odom_msg.twist.covariance[28] = 9999.0; // vpitch
         // --- YAYINLA ---
         rcl_publish(&odom_publisher, &odom_msg, NULL);
       }
    }
  }
}

// ==========================================================================
// --- 6. ENTITY OLUŞTURMA (SENİN YAPIN) ---
// ==========================================================================

bool create_entities(){
  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "esp32_rover", "", &support));

  // create publisher (Odom)
  RCCHECK(rclc_publisher_init_default(
    &odom_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "odom_esp"));

  // create subscriber (Cmd_Vel)
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel_out"));

  // create timer (Odometri Yayını için - 50ms = 20Hz)
  const unsigned int timer_timeout = 10; 
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    publish_odometry_callback));

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator)); // Handle sayısı 2 oldu (Sub + Timer)
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &twist_msg, &cmd_vel_callback, ON_NEW_DATA));

  // --- ZAMAN SENKRONİZASYONU ---
  // Sadece entity oluşturulurken (bağlantı başında) 1 kez yapılır.
  rmw_uros_sync_session(1000);

  return true;
}

void destroy_entities(){
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&odom_publisher, &node);
  rcl_subscription_fini(&subscriber, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// ==========================================================================
// --- 7. SETUP ---
// ==========================================================================

void setup() {
  set_microros_transports();
  
  pixels.begin();
  pixels.setBrightness(30);
  set_led(0, 0, 0);

  Serial2.begin(RC_BAUDRATE, SERIAL_8N1, RX2_PIN, TX2_PIN);
  roboclaw.begin(RC_BAUDRATE);
  roboclaw.ResetEncoders(RC_ADDRESS);

  state = WAITING_AGENT;
}

// ==========================================================================
// --- 8. LOOP (İSTENEN RECONNECT LOGIC) ---
// ==========================================================================

void loop() {
  switch (state) {
    case WAITING_AGENT:
      blink_red_neopixel(); // Görsel geri bildirim
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
      
    case AGENT_AVAILABLE:
      set_led(0, 0, 255); // Mavi (Kuruluyor)
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
      
    case AGENT_CONNECTED:
      
      if(millis() - last_connected_check_time > CONNECTED_CHECK_TIMEOUT_MS){
        EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
        set_led(0, 255, 0); // Yeşil (Bağlı)
      }

      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        if (millis() - last_cmd_vel_time > CMD_VEL_TIMEOUT_MS) {
             roboclaw.SpeedM1M2(RC_ADDRESS, 0, 0);
        }
      }
      break;
      
    case AGENT_DISCONNECTED:
      roboclaw.SpeedM1M2(RC_ADDRESS, 0, 0); // Güvenlik: Dur!
      destroy_entities();
      state = WAITING_AGENT;
      break;
      
    default:
      break;
  }
}