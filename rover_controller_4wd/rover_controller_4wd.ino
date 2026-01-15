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

// --- ROBOCLAW VE MOTOR AYARLARI (4WD - 2 MOTOR DRIVER) ---
// Motor Driver 1: Sol tekerlekler (Adres 0x80)
//   - M1: Sol ön tekerlek
//   - M2: Sol arka tekerlek
// Motor Driver 2: Sağ tekerlekler (Adres 0x81)
//   - M1: Sağ ön tekerlek
//   - M2: Sağ arka tekerlek
#define RC_LEFT_ADDRESS   0x80   // Sol motor sürücü adresi
#define RC_RIGHT_ADDRESS  0x81   // Sağ motor sürücü adresi
#define RC_BAUDRATE       115200 // Baud rate
#define RX2_PIN           16
#define TX2_PIN           17
#define CMD_VEL_TIMEOUT_MS 500
#define CONNECTED_CHECK_TIMEOUT_MS 1000

// --- ROBOT FİZİKSEL ÖZELLİKLERİ ---
#define WHEEL_DIA        0.192   // Tekerlek Çapı (Metre)
#define TRACK_WIDTH      0.495   // İki teker arası mesafe (Metre)
#define TICKS_PER_REV    751.8   // BİR TURDAKİ TICK SAYISI 145.6 1150 rpm motor için, 751.8 223 rpm motor için

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

// Hız bazlı odometri için değişkenler
bool isMoving = false;

unsigned long last_cmd_vel_time = 0;
unsigned long last_connected_check_time = 0;

// Frame ID stringleri (const-safe)
static char odom_frame[] = "odom";
static char base_link_frame[] = "base_link";


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

// Tüm motorları durdur (4WD güvenlik fonksiyonu)
void stop_all_motors() {
  roboclaw.SpeedM1M2(RC_LEFT_ADDRESS, 0, 0);   // Sol tekerlekler
  roboclaw.SpeedM1M2(RC_RIGHT_ADDRESS, 0, 0);  // Sağ tekerlekler
}

// ==========================================================================
// --- 5. CALLBACKLER VE ODOMETRİ ---
// ==========================================================================

void cmd_vel_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  last_cmd_vel_time = millis();

  float linear_x = msg->linear.x;
  float angular_z = msg->angular.z;

  // Diferansiyel sürüş kinematiği (4WD için aynı formül geçerli)
  float v_left = linear_x - (angular_z * TRACK_WIDTH / 2.0);
  float v_right = linear_x + (angular_z * TRACK_WIDTH / 2.0);

  int32_t qpps_left = (int32_t)(v_left * TICKS_PER_METER);
  int32_t qpps_right = (int32_t)(v_right * TICKS_PER_METER);

  // 4WD Motor Kontrolü:
  // Sol motor sürücü (0x80): M1=sol ön, M2=sol arka - aynı hız
  // Sağ motor sürücü (0x81): M1=sağ ön, M2=sağ arka - aynı hız
  roboclaw.SpeedM1M2(RC_LEFT_ADDRESS, qpps_left, qpps_left);   // Sol ön ve arka
  roboclaw.SpeedM1M2(RC_RIGHT_ADDRESS, qpps_right, qpps_right); // Sağ ön ve arka
}

void publish_odometry_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) last_call_time;
  if (timer != NULL) {
    
    uint8_t status;
    bool valid_spd_left_front, valid_spd_left_rear, valid_spd_right_front, valid_spd_right_rear;

    // --- HIZ VERİLERİNİ OKU (4WD - Her tekerlek için) ---
    // Sol motor sürücü (0x80)
    int32_t spd_left_front = roboclaw.ReadSpeedM1(RC_LEFT_ADDRESS, &status, &valid_spd_left_front);
    int32_t spd_left_rear = roboclaw.ReadSpeedM2(RC_LEFT_ADDRESS, &status, &valid_spd_left_rear);
    
    // Sağ motor sürücü (0x81)
    int32_t spd_right_front = roboclaw.ReadSpeedM1(RC_RIGHT_ADDRESS, &status, &valid_spd_right_front);
    int32_t spd_right_rear = roboclaw.ReadSpeedM2(RC_RIGHT_ADDRESS, &status, &valid_spd_right_rear);
    
    // Hız verisi gelmediyse 0 kabul et
    if(!valid_spd_left_front) spd_left_front = 0; 
    if(!valid_spd_left_rear) spd_left_rear = 0;
    if(!valid_spd_right_front) spd_right_front = 0;
    if(!valid_spd_right_rear) spd_right_rear = 0;
   
    // --- 4WD KİNEMATİK: Her taraf için ortalama hız ---
    int32_t current_spd_left = (spd_left_front + spd_left_rear) / 2;
    int32_t current_spd_right = (spd_right_front + spd_right_rear) / 2;
    
    // Hareket durumu kontrolü
    isMoving = (current_spd_left != 0 || current_spd_right != 0);

    // --- ZAMAN DAMGASI ---
    int64_t time_ns = rmw_uros_epoch_nanos();

    odom_msg.header.stamp.sec = time_ns / 1000000000;
    odom_msg.header.stamp.nanosec = time_ns % 1000000000;

    odom_msg.header.frame_id.data = odom_frame;
    odom_msg.header.frame_id.size = 4;
    odom_msg.header.frame_id.capacity = 5;

    odom_msg.child_frame_id.data = base_link_frame;
    odom_msg.child_frame_id.size = 9;
    odom_msg.child_frame_id.capacity = 10;

    // Pozisyon bilgisi yok (sadece hız bazlı odometri)
    odom_msg.pose.pose.position.x = 0.0;
    odom_msg.pose.pose.position.y = 0.0;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = 0.0;
    odom_msg.pose.pose.orientation.w = 1.0;

    // Hız hesaplama (ortalama değerlerden)
    double v_l_ms = (double)current_spd_left / TICKS_PER_METER;
    double v_r_ms = (double)current_spd_right / TICKS_PER_METER;
    
    odom_msg.twist.twist.linear.x = (v_r_ms + v_l_ms) / 2.0;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = (v_r_ms - v_l_ms) / TRACK_WIDTH;

    // POSE Covariance - Yüksek değerler (pozisyon bilgisi yok)
    odom_msg.pose.covariance[0]  = 9999.0; // X - ölçülmüyor
    odom_msg.pose.covariance[7]  = 9999.0; // Y - ölçülmüyor
    odom_msg.pose.covariance[14] = 9999.0; // Z - ölçülmüyor
    odom_msg.pose.covariance[21] = 9999.0; // Roll - ölçülmüyor
    odom_msg.pose.covariance[28] = 9999.0; // Pitch - ölçülmüyor
    odom_msg.pose.covariance[35] = 9999.0; // Yaw - ölçülmüyor

    // TWIST Covariance
    odom_msg.twist.covariance[0]  = 0.1;    // vx - orta güven
    odom_msg.twist.covariance[7]  = 9999.0; // vy - ölçülmüyor
    odom_msg.twist.covariance[14] = 9999.0; // vz - ölçülmüyor
    odom_msg.twist.covariance[21] = 9999.0; // vroll - ölçülmüyor
    odom_msg.twist.covariance[28] = 9999.0; // vpitch - ölçülmüyor
    odom_msg.twist.covariance[35] = 0.5;    // vyaw - düşük güven

    // --- YAYINLA (20Hz) ---
    rcl_publish(&odom_publisher, &odom_msg, NULL);
  }
}

// ==========================================================================
// --- 6. ENTITY OLUŞTURMA ---
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
  const unsigned int timer_timeout = 50; 
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
  
  // Her iki motor sürücünün encoderlarını sıfırla
  roboclaw.ResetEncoders(RC_LEFT_ADDRESS);
  roboclaw.ResetEncoders(RC_RIGHT_ADDRESS);

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
        last_connected_check_time = millis(); // Zamanlayıcıyı güncelle
        EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
        set_led(0, 255, 0); // Yeşil (Bağlı)
      }

      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        if (millis() - last_cmd_vel_time > CMD_VEL_TIMEOUT_MS) {
             stop_all_motors(); // 4WD için tüm motorları durdur
        }
      }
      break;
      
    case AGENT_DISCONNECTED:
      stop_all_motors(); // Güvenlik: Tüm motorları durdur!
      destroy_entities();
      state = WAITING_AGENT;
      break;
      
    default:
      break;
  }
}