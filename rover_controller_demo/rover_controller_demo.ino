#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <RoboClaw.h>
#include <Adafruit_NeoPixel.h>

// ==========================================================================
// --- WIFI AYARLARI ---
// ==========================================================================
const char* ssid = "Asli's Home";      // WiFi ağ adını girin
const char* password = "Korkmaz2002416"; // WiFi şifresini girin

// ==========================================================================
// --- ROBOCLAW AYARLARI (4WD - 2 MOTOR DRIVER) ---
// ==========================================================================
#define RC_LEFT_ADDRESS   0x80   // Sol motor sürücü adresi
#define RC_RIGHT_ADDRESS  0x81   // Sağ motor sürücü adresi
#define RC_BAUDRATE       115200
#define RX2_PIN           16
#define TX2_PIN           17

// --- ROBOT FİZİKSEL ÖZELLİKLERİ ---
#define WHEEL_DIA        0.192   // Tekerlek Çapı (Metre)
#define TRACK_WIDTH      0.495   // İki teker arası mesafe (Metre)
#define TICKS_PER_REV    751.8   // Encoder tick sayısı

const float TICKS_PER_METER = TICKS_PER_REV / (PI * WHEEL_DIA);

// --- DEMO HAREKET PARAMETRELERİ ---
#define LINEAR_SPEED     0.4     // m/s
#define LINEAR_DISTANCE  1.0     // metre
#define TURN_SPEED_DEG   30.0    // derece/saniye
#define TURN_ANGLE       90.0    // derece

// Dönüş hızını rad/s'ye çevir
const float TURN_SPEED_RAD = TURN_SPEED_DEG * PI / 180.0;

// --- NEOPIXEL ---
#define NEOPIXEL_PIN 48
#define NUM_PIXELS 1
Adafruit_NeoPixel pixels(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// ==========================================================================
// --- GLOBAL DEĞİŞKENLER ---
// ==========================================================================

RoboClaw roboclaw(&Serial2, 10000);
WebServer server(80);

bool isMoving = false;
unsigned long moveStartTime = 0;
unsigned long moveDuration = 0;
int32_t targetSpeedLeft = 0;
int32_t targetSpeedRight = 0;

// ==========================================================================
// --- HTML SAYFA ---
// ==========================================================================

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Rover Demo Kontrol</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
      background: linear-gradient(135deg, #1a1a2e 0%, #16213e 50%, #0f3460 100%);
      min-height: 100vh;
      display: flex;
      justify-content: center;
      align-items: center;
      padding: 20px;
    }
    .container {
      background: rgba(255, 255, 255, 0.1);
      backdrop-filter: blur(10px);
      border-radius: 20px;
      padding: 40px;
      box-shadow: 0 8px 32px rgba(0, 0, 0, 0.3);
      border: 1px solid rgba(255, 255, 255, 0.1);
    }
    h1 {
      color: #fff;
      text-align: center;
      margin-bottom: 30px;
      font-size: 2rem;
      text-shadow: 0 2px 10px rgba(0, 0, 0, 0.3);
    }
    .controls {
      display: grid;
      grid-template-columns: repeat(3, 100px);
      grid-template-rows: repeat(3, 100px);
      gap: 10px;
      justify-content: center;
    }
    .btn {
      border: none;
      border-radius: 15px;
      font-size: 2rem;
      cursor: pointer;
      transition: all 0.3s ease;
      display: flex;
      justify-content: center;
      align-items: center;
      box-shadow: 0 4px 15px rgba(0, 0, 0, 0.3);
    }
    .btn:hover {
      transform: translateY(-3px);
      box-shadow: 0 6px 20px rgba(0, 0, 0, 0.4);
    }
    .btn:active {
      transform: translateY(0);
    }
    .btn:disabled {
      opacity: 0.5;
      cursor: not-allowed;
      transform: none;
    }
    .btn-forward {
      grid-column: 2;
      grid-row: 1;
      background: linear-gradient(145deg, #00d9ff, #0099cc);
      color: white;
    }
    .btn-left {
      grid-column: 1;
      grid-row: 2;
      background: linear-gradient(145deg, #ff6b6b, #ee5a5a);
      color: white;
    }
    .btn-right {
      grid-column: 3;
      grid-row: 2;
      background: linear-gradient(145deg, #4ecdc4, #44a08d);
      color: white;
    }
    .btn-backward {
      grid-column: 2;
      grid-row: 3;
      background: linear-gradient(145deg, #a855f7, #7c3aed);
      color: white;
    }
    .btn-stop {
      grid-column: 2;
      grid-row: 2;
      background: linear-gradient(145deg, #ff0000, #cc0000);
      color: white;
      font-size: 1.5rem;
      font-weight: bold;
      border: 3px solid #fff;
      animation: pulse 1.5s infinite;
    }
    .btn-stop:disabled {
      opacity: 1;
      cursor: pointer;
      transform: none;
    }
    @keyframes pulse {
      0% { box-shadow: 0 0 0 0 rgba(255, 0, 0, 0.7); }
      70% { box-shadow: 0 0 0 15px rgba(255, 0, 0, 0); }
      100% { box-shadow: 0 0 0 0 rgba(255, 0, 0, 0); }
    }
    .info {
      margin-top: 30px;
      color: rgba(255, 255, 255, 0.8);
      text-align: center;
      font-size: 0.9rem;
      line-height: 1.6;
    }
    .status {
      margin-top: 20px;
      padding: 15px;
      border-radius: 10px;
      background: rgba(0, 0, 0, 0.2);
      text-align: center;
      color: #4ecdc4;
      font-weight: bold;
    }
  </style>
</head>
<body>
  <div class="container">
    <h1>🚗 Rover Demo</h1>
    <div class="controls">
      <button class="btn btn-forward" onclick="sendCommand('forward')">⬆️</button>
      <button class="btn btn-left" onclick="sendCommand('left')">⬅️</button>
      <button class="btn btn-stop" id="stopBtn" onclick="emergencyStop()">STOP</button>
      <button class="btn btn-right" onclick="sendCommand('right')">➡️</button>
      <button class="btn btn-backward" onclick="sendCommand('backward')">⬇️</button>
    </div>
    <div class="info">
      <p>⬆️⬇️ İleri/Geri: 0.4 m/s | 1 metre</p>
      <p>⬅️➡️ Dönüş: 30°/s | 90 derece</p>
    </div>
    <div class="status" id="status">Hazır</div>
  </div>
  <script>
    function sendCommand(cmd) {
      const btns = document.querySelectorAll('.btn:not(.btn-stop)');
      btns.forEach(b => b.disabled = true);
      document.getElementById('status').textContent = 'Hareket ediliyor...';
      
      fetch('/' + cmd)
        .then(response => response.text())
        .then(data => {
          document.getElementById('status').textContent = data;
          setTimeout(() => {
            btns.forEach(b => b.disabled = false);
            document.getElementById('status').textContent = 'Hazır';
          }, cmd.includes('left') || cmd.includes('right') ? 3000 : 2500);
        })
        .catch(err => {
          document.getElementById('status').textContent = 'Hata: ' + err;
          btns.forEach(b => b.disabled = false);
        });
    }
    
    function emergencyStop() {
      fetch('/stop')
        .then(response => response.text())
        .then(data => {
          document.getElementById('status').textContent = data;
          document.getElementById('status').style.color = '#ff6b6b';
          const btns = document.querySelectorAll('.btn:not(.btn-stop)');
          btns.forEach(b => b.disabled = false);
          setTimeout(() => {
            document.getElementById('status').style.color = '#4ecdc4';
            document.getElementById('status').textContent = 'Hazır';
          }, 2000);
        })
        .catch(err => {
          document.getElementById('status').textContent = 'Hata: ' + err;
        });
    }
  </script>
</body>
</html>
)rawliteral";

// ==========================================================================
// --- MOTOR KONTROL FONKSİYONLARI ---
// ==========================================================================

void stopAllMotors() {
  roboclaw.SpeedM1M2(RC_LEFT_ADDRESS, 0, 0);
  roboclaw.SpeedM1M2(RC_RIGHT_ADDRESS, 0, 0);
  isMoving = false;
}

void setMotorSpeeds(int32_t leftSpeed, int32_t rightSpeed, unsigned long duration) {
  targetSpeedLeft = leftSpeed;
  targetSpeedRight = rightSpeed;
  moveDuration = duration;
  moveStartTime = millis();
  isMoving = true;
  
  // Sol tekerlekler (M1=ön, M2=arka)
  roboclaw.SpeedM1M2(RC_LEFT_ADDRESS, leftSpeed, leftSpeed);
  // Sağ tekerlekler (M1=ön, M2=arka)
  roboclaw.SpeedM1M2(RC_RIGHT_ADDRESS, rightSpeed, rightSpeed);
}

// ==========================================================================
// --- WEB SERVER HANDLERLARI ---
// ==========================================================================

void handleRoot() {
  server.send(200, "text/html", index_html);
}

void handleForward() {
  if (isMoving) {
    server.send(200, "text/plain", "Zaten hareket ediyor!");
    return;
  }
  
  // Süre = mesafe / hız = 1.0 / 0.4 = 2.5 saniye
  unsigned long duration = (unsigned long)((LINEAR_DISTANCE / LINEAR_SPEED) * 1000);
  int32_t speed = (int32_t)(LINEAR_SPEED * TICKS_PER_METER);
  
  setMotorSpeeds(speed, speed, duration);
  server.send(200, "text/plain", "İleri gidiyor (1m)");
}

void handleBackward() {
  if (isMoving) {
    server.send(200, "text/plain", "Zaten hareket ediyor!");
    return;
  }
  
  unsigned long duration = (unsigned long)((LINEAR_DISTANCE / LINEAR_SPEED) * 1000);
  int32_t speed = (int32_t)(LINEAR_SPEED * TICKS_PER_METER);
  
  setMotorSpeeds(-speed, -speed, duration);
  server.send(200, "text/plain", "Geri gidiyor (1m)");
}

void handleLeft() {
  if (isMoving) {
    server.send(200, "text/plain", "Zaten hareket ediyor!");
    return;
  }
  
  // Süre = açı / hız = 90 / 30 = 3 saniye
  unsigned long duration = (unsigned long)((TURN_ANGLE / TURN_SPEED_DEG) * 1000);
  
  // Yerinde dönüş için tekerlek hızı hesapla
  // v = omega * (track_width / 2)
  float wheelSpeed = TURN_SPEED_RAD * (TRACK_WIDTH / 2.0);
  int32_t speed = (int32_t)(wheelSpeed * TICKS_PER_METER);
  
  // Sola dönüş: sol geriye, sağ ileriye
  setMotorSpeeds(-speed, speed, duration);
  server.send(200, "text/plain", "Sola dönüyor (90°)");
}

void handleRight() {
  if (isMoving) {
    server.send(200, "text/plain", "Zaten hareket ediyor!");
    return;
  }
  
  unsigned long duration = (unsigned long)((TURN_ANGLE / TURN_SPEED_DEG) * 1000);
  
  float wheelSpeed = TURN_SPEED_RAD * (TRACK_WIDTH / 2.0);
  int32_t speed = (int32_t)(wheelSpeed * TICKS_PER_METER);
  
  // Sağa dönüş: sol ileriye, sağ geriye
  setMotorSpeeds(speed, -speed, duration);
  server.send(200, "text/plain", "Sağa dönüyor (90°)");
}

void handleStop() {
  stopAllMotors();
  Serial.println("ACİL DURUŞ!");
  server.send(200, "text/plain", "⚠️ ACİL DURUŞ!");
}

// ==========================================================================
// --- LED FONKSİYONLARI ---
// ==========================================================================

void setLed(uint8_t r, uint8_t g, uint8_t b) {
  pixels.setPixelColor(0, pixels.Color(r, g, b));
  pixels.show();
}

// ==========================================================================
// --- SETUP ---
// ==========================================================================

void setup() {
  Serial.begin(115200);
  
  // NeoPixel
  pixels.begin();
  pixels.setBrightness(30);
  setLed(255, 0, 0); // Kırmızı: Başlatılıyor
  
  // RoboClaw
  Serial2.begin(RC_BAUDRATE, SERIAL_8N1, RX2_PIN, TX2_PIN);
  roboclaw.begin(RC_BAUDRATE);
  roboclaw.ResetEncoders(RC_LEFT_ADDRESS);
  roboclaw.ResetEncoders(RC_RIGHT_ADDRESS);
  
  // WiFi Bağlantısı
  Serial.println();
  Serial.print("WiFi'ye bağlanılıyor: ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("WiFi bağlandı!");
    Serial.print("IP Adresi: ");
    Serial.println(WiFi.localIP());
    setLed(0, 255, 0); // Yeşil: Bağlı
  } else {
    Serial.println();
    Serial.println("WiFi bağlantısı başarısız!");
    setLed(255, 100, 0); // Turuncu: Hata
  }
  
  // Web Server Rotaları
  server.on("/", handleRoot);
  server.on("/forward", handleForward);
  server.on("/backward", handleBackward);
  server.on("/left", handleLeft);
  server.on("/right", handleRight);
  server.on("/stop", handleStop);
  
  server.begin();
  Serial.println("Web server başlatıldı!");
}

// ==========================================================================
// --- LOOP ---
// ==========================================================================

void loop() {
  server.handleClient();
  
  // Hareket süresini kontrol et
  if (isMoving && (millis() - moveStartTime >= moveDuration)) {
    stopAllMotors();
    Serial.println("Hareket tamamlandı");
  }
  
  // LED durumu güncelle
  static unsigned long lastBlink = 0;
  if (isMoving) {
    // Hareket ederken mavi yanıp sönsün
    if (millis() - lastBlink > 200) {
      lastBlink = millis();
      static bool ledState = false;
      ledState = !ledState;
      if (ledState) setLed(0, 0, 255);
      else setLed(0, 100, 255);
    }
  } else if (WiFi.status() == WL_CONNECTED) {
    setLed(0, 255, 0); // Yeşil: Hazır
  }
}
