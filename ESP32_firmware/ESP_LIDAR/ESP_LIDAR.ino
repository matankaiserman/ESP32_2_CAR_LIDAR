#include <WiFi.h>
#include <WiFiUdp.h>

// --- CONFIGURATION ---
const char* ssid = "ESP32_JOYSTICK";
const char* password = "password123"; 
IPAddress local_IP(192, 168, 4, 30); 
IPAddress gateway(192, 168, 4, 1);    
IPAddress subnet(255, 255, 255, 0);
const char* udpAddress = "192.168.4.50"; 
const int udpPort = 9876;

WiFiUDP udp;
#define MAX_PAYLOAD 1024
uint8_t payload[MAX_PAYLOAD]; 

#define LIDAR_RX 18
#define LIDAR_TX 17 
#define LED_PIN 15 

long bauds[] = {115200, 121000, 122880, 117000, 114000, 125000, 111000};
int currentBaudIdx = 0;
bool baudLocked = false;
unsigned long lastValidPacketTime = 0;

void sendUDP(int p_len) {
  // לוגיקה זהה לפייתון
  float start_angle = ((payload[3] << 8) | payload[4]) * 0.01f;
  int num_samples = (p_len - 5) / 3; 
  if (num_samples <= 0) return;
  
  float step = 24.0f / num_samples;

  udp.beginPacket(udpAddress, udpPort);
  for (int i = 0; i < num_samples; i++) {
      int off = 5 + (i * 3);
      if (off + 2 >= p_len) break;

      uint8_t q = payload[off];
      uint16_t dist_raw = (payload[off+1] << 8) | payload[off+2];
      float dist = dist_raw * 0.25f;
      float ang = fmod(start_angle + (i * step), 360.0f);

      if (q > 15 && dist > 150.0f) {
        udp.write((uint8_t*)&ang, 4);
        udp.write((uint8_t*)&dist, 4);
      }
  }
  unsigned long currentTime = millis();
  udp.write((uint8_t*)&currentTime, 4); // שולח את ה-4 בתים של ה-unsigned long
  udp.endPacket();
  lastValidPacketTime = millis();
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  WiFi.config(local_IP, gateway, subnet);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nWiFi Connected!");
  Serial1.begin(bauds[currentBaudIdx], SERIAL_8N1, LIDAR_RX, LIDAR_TX);
}

void loop() {
  // מנגנון סריקה/נעילה
  if (!baudLocked) {
    if (Serial1.available()) {
      if (Serial1.read() == 0xAA) { // מחפשים רק 0xAA כמו בפייתון
        baudLocked = true;
        lastValidPacketTime = millis();
        Serial.printf("\nLocked on Baud: %ld\n", bauds[currentBaudIdx]);
      }
    }
    if (millis() - lastValidPacketTime > 1500) {
      currentBaudIdx = (currentBaudIdx + 1) % 7;
      Serial1.begin(bauds[currentBaudIdx], SERIAL_8N1, LIDAR_RX, LIDAR_TX);
      lastValidPacketTime = millis();
    }
    return;
  }

  // לוגיקה משופרת: חיפוש Header וקריאה מבוקרת
  if (Serial1.available() > 0) {
    uint8_t c = Serial1.read();
    
    if (c == 0xAA) { // מצאנו התחלת חבילה
      uint8_t header[7];
      // מחכים שה-Header יגיע (7 בתים)
      if (Serial1.readBytes(header, 7) == 7) {
        if (header[4] == 0xAD) { // בדיקה שהחבילה תקינה לפי הפרוטוקול
          int p_len = (header[5] << 8) | header[6];
          
          if (p_len > 0 && p_len < MAX_PAYLOAD) {
            // קריאת ה-Payload בצורה מהירה
            if (Serial1.readBytes(payload, p_len) == p_len) {
              sendUDP(p_len);
              
              // בלינקר להוכחת חיים
              static unsigned long last_blink = 0;
              if (millis() - last_blink > 50) {
                digitalWrite(LED_PIN, !digitalRead(LED_PIN));
                last_blink = millis();
              }
            }
          }
        }
      }
    }
  }

  // Watchdog חזרה לסריקה אם אין נתונים 3 שניות
  if (millis() - lastValidPacketTime > 3000) {
    baudLocked = false;
    Serial.println("Timeout - Rescanning...");
    lastValidPacketTime = millis();
  }
}