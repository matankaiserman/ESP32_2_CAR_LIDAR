#include <WiFi.h>
#include <WebServer.h>        // Standard library for serving web pages (HTTP)
#include <WebSocketsServer.h> // Stable library for real-time communication (WebSocket)
#include <ArduinoJson.h>      // For parsing the JSON data
#include <WiFiUdp.h> // וודא שהספרייה הזו כלולה
#include <Wire.h>
#include "driver/pcnt.h"




// --- CONFIGURATION ---
const char* ssid = "ESP32_JOYSTICK";
const char* password = "password123"; 


//motor pins
const int enA=23;
const int IN1=22;
const int IN2 = 25; // עודכן מ-1
const int IN3 = 26; // עודכן מ-3
const int IN4=21;
const int enB=19;

const float dead_zone = 0.1; // סף תנועה אחיד לכל המערכת

// משתנים עבור Soft Start
float currentSpeedA = 0.0;
float currentSpeedB = 0.0;
const float accel_step = 0.04; // קצב ההאצה - ככל שהערך קטן יותר, ההאצה רכה יותר


// MPU6500 Config
const int MPU_ADDR = 0x68;
const int I2C_SDA = 13;
const int I2C_SCL = 14;
float gyroX_offset = 0;
float gyroY_offset = 0;
float gyroZ_offset = 0;

float ax = 0, ay = 0, az = 0, gx_rad = 0, gy_rad = 0, gz_rad = 0; // משתני IMU גלובליים


// setting PWM properties
const int freq = 3000;
const int resolution = 10;

// HTTP Server on standard port 80 (to serve the HTML page)
WebServer httpServer(80); 
// WebSocket Server on port 81 (to handle the joystick data)
WebSocketsServer webSocket = WebSocketsServer(81); 

// Global variables to store the control values (-1.0 to 1.0)
float joystickX = 0.0;
float joystickY = 0.0;
bool isClientConnected = false; 

// Variables for non-blocking serial output timing
unsigned long lastSerialPrint = 0;
const long printInterval = 30; // Print every 30 milliseconds (5 times per second)

// Encoder Pins
const int leftEncoderPin = 18;
const int rightEncoderPin = 5;

// הגדרת יחידות ה-PCNT
const pcnt_unit_t PCNT_UNIT_LEFT = PCNT_UNIT_0;
const pcnt_unit_t PCNT_UNIT_RIGHT = PCNT_UNIT_1;

// Counter Variables
volatile long leftEncoderTicks = 0;
volatile long rightEncoderTicks = 0;
unsigned long lastDebounceLeft = 0;
unsigned long lastDebounceRight = 0;
const int debounceDelay = 15; // מ"ש למניעת רעשים מהמתג המגנטי


// הגדרת הכתובות הסטטיות
IPAddress local_IP(192, 168, 4, 2);   // הכתובת הקבועה של המצלמה
IPAddress gateway(192, 168, 4, 1);    // ה-IP של ה-ESP32 הראשי (הג'ויסטיק)
IPAddress subnet(255, 255, 255, 0);

//תקשורת אלחותית להדפסה סיריאלית בשביל ROS
WiFiUDP udp;
const char* broadcastIP = "192.168.4.50"; // ה-IP של המחשב עם ה-ROS
//IPAddress broadcastIP(192, 168, 4, 50); // or 255.255.255.255
//IPAddress broadcastIP(255, 255, 255, 255);
const int udp_port = 8888;
char telemetryBuffer[128];

// --- HTML Content (Embedded Client) ---
// This entire string contains the webpage served by the ESP32 on 192.168.4.1

// --- HTML Content (Embedded Client - Optimized UI) ---
const char* rawHtmlContent = R"rawliteral(
<!DOCTYPE html>
<html lang="he" dir="rtl">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Robot Control Pro</title>
    <script src="https://cdn.tailwindcss.com"></script>
    <style>
        body { font-family: sans-serif; background-color: #f0f4f8; margin: 0; display: flex; justify-content: center; align-items: center; min-height: 100vh; }
        .joystick-base { width: 250px; height: 250px; background: #dfe6e9; border-radius: 50%; position: relative; box-shadow: inset 0 0 20px rgba(0,0,0,0.1); touch-action: none; }
        .joystick-ball { position: absolute; top: 50%; left: 50%; width: 70px; height: 70px; margin: -35px 0 0 -35px; border-radius: 50%; background: linear-gradient(135deg, #3b82f6, #1e40af); box-shadow: 0 5px 15px rgba(0,0,0,0.3); transition: transform 0.05s ease-out; }
    </style>
</head>
<body class="p-4">

    <div class="bg-white rounded-2xl shadow-2xl p-6 w-full max-w-4xl">
        <header class="text-center mb-6">
            <h1 class="text-2xl font-bold text-slate-800">Robot Control Center</h1>
            <div id="ws-status" class="inline-block mt-2 px-4 py-1 rounded-full text-xs font-bold bg-gray-400 text-white uppercase">מתחבר...</div>
        </header>

        <div class="grid grid-cols-1 md:grid-cols-2 gap-8">
            <div class="flex flex-col gap-4">
                <div class="bg-black rounded-lg overflow-hidden aspect-video border-2 border-slate-200 relative">
                    <iframe id="camera-frame" src="http://192.168.4.2:81/stream" class="w-full h-full border-none"></iframe>
                </div>
                <button onclick="window.refreshStream()" class="bg-blue-600 hover:bg-blue-700 text-white font-bold py-2 rounded shadow transition-all active:scale-95">רענן וידאו</button>
            </div>

            <div class="flex flex-col items-center bg-gray-50 p-4 rounded-xl border">
                <div id="zone" class="joystick-base mb-6">
                    <div id="ball" class="joystick-ball"></div>
                </div>

                <div class="w-full mb-4 px-4">
                    <label class="text-xs font-bold text-gray-500 uppercase flex justify-between">
                        <span>מהירות מקסימלית:</span>
                        <span class="text-blue-600"><span id="speed-txt">80</span>%</span>
                    </label>
                    <input type="range" id="speed-range" min="10" max="100" value="80" class="w-full h-2 bg-gray-200 rounded-lg appearance-none cursor-pointer accent-blue-600 mt-2">
                </div>

                <div class="grid grid-cols-2 gap-4 w-full text-center">
                    <div class="bg-white p-3 border rounded shadow-sm">
                        <div class="text-[10px] text-gray-400 font-bold uppercase">Motor L (Y+X)</div>
                        <div id="out-l" class="font-mono text-xl font-bold text-green-600">0.00</div>
                    </div>
                    <div class="bg-white p-3 border rounded shadow-sm">
                        <div class="text-[10px] text-gray-400 font-bold uppercase">Motor R (Y-X)</div>
                        <div id="out-r" class="font-mono text-xl font-bold text-green-600">0.00</div>
                    </div>
                </div>
                <p class="text-[10px] text-gray-400 mt-4 italic">שליטה: WASD / חיצים / ג'ויסטיק</p>
            </div>
        </div>
    </div>

    <script>
        const ball = document.getElementById('ball');
        const zone = document.getElementById('zone');
        const speedRange = document.getElementById('speed-range');
        const speedTxt = document.getElementById('speed-txt');
        const outL = document.getElementById('out-l');
        const outR = document.getElementById('out-r');
        const wsStatus = document.getElementById('ws-status');

        let ws;
        let isMoving = false;
        let speedLimit = 0.8;
        let keys = {};
        let curX = 0, curY = 0; // ערכי ג'ויסטיק גולמיים (-1 עד 1)

        window.refreshStream = function() {
            const iframe = document.getElementById('camera-frame');
            iframe.src = "http://192.168.4.2:81/stream?t=" + Date.now();
        };

        function connectWS() {
            ws = new WebSocket("ws://192.168.4.1:81/");
            ws.onopen = () => {
                wsStatus.innerText = "מחובר";
                wsStatus.className = "inline-block mt-2 px-4 py-1 rounded-full text-xs font-bold bg-green-500 text-white uppercase";
            };
            ws.onclose = () => {
                wsStatus.innerText = "מנותק - מנסה שוב";
                wsStatus.className = "inline-block mt-2 px-4 py-1 rounded-full text-xs font-bold bg-red-500 text-white uppercase";
                setTimeout(connectWS, 2000);
            };
        }

        // פונקציית החישוב והשליחה המרכזית
        function processAndSend(rawX, rawY) {
            // 1. הגבלת המהירות ע"י הסליידר
            const x = rawX * speedLimit;
            const y = rawY * speedLimit;

            // 2. נוסחת Differential Drive (כמו בקוד המקורי שלך)
            // Left = Y + X, Right = Y - X
            let left = y + x;
            let right = y - x;

            // 3. Clamp לטווח של -1 עד 1
            left = Math.max(-1, Math.min(1, left));
            right = Math.max(-1, Math.min(1, right));

            // 4. עדכון ויזואלי של הכדור (מייצג את קלט הג'ויסטיק)
            ball.style.transform = `translate(${rawX * 90}px, ${-rawY * 90}px)`;

            // 5. עדכון מספרי ה-UI (מייצג את פלט המנועים)
            outL.innerText = left.toFixed(2);
            outR.innerText = right.toFixed(2);

            // 6. שידור ל-ESP32
            if (ws && ws.readyState === WebSocket.OPEN) {
                ws.send(JSON.stringify({x: left, y: right}));
            }
        }

        // טיפול בעכבר ומגע
        function handleManualInput(e) {
            if (!isMoving) return;
            const rect = zone.getBoundingClientRect();
            const centerX = rect.left + rect.width / 2;
            const centerY = rect.top + rect.height / 2;
            const clientX = e.clientX || (e.touches ? e.touches[0].clientX : 0);
            const clientY = e.clientY || (e.touches ? e.touches[0].clientY : 0);

            // חישוב מרחק ממרכז הג'ויסטיק
            let dx = (clientX - centerX) / (rect.width / 2 - 35);
            let dy = -(clientY - centerY) / (rect.height / 2 - 35);

            // נרמול לטווח של 1 עד 1-
            dx = Math.max(-1, Math.min(1, dx));
            dy = Math.max(-1, Math.min(1, dy));

            processAndSend(dx, dy);
        }

        zone.addEventListener('mousedown', () => isMoving = true);
        zone.addEventListener('touchstart', (e) => { isMoving = true; e.preventDefault(); }, {passive: false});
        window.addEventListener('mouseup', () => { isMoving = false; processAndSend(0,0); });
        window.addEventListener('touchend', () => { isMoving = false; processAndSend(0,0); });
        window.addEventListener('mousemove', handleManualInput);
        window.addEventListener('touchmove', handleManualInput, {passive: false});

        // טיפול במקלדת
        window.addEventListener('keydown', e => keys[e.code] = true);
        window.addEventListener('keyup', e => keys[e.code] = false);

        function checkKeyboard() {
            if (isMoving) return; // מגע מקבל קדימות
            
            let targetX = 0, targetY = 0;
            if (keys['ArrowUp'] || keys['KeyW']) targetY += 1;
            if (keys['ArrowDown'] || keys['KeyS']) targetY -= 1;
            if (keys['ArrowLeft'] || keys['KeyA']) targetX -= 0.7; // סיבוב מעט איטי יותר לשליטה טובה
            if (keys['ArrowRight'] || keys['KeyD']) targetX += 0.7;

            // החלקה (Smoothing) למקלדת
            curX = curX * 0.8 + targetX * 0.2;
            curY = curY * 0.8 + targetY * 0.2;

            if (Math.abs(curX) < 0.05 && targetX === 0) curX = 0;
            if (Math.abs(curY) < 0.05 && targetY === 0) curY = 0;

            processAndSend(curX, curY);
        }

        speedRange.oninput = () => {
            speedLimit = speedRange.value / 100;
            speedTxt.innerText = speedRange.value;
        };

        // לולאת רענון
        connectWS();
        function mainLoop() {
            checkKeyboard();
            requestAnimationFrame(mainLoop);
        }
        mainLoop();
    </script>
</body>
</html>
)rawliteral";



void setupPCNT(pcnt_unit_t unit, int pin) {
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = pin,
        .ctrl_gpio_num = -1,          
        .lctrl_mode = PCNT_MODE_KEEP, 
        .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_INC,   
        .neg_mode = PCNT_COUNT_DIS,
        .counter_h_lim = 32767,
        .counter_l_lim = -32768,
        .unit = unit,
        .channel = PCNT_CHANNEL_0,
    };
    // חסר היה הקוד שמפעיל את ההגדרה והסוגר הסוגר:
    pcnt_unit_config(&pcnt_config);
    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);
    pcnt_counter_resume(unit);
} 

// --- FUNCTION PROTOTYPE ---
//return output values for motors
//input stick pins
void motoroutputs(float targetX, float targetY) {
  // --- מנגנון SOFT START ---
  // מנוע א' (מחובר ל-targetY לפי הלוגיקה שלך)
  if (currentSpeedA < targetY) currentSpeedA = min(currentSpeedA + accel_step, targetY);
  else if (currentSpeedA > targetY) currentSpeedA = max(currentSpeedA - accel_step, targetY);

  // מנוע ב' (מחובר ל-targetX)
  if (currentSpeedB < targetX) currentSpeedB = min(currentSpeedB + accel_step, targetX);
  else if (currentSpeedB > targetX) currentSpeedB = max(currentSpeedB - accel_step, targetX);

  // עצירת חירום אם אין לקוח מחובר
  if (!isClientConnected) {
    currentSpeedA = 0;
    currentSpeedB = 0;
    ledcWrite(IN1, 1023); ledcWrite(IN2, 1023);
    ledcWrite(IN3, 1023); ledcWrite(IN4, 1023);
    return;
  }

  // --- שליטה פיזית במנוע א' (Y) ---
  if (abs(currentSpeedA) < dead_zone) {
    ledcWrite(IN1, 1023);
    ledcWrite(IN2, 1023);
  } else {
    long dutyCycle = map(round(abs(currentSpeedA) * 100), 0, 100, 0, 1023);
    digitalWrite(enA, HIGH);
    if (currentSpeedA < 0) { // אחורה
      ledcWrite(IN1, dutyCycle);
      ledcWrite(IN2, 0);
    } else { // קדימה
      ledcWrite(IN2, dutyCycle);
      ledcWrite(IN1, 0);
    }
  }

  // --- שליטה פיזית במנוע ב' (X) ---
  if (abs(currentSpeedB) < dead_zone) {
    ledcWrite(IN3, 1023);
    ledcWrite(IN4, 1023);
  } else {
    long dutyCycle = map(round(abs(currentSpeedB) * 100), 0, 100, 0, 1023);
    digitalWrite(enB, HIGH);
    if (currentSpeedB < 0) { // שמאלה/ימינה
      ledcWrite(IN3, dutyCycle);
      ledcWrite(IN4, 0);
    } else {
      ledcWrite(IN4, dutyCycle);
      ledcWrite(IN3, 0);
    }
  }
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);

void handleRoot() {
  // Handler for the root URL: http://192.168.4.1/
  httpServer.send(200, "text/html", rawHtmlContent);
}


//---------------------------------------------------
void setup() {
  Serial.begin(115200);

  // --- MPU6500 SETUP -----------------------------------------------------------
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setTimeOut(50); // מגדיר זמן מקסימלי להמתנה לפני שה-I2C מוותר ולא תוקע את המעבד
  Wire.setClock(400000); 
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0); Wire.endTransmission();
  
  Serial.println("Calibrating IMU... Keep Still!");
  float sumX = 0, sumY = 0, sumZ = 0;
  const int numSamples = 200;

  for(int i = 0; i < numSamples; i++) {
      Wire.beginTransmission(MPU_ADDR);
      // 0x43 היא הכתובת של GYRO_XOUT_H. משם נקרא 6 בתים רצופים (X, Y, Z)
      Wire.write(0x43); 
      Wire.endTransmission(false);
      
      if (Wire.requestFrom(MPU_ADDR, 6, true) == 6) {
          int16_t rawX = (Wire.read() << 8 | Wire.read());
          int16_t rawY = (Wire.read() << 8 | Wire.read());
          int16_t rawZ = (Wire.read() << 8 | Wire.read());
          
          sumX += (rawX / 131.0);
          sumY += (rawY / 131.0);
          sumZ += (rawZ / 131.0);
      }
      delay(5);
  }

  gyroX_offset = sumX / (float)numSamples;
  gyroY_offset = sumY / (float)numSamples;
  gyroZ_offset = sumZ / (float)numSamples;

  Serial.printf("Calibration Done!\nOffsets -> X: %.2f, Y: %.2f, Z: %.2f\n", 
                gyroX_offset, gyroY_offset, gyroZ_offset);
  

  //---------------------------------------------------------------------------

  // Set the Wi-Fi mode explicitly before starting the AP for maximum stability
  Serial.println("Setting WiFi mode...");
  WiFi.mode(WIFI_AP); 

  // --- WIFI SETUP (AP Mode) ---
  Serial.println("Starting SoftAP...");
  if (!WiFi.softAP(ssid, password)) {
      Serial.println("!!! SOFTAP FAILED TO START. Check board type/core version. !!!");
      return; 
  }

// 3. עכשיו מגדירים את ה-IP הסטטי (זה לא יפיל את המערכת כאן)
  Serial.println("Applying Static IP configuration...");
  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("STA Failed to configure Static IP");
  }
  
  Serial.print("Access Point created! SSID: ");
  Serial.println(ssid);
  Serial.print("BROWSER IP (HTML): ");
  Serial.println(WiFi.softAPIP()); 

  // --- HTTP SERVER SETUP (Port 80) ---
  httpServer.on("/", handleRoot);
  httpServer.begin();
  Serial.println("HTTP Server started on Port 80.");

  // --- WEBSOCKET SETUP (Port 81) ---
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("Stable WebSocket server started on Port 81.");

  //setup motor pins
    pinMode(enA, OUTPUT); 
  pinMode(enB, OUTPUT); 
  digitalWrite(enA, HIGH);
  digitalWrite(enB, HIGH);
  
  ledcAttachChannel(IN1, freq, resolution,0);
  ledcAttachChannel(IN2, freq, resolution,1); 
  ledcAttachChannel(IN3, freq, resolution,2);
  ledcAttachChannel(IN4, freq, resolution,3);
  ledcWrite(IN1, 1023);
  ledcWrite(IN2, 1023);
  ledcWrite(IN3, 1023);
  ledcWrite(IN4, 1023);

  //ecoder setup
  pinMode(leftEncoderPin, INPUT_PULLUP);
  pinMode(rightEncoderPin, INPUT_PULLUP);
// החלף את ה-attachInterrupt בזה:
    setupPCNT(PCNT_UNIT_LEFT, leftEncoderPin);
    setupPCNT(PCNT_UNIT_RIGHT, rightEncoderPin);
    
    Serial.println("Hardware Pulse Counter Initialized.");

  udp.begin(udp_port);//serial udp port for ros
}

void loop() {
        // --- NON-BLOCKING Serial Output ---
    // We use millis() instead of delay(100) to keep the loop running fast.
    httpServer.handleClient(); 
    webSocket.loop();          

    unsigned long currentMillis = millis();

    // טיפול בשידור נתונים (UDP וסריאל) - כל 50ms
    if (currentMillis - lastSerialPrint >= printInterval) {
        lastSerialPrint = currentMillis;

        //קריאת אנקודרים---------------------------------------------------------------------------
        // 1. קבלת הערכים הגולמיים מהחומרה
        int16_t countL = 0;
        int16_t countR = 0;
        pcnt_get_counter_value(PCNT_UNIT_LEFT, &countL);
        pcnt_get_counter_value(PCNT_UNIT_RIGHT, &countR);
        pcnt_counter_clear(PCNT_UNIT_LEFT);
        pcnt_counter_clear(PCNT_UNIT_RIGHT);

        // 2. חישוב הערכים שהמנועים קיבלו (הפרשיאל דרייב)
        float motorLeftTarget = joystickX ;
        float motorRightTarget = joystickY ;

        // 3. עדכון האנקודר השמאלי לפי כיוון המנוע השמאלי
        // --- עדכון צד שמאל ---
        if (motorLeftTarget > dead_zone) {
            leftEncoderTicks += countL;
        } else if (motorLeftTarget < -dead_zone) {
            leftEncoderTicks -= countL;
        }

        // --- עדכון צד ימין ---
        if (motorRightTarget > dead_zone) {
            rightEncoderTicks += countR;
        } else if (motorRightTarget < -dead_zone) {
            rightEncoderTicks -= countR;
        }
        //Serial.printf("L_Target: %.2f, R_Target: %.2f, countL: %d, countR: %d\n", 
        //      motorLeftTarget, motorRightTarget, countL, countR);
        // קריאת MPU6500------------------------------------------------------------------------------------------------------

        // 1. נסיון להתחיל תקשורת - בדיקה מהירה
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x3B); 
        if (Wire.endTransmission(false) != 0) {
            // אם יש שגיאה, לא עוצרים את הכל! פשוט מדלגים על הקריאה הסיבוב הזה
            // ומנסים לאפס את ה-Bus פעם בכמה זמן, לא בכל לופ
            static unsigned long lastI2CError = 0;
            if (millis() - lastI2CError > 100) { 
                Wire.begin(I2C_SDA, I2C_SCL);
                Wire.setClock(400000);
                lastI2CError = millis();
            }
            return; 
        }

        // 2. קריאה מהירה של כל הנתונים למאגר אחד
        if (Wire.requestFrom(MPU_ADDR, 14, true) == 14) {
            int16_t data[7]; // מערך ל-7 ערכים (3 ACC, 1 TEMP, 3 GYRO)
            for (int i = 0; i < 7; i++) {
                data[i] = (Wire.read() << 8 | Wire.read());
            }

            ax = data[0] *9.81/ 16384.0;
            ay = data[1] *9.81/ 16384.0;
            az = data[2] *9.81/ 16384.0;
            // data[3] הוא טמפרטורה - מתעלמים
            gx_rad= (data[4] / 131.0 - gyroX_offset) * (PI / 180.0);
            gy_rad= (data[5] / 131.0 - gyroY_offset) * (PI / 180.0);
            gz_rad = (data[6] / 131.0 - gyroZ_offset) * (PI / 180.0);
        }
        //-----------------------------------------------------------------------------------------------------------------------------

        // שליחת נתונים משולבת ב-UDP
        // פורמט: E,Left,Right,AccX,AccY,AccZ,GyroZ
        // במקום String, תשתמש במערך תווים קבוע בראש הלופ או כגלובלי
        // snprintf(telemetryBuffer, sizeof(telemetryBuffer), "E,%ld,%ld,%.3f,%.3f,%.3f,%.4f", 
        //         leftEncoderTicks, rightEncoderTicks, ax, ay, az, gz_rad);
        snprintf(telemetryBuffer, sizeof(telemetryBuffer), "E,%ld,%ld,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%lu", 
                leftEncoderTicks, rightEncoderTicks, ax, ay, az, gx_rad, gy_rad, gz_rad, millis());

        //if (isClientConnected) {
        //    Serial.println(telemetryBuffer);
        //}
        // שליחת אודומטריה ב-Broadcast
        udp.beginPacket(broadcastIP, udp_port);
        udp.print(telemetryBuffer);
        udp.endPacket();
        
    }

    // עדכון המנועים - רץ כל הזמן בשביל תגובה מהירה
    motoroutputs(joystickX, joystickY);

}

// --- WEBSOCKET EVENT HANDLER ---
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Client Disconnected!\n", num);
      isClientConnected = false;
      break;

    case WStype_CONNECTED: {
      IPAddress ip = webSocket.remoteIP(num);
      Serial.printf("[%u] CONNECTED from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
      webSocket.sendTXT(num, "Connection established."); 
      isClientConnected = true;
    }
    break;

    case WStype_TEXT: { 
      // Note: We avoid printing received data here to keep the handler fast.
      StaticJsonDocument<100> doc;
      
      DeserializationError error = deserializeJson(doc, payload);

      if (error) {
        // Only print error messages, not every incoming message
        // Serial.print(F("deserializeJson() failed: "));
        // Serial.println(error.f_str());
        return;
      }

      if (doc.containsKey("x") && doc.containsKey("y")) {
        joystickX = doc["x"].as<float>();
        joystickY = doc["y"].as<float>();
      }
    } 
    break;
      
    default:
      break;
  }
}
