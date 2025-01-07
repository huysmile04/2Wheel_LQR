#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <Wire.h>

// Configurations for static IP
IPAddress local_IP(192, 168, 1, 184);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);    // optional
IPAddress secondaryDNS(8, 8, 4, 4);  // optional

const char* ssid = "Thanh Phuc 4G";
const char* password = "12345678kst";
AsyncWebServer server(80);

float psi = 0;


void setup() {
  Serial.begin(9600);
  Wire.begin();  // Initialize I2C

  Wire.beginTransmission(0x68);  
  Wire.write(0x6B);              
  Wire.write(0x00);              
  Wire.endTransmission(true);

  // Connect to Wi-Fi with static IP
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Serve web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    String html = "<!DOCTYPE html><html><head><title>Balanced 2-wheel vehicle parameter monitoring</title><style>";
    html += "body { font-family: Arial, sans-serif; text-align: center; }";
    html += "button { width: 100px; height: 50px; margin: 10px; border: none; border-radius: 5px; background-color: #4CAF50; color: white; font-size: 16px; cursor: pointer; transition: background-color 0.3s, transform 0.3s; }";
    html += "button:hover { background-color: #45a049; transform: scale(1.1); }";
    html += "button:active { background-color: #3e8e41; transform: scale(0.9); }";
    html += "canvas { width: 100%; height: 400px; }";  // Adjust size for one chart
    html += "</style></head><body>";
    html += "<h1>Control and Supervision</h1>";  // Display text instead of IP address
    html += "<button onclick=\"sendCommand('1')\">Forward</button>";
    html += "<button onclick=\"sendCommand('3')\">Back</button>";
    html += "<button onclick=\"sendCommand('5')\">Left</button>";
    html += "<button onclick=\"sendCommand('7')\">Right</button>";
    html += "<button onclick=\"sendCommand('2')\">Stop</button>";
    html += "<canvas id=\"chart2\"></canvas>";  // Only one canvas for psi
    html += "<script src=\"https://cdn.jsdelivr.net/npm/chart.js\"></script>";
    html += "<script>";
    html += "function sendCommand(cmd) {";
    html += "  var xhttp = new XMLHttpRequest();";
    html += "  xhttp.open('GET', '/control?cmd=' + cmd, true);";
    html += "  xhttp.send();";
    html += "}";
    html += "var ctx2 = document.getElementById('chart2').getContext('2d');";
    html += "var chart2 = new Chart(ctx2, {";
    html += " type: 'line',";
    html += " data: { labels: [], datasets: [{ label: 'psi', data: [], borderColor: 'rgba(192, 75, 75, 1)', borderWidth: 1, pointRadius: 1 }] },";
    html += " options: { scales: {";
    html += " x: { type: 'linear', position: 'bottom', ticks: { callback: function(value, index, values) { return value / 1000 + ' s'; } } },";
    html += " y: { min: -10, max: 10, grid: { drawBorder: true, color: function(context) { return context.tick.value === 0 ? 'red' : 'rgba(0,0,0,0.1)'; } } }";
    html += " }, animation: false }";
    html += "});";
    html += "setInterval(function() {";
    html += " var xhttp = new XMLHttpRequest();";
    html += " xhttp.onreadystatechange = function() {";
    html += " if (this.readyState == 4 && this.status == 200) {";
    html += " var data = JSON.parse(this.responseText);";
    html += " chart2.data.labels.push(data.time);";
    html += " chart2.data.datasets[0].data.push(data.psi);";
    html += " if (chart2.data.labels.length > 100) {";
    html += " chart2.data.labels.shift();";
    html += " chart2.data.datasets[0].data.shift();";
    html += " }";
    html += " chart2.update();";
    html += " }";
    html += " };";
    html += " xhttp.open('GET', '/signal', true);";
    html += " xhttp.send();";
    html += "}, 500);";  
    html += "</script>";
    html += "</body></html>";
    request->send(200, "text/html", html);
  });

  server.on("/signal", HTTP_GET, [](AsyncWebServerRequest* request) {
    DynamicJsonDocument doc(1024);
    doc["time"] = millis();
    doc["psi"] = psi;  // Send filtered psi data
    String jsonResponse;
    serializeJson(doc, jsonResponse);
    request->send(200, "application/json", jsonResponse);
  });
  server.begin();
}

void loop() {
  // Read data from MPU6050
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);  // Start reading accelerometer data
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6, true);  // Request 6 bytes: ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, etc.

  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();
  psi = (atan2(ay, az) * 180 / 3.14159) + 0.45;  // Convert to degrees
  Serial.println(psi);
  delay(200);
}
