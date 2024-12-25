#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

const char *ssid = "Thanh Phuc 4G";
const char *password = "12345678kst";

// Static IP address configuration
IPAddress local_IP(192, 168, 1, 184);  // Your desired static IP
IPAddress gateway(192, 168, 1, 1);     // Your network Gateway (usually your router's IP)
IPAddress subnet(255, 255, 255, 0);    // Your network Subnet

AsyncWebServer server(80);
String controlCommand = "";
String ipAddress = "";     // Khai báo biến toàn cục để lưu địa chỉ IP
String signalData = "";    // Biến để lưu dữ liệu tín hiệu
String receivedData = "";  // Biến để lưu dữ liệu MPU6050 nhận được

// Variables for the moving average filter
const int filterSize = 10;
int16_t thetaBuffer[filterSize];
int16_t psiBuffer[filterSize];
int16_t phiBuffer[filterSize];
int filterIndex = 0;

void connectToWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("Đang kết nối tới WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print('.');
  }
  Serial.println();

  // Hiển thị địa chỉ IP
  ipAddress = WiFi.localIP().toString();
  Serial.print("Đã kết nối tới WiFi, Địa chỉ IP: ");
  Serial.println(ipAddress);
}

void setup() {
  Serial.begin(9600);  // Serial với Arduino Mega 2560

  // Cố gắng cài đặt địa chỉ IP tĩnh
  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("STA Không cấu hình được");
  }

  connectToWiFi();

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    String html = "<!DOCTYPE html><html><head><title>Balanced 2-wheel vehicle parameter monitoring </title><style>";
    html += "body { font-family: Arial, sans-serif; text-align: center; }";
    html += "button { width: 100px; height: 50px; margin: 10px; border: none; border-radius: 5px; background-color: #4CAF50; color: white; font-size: 16px; cursor: pointer; transition: background-color 0.3s, transform 0.3s; }";
    html += "button:hover { background-color: #45a049; transform: scale(1.1); }";
    html += "button:active { background-color: #3e8e41; transform: scale(0.9); }";
    html += ".chart-container { display: flex; justify-content: space-around; flex-wrap: wrap; }";
    html += "canvas { width: 30%; height: 150px; }";  // Điều chỉnh kích thước cho biểu đồ nhỏ hơn
    html += "</style></head><body>";
    html += "<h1>Address IP: " + ipAddress + "</h1>";  // Hiển thị địa chỉ IP
    html += "<button onclick=\"sendCommand('1')\">Foward</button>";
    html += "<button onclick=\"sendCommand('3')\">Back</button>";
    html += "<button onclick=\"sendCommand('5')\">Left</button>";
    html += "<button onclick=\"sendCommand('7')\">Right</button>";
    html += "<button onclick=\"sendCommand('2')\">Stop</button>";
    html += "<div class=\"chart-container\">";
    html += "<canvas id=\"chart1\"></canvas>";  // Sử dụng <canvas> cho Chart.js
    html += "<canvas id=\"chart2\"></canvas>";  // Sử dụng <canvas> cho Chart.js
    html += "<canvas id=\"chart3\"></canvas>";  // Sử dụng <canvas> cho Chart.js
    html += "</div>";
    html += "<script src=\"https://cdn.jsdelivr.net/npm/chart.js\"></script>";
    html += "<script>";
    html += "function sendCommand(cmd) {";
    html += "  var xhttp = new XMLHttpRequest();";
    html += "  xhttp.open('GET', '/control?cmd=' + cmd, true);";
    html += "  xhttp.send();";
    html += "}";
    html += "var ctx1 = document.getElementById('chart1').getContext('2d');";
    html += "var ctx2 = document.getElementById('chart2').getContext('2d');";
    html += "var ctx3 = document.getElementById('chart3').getContext('2d');";
    html += "var chart1 = new Chart(ctx1, {";
    html += "  type: 'line',";
    html += "  data: { labels: [], datasets: [{ label: 'theta', data: [], borderColor: 'rgba(75, 192, 192, 1)', borderWidth: 1, pointRadius: 1 }] },";
    html += "  options: { scales: { x: { type: 'linear', position: 'bottom', ticks: { callback: function(value, index, values) { return value / 1000 + ' s'; } } } } },";
    html += "  options: { animation: false }";  // Disable animation for smooth updates
    html += "});";
    html += "var chart2 = new Chart(ctx2, {";
    html += "  type: 'line',";
    html += "  data: { labels: [], datasets: [{ label: 'psi', data: [], borderColor: 'rgba(192, 75, 192, 1)', borderWidth: 1, pointRadius: 1 }] },";
    html += "  options: { scales: { x: { type: 'linear', position: 'bottom', ticks: { callback: function(value, index, values) { return value / 1000 + ' s'; } } } } },";
    html += "  options: { animation: false }";  // Disable animation for smooth updates
    html += "});";
    html += "var chart3 = new Chart(ctx3, {";
    html += "  type: 'line',";
    html += "  data: { labels: [], datasets: [{ label: 'phi', data: [], borderColor: 'rgba(192, 192, 75, 1)', borderWidth: 1, pointRadius: 1 }] },";
    html += "  options: { scales: { x: { type: 'linear', position: 'bottom', ticks: { callback: function(value, index, values) { return value / 1000 + ' s'; } } } } },";
    html += "  options: { animation: false }";  // Disable animation for smooth updates
    html += "});";
    html += "setInterval(function() {";
    html += "  var xhttp = new XMLHttpRequest();";
    html += "  xhttp.onreadystatechange = function() {";
    html += "    if (this.readyState == 4 && this.status == 200) {";
    html += "      var data = JSON.parse(this.responseText);";
    html += "      chart1.data.labels.push(data.time);";
    html += "      chart1.data.datasets[0].data.push(data.theta);";
    html += "      if (chart1.data.labels.length > 100) {";
    html += "        chart1.data.labels.shift();";
    html += "        chart1.data.datasets[0].data.shift();";
    html += "      }";
    html += "      chart1.update();";
    html += "      chart2.data.labels.push(data.time);";
    html += "      chart2.data.datasets[0].data.push(data.psi);";
    html += "      if (chart2.data.labels.length > 100) {";
    html += "        chart2.data.labels.shift();";
    html += "        chart2.data.datasets[0].data.shift();";
    html += "      }";
    html += "      chart2.update();";
    html += "      chart3.data.labels.push(data.time);";
    html += "      chart3.data.datasets[0].data.push(data.phi);";
    html += "      if (chart3.data.labels.length > 100) {";
    html += "        chart3.data.labels.shift();";
    html += "        chart3.data.datasets[0].data.shift();";
    html += "      }";
    html += "      chart3.update();";
    html += "    }";
    html += "  };";
    html += "  xhttp.open('GET', '/signal', true);";
    html += "  xhttp.send();";
    html += "}, 100);";  // Update every 100 milliseconds
    html += "</script>";
    html += "</body></html>";
    request->send(200, "text/html", html);
  });

  server.on("/control", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("cmd")) {
      controlCommand = request->getParam("cmd")->value();
      request->send(200, "text/plain", "Lệnh đã nhận: " + controlCommand);
      Serial.println(controlCommand);  // Gửi lệnh tới Arduino Mega 2560
    } else {
      request->send(404, "text/plain", "Không có lệnh nào được gửi");
    }
  });

  server.on("/signal", HTTP_GET, [](AsyncWebServerRequest *request) {
    DynamicJsonDocument doc(1024);
    doc["time"] = millis();
    // Phân tích dữ liệu nhận được thành theta, psi, phi
    int16_t theta, psi, phi;
    sscanf(signalData.c_str(), "Accel: %d,%d,%d", &theta, &psi, &phi);

    // Apply moving average filter
    thetaBuffer[filterIndex] = theta;
    psiBuffer[filterIndex] = psi;
    phiBuffer[filterIndex] = phi;
    filterIndex = (filterIndex + 1) % filterSize;

    float sumTheta = 0, sumPsi = 0, sumPhi = 0;
    for (int i = 0; i < filterSize; i++) {
      sumTheta += thetaBuffer[i];
      sumPsi += psiBuffer[i];
      sumPhi += phiBuffer[i];
    }

    float avgTheta = sumTheta / filterSize;
    float avgPsi = sumPsi / filterSize;
    float avgPhi = sumPhi / filterSize;

    doc["theta"] = avgTheta;
    doc["psi"] = avgPsi;
    doc["phi"] = avgPhi;

    String json;
    serializeJson(doc, json);
    request->send(200, "application/json", json);

    // Hiển thị giá trị lên Serial Monitor
    Serial.print("Giá trị nhận được - Theta: ");
    Serial.print(avgTheta);
    Serial.print(", Psi: ");
    Serial.print(avgPsi);
    Serial.print(", Phi: ");
    Serial.println(avgPhi);
  });

  server.begin();
}

void loop() {
  if (Serial.available()) {
    signalData = Serial.readString();
    Serial.println("Nhận từ Arduino: " + signalData);
  }
  if (controlCommand != "") {
    Serial.println(controlCommand);  // Gửi lệnh tới Arduino Mega 2560
    controlCommand = "";
  }
  if (WiFi.status() != WL_CONNECTED) {
    connectToWiFi();
  }
  delay(50);
}
