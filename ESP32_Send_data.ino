#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

// Configurations for static IP
IPAddress local_IP(192, 168, 1, 184);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8); // optional
IPAddress secondaryDNS(8, 8, 4, 4); // optional

const char* ssid = "Thanh Phuc 4G";
const char* password = "12345678kst";

AsyncWebServer server(80);

String data1 = "0", data2 = "0", data3 = "0";

void setup() {
  Serial.begin(9600);

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
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    String html = "<!DOCTYPE html><html><head><title>Balanced 2-wheel vehicle parameter monitoring</title><style>";
    html += "body { font-family: Arial, sans-serif; text-align: center; }";
    html += "button { width: 100px; height: 50px; margin: 10px; border: none; border-radius: 5px; background-color: #4CAF50; color: white; font-size: 16px; cursor: pointer; transition: background-color 0.3s, transform 0.3s; }";
    html += "button:hover { background-color: #45a049; transform: scale(1.1); }";
    html += "button:active { background-color: #3e8e41; transform: scale(0.9); }";
    html += ".chart-container { display: flex; justify-content: center; flex-wrap: wrap; }";
    html += "canvas { width: 30%; height: 300px; }";  // Adjust size for three charts
    html += "</style></head><body>";
    html += "<h1>Address IP: " + WiFi.localIP().toString() + "</h1>";  // Hiển thị địa chỉ IP
    html += "<button onclick=\"sendCommand('1')\">Forward</button>";
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
    html += "  data: { labels: [], datasets: [{ label: 'psi', data: [], borderColor: 'rgba(192, 75, 75, 1)', borderWidth: 1, pointRadius: 1 }] },";
    html += "  options: { scales: { x: { type: 'linear', position: 'bottom', ticks: { callback: function(value, index, values) { return value / 1000 + ' s'; } } } } },";
    html += "  options: { animation: false }";  // Disable animation for smooth updates
    html += "});";
    html += "var chart3 = new Chart(ctx3, {";
    html += "  type: 'line',";
    html += "  data: { labels: [], datasets: [{ label: 'phi', data: [], borderColor: 'rgba(75, 75, 192, 1)', borderWidth: 1, pointRadius: 1 }] },";
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
    html += "      chart2.data.labels.push(data.time);";
    html += "      chart2.data.datasets[0].data.push(data.psi);";
    html += "      chart3.data.labels.push(data.time);";
    html += "      chart3.data.datasets[0].data.push(data.phi);";
    html += "      if (chart1.data.labels.length > 100) {";
    html += "        chart1.data.labels.shift();";
    html += "        chart1.data.datasets[0].data.shift();";
    html += "      }";
    html += "      if (chart2.data.labels.length > 100) {";
    html += "        chart2.data.labels.shift();";
    html += "        chart2.data.datasets[0].data.shift();";
    html += "      }";
    html += "      if (chart3.data.labels.length > 100) {";
    html += "        chart3.data.labels.shift();";
    html += "        chart3.data.datasets[0].data.shift();";
    html += "      }";
    html += "      chart1.update();";
    html += "      chart2.update();";
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

  server.on("/control", HTTP_GET, [](AsyncWebServerRequest *request){
    String command;
    if (request->hasParam("cmd")) {
      command = request->getParam("cmd")->value();
      Serial.println("Command received: " + command);
    }
    request->send(200, "text/plain", "OK");
  });

  server.on("/signal", HTTP_GET, [](AsyncWebServerRequest *request){
    DynamicJsonDocument doc(1024);
    doc["time"] = millis();
    doc["theta"] = data1.toFloat(); // Example value
    doc["psi"] = data2.toFloat(); // Example value
    doc["phi"] = data3.toFloat(); // Example value
    String jsonResponse;
    serializeJson(doc, jsonResponse);
    request->send(200, "application/json", jsonResponse);
  });

  server.begin();
}

void loop() {
  // Kiểm tra nếu có dữ liệu từ Arduino Uno gửi đến
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    Serial.println("Data from Arduino Uno: " + data);

    // Giả sử dữ liệu là chuỗi được phân cách bởi dấu phẩy
    int firstComma = data.indexOf(',');
    int secondComma = data.indexOf(',', firstComma + 1);

    data1 = data.substring(0, firstComma);
    data2 = data.substring(firstComma + 1, secondComma);
    data3 = data.substring(secondComma + 1);
  }
  // delay(1000); // Đợi 1 giây giữa các lần gửi
}
