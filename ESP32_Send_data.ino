#include <WiFi.h>
#include <ESPAsyncWebServer.h>

const char* ssid = "Thanh Phuc 4G";
const char* password = "12345678kst";

// Static IP address configuration
IPAddress local_IP(192, 168, 1, 184); // Your desired static IP
IPAddress gateway(192, 168, 1, 1);    // Your network Gateway (usually your router's IP)
IPAddress subnet(255, 255, 255, 0);   // Your network Subnet

AsyncWebServer server(80);

String controlCommand = "";
String ipAddress = ""; // Khai báo biến toàn cục để lưu địa chỉ IP

void setup() {
  Serial.begin(9600); // Serial với Arduino Mega 2560

  // Attempt to set the static IP address
  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("STA Failed to configure");
  }

  WiFi.begin(ssid, password);

  // Kết nối đến Wi-Fi
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Hiển thị địa chỉ IP
  ipAddress = WiFi.localIP().toString();
  Serial.print("IP Address: ");
  Serial.println(ipAddress);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    String html = "<!DOCTYPE html><html><head><title>Điều khiển xe</title><style>button { width: 100px; height: 50px; margin: 10px; }</style></head><body>";
    html += "<h1>IP Address: " + ipAddress + "</h1>"; // Hiển thị địa chỉ IP
    html += "<button onclick=\"sendCommand('1')\">forward</button>";
    html += "<button onclick=\"sendCommand('3')\">back</button>";
    html += "<button onclick=\"sendCommand('5')\">left</button>";
    html += "<button onclick=\"sendCommand('7')\">right</button>";
    html += "<button onclick=\"sendCommand('2')\">stop</button>";
    html += "<script>function sendCommand(cmd) { var xhttp = new XMLHttpRequest(); xhttp.open('GET', '/control?cmd=' + cmd, true); xhttp.send(); }</script>";
    html += "</body></html>";
    request->send(200, "text/html", html);
  });

  server.on("/control", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("cmd")) {
      controlCommand = request->getParam("cmd")->value();
      request->send(200, "text/plain", "Command received: " + controlCommand);
      Serial.println(controlCommand); // Gửi lệnh tới Arduino Mega 2560
    } else {
      request->send(404, "text/plain", "No command sent");
    }
  });

  server.begin();
}

void loop() {
  if (Serial.available()) {
    String controlCommand = Serial.readString();
    Serial.println("Received from Arduino: " + controlCommand);
  }

  if (controlCommand != "") {
    Serial.println(controlCommand); // Gửi lệnh tới Arduino Mega 2560
    controlCommand = "";
  }
}
