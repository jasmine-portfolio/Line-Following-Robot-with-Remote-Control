/*
 * MIT License
 * 
 * Copyright (c) 2025 jasmine-portfolio
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
/*
  Car Control and PID Web Interface

  Description:
  This program creates a web interface to control a car model and a PID controller.
  It allows users to toggle between car control and PID control modes. In car control mode, 
  users can control the car's state, speed, and speaker state. In PID control mode, 
  users can adjust the PID constants (Kp, Ki, Kd) and publish them.

  Author: Jasmine
  Date: December 3, 2024


*/
#include <WiFi.h>
#include <WebServer.h>

// Replace with your network credentials
const char *ssid = "name";           // WiFi SSID
const char *password = "password";   // WiFi Password

WebServer server(80);  // Set up web server to listen on port 80

// Car control parameters
int carDataSent[] = {0, 0, 0};  // {carState, speed, speakerState}
byte UartState = 0;  // Current UART state, used for communication with external device

// PID control variables (range 0-255 for byte type)
byte Kp = 15;  // Default proportional constant
byte Ki = 0;   // Default integral constant
byte Kd = 0;   // Default derivative constant

// Mode Switch state (0 = Car Control, 1 = PID Control)
bool modeSwitchState = false;

void connectToWiFi() {
  // Begin the WiFi connection process
  WiFi.begin(ssid, password);
  Serial.print("Connecting to ");
  Serial.print(ssid);  // Show the SSID of the WiFi being connected to
  while (WiFi.status() != WL_CONNECTED) {  // Wait for connection
    delay(500);
    Serial.print(".");  // Show progress of connection attempt
  }
  Serial.println("Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());  // Print the device's IP address
}

// Function to display car data sent via Serial Monitor
void displayCarDataSent() {
  Serial.print("Car Data Sent: ");
  Serial.print("CarState: ");
  Serial.print(carDataSent[0]);
  Serial.print(", Speed: ");
  Serial.print(carDataSent[1]);
  Serial.print(", SpeakerState: ");
  Serial.println(carDataSent[2]);
  
  // Send the car data to an external UART device
  Serial1.write((byte)2);  // Send control signal to the external device
  Serial1.write((byte)carDataSent[0]);  // Send car state
  Serial1.write((byte)carDataSent[1]);  // Send speed
  Serial1.write((byte)carDataSent[2]);  // Send speaker state
}

// Main page for the web interface (UI) on the browser
void handleRoot() {
  String html = "<!DOCTYPE html><html lang=\"en\"><head>";
  html += "<meta charset=\"UTF-8\">";
  html += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">";
  html += "<title>Control Switch</title>";
  html += "<style>";
  // Add styling for the HTML elements (form controls, buttons, etc.)
  html += "body { font-family: Arial, sans-serif; background-color: #f5f5f5; margin: 0; padding: 0; display: flex; justify-content: center; align-items: center; height: 100vh; }";
  html += ".container { background: white; padding: 20px; border-radius: 8px; box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1); text-align: center; width: 300px; }";
  html += "h1 { font-size: 24px; color: #333; }";
  html += ".switch { position: relative; display: inline-block; width: 34px; height: 20px; }";
  html += ".switch input { opacity: 0; width: 0; height: 0; }";
  html += "input:checked + .sliderSwitch { background-color: #2196F3; }";
  html += "input:checked + .sliderSwitch:before { transform: translateX(14px); }";
  html += ".sliderSwitch { position: absolute; cursor: pointer; top: 0; left: 0; right: 0; bottom: 0; background-color: #ccc; transition: 0.4s; border-radius: 50px; }";
  html += ".sliderSwitch:before { position: absolute; content: \"\"; height: 12px; width: 12px; border-radius: 50px; left: 4px; bottom: 4px; background-color: white; transition: 0.4s; }";
  html += "</style>";
  html += "</head><body>";
  html += "<div class=\"container\">";
  html += "<h1>Control Mode</h1>";
  
  // Mode switch to toggle between Car Control and PID Control
  html += "<label>Mode Switch:</label><br>";
  html += "<label class=\"switch\"><input type=\"checkbox\" id=\"modeSwitch\" onclick=\"toggleMode(this)\"><span class=\"sliderSwitch\"></span></label>";
  html += "<br><span id=\"modeLabel\">Car Control</span><br>";

  // Car control UI elements (visible when mode is "Car Control")
  html += "<div id=\"carControlUI\">";
  html += "<label>Car State:</label><br>";
  html += "<label class=\"switch\"><input type=\"checkbox\" id=\"carState\" onclick=\"setCarState(this)\"><span class=\"sliderSwitch\"></span></label>";
  html += "<span id=\"carStateValue\">" + String(carDataSent[0]) + "</span><br>";

  html += "<label>Speaker State:</label><br>";
  html += "<label class=\"switch\"><input type=\"checkbox\" id=\"speakerState\" onclick=\"setSpeakerState(this)\"><span class=\"sliderSwitch\"></span></label>";
  html += "<span id=\"speakerStateValue\">" + String(carDataSent[2]) + "</span><br>";

  html += "<label>Speed (0-100):</label><br>";
  html += "<input type=\"range\" id=\"speed\" min=\"0\" max=\"100\" value=\"" + String(carDataSent[1]) + "\" onchange=\"setSpeed(this)\"><br>";
  html += "<span id=\"speedValue\">" + String(carDataSent[1]) + "</span><br>";
  html += "</div>";

  // PID control UI elements (visible when mode is "PID Control")
  html += "<div id=\"pidControlUI\" style=\"display:none;\">";
  html += "<p>Kp: <span id=\"KpValue\">" + String(Kp) + "</span></p>";
  html += "<input type=\"range\" id=\"KpSlider\" min=\"0\" max=\"255\" value=\"" + String(Kp) + "\" class=\"slider\" onchange=\"updateKpValue(this)\"><br>";

  html += "<p>Ki: <span id=\"KiValue\">" + String(Ki) + "</span></p>";
  html += "<input type=\"range\" id=\"KiSlider\" min=\"0\" max=\"255\" value=\"" + String(Ki) + "\" class=\"slider\" onchange=\"updateKiValue(this)\"><br>";

  html += "<p>Kd: <span id=\"KdValue\">" + String(Kd) + "</span></p>";
  html += "<input type=\"range\" id=\"KdSlider\" min=\"0\" max=\"255\" value=\"" + String(Kd) + "\" class=\"slider\" onchange=\"updateKdValue(this)\"><br>";

  html += "<button onclick=\"publishPID()\">Publish PID</button>";
  html += "</div>";

  html += "<script>";
  // JavaScript functions to handle UI interactions and send requests to the server
  html += "function toggleMode(el) {";
  html += "  var modeLabel = document.getElementById('modeLabel');";
  html += "  if (el.checked) {";
  html += "    modeLabel.textContent = 'PID Control';";
  html += "    document.getElementById('carControlUI').style.display = 'none';";
  html += "    document.getElementById('pidControlUI').style.display = 'block';";
  html += "    modeSwitchState = true;";
  html += "  } else {";
  html += "    modeLabel.textContent = 'Car Control';";
  html += "    document.getElementById('carControlUI').style.display = 'block';";
  html += "    document.getElementById('pidControlUI').style.display = 'none';";
  html += "    modeSwitchState = false;";
  html += "  }";
  html += "}";

  // Functions to handle each individual control's update (Car state, speaker state, speed, PID)
  html += "function setCarState(el) {";
  html += "  fetch('/setCarState?state=' + (el.checked ? 1 : 0));";
  html += "  document.getElementById('carStateValue').innerText = el.checked ? '1' : '0';";
  html += "}";

  html += "function setSpeakerState(el) {";
  html += "  fetch('/setSpeakerState?state=' + (el.checked ? 1 : 0));";
  html += "  document.getElementById('speakerStateValue').innerText = el.checked ? '1' : '0';";
  html += "}";

  html += "function setSpeed(el) {";
  html += "  fetch('/setSpeed?speed=' + el.value);";
  html += "  document.getElementById('speedValue').innerText = el.value;";
  html += "}";

  html += "function updateKpValue(el) {";
  html += "  document.getElementById('KpValue').innerText = el.value;";
  html += "}";

  html += "function updateKiValue(el) {";
  html += "  document.getElementById('KiValue').innerText = el.value;";
  html += "}";

  html += "function updateKdValue(el) {";
  html += "  document.getElementById('KdValue').innerText = el.value;";
  html += "}";

  html += "function publishPID() {";
  html += "  fetch('/setPID?Kp=' + document.getElementById('KpSlider').value + '&Ki=' + document.getElementById('KiSlider').value + '&Kd=' + document.getElementById('KdSlider').value);";
  html += "}";

  html += "</script>";
  html += "</body></html>";
  server.send(200, "text/html", html);  // Serve the generated HTML to the browser
}

void setup() {
  Serial.begin(115200);  // Initialize serial communication
  connectToWiFi();  // Connect to WiFi
  server.on("/", handleRoot);  // Handle requests to the root URL
  server.begin();  // Start the web server
}

void loop() {
  server.handleClient();  // Handle incoming client requests
}
