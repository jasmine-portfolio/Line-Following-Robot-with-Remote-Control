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
  WiFi-Controlled Display with Button Input

  Description: This program runs on an ESP32/ESP8266 microcontroller and connects to a Wi-Fi network. 
    It communicates with another ESP32 over UDP, sending button press states and receiving power 
    and RPM data. The received data is displayed on a TFT screen (ST7735 or ST7789) with dynamic updates. 
    The program features button debouncing, state management, and display updates for system status.

  Author: Jasmine
  Date: December 3, 2024
*/
#include <WiFi.h>
#include <WiFiUdp.h>

#include <Adafruit_GFX.h>    // Core graphics library for drawing on TFT screen
#include <Adafruit_I2CDevice.h>
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>

// Pin definitions based on ESP32 or ESP8266 board
#if defined(ARDUINO_FEATHER_ESP32) // Feather Huzzah32
  #define TFT_CS         15   // Chip select pin
  #define TFT_RST         4   // Reset pin
  #define TFT_DC          2   // Data/Command pin

#elif defined(ESP8266)  // ESP8266 specific configuration
  #define TFT_CS         4
  #define TFT_RST        16
  #define TFT_DC         5

#else  // Default configuration for other ESP32 boards or breakout boards
  #define TFT_CS        15
  #define TFT_RST        4 // Or set to -1 and connect to Arduino RESET pin
  #define TFT_DC         2
#endif

// SPI Pins for TFT communication
#define TFT_MOSI 13  // Data out pin for SPI
#define TFT_SCLK 14  // Clock out pin for SPI

// Create the Adafruit_ST7735 object for controlling the TFT display
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

// WiFi credentials for network connection
const char *ssid = "name"; // WiFi SSID (Network name)
const char *password = "password"; // WiFi password
const char *udpAddress = "XXX.XXX.X.XXX"; // IP address of the second ESP32
const int udpPort = 12345;                // UDP port to send/receive data

WiFiUDP Udp;  // Create a UDP object for communication

// Button pins configuration for four buttons
const int buttonPins[] = {6, 8, 7, 5}; // Pins for Button 1, 2, 3, and 4
byte dataSent[] = {0, 0, 0}; // Array to store button states and values
byte controlState = 2;  // Default control state
byte idealSpeed = 40;   // Default speed value

// Variables to store button states and handle debouncing
int buttonStates[4] = {LOW, LOW, LOW, LOW};  
int lastButtonStates[4] = {LOW, LOW, LOW, LOW}; 
unsigned long lastDebounceTimes[4] = {0, 0, 0, 0}; // Time tracking for debouncing

static byte lastUartState = 0; // Store the last UART state
byte UartState = 0; // Current UART state
int powerData = 0; // Variable to store incoming power data
int rpmData = 0; // Variable to store incoming RPM data

const unsigned long debounceDelay = 20; // Debounce time in milliseconds

// Function to check the state of a button and handle debouncing
bool checkButton(int index) {
  int reading = digitalRead(buttonPins[index]);  // Read the button pin

  // If button state has changed, reset debounce timer
  if (reading != lastButtonStates[index]) {
    lastDebounceTimes[index] = millis(); // Reset the debounce timer
  }

  // If the debounce delay has passed, check the button state
  if ((millis() - lastDebounceTimes[index]) > debounceDelay) {
    // If button state has changed, update the button state
    if (reading != buttonStates[index]) {
      buttonStates[index] = reading;

      // If the button is pressed (LOW state), return true
      if (buttonStates[index] == LOW) {
        return true; // Button pressed
      }
    }
  }

  // Save the reading for the next loop
  lastButtonStates[index] = reading;
  return false; // Button not pressed
}

// Function to update and draw display
void drawDisplay(){
  tft.fillScreen(ST77XX_BLACK);  // Clear the display
  
  // Display the state of the On/Off button based on the UART state
  if (UartState & 0b10000000) {  
    tft.setCursor(10, 10);
    tft.setTextColor(ST77XX_GREEN);
    tft.setTextSize(2);
    tft.println("On");
  } else {
    tft.setCursor(10, 10);
    tft.setTextColor(ST77XX_RED);
    tft.setTextSize(2);
    tft.println("Off");
  }

  // Display the state of the Speaker button based on the UART state
  if (UartState & 0b01000000) {  
    tft.setCursor(10, 30);
    tft.setTextColor(ST77XX_GREEN);
    tft.setTextSize(2);
    tft.println("Spk On");
  } else {
    tft.setCursor(10, 30);
    tft.setTextColor(ST77XX_RED);
    tft.setTextSize(2);
    tft.println("Spk Off");
  }

  // Extract and display the ideal speed and power data
  byte mask = 0b00111110; // Mask for bits 5-1
  byte extractedBits = (UartState & mask) >> 1; // Extract bits for speed control
  tft.setCursor(20, 50);
  tft.setTextColor(ST77XX_BLUE);
  tft.setTextSize(2);
  tft.println(idealSpeed);
  
  tft.setCursor(80, 50);
  tft.setTextColor(ST77XX_BLUE);
  tft.setTextSize(2);
  tft.println("cyc");

  // Display power data on the screen
  tft.setCursor(20, 70);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setTextSize(2);
  tft.println(powerData);
  
  tft.setCursor(80, 70);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setTextSize(2);
  tft.println("mW");

  // Display RPM data on the screen
  tft.setCursor(20, 100);
  tft.setTextColor(ST77XX_GREEN);
  tft.setTextSize(2);
  tft.println(rpmData);
  
  tft.setCursor(80, 100);
  tft.setTextColor(ST77XX_GREEN);
  tft.setTextSize(2);
  tft.println("rpm");
}

void setup() {
  // Initialize button pins with pull-up resistors
  for (int i = 0; i < 4; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
  }

  Serial.begin(115200);  // Initialize serial communication

  // Connect to WiFi network
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);  // Start WiFi connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);  // Wait for WiFi connection
    Serial.print(".");
  }
  Serial.println("Connected to WiFi!");
  Serial.print("Sender ESP32 IP Address: ");
  Serial.println(WiFi.localIP());  // Display the IP address

  Udp.begin(udpPort);  // Start the UDP communication

  // Initialize the TFT display
  tft.initR(INITR_144GREENTAB); // Initialize display (green tab version)
}

void loop() {
  // Handle On/Off button
  if (checkButton(0)) {
    dataSent[0] = !dataSent[0]; // Toggle the On/Off state
  }

  // Handle increase speed button
  if (checkButton(1) && (dataSent[1] <= 5)) {
    dataSent[1]++;
    idealSpeed += 10;  // Increase ideal speed by 10
  }

  // Handle decrease speed button
  if (checkButton(2) && (dataSent[1] > 0)) {
    dataSent[1]--;
    idealSpeed -= 10;  // Decrease ideal speed by 10
  }

  // Read the speaker button state
  dataSent[2] = digitalRead(buttonPins[3]); 

  // Construct UART state using bitwise operations
  UartState = (dataSent[0] << 7) | // On/Off state (bit 7)
              (dataSent[2] << 6) | // Speaker state (bit 6)
              (dataSent[1] & 0x1F) | // Speed control (bits 5-1)
              0b00000001; // Constant 1 (bit 0)

  // Send the UART state over UDP if it has changed
  if (UartState != lastUartState) {
    Serial.println(UartState, BIN); // Display the UART state in binary

    // Send the data over UDP
    Udp.beginPacket(udpAddress, udpPort); // Start UDP packet
    Udp.write(dataSent[0]); // Send On/Off state
    Udp.write(idealSpeed); // Send ideal speed
    Udp.write(dataSent[2]); // Send Speaker state
    Udp.endPacket(); // End the UDP packet

    Serial.println("UartState sent over WiFi!");

    lastUartState = UartState;  // Update the last UART state
  }

  // Handle receiving data over UDP
  int packetSize = Udp.parsePacket(); // Check for incoming packet
  if (packetSize) {
    Udp.read((uint8_t*)&powerData, sizeof(powerData));  // Read power data
    Serial.print("Received power: ");
    Serial.println(powerData);

    Udp.read((uint8_t*)&rpmData, sizeof(rpmData));  // Read RPM data
    Serial.print("Received rpm: ");
    Serial.println(rpmData);
  }

  // Update display if there was a packet or UART state change
  if (packetSize || (UartState != lastUartState)) {
    drawDisplay();  // Draw the updated display
  }

  delay(10); // Short delay for better responsiveness
}
