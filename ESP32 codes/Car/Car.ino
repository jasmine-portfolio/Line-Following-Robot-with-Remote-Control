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
  ESP32 UDP Receiver and UART Communication

  Description:  * This program sets up an ESP32 to receive UDP packets containing 
 data (3 bytes) and sends them over UART to another device. It also 
 listens for data from the UART, processes it, and sends it back 
 via UDP to another ESP32 device. The program uses WiFi for network 
 communication and UDP for data transmission, allowing real-time 
 exchange of data between two ESP32 devices.
  
 Author: Jasmine
 Date: December 3, 2024
 **************************************************************/
#include <WiFi.h>
#include <WiFiUdp.h>

// WiFi credentials - Replace with your own network details
const char *ssid = "name"; // WiFi SSID (Network name)
const char *password = "password"; // WiFi password
const char *udpAddress = "XXX.XXX.X.XXX"; // IP address of the second ESP32
const int udpPort = 12345;       // UDP port to listen on

// Variables for power consumption and RPM values
int powerConsumption = 260; // Initial power consumption value
int rmp = 14;               // Initial RPM value

// Create a WiFiUDP object to handle UDP communication
WiFiUDP Udp;

void setup() {
    Serial.begin(115200);  // Initialize Serial communication for debugging
     
    Serial.println("ESP32 UDP Receiver");  // Output to indicate that the ESP32 has started

    // Connect to WiFi
    Serial.print("Connecting to WiFi...");
    WiFi.begin(ssid, password);  // Start WiFi connection

    // Wait for connection
    int attempts = 0;  // Keep track of connection attempts
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);  // Wait for 500ms before checking again
        Serial.print(".");  // Print a dot to show progress
        attempts++;  // Increment attempt counter
    }

    // Check if connected to WiFi
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("Connected to WiFi!");  // Connection successful
        Serial.print("Receiver ESP32 IP Address: ");
        Serial.println(WiFi.localIP());  // Print the IP address of the ESP32
        
    } else {
        Serial.println("Failed to connect to WiFi.");  // Connection failed
    }

    Udp.begin(udpPort); // Start listening for UDP packets on the specified port

    // Initialize Serial1 communication for UART (TX on GPIO 17, RX on GPIO 16)
    Serial1.begin(115200, SERIAL_8N1, 16, 17); 
}

void loop() {
    int packetSize = Udp.parsePacket(); // Check if there's an incoming UDP packet
    if (packetSize) {  // If a packet is received
        byte incomingByte[3]; // Variable to store incoming bytes

        Serial.print("Received packet: "); // Print message when packet is received

        // Read the three bytes from the UDP packet
        Udp.read(incomingByte, sizeof(incomingByte));

        // Store the bytes into variables for further processing
        byte dataSent0 = incomingByte[0]; // First byte of the data
        byte dataSent1 = incomingByte[1]; // Second byte of the data
        byte dataSent2 = incomingByte[2]; // Third byte of the data

        // Print the received bytes in binary format for debugging
        Serial.print("dataSent[0]: ");
        Serial.println(dataSent0, BIN);
        Serial.print("dataSent[1]: ");
        Serial.println(dataSent1, BIN);
        Serial.print("dataSent[2]: ");
        Serial.println(dataSent2, BIN);

        // Send the received data over Serial1 (UART) to another device
        Serial1.write((byte)2); // Send a control byte
        Serial1.write((byte)dataSent0); // Send the first byte of the received data
        Serial1.write((byte)dataSent1); // Send the second byte of the received data
        Serial1.write((byte)dataSent2); // Send the third byte of the received data
    }

    // Check if data is available from Serial1 (UART)
    uint16_t receivedData1, receivedData2;
    byte num1, num2, num3, num4;
    while (Serial1.available() > 0) {  // If there are bytes available to read
        num1 = Serial1.read();  // Read the first byte
        num2 = Serial1.read();  // Read the second byte
        num3 = Serial1.read();  // Read the third byte
        num4 = Serial1.read();  // Read the fourth byte
        receivedData1 = num2 * 256 + num1;  // Combine bytes to form a 16-bit value
        receivedData2 = num4 * 256 + num3;  // Combine bytes to form a second 16-bit value
        Serial.print("Received: ");
        Serial.println(receivedData1);  // Print the first received value to Serial Monitor

        // Update the power consumption and RPM values with the received data
        powerConsumption = receivedData1;
        rmp = receivedData2;

        // Start a new UDP packet and send the updated values
        Udp.beginPacket(udpAddress, udpPort);  // Start the UDP packet to the specified address and port
        Udp.write((const uint8_t*)&powerConsumption, sizeof(int)); // Send the power consumption as an integer
        Udp.write((const uint8_t*)&rmp, sizeof(int)); // Send the RPM as an integer
        Udp.endPacket();  // End the UDP packet
    }

    delay(100);  // Small delay to prevent excessive CPU usage
}
