#include <Servo.h>
#include <SoftwareSerial.h> // Include the SoftwareSerial library
#include <HardwareSerial.h>
#include <Arduino.h>

// --- HC-05 SoftwareSerial Pins ---
// Define RX and TX pins for SoftwareSerial.
// Choose any digital pins other than 0 (RX) and 1 (TX) which are used by hardware Serial.
// Common choices are pins 2 and 3, or 10 and 11, etc.
// IMPORTANT: Connect HC-05's TX to Arduino's RX_PIN (e.g., 2)
//            Connect HC-05's RX to Arduino's TX_PIN (e.g., 3) -- REMEMBER VOLTAGE DIVIDER FOR HC-05 RX!
const int HC05_RX_PIN = 2; // Arduino pin connected to HC-05 TX
const int HC05_TX_PIN = 3; // Arduino pin connected to HC-05 RX (via voltage divider)

SoftwareSerial bluetooth(HC05_RX_PIN, HC05_TX_PIN); // Create SoftwareSerial object

// --- Servo Configuration ---
const int NUM_SERVOS = 5;
Servo myServos[NUM_SERVOS];

// Assign PWM pins for each servo. These should be PWM capable pins (marked with ~ on Uno).
const int servoPins[NUM_SERVOS] = {3, 5, 6, 9, 10}; // Example PWM pins.
                                                   // Note: Pin 3 is used by SoftwareSerial TX here,
                                                   //       so if it's a PWM pin, it might conflict.
                                                   //       Let's adjust to avoid overlap if possible.
                                                   //       We'll change HC05_TX_PIN to 4.

/*
Let's re-evaluate the pins to avoid potential conflicts:
- Hardware Serial: Pins 0 (RX) and 1 (TX) - Used for USB communication/uploading.
- SoftwareSerial: HC05_RX_PIN (e.g., 2), HC05_TX_PIN (e.g., 4)
- Servos: PWM pins like 5, 6, 9, 10, 11
So, our chosen pins look good.
*/

// Updated HC-05 SoftwareSerial Pins to avoid conflicts if servo 3 was intended for PWM
const int New_HC05_RX_PIN = 2; // Arduino pin connected to HC-05 TX
const int New_HC05_TX_PIN = 4; // Arduino pin connected to HC-05 RX (via voltage divider)

SoftwareSerial newBluetooth(New_HC05_RX_PIN, New_HC05_TX_PIN); // Create SoftwareSerial object

// Updated Servo Pins after re-evaluation
const int newServoPins[NUM_SERVOS] = {5, 6, 9, 10, 11}; // Example PWM pins

void setup() {
  // Initialize hardware serial for debugging (to Serial Monitor)
  Serial.begin(9600);
  Serial.println("Arduino Initializing...");

  // Initialize SoftwareSerial for HC-05 communication
  newBluetooth.begin(9600); // HC-05 default baud rate is often 9600 or 38400
  newBluetooth.println("HC-05 Ready!");

  // Attach servos to their respective pins and set initial position
  for (int i = 0; i < NUM_SERVOS; i++) {
    myServos[i].attach(newServoPins[i]);
    myServos[i].write(90); // Initial position (center 90 degrees)
    Serial.print("Servo ");
    Serial.print(i);
    Serial.println(" attached and set to 90 degrees.");
  }

  Serial.println("Ready to receive servo commands via Bluetooth.");
}

void loop() {
  if (newBluetooth.available()) { // Check if data is available from HC-05
    String data = newBluetooth.readStringUntil('\n'); // Read until newline character
    data.trim(); // Remove any leading/trailing whitespace

    // Optional: Print received data to Serial Monitor for debugging
    Serial.print("Received data: \"");
    Serial.print(data);
    Serial.println("\"");

    int servoValues[NUM_SERVOS];
    int currentServo = 0;
    String currentNumber = "";

    // Parse the comma-separated values (CSV) string
    for (int i = 0; i < data.length(); i++) {
      char c = data.charAt(i);
      if (c == ',') {
        if (currentServo < NUM_SERVOS) {
          servoValues[currentServo] = currentNumber.toInt();
          currentServo++;
          currentNumber = "";
        }
      } else {
        currentNumber += c;
      }
    }
    // Add the last number after the loop
    if (currentServo < NUM_SERVOS) {
      servoValues[currentServo] = currentNumber.toInt();
    }

    // Write the received values to the servos
    for (int i = 0; i < NUM_SERVOS; i++) {
      // Basic bounds checking on Arduino side for robustness.
      // Python app should already send values within 0-180,
      // but this adds an extra layer of safety.
      int angle = constrain(servoValues[i], 0, 180);
      myServos[i].write(angle);
      Serial.print("Servo ");
      Serial.print(i);
      Serial.print(" set to ");
      Serial.print(angle);
      Serial.println(" degrees.");
    }
  }

  // You can add other Arduino logic here if needed
}