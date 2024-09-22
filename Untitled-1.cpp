#include <PS3BT.h>
#include <BluetoothSerial.h>

// Pin definitions for motor control
#define IN1 14   // Pin for Motor A forward
#define IN2 27   // Pin for Motor A backward
#define IN3 26   // Pin for Motor B forward
#define IN4 25   // Pin for Motor B backward
#define ENA 32   // PWM pin for Motor A speed
#define ENB 33   // PWM pin for Motor B speed

BluetoothSerial SerialBT;   // Bluetooth Serial object

// Variables to store motor speed
int motorSpeedA = 0;
int motorSpeedB = 0;

// Setup function
void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);

  // Setup pins for motor control
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Initialize Bluetooth
  if (!SerialBT.begin("ESP32")) {
    Serial.println("Bluetooth initialization failed");
  } else {
    Serial.println("Bluetooth started. Waiting for PS3 controller...");
  }
  
  PS3.begin("XX:XX:XX:XX:XX:XX");  // Replace XX:XX:XX:XX:XX:XX with the MAC address of your PS3 controller
}

// Function to control motors
void controlMotors(int motorA, int motorB) {
  if (motorA > 0) {
    // Move motor A forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (motorA < 0) {
    // Move motor A backward
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    // Stop motor A
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  if (motorB > 0) {
    // Move motor B forward
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else if (motorB < 0) {
    // Move motor B backward
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    // Stop motor B
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  // Set motor speeds using PWM
  analogWrite(ENA, abs(motorA));
  analogWrite(ENB, abs(motorB));
}

void loop() {
  if (PS3.isConnected()) {
    // Read joystick values for movement
    int leftY = PS3.data.analog.stick.ly;  // Left joystick Y-axis (Motor A control)
    int rightY = PS3.data.analog.stick.ry; // Right joystick Y-axis (Motor B control)

    // Map joystick values to motor speed (-255 to 255)
    motorSpeedA = map(leftY, 0, 255, -255, 255);
    motorSpeedB = map(rightY, 0, 255, -255, 255);

    // Control motors based on joystick input
    controlMotors(motorSpeedA, motorSpeedB);

    // Check for button presses
    if (PS3.data.button.cross) {
      // If the cross button is pressed, stop the motors
      controlMotors(0, 0);
      Serial.println("Cross button pressed. Motors stopped.");
    }
  } else {
    Serial.println("PS3 controller not connected");
  }
  
  delay(50);  // Small delay to avoid overloading the loop
}
