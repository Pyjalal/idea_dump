
// Include Arduino standard library to access standard functions
#include <Arduino.h>

#define leftEncoder A1
#define rightEncoder A0

// Store pulse counts
volatile int pulsesLeft = 0;
volatile int pulsesRight = 0;

// Track previous states
int leftLastState = LOW;
int rightLastState = LOW;

const float wheelCircumference = PI * 6.5;   // 6.5 cm diameter
const int pulsesPerRotation = 20;            // 20 pulses per rotation

// Encoder setup
void encoderSetup() {
    pinMode(leftEncoder, INPUT);
    pinMode(rightEncoder, INPUT);
   
}

// Update encoder pulses manually
void updateEncoders() {
    int leftState = digitalRead(leftEncoder);
    int rightState = digitalRead(rightEncoder);

    if (leftState == HIGH && leftLastState == LOW) {
        pulsesLeft++;
    }
    if (rightState == HIGH && rightLastState == LOW) {
        pulsesRight++;
    }

    // Update last states
    leftLastState = leftState;
    rightLastState = rightState;
}

// Calculate distance
float getLeftMovingDistance() {
    noInterrupts();
    return (pulsesLeft / (float)pulsesPerRotation) * wheelCircumference;
    interrupts();
}

float getRightMovingDistance() {
    noInterrupts();
    return (pulsesRight / (float)pulsesPerRotation) * wheelCircumference;
    interrupts();
}

float getAverageDistance() {
    return ((getLeftMovingDistance() + getRightMovingDistance()) / 2);
}

// Reset distances
void resetLeftDistance() {
    pulsesLeft = 0;
}

void resetRightDistance() {
    pulsesRight = 0;
}
//Gyro

#include <Arduino.h>
#include <Wire.h>

// MPU6050 I2C address
const int MPU = 0x68;

// Gyroscope sensitivity scale factor (MPU6050 default: 131 LSB per dps)
const float GYRO_SCALE = 1.0 / 131.0;

// Variables for gyro readings
float gyroOutputBuffer = 0;
float GyroErrorZ = 0;

// Time variables
float previousTime, currentTime, elapsedTime;

// Angle calculation
float yaw = 0;
volatile float angle = 0;

// Auto drift reset
unsigned long lastMovementTime = 0;
const unsigned long resetThreshold = 3000; // 3 seconds

// Function to get Z-axis rotation (yaw) from MPU6050
void getOrientation() {
    gyroOutputBuffer = 0;
    
    Wire.beginTransmission(MPU);
    Wire.write(0x47); // Gyroscope Z-axis high byte register
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 2, true);
    
    // Read and combine high and low byte
    gyroOutputBuffer = (Wire.read() << 8 | Wire.read()) * GYRO_SCALE;
}

// Function to calculate gyro drift error
void calculateError() {
    byte count = 0;
    GyroErrorZ = 0;

    // Take 200 readings to calculate average bias
    /*while (count < 200) {
        getOrientation();
        GyroErrorZ += gyroOutputBuffer;
        count++;
    }*/

    GyroErrorZ /= 200; // Average error
    Serial.println("MPU6050 gyroscope calibrated.");
}

// Function to initialize MPU6050
void mpuSetup() {
    Wire.begin();                    // Start I2C communication
    Wire.beginTransmission(MPU);
    Wire.write(0x6B);                 // Power management register
    Wire.write(0x00);                 // Wake up MPU6050
    Wire.endTransmission(true);

    // Calibrate the gyroscope
    calculateError();

    // Initialize timing variables
    previousTime = millis();
}

// Function to update yaw angle
void update() {
    // Get current time
    currentTime = millis();
    elapsedTime = (currentTime - previousTime) / 1000.0; // Convert to seconds
    previousTime = currentTime;

    // Get gyro data
    getOrientation();

    // Apply drift correction
    gyroOutputBuffer -= GyroErrorZ;

    // Ignore small noise below threshold
    if (abs(gyroOutputBuffer) < 0.05) {
        gyroOutputBuffer = 0;
    } else {
        lastMovementTime = millis(); // Reset timer when movement is detected
    }

    // Integrate to calculate yaw
    yaw += gyroOutputBuffer * elapsedTime;
    angle = round(yaw * 10) / 10.0;

    // Reset drift if stationary for too long
    if (millis() - lastMovementTime > resetThreshold) {
        yaw = 0;
        angle = 0;
    }
}
//Ultrasonic
// Include Arduino standard library to access standard functions
#include <Arduino.h>

// Front ultrasonic sensor
#define frontTrig 6
#define frontEcho 7
#define leftTrig 8
#define leftEcho 11
#define rightTrig 12
#define rightEcho 13

// Ultrasonic part setup code
void ultrasonicSetup(){
    pinMode(frontEcho, INPUT);
    pinMode(frontTrig, OUTPUT);
    pinMode(leftEcho, INPUT);
    pinMode(leftTrig, OUTPUT);
    pinMode(rightEcho, INPUT);
    pinMode(rightTrig, OUTPUT);
}

// Return distance in cm
float getFrontDistance(){
    digitalWrite(frontTrig,LOW);
    delay(2);
    digitalWrite(frontTrig,HIGH);
    delay(10);
    digitalWrite(frontTrig,LOW);
    return (pulseIn(frontEcho,HIGH) * 0.034613 / 2.00);
}

float getLeftDistance(){
    digitalWrite(leftTrig,LOW);
    delay(2);
    digitalWrite(leftTrig,HIGH);
    delay(10);
    digitalWrite(leftTrig,LOW);
    return (pulseIn(leftEcho,HIGH) * 0.034613 / 2.00);
}

float getRightDistance(){
    digitalWrite(rightTrig,LOW);
    delay(2);
    digitalWrite(rightTrig,HIGH);
    delay(10);
    digitalWrite(rightTrig,LOW);
    return (pulseIn(rightEcho,HIGH) * 0.034613 / 2.00);
}
// How to use?
// ultrasonicSetup(); in setup

// getDistance(); to obtain distance
//Motor
// Include Arduino standard library to access standard functions
#include <Arduino.h>

// Direction control pins
#define IN1 5
#define IN2 4
#define IN3 3
#define IN4 2
#define ENA 10
#define ENB 9


// Stop motors
void stopMotors() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}


// Move forward
void moveForward() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

// Move backward
void moveBackward() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

// Turn left
void turnLeft() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

// Turn right
void turnRight() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

// Motor setup
void motorSetup() {
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    analogWrite(ENA, 80); // Full speed (0–255)
    analogWrite(ENB, 80);
    stopMotors();
}
// How to use?
// motorSetup(); in setup

// resetMotor1(); to stop the motor
// goForwardMotor1(); to move the motor
//memory
// Include Arduino standard library to access standard functions
#include <Arduino.h>
// Include EEPROM to access it internal functions
#include <EEPROM.h>

// Reset memory in EEPROM
void memoryReset(){
    for(int i = 0; i < EEPROM.length(); i++){
        EEPROM.write(i, 0);
    }
}

// Write memory to EEPROM
void memoryWrite(String input){
    for(int i = 0; i < int(input.length()); i++){
        EEPROM.write(i, input[i]);
    }
}

// Read and Return all value from EEPROM
String memoryRead(){
    String buffer = "";
    for(int i = 0; i < EEPROM.length(); i++){
        if(EEPROM.read(i) != 0){
            buffer += char(EEPROM.read(i));
        }else{
            break;
        }
    }
    return buffer;
}

// How to use?
// memoryReset(); to reset the existing memory
// memoryWrite(input message); to store value in EEPROM

// memoryRead(); to reset values from existing memory
