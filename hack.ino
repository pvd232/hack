/*
 * Combined HC-SR04 Ultrasonic Sensor and Motor Control Example
 *
 * This sketch integrates an HC-SR04 ultrasonic sensor with motor control functionalities.
 * It measures distance using the ultrasonic sensor and controls two motors to move
 * forward and backward based on predefined intervals.
 */

// ==============================
// ==== Ultrasonic Sensor Setup ===
// ==============================

// Define ultrasonic sensor pins
const int TRIG_PIN = 11;
const int ECHO_PIN = 12;

// Constants for distance calculation
const float SOUND_SPEED = 0.0343;           // cm/us (speed of sound at 20°C)
const unsigned long SENSOR_TIMEOUT = 30000; // 30ms timeout for echo

// Measurement interval (milliseconds)
const unsigned long SENSOR_MEASUREMENT_INTERVAL = 100;

// Function Prototypes for Ultrasonic Sensor
void setupUltrasonicSensor();
void triggerUltrasonic();
float measureDistance();
void displayDistance(float distance);

// ==============================
// ======= Motor Control Setup ===
// ==============================

// Define motor control pins
const int ENA = 9; // Enable pin for Motor 1 (PWM)
const int IN6 = 7; // Direction pin 1 for Motor 1
const int IN7 = 8; // Direction pin 2 for Motor 1

const int ENA2 = 10; // Enable pin for Motor 2 (PWM)
const int IN3 = 5;   // Direction pin 1 for Motor 2
const int IN4 = 6;   // Direction pin 2 for Motor 2

// Define motor speed
const int MOTOR_SPEED = 150; // Speed value (0-255)

// Timing intervals for motor control (milliseconds)
const unsigned long MOTOR_RUN_INTERVAL = 3000;  // 3 seconds
const unsigned long MOTOR_STOP_INTERVAL = 2000; // 2 seconds

// Function Prototypes for Motor Control
void setupMotorPins();
void runMotorsForward(int speed);
void runMotorsBackward(int speed);
void stopMotors();
void controlMotor(int enaPin, int in1Pin, int in2Pin, bool forward, int speed);

// ==============================
// ========= Setup Function ======
// ==============================

void setup()
{
    // Initialize Serial Communication
    Serial.begin(9600);

    // Initialize Ultrasonic Sensor
    setupUltrasonicSensor();

    // Initialize Motor Control
    setupMotorPins();
}

// ==============================
// ========= Loop Function =======
// ==============================

void loop()
{
    // Ultrasonic Sensor Measurement
    triggerUltrasonic();
    float distance = measureDistance();
    displayDistance(distance);

    // Motor Control Sequence
    runMotorsForward(MOTOR_SPEED);
    delay(MOTOR_RUN_INTERVAL); // Run motors forward for 3 seconds

    stopMotors();
    delay(MOTOR_STOP_INTERVAL); // Stop motors for 2 seconds

    runMotorsBackward(MOTOR_SPEED);
    delay(MOTOR_RUN_INTERVAL); // Run motors backward for 3 seconds

    stopMotors();
    delay(MOTOR_STOP_INTERVAL); // Stop motors for 2 seconds

    // Small delay before next sensor measurement
    delay(SENSOR_MEASUREMENT_INTERVAL);
}

// ==============================
// ==== Ultrasonic Sensor Functions ===
// ==============================

/**
 * @brief Initializes the ultrasonic sensor pins.
 */
void setupUltrasonicSensor()
{
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    digitalWrite(TRIG_PIN, LOW); // Ensure trigger pin is LOW initially
}

/**
 * @brief Sends a trigger pulse to initiate ultrasonic burst.
 */
void triggerUltrasonic()
{
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10); // Trigger pulse duration of 10µs
    digitalWrite(TRIG_PIN, LOW);
}

/**
 * @brief Measures the duration of the echo pulse and calculates distance.
 *
 * @return float The calculated distance in centimeters. Returns -1 if out of range.
 */
float measureDistance()
{
    // Measure the duration of the echo pulse in microseconds
    float duration = pulseIn(ECHO_PIN, HIGH, SENSOR_TIMEOUT); // Timeout after 30ms to prevent blocking

    // Check for timeout (no echo received)
    if (duration == 0)
    {
        return -1; // Indicates out of range or no object detected
    }

    // Calculate distance: (duration * speed of sound) / 2
    float distance = (duration * SOUND_SPEED) / 2;
    return distance;
}

/**
 * @brief Displays the measured distance on the serial monitor.
 *
 * @param distance The distance to display.
 */
void displayDistance(float distance)
{
    if (distance >= 0)
    {
        Serial.print("Distance: ");
        Serial.print(distance);
        Serial.println(" cm");
    }
    else
    {
        Serial.println("Distance: Out of range");
    }
}

// ==============================
// ==== Motor Control Functions ===
// ==============================

/**
 * @brief Initializes motor control pins as outputs.
 */
void setupMotorPins()
{
    // Initialize Motor 1 pins
    pinMode(ENA, OUTPUT);
    pinMode(IN6, OUTPUT);
    pinMode(IN7, OUTPUT);

    // Initialize Motor 2 pins
    pinMode(ENA2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
}

/**
 * @brief Runs both motors forward at the specified speed.
 *
 * @param speed PWM value (0-255)
 */
void runMotorsForward(int speed)
{
    controlMotor(ENA, IN6, IN7, true, speed);  // Motor 1 forward
    controlMotor(ENA2, IN3, IN4, true, speed); // Motor 2 forward
}

/**
 * @brief Runs both motors backward at the specified speed.
 *
 * @param speed PWM value (0-255)
 */
void runMotorsBackward(int speed)
{
    controlMotor(ENA, IN6, IN7, false, speed);  // Motor 1 backward
    controlMotor(ENA2, IN3, IN4, false, speed); // Motor 2 backward
}

/**
 * @brief Stops both motors by setting direction pins LOW and PWM to 0.
 */
void stopMotors()
{
    // Stop Motor 1
    digitalWrite(IN6, LOW);
    digitalWrite(IN7, LOW);
    analogWrite(ENA, 0); // Optional: Set speed to 0

    // Stop Motor 2
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENA2, 0); // Optional: Set speed to 0
}

/**
 * @brief Controls a single motor's direction and speed.
 *
 * @param enaPin Enable pin connected to PWM for speed control.
 * @param in1Pin Direction pin 1.
 * @param in2Pin Direction pin 2.
 * @param forward Boolean indicating direction. True for forward, false for backward.
 * @param speed PWM value (0-255).
 */
void controlMotor(int enaPin, int in1Pin, int in2Pin, bool forward, int speed)
{
    if (forward)
    {
        digitalWrite(in1Pin, HIGH);
        digitalWrite(in2Pin, LOW);
    }
    else
    {
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, HIGH);
    }
    analogWrite(enaPin, speed);
}
