/*
 * Autonomous Navigation Robot with Ultrasonic Sensor
 *
 * This sketch allows a robot to navigate a room by moving forward until it
 * detects an obstacle, then turning to avoid it using only a front-facing
 * ultrasonic sensor.
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

// Obstacle detection threshold (centimeters)
const float OBSTACLE_THRESHOLD = 20.0;

// Function Prototypes for Ultrasonic Sensor
void setupUltrasonicSensor();
float measureDistance();
void displayDistance(float distance);

// ==============================
// ======= Motor Control Setup ===
// ==============================

// Define motor control pins
const int ENA = 9; // Enable pin for Motor 1 (PWM)
const int IN1 = 7; // Direction pin 1 for Motor 1
const int IN2 = 8; // Direction pin 2 for Motor 1

const int ENB = 10; // Enable pin for Motor 2 (PWM)
const int IN3 = 5;  // Direction pin 1 for Motor 2
const int IN4 = 6;  // Direction pin 2 for Motor 2

// Define motor speed
const int MOTOR_SPEED = 150; // Speed value (0-255)

// Turn duration (milliseconds) - Adjust experimentally
const unsigned long TURN_DURATION = 650; // Approximate duration for 90-degree turn

// Function Prototypes for Motor Control
void setupMotorPins();
void moveForward(int speed);
void stopMotors();
void turnRight();
void turnLeft();

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
 * @brief Measures the duration of the echo pulse and calculates distance.
 *
 * @return float The calculated distance in centimeters. Returns -1 if out of range.
 */
float measureDistance()
{
    // Send a trigger pulse
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10); // Trigger pulse duration of 10µs
    digitalWrite(TRIG_PIN, LOW);

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
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);

    // Initialize Motor 2 pins
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
}

/**
 * @brief Moves the robot forward at the specified speed.
 *
 * @param speed PWM value (0-255)
 */
void moveForward(int speed)
{
    // Motor 1 forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, speed);

    // Motor 2 forward
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, speed);
}

/**
 * @brief Stops both motors.
 */
void stopMotors()
{
    // Stop Motor 1
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);

    // Stop Motor 2
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
}

/**
 * @brief Turns the robot right by running motors in opposite directions.
 */
void turnRight()
{
    // Motor 1 forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, MOTOR_SPEED);

    // Motor 2 backward
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, MOTOR_SPEED);

    delay(TURN_DURATION); // Turn duration to approximate 90 degrees

    stopMotors(); // Stop after turning
}

/**
 * @brief Turns the robot left by running motors in opposite directions.
 */
void turnLeft()
{
    // Motor 1 backward
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, MOTOR_SPEED);

    // Motor 2 forward
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, MOTOR_SPEED);

    delay(TURN_DURATION); // Turn duration to approximate 90 degrees

    stopMotors(); // Stop after turning
}

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

// Variable to alternate turn directions
bool turnRightNext = true;

void loop()
{
    // Measure distance ahead
    float distance = measureDistance();
    displayDistance(distance);

    if (distance > 0 && distance < OBSTACLE_THRESHOLD)
    {
        // Obstacle detected within threshold
        stopMotors(); // Stop before turning
        delay(200);   // Brief pause

        if (turnRightNext)
        {
            turnRight();
            turnRightNext = false; // Next time, turn left
        }
        else
        {
            turnLeft();
            turnRightNext = true; // Next time, turn right
        }
    }
    else
    {
        // No obstacle detected, move forward
        moveForward(MOTOR_SPEED);
    }

    // Small delay before next sensor measurement
    delay(SENSOR_MEASUREMENT_INTERVAL);
}
