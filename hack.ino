/*
 * Autonomous Navigation Robot with Ultrasonic Sensors
 *
 * This sketch allows a robot to navigate a square room by finding the closest wall,
 * moving towards it, following along the walls to reach each corner, and at each corner,
 * rotating inward toward the center of the room, resting for 10 seconds, then proceeding
 * to the next corner. This process repeats indefinitely.
 */

// ==============================
// ==== Ultrasonic Sensor Setup ===
// ==============================

// Define ultrasonic sensor pins
const int TRIG_PIN_FRONT = 11;
const int ECHO_PIN_FRONT = 12;

const int TRIG_PIN_LEFT = 13; // Assign appropriate pins
const int ECHO_PIN_LEFT = 14;

const int TRIG_PIN_RIGHT = 15;
const int ECHO_PIN_RIGHT = 16;

// Constants for distance calculation
const float SOUND_SPEED = 0.0343;           // cm/us (speed of sound at 20°C)
const unsigned long SENSOR_TIMEOUT = 30000; // 30ms timeout for echo

// Obstacle detection threshold (centimeters)
const float OBSTACLE_THRESHOLD = 20.0;

// Desired distance from the wall for wall-following (centimeters)
const float DESIRED_WALL_DISTANCE = 15.0;

// Measurement interval (milliseconds)
const unsigned long SENSOR_MEASUREMENT_INTERVAL = 100;

// Function Prototypes for Ultrasonic Sensors
void setupUltrasonicSensors();
float measureDistance(int trigPin, int echoPin);
void displayDistances(float frontDist, float leftDist, float rightDist);

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
const unsigned long TURN_DURATION = 650;                  // Approximate duration for 90-degree turn
const unsigned long TURN_45_DURATION = TURN_DURATION / 2; // Approximate duration for 45-degree turn

// Function Prototypes for Motor Control
void setupMotorPins();
void moveForward(int speed);
void stopMotors();
void turnRight();
void turnLeft();
void turn45DegreesRight();
void turn45DegreesLeft();
void adjustCourse(float sideDist);

// ==============================
// ==== New Functionality Setup ===
// ==============================

enum RobotState
{
    FIND_CLOSEST_WALL,
    MOVE_TOWARD_WALL,
    TURN_ALONG_WALL,
    MOVE_ALONG_WALL,
    AT_CORNER,
    RESTING
};

RobotState robotState = FIND_CLOSEST_WALL;

// For rest timing
unsigned long restStartTime = 0;

// Variable to determine which side to follow (left or right)
bool wallOnRight = true;

// Function Prototypes for new functionalities
void findClosestWall();
void moveForwardUntilObstacle();
void turnAlongWall();
void moveAlongWall();
void atCorner();
void rest();

// ==============================
// ========= Setup Function ======
// ==============================

void setup()
{
    // Initialize Serial Communication
    Serial.begin(9600);

    // Initialize Ultrasonic Sensors
    setupUltrasonicSensors();

    // Initialize Motor Control
    setupMotorPins();
}

// ==============================
// ========= Loop Function =======
// ==============================

void loop()
{
    switch (robotState)
    {
    case FIND_CLOSEST_WALL:
        findClosestWall();
        break;
    case MOVE_TOWARD_WALL:
        moveForwardUntilObstacle();
        break;
    case TURN_ALONG_WALL:
        turnAlongWall();
        break;
    case MOVE_ALONG_WALL:
        moveAlongWall();
        break;
    case AT_CORNER:
        atCorner();
        break;
    case RESTING:
        rest();
        break;
    }
}

// ==============================
// ==== Ultrasonic Sensor Functions ===
// ==============================

void setupUltrasonicSensors()
{
    // Front sensor
    pinMode(TRIG_PIN_FRONT, OUTPUT);
    pinMode(ECHO_PIN_FRONT, INPUT);
    digitalWrite(TRIG_PIN_FRONT, LOW);

    // Left sensor
    pinMode(TRIG_PIN_LEFT, OUTPUT);
    pinMode(ECHO_PIN_LEFT, INPUT);
    digitalWrite(TRIG_PIN_LEFT, LOW);

    // Right sensor
    pinMode(TRIG_PIN_RIGHT, OUTPUT);
    pinMode(ECHO_PIN_RIGHT, INPUT);
    digitalWrite(TRIG_PIN_RIGHT, LOW);
}

float measureDistance(int trigPin, int echoPin)
{
    // Send a trigger pulse
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Measure the duration of the echo pulse in microseconds
    float duration = pulseIn(echoPin, HIGH, SENSOR_TIMEOUT);

    // Check for timeout (no echo received)
    if (duration == 0)
    {
        return -1; // Indicates out of range or no object detected
    }

    // Calculate distance: (duration * speed of sound) / 2
    float distance = (duration * SOUND_SPEED) / 2;
    return distance;
}

void displayDistances(float frontDist, float leftDist, float rightDist)
{
    Serial.print("Front: ");
    Serial.print(frontDist);
    Serial.print(" cm, Left: ");
    Serial.print(leftDist);
    Serial.print(" cm, Right: ");
    Serial.print(rightDist);
    Serial.println(" cm");
}

// ==============================
// ==== Motor Control Functions ===
// ==============================

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

    delay(TURN_DURATION); // Adjust experimentally

    stopMotors();
}

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

    delay(TURN_DURATION); // Adjust experimentally

    stopMotors();
}

void turn45DegreesRight()
{
    // Motor 1 forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, MOTOR_SPEED);

    // Motor 2 backward
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, MOTOR_SPEED);

    delay(TURN_45_DURATION); // Adjust experimentally

    stopMotors();
}

void turn45DegreesLeft()
{
    // Motor 1 backward
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, MOTOR_SPEED);

    // Motor 2 forward
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, MOTOR_SPEED);

    delay(TURN_45_DURATION); // Adjust experimentally

    stopMotors();
}

// Adjust the robot's course based on side distance
void adjustCourse(float sideDist)
{
    const int adjustmentSpeed = 50; // Adjust as necessary

    if (sideDist > DESIRED_WALL_DISTANCE + 5)
    {
        // Too far from wall, adjust towards wall
        if (wallOnRight)
        {
            // Turn slightly right
            analogWrite(ENA, MOTOR_SPEED + adjustmentSpeed);
            analogWrite(ENB, MOTOR_SPEED - adjustmentSpeed);
        }
        else
        {
            // Turn slightly left
            analogWrite(ENA, MOTOR_SPEED - adjustmentSpeed);
            analogWrite(ENB, MOTOR_SPEED + adjustmentSpeed);
        }
    }
    else if (sideDist > 0 && sideDist < DESIRED_WALL_DISTANCE - 5)
    {
        // Too close to wall, adjust away from wall
        if (wallOnRight)
        {
            // Turn slightly left
            analogWrite(ENA, MOTOR_SPEED - adjustmentSpeed);
            analogWrite(ENB, MOTOR_SPEED + adjustmentSpeed);
        }
        else
        {
            // Turn slightly right
            analogWrite(ENA, MOTOR_SPEED + adjustmentSpeed);
            analogWrite(ENB, MOTOR_SPEED - adjustmentSpeed);
        }
    }
    else
    {
        // Move straight
        moveForward(MOTOR_SPEED);
    }
}

// ==============================
// ==== New Function Definitions ===
// ==============================

void findClosestWall()
{
    float minDistance = 9999.0;
    int minSteps = 0;

    // Rotate 360 degrees in 45-degree increments
    for (int i = 0; i < 8; i++)
    {
        float dist = measureDistance(TRIG_PIN_FRONT, ECHO_PIN_FRONT);
        Serial.print("Distance at step ");
        Serial.print(i);
        Serial.print(": ");
        Serial.print(dist);
        Serial.println(" cm");

        if (dist > 0 && dist < minDistance)
        {
            minDistance = dist;
            minSteps = i;
        }

        turn45DegreesRight();
        delay(200); // Small delay after turning
    }

    // Rotate back to the orientation with the minimum distance
    int stepsToRotateBack = (8 - minSteps) % 8;

    for (int i = 0; i < stepsToRotateBack; i++)
    {
        turn45DegreesRight();
        delay(200);
    }

    // Determine which side the wall is on for wall following
    float leftDist = measureDistance(TRIG_PIN_LEFT, ECHO_PIN_LEFT);
    float rightDist = measureDistance(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);

    if (leftDist > 0 && (leftDist < rightDist || rightDist < 0))
    {
        wallOnRight = false; // Follow wall on left side
        Serial.println("Following wall on left side.");
    }
    else
    {
        wallOnRight = true; // Follow wall on right side
        Serial.println("Following wall on right side.");
    }

    robotState = MOVE_TOWARD_WALL;
}

void moveForwardUntilObstacle()
{
    moveForward(MOTOR_SPEED);
    float frontDist = measureDistance(TRIG_PIN_FRONT, ECHO_PIN_FRONT);

    if (frontDist > 0 && frontDist < OBSTACLE_THRESHOLD)
    {
        stopMotors();
        robotState = TURN_ALONG_WALL;
    }
}

void turnAlongWall()
{
    // Turn towards the wall to align along it
    if (wallOnRight)
    {
        turnRight();
    }
    else
    {
        turnLeft();
    }
    delay(200);
    robotState = MOVE_ALONG_WALL;
}

void moveAlongWall()
{
    float frontDist = measureDistance(TRIG_PIN_FRONT, ECHO_PIN_FRONT);
    float leftDist = measureDistance(TRIG_PIN_LEFT, ECHO_PIN_LEFT);
    float rightDist = measureDistance(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);
    float sideDist = wallOnRight ? rightDist : leftDist;

    displayDistances(frontDist, leftDist, rightDist);

    // Adjust course to follow wall
    adjustCourse(sideDist);

    // Check for corner: walls on front and side
    if ((frontDist > 0 && frontDist < OBSTACLE_THRESHOLD) && (sideDist > 0 && sideDist < OBSTACLE_THRESHOLD))
    {
        stopMotors();
        robotState = AT_CORNER;
    }

    delay(SENSOR_MEASUREMENT_INTERVAL);
}

void atCorner()
{
    // Rotate 45 degrees inward
    if (wallOnRight)
    {
        turn45DegreesLeft();
    }
    else
    {
        turn45DegreesRight();
    }
    stopMotors();
    Serial.println("Arrived at corner. Resting for 10 seconds.");
    restStartTime = millis();
    robotState = RESTING;
}

void rest()
{
    if (millis() - restStartTime >= 10000) // Rest for 10 seconds
    {
        // Rest is over
        // Rotate back to face along the wall
        if (wallOnRight)
        {
            turn45DegreesRight();
        }
        else
        {
            turn45DegreesLeft();
        }
        delay(200);
        // Turn to follow the next wall
        if (wallOnRight)
        {
            turnRight();
        }
        else
        {
            turnLeft();
        }
        delay(200);
        robotState = MOVE_ALONG_WALL;
    }
}
