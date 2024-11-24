/*
 * Autonomous Navigation Robot with Ultrasonic Sensors
 *
 * This sketch allows a robot to navigate a square room by finding the closest wall,
 * moving towards it, following along the walls to reach each corner, and at each corner,
 * continuously turning and searching for a person using ultrasonic sensors.
 * If a person is found, the robot approaches the person and starts following them.
 * The robot also detects if the person falls and responds accordingly.
 * This process repeats indefinitely.
 */

// ==============================
// ==== Ultrasonic Sensor Setup ===
// ==============================

// Define ultrasonic sensor pins
const int TRIG_PIN_FRONT = 13;
const int ECHO_PIN_FRONT = 12;

const int TRIG_PIN_LEFT = A0;
const int ECHO_PIN_LEFT = A1;

const int TRIG_PIN_RIGHT = A2;
const int ECHO_PIN_RIGHT = A3;

// Upward-facing sensor for person detection and fall detection
const int TRIG_PIN_ABOVE = 11;
const int ECHO_PIN_ABOVE = 10;

// Constants for distance calculation
const float SOUND_SPEED = 0.0343;           // cm/us (speed of sound at 20°C)
const unsigned long SENSOR_TIMEOUT = 30000; // 30ms timeout for echo

// Obstacle detection threshold (centimeters)
const float OBSTACLE_THRESHOLD = 20.0;

// Desired distance from the wall for wall-following (centimeters)
const float DESIRED_WALL_DISTANCE = 15.0;

// Measurement interval (milliseconds)
const unsigned long SENSOR_MEASUREMENT_INTERVAL = 100;

// Constants for person detection
const float UPWARD_SENSOR_THRESHOLD = 150.0; // Adjust based on expected person height/distance
const float SUDDEN_CHANGE_THRESHOLD = 30.0;  // Adjust based on expected change in front sensor

// Corner detection threshold (centimeters)
const float CORNER_DETECTION_THRESHOLD = 20.0; // Adjust based on distance to detect adjacent wall

// =============================
// ======= Motor Control Setup ===
// =============================

// Define motor control pins
const int ENA = 2; // Enable pin for Motor 1 (PWM) (Left)
const int IN1 = 3; // Direction pin 1 for Motor 1
const int IN2 = 4; // Direction pin 2 for Motor 1

const int ENB = 5; // Enable pin for Motor 2 (PWM)
const int IN3 = 6; // Direction pin 1 for Motor 2
const int IN4 = 7; // Direction pin 2 for Motor 2

// Define motor speed
const int MOTOR_SPEED = 150; // Speed value (0-255)

// Turn duration (milliseconds) - Adjust experimentally
const unsigned long TURN_DURATION = 650;                  // Approximate duration for 90-degree turn
const unsigned long TURN_45_DURATION = TURN_DURATION / 2; // Approximate duration for 45-degree turn

// ==============================
// ======== LED Setup ===========
// ==============================

const int ledPin = 9; // LED indicator pin

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
    SEARCHING_FOR_PERSON,
    APPROACH_PERSON,
    AT_PERSON,
    FOLLOW_PERSON,
    DETECT_FALL,
    RESPOND_PERSON
};

RobotState robotState = FIND_CLOSEST_WALL;

// Variable to determine which side to follow (left or right)
bool wallOnRight = true;

// Variable for detecting sudden changes in front distance
float previousFrontDist = -1.0; // Initialize to an invalid distance

// Function Prototypes
void setupUltrasonicSensors();
float measureDistance(int trigPin, int echoPin);
void displayDistances(float frontDist, float leftDist, float rightDist);

void setupMotorPins();
void moveForward(int speed);
void stopMotors();
void turnRight();
void turnLeft();
void turn45DegreesRight();
void turnContinuous(bool turnRightDirection);
void adjustCourse(float sideDist);

void findClosestWall();
void moveForwardUntilObstacle();
void turnAlongWall();
void moveAlongWall();
void atCorner();
void searchForPerson();
void approachPerson();
void atPerson();
void followPerson();
void detectFall();
void respondPerson();

// ==============================
// ========= Setup Function ======
// ==============================

void setup()
{
    // Initialize Serial Communication
    Serial.begin(9600);

    // Initialize LED
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW);

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
    case SEARCHING_FOR_PERSON:
        searchForPerson();
        break;
    case APPROACH_PERSON:
        approachPerson();
        break;
    case AT_PERSON:
        atPerson();
        break;
    case FOLLOW_PERSON:
        followPerson();
        break;
    case DETECT_FALL:
        detectFall();
        break;
    case RESPOND_PERSON:
        respondPerson();
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

    // Upward-facing sensor
    pinMode(TRIG_PIN_ABOVE, OUTPUT);
    pinMode(ECHO_PIN_ABOVE, INPUT);
    digitalWrite(TRIG_PIN_ABOVE, LOW);
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

void turnContinuous(bool turnRightDirection)
{
    if (turnRightDirection)
    {
        // Motor 1 forward
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, MOTOR_SPEED);

        // Motor 2 backward
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        analogWrite(ENB, MOTOR_SPEED);
    }
    else
    {
        // Motor 1 backward
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        analogWrite(ENA, MOTOR_SPEED);

        // Motor 2 forward
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        analogWrite(ENB, MOTOR_SPEED);
    }
}

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
    stopMotors();
    Serial.println("Arrived at corner. Starting to search for person.");
    robotState = SEARCHING_FOR_PERSON;
}

void searchForPerson()
{
    // Start continuous turning
    if (wallOnRight)
    {
        turnContinuous(false); // Turn left
    }
    else
    {
        turnContinuous(true); // Turn right
    }

    // Measure distances
    float frontDist = measureDistance(TRIG_PIN_FRONT, ECHO_PIN_FRONT);
    float upwardDist = measureDistance(TRIG_PIN_ABOVE, ECHO_PIN_ABOVE);
    float oppositeSideDist = wallOnRight ? measureDistance(TRIG_PIN_LEFT, ECHO_PIN_LEFT) : measureDistance(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);

    // Initialize previousFrontDist if it's invalid
    if (previousFrontDist < 0 && frontDist > 0)
    {
        previousFrontDist = frontDist;
    }

    // Check for person detection
    if (upwardDist > 0 && upwardDist < UPWARD_SENSOR_THRESHOLD)
    {
        Serial.println("Person detected with upward sensor!");
        stopMotors();
        robotState = APPROACH_PERSON;
        return;
    }

    if (frontDist > 0 && previousFrontDist > 0 && (previousFrontDist - frontDist) > SUDDEN_CHANGE_THRESHOLD)
    {
        Serial.println("Person detected due to sudden decrease in front distance!");
        stopMotors();
        robotState = APPROACH_PERSON;
        return;
    }

    // Update previous front distance
    if (frontDist > 0)
    {
        previousFrontDist = frontDist;
    }

    // Check if the robot has reached the adjacent wall
    if (oppositeSideDist > 0 && oppositeSideDist < CORNER_DETECTION_THRESHOLD)
    {
        Serial.println("Reached adjacent wall. Person not found. Proceeding to next corner.");
        stopMotors();

        // Reset previousFrontDist
        previousFrontDist = -1.0;

        // Rotate to align with the next wall
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
        return;
    }
}

void approachPerson()
{
    moveForward(MOTOR_SPEED);
    float frontDist = measureDistance(TRIG_PIN_FRONT, ECHO_PIN_FRONT);

    if (frontDist > 0 && frontDist < OBSTACLE_THRESHOLD)
    {
        stopMotors();
        Serial.println("Reached person. Starting to follow.");
        robotState = AT_PERSON;
    }
}

void atPerson()
{
    // Robot has reached the person and is ready to follow
    // Implement any necessary initialization here
    robotState = FOLLOW_PERSON;
}

void followPerson()
{
    float frontDist = measureDistance(TRIG_PIN_FRONT, ECHO_PIN_FRONT);

    if (frontDist > OBSTACLE_THRESHOLD || frontDist == -1)
    {
        moveForward(MOTOR_SPEED);
    }
    else
    {
        stopMotors();
        robotState = DETECT_FALL; // Transition to detecting fall
    }
}

void detectFall()
{
    float frontDistInitial = measureDistance(TRIG_PIN_FRONT, ECHO_PIN_FRONT);
    float aboveDistInitial = measureDistance(TRIG_PIN_ABOVE, ECHO_PIN_ABOVE);
    float frontDistCurrent, aboveDistCurrent;

    delay(2000); // Time to wait for person falling

    frontDistCurrent = measureDistance(TRIG_PIN_FRONT, ECHO_PIN_FRONT);
    aboveDistCurrent = measureDistance(TRIG_PIN_ABOVE, ECHO_PIN_ABOVE);

    float frontDiff = frontDistCurrent - frontDistInitial;
    float aboveDiff = aboveDistCurrent - aboveDistInitial;

    if (abs(frontDiff) > OBSTACLE_THRESHOLD)
    {
        // Object moved away; resume following
        robotState = FOLLOW_PERSON;
    }
    else if (aboveDiff > OBSTACLE_THRESHOLD)
    {
        // Person fell detected
        robotState = RESPOND_PERSON;
    }
    else
    {
        // Re-evaluate detection
        robotState = DETECT_FALL; // Stay in DETECT_FALL state
    }
}

void respondPerson()
{
    Serial.println("Person fell! Activating alert.");
    digitalWrite(ledPin, HIGH);
    delay(10000); // 10 seconds
    digitalWrite(ledPin, LOW);
    robotState = FIND_CLOSEST_WALL; // Restart the process
}
