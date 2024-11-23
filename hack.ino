/*
 * Autonomous Navigation Robot with Ultrasonic Sensors and Mapping
 *
 * This sketch allows a robot to navigate a room by moving forward until it
 * detects an obstacle, then turning to avoid it using front, left, and right
 * ultrasonic sensors. It also creates a simple grid map of the environment.
 *
 * After mapping, the robot navigates to different quadrants of the room,
 * rests for 10 seconds at each, and continues the cycle.
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
const unsigned long TURN_DURATION = 650; // Approximate duration for 90-degree turn

// Function Prototypes for Motor Control
void setupMotorPins();
void moveForward(int speed);
void stopMotors();
void turnRight();
void turnLeft();

// ==============================
// ======= Mapping Setup ===
// ==============================

const int GRID_SIZE = 20;           // Adjust based on room size and resolution
byte gridMap[GRID_SIZE][GRID_SIZE]; // 0: Unknown, 1: Free, 2: Obstacle

// Robot's current position and orientation
int posX = GRID_SIZE / 2;
int posY = GRID_SIZE / 2;
enum Orientation
{
    NORTH,
    EAST,
    SOUTH,
    WEST
};
Orientation currentOrientation = NORTH;

// Function Prototypes for Mapping
void updateMap(float frontDist, float leftDist, float rightDist);
void updatePosition();
void printGridMap();

// ==============================
// ==== New Functionality Setup ===
// ==============================

// Flag to indicate if mapping is complete
bool mappingComplete = false;

// Define quadrant positions
struct Position
{
    int x;
    int y;
};

Position quadrants[4];

// Index of the current quadrant to visit
int currentQuadrantIndex = 0;

// Function Prototypes for new functionalities
bool isMappingComplete();
void navigateTo(Position target);
void rotateToOrientation(Orientation targetOrientation);

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

    // Initialize Grid Map
    for (int i = 0; i < GRID_SIZE; i++)
    {
        for (int j = 0; j < GRID_SIZE; j++)
        {
            gridMap[i][j] = 0; // Unknown
        }
    }

    // Mark starting position as free space
    gridMap[posX][posY] = 1; // Free

    // Initialize Quadrant Positions
    quadrants[0] = {GRID_SIZE / 4, GRID_SIZE / 4};         // Top-left quadrant
    quadrants[1] = {3 * GRID_SIZE / 4, GRID_SIZE / 4};     // Top-right quadrant
    quadrants[2] = {3 * GRID_SIZE / 4, 3 * GRID_SIZE / 4}; // Bottom-right quadrant
    quadrants[3] = {GRID_SIZE / 4, 3 * GRID_SIZE / 4};     // Bottom-left quadrant
}

// ==============================
// ========= Loop Function =======
// ==============================

void loop()
{
    if (!mappingComplete)
    {
        // Measure distances
        float frontDist = measureDistance(TRIG_PIN_FRONT, ECHO_PIN_FRONT);
        float leftDist = measureDistance(TRIG_PIN_LEFT, ECHO_PIN_LEFT);
        float rightDist = measureDistance(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);
        displayDistances(frontDist, leftDist, rightDist);

        // Update the map based on sensor readings
        updateMap(frontDist, leftDist, rightDist);

        // Decision-making based on sensor data
        if (frontDist > 0 && frontDist < OBSTACLE_THRESHOLD)
        {
            // Obstacle detected in front
            stopMotors();
            delay(200); // Brief pause

            // Decide turn direction based on left and right distances
            if (leftDist > rightDist)
            {
                turnLeft();
                currentOrientation = (Orientation)((currentOrientation + 3) % 4); // Turned left
            }
            else
            {
                turnRight();
                currentOrientation = (Orientation)((currentOrientation + 1) % 4); // Turned right
            }
        }
        else
        {
            // No obstacle in front, move forward
            moveForward(MOTOR_SPEED);
            updatePosition(); // Update position after moving forward
        }

        // Small delay before next sensor measurement
        delay(SENSOR_MEASUREMENT_INTERVAL);

        // Optional: Print the grid map
        // printGridMap();

        // Check if mapping is complete
        if (isMappingComplete())
        {
            mappingComplete = true;
            stopMotors();
            Serial.println("Mapping complete. Starting quadrant navigation.");
        }
    }
    else
    {
        // Navigate to quadrants
        Position target = quadrants[currentQuadrantIndex];
        navigateTo(target);

        // Wait for 10 seconds
        Serial.print("Arrived at quadrant ");
        Serial.println(currentQuadrantIndex + 1);
        stopMotors();
        delay(10000); // 10 seconds

        // Move to the next quadrant
        currentQuadrantIndex = (currentQuadrantIndex + 1) % 4;
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

// ==============================
// ==== Mapping Functions ===
// ==============================

void updateMap(float frontDist, float leftDist, float rightDist)
{
    // Grid cell size in cm
    const float CELL_SIZE = 20.0;

    // Mark the cell in front
    if (frontDist > 0 && frontDist <= OBSTACLE_THRESHOLD)
    {
        int cellAheadX = posX;
        int cellAheadY = posY;

        switch (currentOrientation)
        {
        case NORTH:
            cellAheadY -= 1;
            break;
        case EAST:
            cellAheadX += 1;
            break;
        case SOUTH:
            cellAheadY += 1;
            break;
        case WEST:
            cellAheadX -= 1;
            break;
        }

        if (cellAheadX >= 0 && cellAheadX < GRID_SIZE && cellAheadY >= 0 && cellAheadY < GRID_SIZE)
        {
            gridMap[cellAheadX][cellAheadY] = 2; // Obstacle
        }
    }

    // Similarly, mark the cells to the left and right
    // Left
    if (leftDist > 0 && leftDist <= OBSTACLE_THRESHOLD)
    {
        int cellLeftX = posX;
        int cellLeftY = posY;

        switch (currentOrientation)
        {
        case NORTH:
            cellLeftX -= 1;
            break;
        case EAST:
            cellLeftY -= 1;
            break;
        case SOUTH:
            cellLeftX += 1;
            break;
        case WEST:
            cellLeftY += 1;
            break;
        }

        if (cellLeftX >= 0 && cellLeftX < GRID_SIZE && cellLeftY >= 0 && cellLeftY < GRID_SIZE)
        {
            gridMap[cellLeftX][cellLeftY] = 2; // Obstacle
        }
    }

    // Right
    if (rightDist > 0 && rightDist <= OBSTACLE_THRESHOLD)
    {
        int cellRightX = posX;
        int cellRightY = posY;

        switch (currentOrientation)
        {
        case NORTH:
            cellRightX += 1;
            break;
        case EAST:
            cellRightY += 1;
            break;
        case SOUTH:
            cellRightX -= 1;
            break;
        case WEST:
            cellRightY -= 1;
            break;
        }

        if (cellRightX >= 0 && cellRightX < GRID_SIZE && cellRightY >= 0 && cellRightY < GRID_SIZE)
        {
            gridMap[cellRightX][cellRightY] = 2; // Obstacle
        }
    }

    // Mark current position as free space
    gridMap[posX][posY] = 1; // Free
}

void updatePosition()
{
    // Since we don't have encoders, we'll assume that each forward movement moves one cell
    switch (currentOrientation)
    {
    case NORTH:
        if (posY > 0)
            posY -= 1;
        break;
    case EAST:
        if (posX < GRID_SIZE - 1)
            posX += 1;
        break;
    case SOUTH:
        if (posY < GRID_SIZE - 1)
            posY += 1;
        break;
    case WEST:
        if (posX > 0)
            posX -= 1;
        break;
    }

    // Mark new position as free space
    gridMap[posX][posY] = 1; // Free

    // Move for a fixed duration to simulate moving one cell
    // Only delay during mapping to avoid unnecessary pauses during navigation
    if (!mappingComplete)
    {
        delay(1000); // Adjust duration as needed
    }
}

void printGridMap()
{
    for (int y = 0; y < GRID_SIZE; y++)
    {
        for (int x = 0; x < GRID_SIZE; x++)
        {
            if (x == posX && y == posY)
            {
                Serial.print("R "); // Robot's current position
            }
            else
            {
                Serial.print(gridMap[x][y]);
                Serial.print(" ");
            }
        }
        Serial.println();
    }
    Serial.println("---------------------------");
}

// ==============================
// ==== New Function Definitions ===
// ==============================

/**
 * @brief Checks if all accessible cells have been visited.
 *
 * @return true if mapping is complete, false otherwise.
 */
bool isMappingComplete()
{
    for (int i = 0; i < GRID_SIZE; i++)
    {
        for (int j = 0; j < GRID_SIZE; j++)
        {
            // If there is any unknown (0) cell adjacent to a free space (1), mapping is not complete
            if (gridMap[i][j] == 0)
            {
                // Check if this unknown cell is adjacent to a free space
                if ((i > 0 && gridMap[i - 1][j] == 1) ||
                    (i < GRID_SIZE - 1 && gridMap[i + 1][j] == 1) ||
                    (j > 0 && gridMap[i][j - 1] == 1) ||
                    (j < GRID_SIZE - 1 && gridMap[i][j + 1] == 1))
                {
                    return false; // Still areas to explore
                }
            }
        }
    }
    return true; // All accessible areas have been explored
}

/**
 * @brief Navigates the robot to the target grid position.
 *
 * @param target The target position to navigate to.
 */
void navigateTo(Position target)
{
    // Simplified navigation: Move in grid steps towards the target
    while (posX != target.x || posY != target.y)
    {
        // Determine the direction to move in X axis
        if (posX < target.x)
        {
            rotateToOrientation(EAST);
            moveForward(MOTOR_SPEED);
            updatePosition();
        }
        else if (posX > target.x)
        {
            rotateToOrientation(WEST);
            moveForward(MOTOR_SPEED);
            updatePosition();
        }
        // Determine the direction to move in Y axis
        else if (posY < target.y)
        {
            rotateToOrientation(SOUTH);
            moveForward(MOTOR_SPEED);
            updatePosition();
        }
        else if (posY > target.y)
        {
            rotateToOrientation(NORTH);
            moveForward(MOTOR_SPEED);
            updatePosition();
        }

        // Add obstacle avoidance during navigation
        float frontDist = measureDistance(TRIG_PIN_FRONT, ECHO_PIN_FRONT);
        if (frontDist > 0 && frontDist < OBSTACLE_THRESHOLD)
        {
            // Obstacle detected, need to avoid
            stopMotors();
            delay(200);
            // Try to turn and bypass the obstacle
            turnRight();
            currentOrientation = (Orientation)((currentOrientation + 1) % 4);
        }

        // Update the map with new sensor readings
        float leftDist = measureDistance(TRIG_PIN_LEFT, ECHO_PIN_LEFT);
        float rightDist = measureDistance(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);
        updateMap(frontDist, leftDist, rightDist);
    }
}

/**
 * @brief Rotates the robot to face the target orientation.
 *
 * @param targetOrientation The orientation to rotate to.
 */
void rotateToOrientation(Orientation targetOrientation)
{
    while (currentOrientation != targetOrientation)
    {
        turnRight();
        currentOrientation = (Orientation)((currentOrientation + 1) % 4);
        delay(200); // Brief pause after turning
    }
}
