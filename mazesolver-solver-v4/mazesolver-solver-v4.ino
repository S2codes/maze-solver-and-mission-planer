#include <Wire.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>



// Maze Solver Code for Two-Wheel Caster Robot

// Pin Configuration
#define LEFT_MOTOR_IN1 27
#define LEFT_MOTOR_IN2 14
#define RIGHT_MOTOR_IN3 12
#define RIGHT_MOTOR_IN4 13

// Encoder Pins
#define LEFT_ENCODER_A 34
#define LEFT_ENCODER_B 35
#define RIGHT_ENCODER_A 32
#define RIGHT_ENCODER_B 33

// Ultrasonic Sensor Pins
#define ULTRASONIC1_TRIG 25
#define ULTRASONIC1_ECHO 26
#define ULTRASONIC2_TRIG 18
#define ULTRASONIC2_ECHO 19
#define ULTRASONIC3_TRIG 4
#define ULTRASONIC3_ECHO 2


WebSocketsServer websockets(81);  // port for web socket

volatile int leftEncoderCount = 0;   // Variable to store left encoder count (volatile because it's changed in ISR)
volatile int rightEncoderCount = 0;  // Variable to store right encoder count (volatile because it's changed in ISR)


long frontDist, leftDist, rightDist;  // Variables to store ultrasonic sensor readings


unsigned long lastTime = 0;  // Last time sensor data was sent
int leftRPM = 0;             // Left wheel RPM
int rightRPM = 0;            // Right wheel RPM

// wheel properties
float wheelRadius = 0.022;  // Wheel radius in meters (22mm)
float wheelBase = 0.125;    // Distance between wheels in meters (12.5cm)


// Odometry variables
float robotX = 0.0;            // Robot's X position in meters
float robotY = 0.0;            // Robot's Y position in meters
float robotTheta = 0.0;        // Robot's orientation in radians
float distanceTraveled = 0.0;  // Total distance traveled in meters

// Add these near your other global variables
float linearVelocity = 0.0;     // m/s
float angularVelocity = 0.0;    // rad/s
unsigned long lastRPMCalc = 0;  // For velocity calculations


// Previous encoder values for calculating changes
int prevLeftEncoder = 0;   // Previous left encoder count
int prevRightEncoder = 0;  // Previous right encoder count

// Time tracking for odometry
unsigned long lastOdometryUpdate = 0;   // Last time odometry was updated
const float ticksPerRevolution = 1655;  // Encoder ticks per wheel revolution (adjust based on your encoder)


// Maze Solving Algorithm - Right Wall Following
// This algorithm keeps the robot's right side close to a wall, following it until exit

bool mazeSolvingActive = false;  // Control flag for maze solving

// Constants for maze solving
#define WALL_DISTANCE 6    // Ideal distance from wall in cm
#define MIN_DISTANCE 5      // Minimum distance to consider an obstacle
#define FRONT_CLEARANCE 4  // Minimum front clearance to move forward

bool isWallFind = false;
String mazeStatus = "";


// Function to solve maze using Right Wall Following algorithm
// void solveMaze() {

//   if (frontDist < FRONT_CLEARANCE || leftDist < WALL_DISTANCE || rightDist < WALL_DISTANCE) {
//     isWallFind = true;
//   }else{
//     forwardMotors();
//   }


//   if (frontDist >= FRONT_CLEARANCE) {
//     forwardMotors();
//     mazeStatus = "forward";  // Update status
//     Serial.println(" robot forward");
//   } else if (rightDist >= WALL_DISTANCE + 10) {
//     // Right is open, turn right
//     mazeStatus = "Stop";  // Update status
//     stopMotors();
//     Serial.println(" robot stop");
//     delay(300);
//     turnRight();
//     mazeStatus = "right";  // Update status
//     Serial.println(" robot turnRight");
//     delay(500);
//   } else {
//     // Can't go forward or right, turn left
//     stopMotors();
//     mazeStatus = "Stop";  // Update status
//     Serial.println(" robot stop");
//     delay(300);
//     turnLeft();
//     mazeStatus = "left";  // Update status
//     Serial.print("  robot turnLeft");
//     delay(500);
//   }
//     Serial.println(" ");
// }


// v2
// void solveMaze() {
//   // If no wall detected, move forward (faster exploration)
//   if (frontDist >= FRONT_CLEARANCE && rightDist >= WALL_DISTANCE + 5) {
//     forwardMotors();
//     mazeStatus = "forward";
//   }
//   // If right wall is too far, curve right to follow it
//   else if (rightDist > WALL_DISTANCE + 10) {
//     turnRight();
//     mazeStatus = "right";
//     delay(300); // Short delay to stabilize
//   }
//   // If front blocked, turn left (avoid dead-ends)
//   else if (frontDist < FRONT_CLEARANCE) {
//     turnLeft();
//     mazeStatus = "left";
//     delay(300);
//   }
//   // Default: Follow the right wall
//   else {
//     forwardMotors();
//     mazeStatus = "following";
//   }
// }

void solveMaze() {
  // If no wall ahead and safe right distance → Move forward
  if (frontDist >= FRONT_CLEARANCE && rightDist >= WALL_DISTANCE) {
    // Dynamic wall-following: Adjust if too close/far from the right wall
    if (rightDist < WALL_DISTANCE - 2) {  // Too close to the wall
      slightLeft();  // Curve away from the wall gently
      mazeStatus = "correcting_left";
    } 
    else if (rightDist > WALL_DISTANCE + 2) {  // Too far from the wall
      slightRight();  // Curve toward the wall gently
      mazeStatus = "correcting_right";
    } 
    else {  // Perfect distance → Go straight
      forwardMotors();
      mazeStatus = "forward";
    }
  }
  // If right wall is missing → Turn right to find it
  else if (rightDist > WALL_DISTANCE + 10) {
    turnRight();
    mazeStatus = "right";
    delay(300);
  }
  // If front blocked → Turn left (dead-end escape)
  else if (frontDist < FRONT_CLEARANCE) {
    turnLeft();
    mazeStatus = "left";
    delay(300);
  }
  // Default: Move forward cautiously (e.g., in corners)
  else {
    forwardMotors();
    mazeStatus = "following";
  }
}

// Function to update robot's position and orientation based on encoder data
void updateOdometry() {
  unsigned long currentTime = millis();                           // Get current time
  float deltaTime = (currentTime - lastOdometryUpdate) / 1000.0;  // Convert time difference to seconds

  if (deltaTime < 0.01) return;  // Skip very small time intervals for stability

  // Calculate encoder changes since last update
  int deltaLeftEncoder = leftEncoderCount - prevLeftEncoder;     // Change in left encoder count
  int deltaRightEncoder = rightEncoderCount - prevRightEncoder;  // Change in right encoder count

  // Convert encoder ticks to wheel distances
  float leftWheelDist = (deltaLeftEncoder / ticksPerRevolution) * 2 * PI * wheelRadius;    // Left wheel distance traveled
  float rightWheelDist = (deltaRightEncoder / ticksPerRevolution) * 2 * PI * wheelRadius;  // Right wheel distance traveled

  // Calculate velocities (for smoother web display)
  linearVelocity = (leftDist + rightDist) / (2 * deltaTime);
  angularVelocity = (rightDist - leftDist) / (wheelBase * deltaTime);

  // Calculate robot movement
  float deltaDistance = (leftWheelDist + rightWheelDist) / 2.0;     // Linear distance traveled by robot center
  float deltaTheta = (rightWheelDist - leftWheelDist) / wheelBase;  // Change in orientation

  // Update robot position and orientation (odometry equations)
  robotX += deltaDistance * cos(robotTheta + (deltaTheta / 2.0));  // Update X position
  robotY += deltaDistance * sin(robotTheta + (deltaTheta / 2.0));  // Update Y position
  robotTheta += deltaTheta;                                        // Update orientation

  // Normalize angle to [-π, π]
  while (robotTheta > PI) robotTheta -= 2 * PI;   // Keep angle in standard range
  while (robotTheta < -PI) robotTheta += 2 * PI;  // Keep angle in standard range

  // Update total distance traveled
  distanceTraveled += abs(deltaDistance);  // Add to total distance traveled

  // Save current values for next update
  prevLeftEncoder = leftEncoderCount;    // Save current left encoder count
  prevRightEncoder = rightEncoderCount;  // Save current right encoder count
  lastOdometryUpdate = currentTime;      // Save current time
}


void sendSensorData() {
  // Create JSON document
  DynamicJsonDocument doc(1024);  // Create JSON document with 1KB capacity

  // Add sensor and encoder data
  doc["frontDistance"] = frontDist;  // Add front ultrasonic reading
  doc["leftDistance"] = leftDist;    // Add left ultrasonic reading
  doc["rightDistance"] = rightDist;  // Add right ultrasonic reading

  doc["leftEncoder"] = leftEncoderCount;    // Add left encoder count
  doc["rightEncoder"] = rightEncoderCount;  // Add right encoder count
  doc["leftRPM"] = leftRPM;                 // Add left wheel RPM
  doc["rightRPM"] = rightRPM;               // Add right wheel RPM

  // Add odometry data
  doc["posX"] = robotX;                          // Add robot X position
  doc["posY"] = robotY;                          // Add robot Y position
  doc["orientation"] = robotTheta * 180.0 / PI;  // Convert orientation to degrees
  doc["distanceTraveled"] = distanceTraveled;    // Add total distance traveled

  // Velocity data
  doc["velocity"]["linear"] = linearVelocity;
  doc["odometry"]["angular"] = angularVelocity;


  doc["mazeStatus"] = mazeStatus;
  doc["isWallFind"] = isWallFind;

  // Serialize to string
  String jsonString;               // String to hold JSON data
  serializeJson(doc, jsonString);  // Convert JSON to string

  // Send to all connected clients
  websockets.broadcastTXT(jsonString);  // Send data to all WebSocket clients

  // Print to serial
  // Serial.println("Sending sensor data:");  // Log sending action
  // Serial.println(jsonString);              // Print JSON data
}


void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      break;
    case WStype_CONNECTED:
      {
        IPAddress ip = websockets.remoteIP(num);
        Serial.printf("[%u] Connected from IP: %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
        websockets.sendTXT(num, "Connected from server");
        break;
      }
    case WStype_TEXT:
      {
        Serial.printf("[%u] Received Text: %s\n", num, payload);
        String message = String((char*)(payload));

        DynamicJsonDocument doc(500);
        DeserializationError error = deserializeJson(doc, message);

        if (error) {
          Serial.print("JSON deserialization failed: ");
          Serial.println(error.c_str());
          return;
        }

        if (doc.containsKey("command")) {
          String command = doc["command"];

          // maze solve start command
          if (command == "startMaze") {
            mazeSolvingActive = true;
            Serial.println("Maze solving activated");
          } else if (command == "stopMaze") {
            mazeSolvingActive = false;
            stopMotors();
            Serial.println("Maze solving deactivated");
          }

          if (command == "forward") {
            // forwardMotors();
            forwardMotorsWithCorrection();
            Serial.println("Command received: forward");
          } else if (command == "backward") {
            // Add backward function or implementation
            backwardMotors();
            Serial.println("Command received: backward");
          } else if (command == "left") {
            Serial.println("Command received: left");
          } else if (command == "right") {
            Serial.println("Command received: right");
          } else if (command == "stop") {
            Serial.println("Command received: stop");
            stopMotors();
            // } else if (command == "turnLeft" && doc.containsKey("angle")) {
          } else if (command == "turnLeft") {
            // int angle = doc["angle"];
            Serial.print("Command received: turnLeft with angle: ");
            // Serial.println(angle);
            turnLeft();
            // } else if (command == "turnRight" && doc.containsKey("angle")) {
          } else if (command == "turnRight") {
            // int angle = doc["angle"];
            Serial.print("Command received: turnRight with angle: ");
            // Serial.println(angle);
            turnRight();
          }
        }


        break;
      }
    default:
      break;
  }
}

// Gentle left curve (right motor faster)
void slightLeft() {
  digitalWrite(LEFT_MOTOR_IN1, HIGH);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN3, HIGH);  // Right motor stronger
  digitalWrite(RIGHT_MOTOR_IN4, LOW);
}

// Gentle right curve (left motor faster)
void slightRight() {
  digitalWrite(LEFT_MOTOR_IN1, HIGH);  // Left motor stronger
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN3, HIGH);
  digitalWrite(RIGHT_MOTOR_IN4, LOW);
}

// Motor Control Functions
void stopMotors() {
  // Serial.println("stop");`
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN3, LOW);
  digitalWrite(RIGHT_MOTOR_IN4, LOW);
}

void forwardMotors() {
  Serial.println("Forward");
  digitalWrite(LEFT_MOTOR_IN1, HIGH);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN3, HIGH);
  digitalWrite(RIGHT_MOTOR_IN4, LOW);
}



void forwardMotorsWithCorrection() {
  // Calculate error (difference between left and right encoder counts)
  float error = leftEncoderCount - rightEncoderCount;
  
  // PID terms
  integral += error;
  float derivative = error - prevError;
  float correction = Kp*error + Ki*integral + Kd*derivative;
  
  // Apply correction (adjust motor speeds)
  int baseSpeed = 255;  // Adjust as needed
  int leftSpeed = constrain(baseSpeed + correction, 0, 255);
  int rightSpeed = constrain(baseSpeed - correction, 0, 255);
  
  // Apply to motors (you'll need to modify for PWM control)
  analogWrite(LEFT_MOTOR_IN1, leftSpeed);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  analogWrite(RIGHT_MOTOR_IN3, rightSpeed);
  digitalWrite(RIGHT_MOTOR_IN4, LOW);
  
  prevError = error;
}



void backwardMotors() {
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, HIGH);
  digitalWrite(RIGHT_MOTOR_IN3, LOW);
  digitalWrite(RIGHT_MOTOR_IN4, HIGH);
}

// void turnLeft() {
//   Serial.println("left");
//   digitalWrite(LEFT_MOTOR_IN1, LOW);
//   digitalWrite(LEFT_MOTOR_IN2, HIGH);
//   digitalWrite(RIGHT_MOTOR_IN3, HIGH);
//   digitalWrite(RIGHT_MOTOR_IN4, LOW);
// }

// void turnRight() {
//   Serial.println("right");
//   digitalWrite(LEFT_MOTOR_IN1, HIGH);
//   digitalWrite(LEFT_MOTOR_IN2, LOW);
//   digitalWrite(RIGHT_MOTOR_IN3, LOW);
//   digitalWrite(RIGHT_MOTOR_IN4, HIGH);
// }

// Improved turning functions for precise 90-degree spot turns
void turnLeft() {
  Serial.println("Turning Left 90 degrees");
  // Reset encoder counts before turn
  leftEncoderCount = 0;
  rightEncoderCount = 0;

  // Calculate required encoder ticks for 90 degree turn
  // Circumference of turning circle = PI * wheelBase
  // 90 degrees is 1/4 of full rotation
  // float distancePerWheel = (PI * wheelBase) / 4.0;

   // Calculate required encoder ticks for 87 degree turn
  // 87° = 87/360 of full rotation (0.2417 ratio instead of 0.25 for 90°)
  float distancePerWheel = (PI * wheelBase) * (85.0 / 360.0);

  float requiredTicks = (distancePerWheel / (2 * PI * wheelRadius)) * ticksPerRevolution;

  // Start turning (left wheel backward, right wheel forward)
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, HIGH);
  digitalWrite(RIGHT_MOTOR_IN3, HIGH);
  digitalWrite(RIGHT_MOTOR_IN4, LOW);

  // Keep turning until encoder counts reach target
  while (abs(leftEncoderCount) < requiredTicks || abs(rightEncoderCount) < requiredTicks) {
    // Small delay to prevent overwhelming the processor
    delay(10);
  }

  // Stop motors after turn completes
  stopMotors();
}

void turnRight() {
  Serial.println("Turning Right 90 degrees");
  // Reset encoder counts before turn
  leftEncoderCount = 0;
  rightEncoderCount = 0;

  // Calculate required encoder ticks for 90 degree turn
  // float distancePerWheel = (PI * wheelBase) / 4.0;

   // Calculate required encoder ticks for 87 degree turn
  // 87° = 87/360 of full rotation (0.2417 ratio instead of 0.25 for 90°)
  float distancePerWheel = (PI * wheelBase) * (85.0 / 360.0);

  float requiredTicks = (distancePerWheel / (2 * PI * wheelRadius)) * ticksPerRevolution;

  // Start turning (left wheel forward, right wheel backward)
  digitalWrite(LEFT_MOTOR_IN1, HIGH);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN3, LOW);
  digitalWrite(RIGHT_MOTOR_IN4, HIGH);

  // Keep turning until encoder counts reach target
  while (abs(leftEncoderCount) < requiredTicks || abs(rightEncoderCount) < requiredTicks) {
    delay(10);
  }

  // Stop motors after turn completes
  stopMotors();
}



// Interrupt Service Routines
void IRAM_ATTR handleLeftEncoder() {
  leftEncoderCount++;
}

void IRAM_ATTR handleRightEncoder() {
  rightEncoderCount++;
}

// Distance Measurement
long getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2;
}

void setup() {

  Serial.begin(115200);
  Serial.println("Robot init");

  WiFi.softAP("ONESTEP", "");

  Serial.println("softap");
  Serial.println("");

  Serial.println(WiFi.softAPIP());
  websockets.begin();  //it will start web scoket
  websockets.onEvent(webSocketEvent);

  // Motor Pins Setup
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_IN3, OUTPUT);
  pinMode(RIGHT_MOTOR_IN4, OUTPUT);

  // Encoder Pins Setup
  pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), handleLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), handleRightEncoder, RISING);

  // Ultrasonic Sensor Pins Setup
  pinMode(ULTRASONIC1_TRIG, OUTPUT);
  pinMode(ULTRASONIC1_ECHO, INPUT);
  pinMode(ULTRASONIC2_TRIG, OUTPUT);
  pinMode(ULTRASONIC2_ECHO, INPUT);
  pinMode(ULTRASONIC3_TRIG, OUTPUT);
  pinMode(ULTRASONIC3_ECHO, INPUT);

  // Initialize odometry tracking variables
  lastOdometryUpdate = millis();  // Initialize odometry timer
  prevLeftEncoder = 0;            // Initialize previous encoder counts
  prevRightEncoder = 0;

  stopMotors();  // Ensure motors are stopped on startup
}

void loop() {

  websockets.loop();


  leftDist = getDistance(ULTRASONIC1_TRIG, ULTRASONIC1_ECHO);
  rightDist = getDistance(ULTRASONIC2_TRIG, ULTRASONIC2_ECHO);
  frontDist = getDistance(ULTRASONIC3_TRIG, ULTRASONIC3_ECHO);

  Serial.print(" frontDist: ");
  Serial.print(frontDist);

  Serial.print("leftDist: ");
  Serial.print(leftDist);
  // Serial.println(" cm");



  Serial.print(" rightDist: ");
  Serial.print(rightDist);


  // Update odometry
  updateOdometry();  // Calculate robot position and orientation

  // Solve the maze autonomously
  // Only solve maze if activated
  if (mazeSolvingActive) {
    solveMaze();
  }

  unsigned long currentTime = millis();
  if (currentTime - lastTime >= 500) {  // Send data every 500ms

    float timeDiff = (currentTime - lastTime) / 1000.0;  // in seconds

    // // Calculate RPM (assuming you send data every 500ms)
    // leftRPM = (leftEncoderCount * 60 * 2) / (currentTime - lastTime);  // multiply by 2 since we count every 500ms
    // rightRPM = (rightEncoderCount * 60 * 2) / (currentTime - lastTime);

    // Correct RPM calculation
    leftRPM = (leftEncoderCount / ticksPerRevolution) * (60.0 / timeDiff);
    rightRPM = (rightEncoderCount / ticksPerRevolution) * (60.0 / timeDiff);


    sendSensorData();

    // Reset encoder counts after sending
    leftEncoderCount = 0;
    rightEncoderCount = 0;
    lastTime = currentTime;
  }


  delay(50);
}
