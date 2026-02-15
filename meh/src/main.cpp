#include <Arduino.h>
#include <ESP32Servo.h>
// Define servo pins
#define BASE_PIN 8
#define ARM_PIN 5
#define WRIST_PIN 4
#define GRIPPER_ROTATION_PIN 3
#define GRIPPER_PIN 2
#define BUZZER_PIN 7
// Define color sensor pins
#define S0_PIN 10
#define S1_PIN 11
#define S2_PIN 12
#define S3_PIN 13
#define OUT_PIN 9
// Create servo objects
Servo baseServo;
Servo armServo;
Servo wristServo;
Servo gripperRotationServo;
Servo gripperServo;
// Define movement speed (larger number = slower movement)
// This is the delay in ms between each degree of movement
int movementDelay = 12; // at 10 it works fine but the impact is a bit high whicch mackes small oscillations, 12 works perfectly
// Current positions, WHICH SHOULD BE THE POSTUION WHEN THE ROBOT FINISHES A JOB..
int currentBasePos = 90;
int currentArmPos = 75;
int currentWristPos = 180;
int currentGripperRotationPos = 58;
int currentGripperPos = 145;
// Color values
int redValue, greenValue, blueValue;
// Number of readings to average for color detection
#define NUM_READINGS 5
// Color thresholds with wider ranges for better detection
// Blue ball
#define BLUE_R_MIN 54  // Slightly lower minimum
#define BLUE_R_MAX 100 // Slightly higher maximum
#define BLUE_G_MIN 82  // Slightly lower minimum
#define BLUE_G_MAX 130 // Slightly higher maximum
#define BLUE_B_MIN 57  // Slightly lower minimum
#define BLUE_B_MAX 95  // Slightly higher maximum
// White ball
#define WHITE_R_MIN 25 // Slightly lower minimum
#define WHITE_R_MAX 60 // Slightly higher maximum
#define WHITE_G_MIN 35 // Slightly lower minimum
#define WHITE_G_MAX 60 // Slightly higher maximum
#define WHITE_B_MIN 28 // Slightly lower minimum
#define WHITE_B_MAX 50 // Slightly higher maximum
// Red ball (placeholder values - to be updated)
#define RED_R_MIN 28
#define RED_R_MAX 60
#define RED_G_MIN 67
#define RED_G_MAX 120
#define RED_B_MIN 58
#define RED_B_MAX 110
// Flag to prevent multiple detections in a short time
bool ballProcessed = false;
unsigned long lastDetectionTime = 0;
#define DETECTION_COOLDOWN 3000 // 3 seconds cooldown between detections
void setup()
{
    Serial.begin(9600);
    // Attach servos to pins
    baseServo.attach(BASE_PIN);
    armServo.attach(ARM_PIN);
    wristServo.attach(WRIST_PIN);
    gripperRotationServo.attach(GRIPPER_ROTATION_PIN);
    gripperServo.attach(GRIPPER_PIN);
    // Set buzzer pin as output
    pinMode(BUZZER_PIN, OUTPUT);
    // Set color sensor pins
    pinMode(S0_PIN, OUTPUT);
    pinMode(S1_PIN, OUTPUT);
    pinMode(S2_PIN, OUTPUT);
    pinMode(S3_PIN, OUTPUT);
    pinMode(OUT_PIN, INPUT);
    // Set sensor frequency scaling to 20%
    digitalWrite(S0_PIN, HIGH);
    digitalWrite(S1_PIN, LOW);
    // Play a startup tone to indicate the system is starting
    tone(BUZZER_PIN, 440, 100);
    delay(150);
    tone(BUZZER_PIN, 523, 100);
    delay(150);
    tone(BUZZER_PIN, 659, 100);
    delay(150);
    noTone(BUZZER_PIN);
    Serial.println("Robot Arm initialized");
    // Initialize all servos one by one, with delays between movements
    Serial.println("Moving to resting position...");
    // Move each servo to initial position separately
    baseServo.write(90);
    delay(5);
    armServo.write(75);
    delay(5);
    wristServo.write(180);
    delay(5);
    gripperRotationServo.write(58);
    delay(5);
    gripperServo.write(145);
    delay(5);
    // Update current positions
    currentBasePos = 90;
    currentArmPos = 75;
    currentWristPos = 180;
    currentGripperRotationPos = 58;
    currentGripperPos = 145;
    Serial.println("Initialization complete");
    delay(1000);
}
// Function to move a servo gradually with more careful control
void moveServo(Servo &servo, int &currentPos, int targetPos, String servoName)
{
    if (currentPos == targetPos)
        return;
    // Print movement info
    Serial.print("Moving ");
    Serial.print(servoName);
    Serial.print(" from ");
    Serial.print(currentPos);
    Serial.print(" to ");
    Serial.println(targetPos);
    // Calculate step size based on distance to move
    int distance = abs(targetPos - currentPos);
    int stepDelay = movementDelay;
    // For larger movements, move in smaller increments with larger delays for smoother motion
    if (distance > 30)
    {
        stepDelay = movementDelay * 1.2;
    }
    if (currentPos < targetPos)
    {
        for (int pos = currentPos; pos <= targetPos; pos++)
        {
            servo.write(pos);
            delay(stepDelay);
        }
    }
    else
    {
        for (int pos = currentPos; pos >= targetPos; pos--)
        {
            servo.write(pos);
            delay(stepDelay);
        }
    }
    currentPos = targetPos;
    // Ensure the servo has time to reach its position
    delay(200);
}
// Function to read color sensor values with averaging for stability
void readColorSensor()
{
    long redSum = 0, greenSum = 0, blueSum = 0;
    for (int i = 0; i < NUM_READINGS; i++)
    {
        // Read red value
        digitalWrite(S2_PIN, LOW);
        digitalWrite(S3_PIN, LOW);
        redSum += pulseIn(OUT_PIN, LOW);
        delay(10);
        // Read green value
        digitalWrite(S2_PIN, HIGH);
        digitalWrite(S3_PIN, HIGH);
        greenSum += pulseIn(OUT_PIN, LOW);
        delay(10);
        // Read blue value
        digitalWrite(S2_PIN, LOW);
        digitalWrite(S3_PIN, HIGH);
        blueSum += pulseIn(OUT_PIN, LOW);
        delay(10);
    }
    // Calculate averages
    redValue = redSum / NUM_READINGS;
    greenValue = greenSum / NUM_READINGS;
    blueValue = blueSum / NUM_READINGS;
    // Print the values to serial for debugging
    Serial.print("Red: ");
    Serial.print(redValue);
    Serial.print(" Green: ");
    Serial.print(greenValue);
    Serial.print(" Blue: ");
    Serial.println(blueValue);
}
// Function to detect color
String detectColor()
{
    // Read the sensor values
    readColorSensor();
    // Check if it's a blue ball
    if (redValue >= BLUE_R_MIN && redValue <= BLUE_R_MAX &&
        greenValue >= BLUE_G_MIN && greenValue <= BLUE_G_MAX &&
        blueValue >= BLUE_B_MIN && blueValue <= BLUE_B_MAX)
    {
        Serial.println("*** BLUE BALL DETECTED ***");
        return "blue";
    }
    // Check if it's a white ball
    if (redValue >= WHITE_R_MIN && redValue <= WHITE_R_MAX &&
        greenValue >= WHITE_G_MIN && greenValue <= WHITE_G_MAX &&
        blueValue >= WHITE_B_MIN && blueValue <= WHITE_B_MAX)
    {
        Serial.println("*** WHITE BALL DETECTED ***");
        return "white";
    }
    // Check if it's a red ball (placeholder - values to be updated)
    if (redValue >= RED_R_MIN && redValue <= RED_R_MAX &&
        greenValue >= RED_G_MIN && greenValue <= RED_G_MAX &&
        blueValue >= RED_B_MIN && blueValue <= RED_B_MAX)
    {
        Serial.println("*** RED BALL DETECTED ***");
        return "red";
    }
    // Return "unknown" if no match
    return "unknown";
}
// Function to play a beep sound
void playBeep()
{
    tone(BUZZER_PIN, 440, 250); // 440Hz for shorter duration
    delay(200);
    noTone(BUZZER_PIN);
}
// DEFINE ALL POSTIONS NEEDED.
//  Function to move to resting position
void goToRestingPosition()
{
    Serial.println("Going to resting position...");
    moveServo(baseServo, currentBasePos, 90, "Base");
    delay(100);
    moveServo(armServo, currentArmPos, 75, "Arm");
    delay(100);
    moveServo(wristServo, currentWristPos, 180, "Wrist");
    delay(100);
    moveServo(gripperRotationServo, currentGripperRotationPos, 58, "Gripper Rotation");
    delay(100);
    moveServo(gripperServo, currentGripperPos, 145, "Gripper");
    delay(100);
    Serial.println("Now in resting position");
}
// Function to move to pickup position
void goToPickupPosition()
{
    Serial.println("Going to pickup position...");
    moveServo(baseServo, currentBasePos, 90, "Base");
    delay(100);
    moveServo(wristServo, currentWristPos, 4, "Wrist");
    delay(150);
    moveServo(gripperRotationServo, currentGripperRotationPos, 38, "Gripper Rotation");
    delay(200);
    moveServo(armServo, currentArmPos, 41, "Arm");
    delay(500);
    moveServo(gripperServo, currentGripperPos, 125, "Gripper");
    delay(200);
    Serial.println("Now in pickup position");
}
// Function to move to upright position
void goToUprightPosition()
{
    Serial.println("Going to upright position...");
    moveServo(armServo, currentArmPos, 90, "Arm");
    delay(150);
    moveServo(wristServo, currentWristPos, 90, "Wrist");
    delay(150);
    moveServo(gripperRotationServo, currentGripperRotationPos, 58, "Gripper Rotation");
    delay(50);
    Serial.println("Now in upright position");
}
// Function to move to CW rotation position
void goToCWRotationPosition()
{
    Serial.println("Going to CW rotation position...");
    moveServo(armServo, currentArmPos, 90, "Arm");
    delay(50);
    moveServo(wristServo, currentWristPos, 90, "Wrist");
    delay(50);
    moveServo(gripperRotationServo, currentGripperRotationPos, 148, "Gripper Rotation");
    delay(150);
    moveServo(baseServo, currentBasePos, 0, "Base");
    delay(400);
    Serial.println("Now in CW rotation position");
}
// Function to move to CCW rotation position
void goToCCWRotationPosition()
{
    Serial.println("Going to CCW rotation position...");
    moveServo(armServo, currentArmPos, 90, "Arm");
    delay(50);
    moveServo(wristServo, currentWristPos, 90, "Wrist");
    delay(50);
    moveServo(gripperRotationServo, currentGripperRotationPos, 148, "Gripper Rotation");
    delay(100);
    moveServo(baseServo, currentBasePos, 180, "Base");
    delay(400);
    Serial.println("Now in CCW rotation position");
}
// Function to move to deploy position
void goToDeployPosition()
{
    Serial.println("Going to deploy position...");
    moveServo(gripperRotationServo, currentGripperRotationPos, 58, "Gripper Rotation");
    delay(100);
    moveServo(armServo, currentArmPos, 140, "Arm");
    delay(200);
    moveServo(wristServo, currentWristPos, 160, "Wrist");
    delay(200);
    Serial.println("Now in deploy position");
}
// HANDLING THE BALLSZZ
//  Function to handle blue ball
void handleBlueBall()
{
    Serial.println("I guess I saw a blue there let me take it to its place ...Processing blue ball...");
    playBeep();
    // Go to pickup position
    goToPickupPosition();
    // Close gripper to grab the ball
    Serial.println("Closing gripper to grab ball...");
    moveServo(gripperServo, currentGripperPos, 125, "Gripper");
    delay(500); // Longer delay to ensure grip is secure
    // Go to upright position upright postion is not needed as the base wonbt rotate for thus ball.
    goToUprightPosition();
    // Go to deploy position
    goToDeployPosition();
    // Open gripper to release ball
    Serial.println("Opening gripper to release ball...");
    moveServo(gripperServo, currentGripperPos, 145, "Gripper");
    delay(300); // Longer delay to ensure release
    // Return to resting position
    goToRestingPosition();
    // Play beep to signal completion
    playBeep();
    Serial.println("Blue ball handling COMPLETED ;) waiting for your next order SIR ");
}
// Function to handle white ball
void handleWhiteBall()
{
    Serial.println("That is a white ball there I guess.. let me take it to its place...");
    playBeep();
    // Go to pickup position
    goToPickupPosition();
    // Close gripper to grab the ball
    Serial.println("Closing gripper to grab ball...");
    moveServo(gripperServo, currentGripperPos, 125, "Gripper");
    delay(400); // Longer delay to ensure grip is secureSpring 2024 MECH 458 Robot Arm with pick and place functionality Instructor: Esref

    // go to upright postion so the base can rotate easily
    goToUprightPosition();
    // Go to CCW rotation position
    goToCCWRotationPosition();
    // go to upright postion so the base can rotate easily
    goToUprightPosition();
    // Go to deploy position
    goToDeployPosition();
    // Open gripper to release ball
    Serial.println("Opening gripper to release ball...");
    moveServo(gripperServo, currentGripperPos, 145, "Gripper");
    delay(300); // Longer delay to ensure release
    // Return to resting position
    goToRestingPosition();
    // Play beep to signal completion
    playBeep();
    Serial.println("White ball handling completed sir, waiting for your next order");
}
// Function to handle red ball ()
void handleRedBall()
{
    Serial.println("I can see a RED ball there.. Let me take it to its place...");
    playBeep();
    // Go to pickup position
    goToPickupPosition();
    // Close gripper to grab the ball
    Serial.println("Closing gripper to grab ball...");
    moveServo(gripperServo, currentGripperPos, 125, "Gripper");
    delay(500); // Longer delay to ensure grip is secure
    // go to upright postion so the base can rotate easily
    goToUprightPosition();
    // Go to CW rotation position
    goToCWRotationPosition();
    // go to upright postion so the base can rotate easily
    goToUprightPosition();
    // Go to deploy position
    goToDeployPosition();
    // Open gripper to release ball
    Serial.println("Opening gripper to release ball...");
    moveServo(gripperServo, currentGripperPos, 145, "Gripper");
    delay(300); // Longer delay to ensure release
    // Return to resting position
    goToRestingPosition();
    // Play beep to signal completion
    playBeep();
    Serial.println("Red ball handling completed, waiting for my next task :) ");
}
void loop()
{
    Serial.println("Scanning for balls...");
    // Take multiple readings for stability
    for (int i = 0; i < 3; i++)
    {
        // Read color sensor
        String ballColor = detectColor();
        // Check if cooldown period has passed since last detection
        unsigned long currentTime = millis();
        if (currentTime - lastDetectionTime < DETECTION_COOLDOWN)
        {
            delay(100);
            continue;
        }

        if (ballColor != "unknown")
        {
            lastDetectionTime = currentTime;
            if (ballColor == "blue")
            {
                handleBlueBall();
                break;
            }
            else if (ballColor == "white")
            {
                handleWhiteBall();
                break;
            }
            else if (ballColor == "red")
            {
                handleRedBall();
                break;
            }
        }
        // Brief delay between readings
        delay(100);
    }
    // Wait before scanning again
    delay(500);
}