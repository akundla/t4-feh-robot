// INCLUDE LIBRARIES
#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>

//Include the FEHMotor library
#include <FEHMotor.h>
//Include the FEHServo library
#include <FEHServo.h>

// DECLARE GLOBAL CONSTANTS

// SENSORS
//Declare a CdS Cell sensor as an analog input and assign it to an IO port
AnalogInputPin cdsCell (FEHIO::P2_0);

//Declare a microswitch as a digital input and assign it to an IO port
DigitalInputPin frontLeftBump (FEHIO::P3_6);
DigitalInputPin frontRightBump (FEHIO::P0_1);
DigitalInputPin backLeftBump (FEHIO::P3_7);
DigitalInputPin backRightBump (FEHIO::P0_0);

AnalogInputPin leftOptosensor (FEHIO::P2_2);
AnalogInputPin middleOptosensor (FEHIO::P2_1);
AnalogInputPin rightOptosensor (FEHIO::P2_0);

//MOTORS
//Assign the right and left motors to motor ports with a max voltage of 9.0V
FEHMotor leftMotor (FEHMotor :: Motor0, 9.0);
FEHMotor rightMotor (FEHMotor :: Motor1, 9.0);

//Declare a servo motor and assign it to a servo port
FEHServo servo (FEHServo::Servo0);

// LIGHT CONSTANTS
// Voltage reading from CDS cell when cell is covered (no light, pitch black)
double NO_LIGHT_V = 3.400;

// CDS Cell average Voltage readings
double BLUE_LIGHT_NO_FILTER_V_AVG = 0.524;
double RED_LIGHT_NO_FILTER_V_AVG = 0.188;
double BLUE_BACKLGROUND_V_AVG = 3.200;

// Ratio that a CDS cell reading is multiplied by to create a degree (0 - 180) to which to move the servo
// CDS cell has an approximate voltage range of 0 - 3.400 volts. Servo has range of 0 - 180 degrees. 180 / 3.400 = 52.94
double RATIO_SERVO_DEGREE_TO_CDS_CELL = 52.94;

// CALIBRATION VALUES
// Calibration values for Exploration 1 servo
int EXP_1_SERVO_MIN = 500;
int EXP_1_SERVO_MAX = 2340;


// CALIBRATION FUNCTIONS

// Calibrates the servo tested in Exploration 1
void calibrateServo() {
    // Calibrate Servo
    servo.SetMin(EXP_1_SERVO_MIN);
    servo.SetMax(EXP_1_SERVO_MAX);
}


// DRIVE FUNCTIONS

// TODO: Fine-tune for each motor setup
//Create a function to drive motors forward until microswitch is pressed.
void DriveForwardUntilHitWall(int left_motor_percent, int right_motor_percent)
{
    //Turn both motors on at given percent motor power.
    leftMotor.SetPercent(left_motor_percent);
    rightMotor.SetPercent(right_motor_percent);

    // Psuedo-infinite loop to burn time while both switches are not pressed.
    // Note that bump switches are "true" when not pressed and "false" when pressed
    while (frontLeftBump.Value() == 1 || frontRightBump.Value() == 1) { }

    // Stops motors
    leftMotor.Stop();
    rightMotor.Stop();
}

// TODO: Fine-tune for each motor setup
// Function to reverse the robot to the left until a wall is hit
void BackLeftTurnUntilHitWall()
{
    // Set both motors to a low power level
    leftMotor.SetPercent(-10);
    rightMotor.SetPercent(-25);

    // Psuedo-infinite loop to burn time while both switches are not pressed.
    // Note that bump switches are "true" when not pressed and "false" when pressed
    while (backLeftBump.Value() == 1 || backRightBump.Value() == 1) { }

    // Stops motors
    leftMotor.Stop();
    rightMotor.Stop();
}

// TODO: Fine-tune for each motor setup
// Function to reverse the robot back to the right until one switch hits a wall
void BackRightTurnUntilCornerHitWall()
{
    // Set both motors to a low power level
    leftMotor.SetPercent(-50);
    rightMotor.SetPercent(-10);

    // Psuedo-infinite loop to burn time while one switch is not pressed.
    // Note that bump switches are "true" when not pressed and "false" when pressed
    while (backLeftBump.Value() == 1 && backRightBump.Value() == 1) { }

    // Stops left motor
    leftMotor.Stop();

    // Drives the right motor forward to make the robot square in the course
    rightMotor.SetPercent(25);

    // Allows the right motor to run for 2 tenths of a second
    Sleep(0.2);

    // Stops right motor
    rightMotor.Stop();
}


// LIGHT FUNCTIONS

// Turns a servo motor in response to the reading from a CDS light cell
void moveServoToLight() {
    // Runs continuously
    while(true) {
        //Print the value of the CdS cell to the screen.
        LCD.Write("Voltage from CDS cell: ");
        LCD.Write(cdsCell.Value());
        LCD.Write("Volts");

        // Set the servo arm depending on the amount of light from the CDS cell
        // 0 light corresponds to 0 degress and full light to 180 degrees
        servo.SetDegree( (NO_LIGHT_V - cdsCell.Value() ) * RATIO_SERVO_DEGREE_TO_CDS_CELL);
    }
}

// Runs continuously and prints to the screen if the CDS cell detects blue light
void detectBlueLight() {

    // Acceptable margin of error (+/-) in voltage value (determines window in which light can be detected)
    double LightV_MoE = 0.01;

    // If the voltage is within the MoE of blue light, prints to the screen that the light is blue
    if (cdsCell.Value() < BLUE_LIGHT_NO_FILTER_V_AVG + LightV_MoE && cdsCell.Value() > BLUE_LIGHT_NO_FILTER_V_AVG - LightV_MoE) {
        LCD.WriteLine("Blue Light detected");
    } else {
        LCD.WriteLine("Blue Light not detected");
    }
}


// COMBINATION FUNCTIONS

// TODO: Recalibrate offset for new motor setup
// Uses functions written above to navigate the Exploration 1 course
void navigateExploration1Course() {

    double OFFSET_TO_DRIVE_STRAIGHT = 1.5;
    int QUARTER_POWER_PERCENT = 25;

    DriveForwardUntilHitWall(QUARTER_POWER_PERCENT, QUARTER_POWER_PERCENT - OFFSET_TO_DRIVE_STRAIGHT);
    BackLeftTurnUntilHitWall();
    DriveForwardUntilHitWall(QUARTER_POWER_PERCENT, QUARTER_POWER_PERCENT - OFFSET_TO_DRIVE_STRAIGHT);
    BackRightTurnUntilCornerHitWall();
    DriveForwardUntilHitWall(QUARTER_POWER_PERCENT, QUARTER_POWER_PERCENT - OFFSET_TO_DRIVE_STRAIGHT);
}

void lineFollowerPrintValues() {

    float trashX, trashY;

    while (true) {

        LCD.WriteLine("Left optosensor voltages: ");
        for (int i = 0; i < 5; i++) {
            LCD.WriteLine(leftOptosensor.Value());
        }

        while (!LCD.Touch(&trashX, &trashY)) {}
        while (LCD.Touch(&trashX, &trashY)) {}

        LCD.WriteLine("Middle optosensor voltages: ");
        for (int i = 0; i < 5; i++) {
            LCD.WriteLine(middleOptosensor.Value());
        }

        while (!LCD.Touch(&trashX, &trashY)) {}
        while (LCD.Touch(&trashX, &trashY)) {}

        LCD.WriteLine("Right optosensor voltages: ");
        for (int i = 0; i < 5; i++) {
            LCD.WriteLine(rightOptosensor.Value());
        }

        while (!LCD.Touch(&trashX, &trashY)) {}
        while (LCD.Touch(&trashX, &trashY)) {}
    }
}

// Constants from exploration 1
// Left optosensor
#define LEFT_RED 1.250
#define LEFT_LIGHT_BACKGROUND 1.635
#define LEFT_DARK_BACKGROUND 1.691
#define LEFT_BLACK 2.293

// Middle optosensor
#define MID_RED 1.440
#define MID_LIGHT_BACKGROUND 1.626
#define MID_DARK_BACKGROUND 1.685
#define MID_BLACK 2.358

// Right optosensor
#define RIGHT_RED 1.650
#define RIGHT_LIGHT_BACKGROUND 1.912
#define RIGHT_DARK_BACKGROUND 2.028
#define RIGHT_BLACK 2.434

// This margin is wider than necessary for easy detection on clear-contrast backgrounds
#define MoE 0.100

// Navigates the robot along the black line
void FollowBlackLine(){

    // Robot navigation state
    bool leftOfLine = false;
    bool rightOfLine = false;
    bool onLine = false;

    float leftVal, midVal, rightVal;

    while(true) {
        // Take values
        leftVal = leftOptosensor.Value();
        midVal = middleOptosensor.Value();
        rightVal = rightOptosensor.Value();

        // if left sees line, position = right of line
        if (leftVal > LEFT_BLACK - MoE && leftVal < LEFT_BLACK + MoE) {
            leftOfLine = false;
            rightOfLine = true;
            onLine = false;
        }

        // if right sees line, position = left of line
        else if (rightVal > RIGHT_BLACK - MoE && rightVal < RIGHT_BLACK + MoE) {
            leftOfLine = true;
            rightOfLine = false;
            onLine = false;
        }

        // if middle sees line, position = on line
        else if (midVal > MID_BLACK - MoE && midVal < MID_BLACK + MoE) {
            leftOfLine = false;
            rightOfLine = false;
            onLine = true;
        }

        if (onLine) {

            LCD.WriteLine("On line");

            leftMotor.SetPercent(15);
            rightMotor.SetPercent(15);
        } else if (rightOfLine) {

            LCD.WriteLine("RightOfLine");

            leftMotor.SetPercent(10);
            rightMotor.SetPercent(30);
        } else if (leftOfLine) {

            LCD.WriteLine("LeftOfLine");

            leftMotor.SetPercent(30);
            rightMotor.SetPercent(10);
        }
    }
}

void FollowRedLine(){

    bool leftOfLine = false;
    bool rightOfLine = false;
    bool onLine = false;

    float leftVal, midVal, rightVal;

    while(true) {
        // Take values
        leftVal = leftOptosensor.Value();
        midVal = middleOptosensor.Value();
        rightVal = rightOptosensor.Value();

        // if left sees line, position = right of line
        if (leftVal > LEFT_RED - MoE && leftVal < LEFT_RED + MoE) {
            leftOfLine = false;
            rightOfLine = true;
            onLine = false;
        }

        // if right sees line, position = left of line
        else if (rightVal > RIGHT_RED - MoE && rightVal < RIGHT_RED + MoE) {
            leftOfLine = true;
            rightOfLine = false;
            onLine = false;
        }

        // if middle sees line, position = on line
        else if (midVal > MID_RED - MoE && midVal < MID_RED + MoE) {
            leftOfLine = false;
            rightOfLine = false;
            onLine = true;
        }

        if (onLine) {

            LCD.WriteLine("On line");

            leftMotor.SetPercent(25);
            rightMotor.SetPercent(25);
        } else if (rightOfLine) {

            LCD.WriteLine("RightOfLine");

            leftMotor.SetPercent(15);
            rightMotor.SetPercent(25);
        } else if (leftOfLine) {

            LCD.WriteLine("LeftOfLine");

            leftMotor.SetPercent(25);
            rightMotor.SetPercent(15);
        }
    }
}





// MAIN FUNCTION
int main(void)
{
    // Consider calling servo.TouchCalibrate(); if this is the first run with servos

    FollowBlackLine();

    // Just a conventional best practice
    return 0;
}
