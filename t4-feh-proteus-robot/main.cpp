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

//Create a function to drive motors forward until microswitch is pressed.
void DriveForwardUntilHitWall(int left_motor_percent, int right_motor_percent)
{
    //Turn both motors on at given percent motor power.
    leftMotor.SetPercent(motor_percent);
    rightMotor.SetPercent(motor_percent);

    // Psuedo-infinite loop to burn time while both switches are not pressed.
    // Note that bump switches are "true" when not pressed and "false" when pressed
    while (frontLeftBump || frontRightBump);

    // Stops motors
    leftMotor.stop();
    rightMotor.stop();
}

// Function to reverse the robot to the left until a wall is hit
void BackLeftTurnUntilHitWall()
{
    // Set both motors to a low power level
    leftMotor.SetPercent(10);
    rightMotor.SetPercent(25);

    // Psuedo-infinite loop to burn time while both switches are not pressed.
    // Note that bump switches are "true" when not pressed and "false" when pressed
    while (backLeftBump || backRightBump);

    // Stops motors
    leftMotor.stop();
    rightMotor.stop();
}

// Function to reverse the robot back to the right until one switch hits a wall
void BackRightTurnUntilCornerHitWall()
{
    // Set both motors to a low power level
    leftMotor.SetPercent(25);
    rightMotor.SetPercent(10);

    // Psuedo-infinite loop to burn time while one switch is not pressed.
    // Note that bump switches are "true" when not pressed and "false" when pressed
    while (backLeftBump && backRightBump);

    // Stops right motor
    rightMotor.stop();

    // Accellerates the left motor to complete the turn
    leftMotor.setPercent(50);

    // Allows the left motor to run for a few tenths of a second
    Sleep(0.2);

    // Stops left motor
    leftMotor.stop();
}


// LIGHT FUNCTIONS

// Turns a servo motor in response to the reading from a CDS light cell
void moveServoToLight() {
    // Runs continuously
    while(true) {
        //Print the value of the CdS cell to the screen.
        LCD.WriteLine("Voltage from CDS cell: " + cdsCell.Value() + "Volts");

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

// Uses functions written above to navigate the Exploration 1 course
void navigateExploration1Course() {

    DriveForwardUntilHitWall(QUARTER_POWER, QUARTER_POWER);
    BackLeftTurnUntilHitWall();
    DriveForwardUntilHitWall(QUARTER_POWER, QUARTER_POWER);
    BackRightTurnUntilCornerHitWall();
    DriveForwardUntilHitWall(QUARTER_POWER, QUARTER_POWER);
}


// MAIN FUNCTION
int main(void)
{
    navigateExploration1Course();

    // Just a conventional best practice
    return 0;
}
