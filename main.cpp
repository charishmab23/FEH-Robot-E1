#include <FEHLCD.h>
#include <FEHMotor.h>
#include <FEHIO.h>
#include <FEHBattery.h>
#include <FEHSD.h>
#include <FEHServo.h>
#include <math.h>
#include <FEHRCS.h>
#include <FEHUtility.h>

// Define constants
#define DC_VOLTAGE 9.0
#define TARGET_LOW_POWER 25.0
#define TARGET_HIGH_POWER 75.0
#define RAIL_WIDTH 5.0
#define PI 3.14159265358979323846
#define COUNTS_PER_INCH 40.4890175226 // 318 counts per revolution / 2.5 pi inches per revolution 
#define WAIT_TIME 0.25
#define MIN_MOTOR -100.0
#define MAX_MOTOR 100.0
#define TIMEOUT 5.0
#define LIGHT_TIMEOUT 30.0
#define LIGHT_THRESHOLD 1.05
#define BUFFER 0.05
#define BLUE_CDS 1.45
#define LIGHT_ERROR 0.4
#define LEFT_ADJUST 1.65
#define P_CONST 0.75
#define I_CONST 0.1
#define D_CONST 0.0001
#define ACC_DIST 2.0
#define SERVO_MIN 500
#define SERVO_MAX 2250
#define SERVO_HORIZONTAL 110.0
#define TURN_VELOCITY 5.0
#define MAX_SERVO_DEGREE 180.0
#define PRESS_LEVER 130.0
#define ROBBIE "E1z9DCs8Z"

//Set this to skip RCS
#define DEBUG false


enum Direction {
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    STOPPED,
};

enum Light {
    WHITELIGHT,
    REDLIGHT,
    BLUELIGHT,
    OTHERLIGHT,
    OFFLIGHT,
};

// Define hardware
FEHMotor RIGHT_MOTOR(FEHMotor::Motor2, 9.0);
FEHMotor LEFT_MOTOR(FEHMotor::Motor0, 9.0);

DigitalEncoder RIGHT_ENCODER(FEHIO::P1_0);
DigitalEncoder LEFT_ENCODER(FEHIO::P1_3);
DigitalInputPin RIGHT_SWITCH(FEHIO::P2_5);
DigitalInputPin LEFT_SWITCH(FEHIO::P2_7);
AnalogInputPin CDS(FEHIO::P2_0);
FEHServo ARM(FEHServo::Servo7);

/**
 * The PIDController runs a Proportion-Integral-Derivative
 * loop in order to adjust the power to each DC motor to the
 * values necessary to get the robot to drive straight.
 * 
 * Designed using the following Wikipedia article:
 * https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller
 * 
 * Also used the information in the robot resources folder
*/
class PIDController {
    private:
        float leftPower = 0.0;
        float rightPower = 0.0;
        float leftTotalDistance = 0.0;
        float rightTotalDistance = 0.0;

        float backupLeftPower = 0.0;

        int leftLastCounts = 0;
        int rightLastCounts = 0;
        float leftPrevError = 0.0;
        float rightPrevError = 0.0;
        float leftAccError = 0.0;
        float rightAccError = 0.0;
        float leftLastError = 0.0;
        float rightLastError = 0.0;
        float targetSpeed = 0.0;
        float lastTime;
        float proportionalConstant;
        float integralConstant;
        float derivativeConstant;
    public:
        PIDController(float time, float p = P_CONST, float i = I_CONST, float d = D_CONST) {
            lastTime = time;
            proportionalConstant = p;
            integralConstant = i;
            derivativeConstant = d;
            LCD.Write("prop const: ");
        }
        void dumpSD() {
            FEHFile* sdcard = SD.FOpen("stuff.txt", "w");
            
            SD.FPrintf(sdcard, "%f", leftPower);
            SD.FPrintf(sdcard, "%f", rightPower);
            SD.FPrintf(sdcard, "%f", leftTotalDistance);
            SD.FPrintf(sdcard, "%f", rightTotalDistance);
            SD.FPrintf(sdcard, "%f", backupLeftPower);
            SD.FPrintf(sdcard, "%f", leftLastCounts);
            SD.FPrintf(sdcard, "%f", rightLastCounts);
            SD.FPrintf(sdcard, "%f", leftPrevError);
            SD.FPrintf(sdcard, "%f", rightPrevError);
            SD.FPrintf(sdcard, "%f", leftAccError);
            SD.FPrintf(sdcard, "%f", rightAccError);
            SD.FPrintf(sdcard, "%f", leftLastError);
            SD.FPrintf(sdcard, "%f", rightLastError);
            SD.FPrintf(sdcard, "%f", targetSpeed);
            SD.FPrintf(sdcard, "%f", lastTime);
            SD.FPrintf(sdcard, "%f", proportionalConstant);
            SD.FPrintf(sdcard, "%f", integralConstant);
            SD.FPrintf(sdcard, "%f", derivativeConstant);

            SD.FClose(sdcard);

            LCD.Write("     ");
            LCD.WriteLine(proportionalConstant);
            LCD.Write("     ");
            LCD.WriteLine(integralConstant);
            LCD.Write("     ");
            LCD.WriteLine(derivativeConstant);
        }
        float distanceTraveled() {
            return (leftTotalDistance + rightTotalDistance) / 2;
        }
        float leftDCPower() {
            return leftPower;
        }
        float rightDCPower() {
            return rightPower;
        }
        void reset(float time) {
            LEFT_ENCODER.ResetCounts();
            RIGHT_ENCODER.ResetCounts();

            leftPower = 0.0;
            rightPower = 0.0;

            leftTotalDistance = 0.0;
            rightTotalDistance = 0.0;

            leftLastCounts = 0;
            rightLastCounts = 0;
            leftPrevError = 0.0;
            rightPrevError = 0.0;
            leftAccError = 0.0;
            rightAccError = 0.0;
            leftLastError = 0.0;
            rightLastError = 0.0;
            lastTime = time;
        }
        /**
         * This sets the target speed IN INCHES/SECOND
        */
        void setTargetSpeed(float newSpeed) {
            targetSpeed = newSpeed;
        }

        void getCorrection() {
            // DO NOT REMOVE!!!!!!! FOR SOME REASON leftPower GETS RESET AT THE END!!!!!!!!
            //leftPower = backupLeftPower;

            int newLeftCounts = LEFT_ENCODER.Counts() - leftLastCounts;
            int newRightCounts = RIGHT_ENCODER.Counts() - rightLastCounts;
            float deltaTime = TimeNow() - lastTime;

            float leftDistance = newLeftCounts / COUNTS_PER_INCH;
            float rightDistance = newRightCounts / COUNTS_PER_INCH;

            float leftV = leftDistance / deltaTime;
            float rightV = rightDistance / deltaTime;

            float leftError = targetSpeed - leftV;
            float rightError = targetSpeed - rightV;
            leftAccError += leftError;
            rightAccError += rightError;

            float leftPTerm =  proportionalConstant * leftError;
            float rightPTerm = proportionalConstant * rightError;

            float leftITerm =  integralConstant * leftAccError;
            float rightITerm = integralConstant * rightAccError;

            float leftDTerm =  derivativeConstant * ((leftError - leftPrevError) / deltaTime);
            float rightDTerm = derivativeConstant * ((rightError - rightPrevError) / deltaTime);

            leftLastCounts += newLeftCounts;
            rightLastCounts += newRightCounts;
            lastTime += deltaTime;
            leftPrevError = leftError;
            rightPrevError = rightError;
            leftTotalDistance += leftDistance;
            rightTotalDistance += rightDistance;

            leftPower = leftPower + leftPTerm + leftITerm + leftDTerm;
            rightPower = rightPower + rightPTerm + rightITerm + rightDTerm;

            //backupLeftPower = leftPower;

            // Throw an error if motor power exceeds the maximum
            /*
            if(leftPower > 100.0 || leftPower < -100.0) {
                leftPower = 0.0;
            }
            if(rightPower > 100.0 || rightPower < -100.0) {
                rightPower = 0.0;
            }*/
            if(leftPower > 99.0 || rightPower > 99.0) {
                leftPower = 100.0;
                rightPower = 100.0;
            }
        }
};

class Robot {
    private:
        // Control Structures
        //PIDController* pid;

        // Software Defined Values
        int leftMotorPower;
        int rightMotorPower;
        enum Direction direction;
        float redCutoff = 0;
        float blueCutoff = 0;
    public:
        Robot() {
            // Initialize Debug Screen
            LCD.Clear(BLACK);
            LCD.WriteLine("Init Debug");

            // Initialize Hardware
            LCD.WriteLine("Init Hardware");

            // Initialize Control Structures
            //PIDController pid(10);
            LCD.WriteLine("Init Control");

            // Initialize Software Defined Values
            leftMotorPower = TARGET_LOW_POWER;
            rightMotorPower = TARGET_LOW_POWER;
            LCD.WriteLine("Init Software");
            direction = STOPPED;

            ARM.SetMin(SERVO_MIN);
            ARM.SetMax(SERVO_MAX);

            // Raise servo
            ARM.SetDegree(MAX_SERVO_DEGREE);

            // Initialize RCS - Last code before robot movement
            if(!DEBUG) {
                RCS.InitializeTouchMenu(ROBBIE);
            }
        }

        /**
         * Calibrates the servo
        */
        void calibrateServo() {
            ARM.TouchCalibrate();
        }

        /**
         * Straightens out the robot arm
        */
        void straightenArm() {
            ARM.SetDegree(SERVO_HORIZONTAL);
        }

        void pressLever() {
            ARM.SetDegree(MAX_SERVO_DEGREE - 40);
        }

        void raiseArm() {
            ARM.SetDegree(MAX_SERVO_DEGREE);
        }

        void pressLeverOld() {
            ARM.SetDegree(PRESS_LEVER);
            Sleep(3.0);
            ARM.SetDegree(MAX_SERVO_DEGREE);
        }

        void raiseLever() {
            ARM.SetDegree(MAX_SERVO_DEGREE - 25);
        }

        void raisePassport() {
            ARM.SetDegree(MAX_SERVO_DEGREE - 21);
        }

        int getLever() {
            return RCS.GetCorrectLever();
        }

        /**
         * Powers the motors to the specific powers.
         * 
         * @param newLeftPower
         *      (optional) The new power for the left motor
         * @param newRightPower
         *      (optional) The new power for the right motor
         * @requries -100 <= newLeftPower <= 100
         * @requires -100 <= newRightPower <= 100
         * @ensures motors powered to the specified values
        */
        void setDCPower(int newLeftPower = MAX_MOTOR + 1, int newRightPower = MAX_MOTOR + 1) {
            if(MIN_MOTOR <= newLeftPower && newLeftPower <= MAX_MOTOR) {
                leftMotorPower = newLeftPower;
            }
            if(MIN_MOTOR <= newRightPower && newRightPower <= MAX_MOTOR) {
                rightMotorPower = newRightPower;
            }

            LEFT_MOTOR.SetPercent(leftMotorPower);
            RIGHT_MOTOR.SetPercent(rightMotorPower);
        }

        /**
         * Detects if any of the bumpers were hit
         * 
         * @returns true if a bumper is currently being hit
        */
        bool hitBumper() {
            //TODO: Add bumper detection
            return false;
        }

        int bumperHits() {
            int hits = 0;
            hits += !LEFT_SWITCH.Value();
            hits += !RIGHT_SWITCH.Value();
            return hits;
        }

        /**
         * Turns off both of the DC motors and waits for a quarter of a second
         * 
         * @ensures both DC motors stop rotating
        */
        void stop() {
            LEFT_MOTOR.Stop();
            RIGHT_MOTOR.Stop();

            direction = STOPPED;

            Sleep(WAIT_TIME);
        }
        
        /**
         * Adjusts the power to the robot so that it will continue to drive straight
         * 
         * @param time
         *      (optional) the amount of time until the robot stops driving
         * @ensures the robot will drive until it hits a wall, the specified time runs out,
         * or more than 30 seconds have elapsed.
        */
        void keepStraightTime(double time = 30.0) {
            LCD.WriteLine("Driving Straight");
            double initialTime = TimeNow();
            double timeElapsed = 0;

            int newLeftPower = leftMotorPower;
            int newRightPower = rightMotorPower;

            direction = FORWARD;

            // Continue until end conditions
            while(timeElapsed < time && !hitBumper()) {
                // TODO: Use a PID loop here
                LCD.WriteLine("Setting power:");
                LCD.WriteLine("Left:");
                LCD.WriteLine(newLeftPower);
                LCD.WriteLine("Right: ");
                LCD.WriteLine(newRightPower);
                setDCPower(newLeftPower, newRightPower);

                // Update elapsed time
                timeElapsed = TimeNow() - initialTime;
            }

            stop();
        }

        /**
         * Makes the robot turn for a specified period of time.
         * 
         * @param time
         *      The amount of time to turn in seconds
         * @param towards
         *      The direction to turn towards
         * @requires towards is LEFT or RIGHT
         * @ensures the robot will turn for the specified amount of time
        */
        void turnTime(float time, enum Direction towards) {
            switch(towards) {
                case LEFT:
                    LEFT_MOTOR.SetPercent(-1 * TARGET_LOW_POWER);
                    RIGHT_MOTOR.SetPercent(TARGET_LOW_POWER);
                    break;
                case RIGHT:
                    LEFT_MOTOR.SetPercent(TARGET_LOW_POWER);
                    RIGHT_MOTOR.SetPercent(-1 * TARGET_LOW_POWER);
                    break;
            }
            stop();
        }

        /**
         * Makes the robot go straight for the specified distance in inches
         * @param inches
         *      The distance to travel in inches
         * @param towards
         *      (optional) The direction to drive towards. Defaults to FORWARD
         * @param power
         *      (optional) The power to travel. Defualts to TARGET_LOW_POWER
         * @requires towards is FORWARD or BACKWARD
         * @ensures the robot travels straight for the specified distance
        */
        void keepStraight(double inches, enum Direction towards = FORWARD, float targetPower = TARGET_LOW_POWER, float timeout = TIMEOUT) {
            // For now, take the average of both sides. In the future, use a PIDLoop to keep straight
            if(towards == BACKWARD) {
                targetPower *= -1;
            }
            LEFT_MOTOR.SetPercent(targetPower);
            RIGHT_MOTOR.SetPercent(targetPower);

            LEFT_ENCODER.ResetCounts();
            RIGHT_ENCODER.ResetCounts();

            double inchesTraveled = 0.0;

            float startTime = TimeNow();

            while(inchesTraveled < inches && TimeNow() - startTime < timeout) {
                int leftCount = LEFT_ENCODER.Counts();
                int rightCount = RIGHT_ENCODER.Counts();

                double leftInches = leftCount / COUNTS_PER_INCH;
                double rightInches = rightCount / COUNTS_PER_INCH;

                inchesTraveled = leftInches + rightInches / 2;
            }

            stop();
        }

        /**
         * Turns the robot a specified number of radians
         * 
         * @param theta
         *      Number of radians to turn. Negative is left, positive is right
         * @requires theta is not 0
         * @ensures the robot turns the specified number of degrees.
        */
        void turnOld(double theta) {
            // Theta is the ratio between the arc lengths.
            if(theta > 0) {
                setDCPower(TARGET_LOW_POWER, -1 * TARGET_LOW_POWER);
                direction = RIGHT;
            } else {
                setDCPower(-1 * TARGET_LOW_POWER, TARGET_LOW_POWER);
                theta *= -1;
                direction = LEFT;
            }

            LEFT_ENCODER.ResetCounts();
            RIGHT_ENCODER.ResetCounts();

            // Continuously calculate the average of the two thetas
            // and continue untli traveled more than theta
            double radTraveled = 0.0;
            while(radTraveled < theta) {
                LCD.Clear();
                double leftCounts = LEFT_ENCODER.Counts();
                int rightCounts = RIGHT_ENCODER.Counts();

                double leftArc = leftCounts / COUNTS_PER_INCH;
                double rightArc = rightCounts / COUNTS_PER_INCH;

                double totalArc = leftArc + rightArc;

                radTraveled = totalArc / RAIL_WIDTH;

                LCD.WriteLine(radTraveled);
                LCD.WriteLine("Radians Traveled");
            }
            stop();
        }

        /**
         * Drived the robot straight with PID Control
         * 
         * Power is determined with a target velocity.
         * 
         * @param velocity
         *      velocity in inches per second
         * @param distance
         *      distance in inches to travel
         * @param towards
         *      direction to travel towards
         * @param timeout
         *      time before the motors timeout
        */
        void correctedStraight(float velocity, float distance, enum Direction towards = FORWARD, float timeout = TIMEOUT, bool override = false) {
            int multiplier = 1;
            if(towards == BACKWARD) {
                multiplier *= -1;
            }

            PIDController newPID(TimeNow());

            newPID.reset(TimeNow());
            float startTime = TimeNow();

            while(newPID.distanceTraveled() < distance && TimeNow() - startTime < timeout) {
                // Slowly accelerate and decellerate
                
                float targetVelocity = velocity;
                
                if(!override) {
                    if(distance - newPID.distanceTraveled() < ACC_DIST) {
                        targetVelocity *= (distance - newPID.distanceTraveled()) / ACC_DIST;
                    } else if(newPID.distanceTraveled() < ACC_DIST) {
                        targetVelocity *= newPID.distanceTraveled() / ACC_DIST;
                        if(targetVelocity < velocity / 4) {
                            targetVelocity = velocity / 4;
                        }
                    }
                }
                
                newPID.setTargetSpeed(targetVelocity);

                newPID.getCorrection();

                setDCPower(newPID.leftDCPower() * multiplier, newPID.rightDCPower() * multiplier);
                Sleep(0.05);
            }

            stop();
        }

        void correctedAlign(float velocity, Direction direction = FORWARD) {
            int multiplier = 1;
            if(direction == BACKWARD) {
                multiplier *= -1;
            }

            PIDController newPID(TimeNow());

            newPID.reset(TimeNow());
            float startTime = TimeNow();

            while(bumperHits() < 2 && TimeNow() - startTime < TIMEOUT) {
                float targetVelocity = velocity;
                
                if(newPID.distanceTraveled() < ACC_DIST) {
                    targetVelocity *= newPID.distanceTraveled() / ACC_DIST;
                    if(targetVelocity < velocity / 4) {
                        targetVelocity = velocity / 4;
                    }
                }

                newPID.setTargetSpeed(targetVelocity);

                newPID.getCorrection();

                setDCPower(newPID.leftDCPower() * multiplier, newPID.rightDCPower() * multiplier);
                Sleep(0.05);
            }

            stop();
        }

        void correctedTurn(double theta) {
            // Theta is the ratio between the arc lengths.
            int dirMultiplier;
            if(theta > 0) {
                dirMultiplier = 1;
            } else {
                dirMultiplier = -1;
            }

            PIDController newPID(TimeNow());

            newPID.reset(TimeNow());
            float startTime = TimeNow();

            float thetaTraveled = 0;

            while(thetaTraveled < abs(theta) && TimeNow() - startTime < TIMEOUT) {
                // Slowly accelerate and decellerate
                
                float targetVelocity = TURN_VELOCITY;

                thetaTraveled = (newPID.distanceTraveled() * 2) / RAIL_WIDTH;

                float theta_acc = 4 * ACC_DIST / RAIL_WIDTH;
                if(abs(theta) - thetaTraveled < theta_acc) {
                    targetVelocity *= (abs(theta) - thetaTraveled) / theta_acc;
                } else if(thetaTraveled < theta_acc) {
                    targetVelocity *= (thetaTraveled) / theta_acc;
                    if(targetVelocity < TURN_VELOCITY / 2) {
                        targetVelocity = TURN_VELOCITY / 2;
                    }
                }
                
                newPID.setTargetSpeed(targetVelocity);

                newPID.getCorrection();
                
                LCD.Clear();
                LCD.Write("\tLeft: ");
                LCD.WriteLine(newPID.leftDCPower() * dirMultiplier);
                LCD.Write("\tRight: ");
                LCD.WriteLine(newPID.rightDCPower() * dirMultiplier * -1);
                Sleep(0.1);

                setDCPower(newPID.leftDCPower() * dirMultiplier, newPID.rightDCPower() * dirMultiplier * -1);
                Sleep(0.05);
            }

            stop();
        }
        
        void turn(double theta) {
            correctedTurn(theta);
        }


        /**
         * Waits for the user to touch the screen to continue
        */
        void waitForTouch() {
            LCD.WriteLine("Waiting for touch");
            int x, y;
            while(!LCD.Touch(&x, &y)) {}
        }

        /**
         * Finds and reports the values associated with each color of light
        */
        void calibrateRedLight() {
            Sleep(2);
            float value = CDS.Value();
            redCutoff = value + BUFFER;
            LCD.WriteAt(value, 200, 200);
        }

        void calibrateBlueLight() {
            Sleep(2);
            float value = CDS.Value();
            blueCutoff = value + BUFFER;
            LCD.WriteAt(value, 200, 215);
        }

        /**
         * Reads the light
         * 
         * @returns the light read by the CdS cell
        */
        enum Light readLight(float timeout = LIGHT_TIMEOUT) {
            float startTime = TimeNow();
            enum Light returnLight = OFFLIGHT;
            while(returnLight == OFFLIGHT && TimeNow() - startTime < timeout) {
                float blueError = CDS.Value() - BLUE_CDS;
                if(abs(blueError) < LIGHT_ERROR) {
                    returnLight = BLUELIGHT;
                    LCD.Clear(BLUE);
                } else if (CDS.Value() < LIGHT_THRESHOLD) {
                    LCD.Clear(RED);
                    returnLight = REDLIGHT;
                }
            }
            return returnLight;
        }

        /**
         * Reads the light and drives to the start button.
        */
        void driveToStartButton() {
            double startTime = TimeNow();
            while(readLight() == OFFLIGHT && TimeNow() - startTime < TIMEOUT) {}
            const int startDistance = 2.0; //TODO: Update with actual measured values
            keepStraight(startDistance, BACKWARD);
        }

        /**
         * Used to complete Checkpoint 1. This includes driving
         * up the ramp and hitting the ticket counter.
        */
        void finishCheckpoint1() {
            readLight();
            turn(0.75);
            keepStraight(20);
            keepStraight(25, FORWARD, TARGET_HIGH_POWER);
            turn(-0.2);
            keepStraight(50);
        }

        /**
         * Used to get bonus points on Checkpoint 1.
         * It drives back down the ramp :)
        */
        void checkpoint1Bonus() {
            keepStraight(3.0, BACKWARD);
            turn(PI - 0.6);
            keepStraight(30);
            turn(.3);
            keepStraight(40);
        }

        /**
         * Used to align the robot against the right wall for checkpoint 2
        */
        void alignCheckpoint2() {
            const float backupFromTicket = 15.0;
            const float alignBackup = PI / -2.0 + PI / 8;
            const float backupToWall = 35.0;

            keepStraight(backupFromTicket, BACKWARD);
            turn(alignBackup);
            keepStraight(backupToWall, BACKWARD);
        }

        /**
         * Drives to the back wall from the ticket counter light
        */
        enum Light checkpoint2Light() {
            const float distanceToTurn = 10;
            const float turnToLight = PI / 4.0 - PI / 32;
            const float distanceToLight = 28.5;
            const float hitWall = 10.0;

            keepStraight(distanceToTurn);
            turn(turnToLight);
            keepStraight(distanceToLight);
            
            enum Light lightValue = readLight();
            keepStraight(hitWall);
            return lightValue;
        }

        /**
         * Drives the specified distance back from the
         * light sensor to hit a button
        */
        void checkpoint2Button(float backup) {
            const float backupToLight = 9.0;
            const float turnToAlign = PI / -4.0 + PI / 16;
            const float turnToButton = PI / 2.0;
            const float driveToButton = 10;
            const float buttonTimeout = 5.0;

            keepStraight(backupToLight, BACKWARD);
            turn(turnToAlign);
            keepStraight(backup, BACKWARD);
            turn(turnToButton);
            keepStraight(driveToButton, FORWARD, TARGET_LOW_POWER, buttonTimeout);
        }

        /**
         * Drives from the wall in front of the ticket light
         * to the red button
        */
        void checkpoint2Red() {
            const float distanceToButton = 12.0;
            const float smallBackup = 10.0;
            const float turningAmount = PI / -2;
            const float largeAlign = 15.0;

            checkpoint2Button(distanceToButton);
            keepStraight(smallBackup, BACKWARD);
            turn(turningAmount);
            keepStraight(smallBackup);
            turn(turningAmount * -1);
            keepStraight(largeAlign);
        }

        /**
         * Drives from the wall in front of the ticket light
         * to the blue button
        */
        void checkpoint2Blue() {
            const float distanceToButton = 6.0;
            checkpoint2Button(distanceToButton);
        }

        /**
         * Used to do the encoding stuff for exploration three
        */
        void explorationThreeEncoding() {
            
        }

        void driveToCorrectLeverOld() {

            //RCS.InitializeTouchMenu(ROBBIE);
            // Get correct lever from the RCS
            int correctLever =0; //RCS.GetCorrectLever();
          
            if(correctLever == 0) {
                // Perform actions to flip left lever
            } else if(correctLever == 1) {
                // Perform actions to flip middle lever
            } else if(correctLever == 2) {
                // Perform actions to flip right lever
            }
        }

        void alignCheckpoint3() {
            const float partRight = PI / 4;
            const float fullLeft = PI / -2;
            const float speed = 10.0;
            const float getToAlignment = 10.0;
            const float alignDistance = 30.0;
            const float timeout = 5.0;

            turn(partRight);
            correctedStraight(speed, getToAlignment);
            turn(fullLeft);
            correctedStraight(speed, alignDistance, BACKWARD, timeout);
            correctedStraight(speed, getToAlignment);
            turn(fullLeft * -1);
            correctedStraight(speed, alignDistance, BACKWARD, timeout);
        }

        void driveToLevers() {
            const float speed = 10.0;
            const float toLevers = 10.0;
            const float turnToLevers = PI / -4;
            const float alignToLevers = 11.0;
            const float turnToLever = (PI / -4) - (PI / 16);// - (3*PI / 64);
            const float lastTurn = (PI / 48);

            correctedStraight(speed, toLevers);
            turn(turnToLevers);
            correctedStraight(speed, alignToLevers);
            turn(turnToLever);

            //correctedStraight(speed, alignToLevers);
            turn(turnToLever);
            correctedStraight(3.0, 2.0);

            //turn(lastTurn);
        }

        void unpressLever() {
            correctedStraight(3.0, 4.0, BACKWARD);
            straightenArm();
            correctedStraight(3.0, 2.0);
            raiseLever();
            correctedStraight(4.0, 5.0, BACKWARD);
        }

        void checkpoint4() {
            const float alignToRampTurn1 = PI / 4;
            const float alignToRampTurn2 = alignToRampTurn1 * -1;
            const float alignToRampDistance = 5.0;
            const float velocity = 10.0;

            const float rampToPassport = 32.5;
            const float alignToPassport1 = PI / -3;
            const float alignToPassportDistance = 5.0;
            const float passportTimeout = 2.0;
            const float alignToPassport2 = alignToPassport1 * -1;

            const float forwardDistance = 15.0;

            turn(alignToRampTurn1);
            //correctedStraight(velocity, alignToRampDistance);
            //turn(alignToRampTurn2);

            correctedStraight(velocity*2.5, rampToPassport);

            // CORRECTION BECAUSE IT KEEPS DRIFTING LEFT
            //turn(PI / -128);

            turn(alignToPassport1);
            straightenArm();
            correctedStraight(velocity, alignToPassportDistance, FORWARD, passportTimeout);
            turn(alignToPassport2);

            raiseArm();
            keepStraight(forwardDistance);
            keepStraight(forwardDistance, BACKWARD);
            straightenArm();

            keepStraight(forwardDistance / 2);
            raisePassport();
            keepStraight(forwardDistance);

        }

        void hitStart() {
            readLight();
            correctedStraight(10.0, 5.0, BACKWARD, 2.0);
            Sleep(WAIT_TIME);
        }

        void depositLuggage() {
            const float velocity = 10.0;
            const float depositVelocity = 5.0;
            const float buttonToWallAlignment = 15.0;
            const float turnToWall = PI / -4;

            correctedStraight(velocity, buttonToWallAlignment);
            turn(turnToWall);
            correctedAlign(velocity, BACKWARD);

            const float xToLuggage = 18.0;
            const float turnToLuggage = PI / 2;
            const float yToLuggage = 10.0;
            const float yToLuggageBack = 4.0;

            correctedStraight(velocity, xToLuggage);
            turn(turnToLuggage);
            correctedStraight(depositVelocity, yToLuggage);
            pressLever();
            Sleep(WAIT_TIME);
            correctedStraight(velocity, yToLuggageBack, BACKWARD, TIMEOUT, true);
            raiseArm();
        }

        void alignForFuel() {
            const float velocity = 10.0;
            const float turnFromLuggage = PI / -2;

            turn(turnFromLuggage);
            correctedAlign(velocity, BACKWARD);
        }

        void finishCheckpoint5() {
            const float velocity = 10.0;
            const float alignWithButton = 11.0;
            const float turnToButton = PI / 4;

            correctedStraight(velocity, alignWithButton);
            turn(turnToButton);
            correctedAlign(velocity, BACKWARD);
        }

        void driveToCorrectLever() {
            const float velocity = 10.0;

            const float toFirstLever = 17.75;
            const float betweenLevers = 3.5;

            correctedStraight(velocity, toFirstLever + betweenLevers * getLever());
        }

        void pressCorrectLever() {
            const float velocity = 10.0;
            const float turnToLever = (PI / -2) - (PI / 48);
            const float backStep = 1.0;
            const float stepTime = 0.5;
            const float backClear = 4.0;

            turn(turnToLever);
            pressLever();
            Sleep(stepTime);
            raiseArm();
            Sleep(stepTime);

            /*
            correctedStraight(velocity, backStep * 2, BACKWARD, stepTime * 4);
            straightenArm();
            Sleep(stepTime);
            raiseArm();
            Sleep(stepTime);
            */
            correctedStraight(velocity, backClear, BACKWARD, backClear);

            const float reqSleep = 5.0;
            Sleep(reqSleep);

            straightenArm();
            correctedStraight(velocity, backClear - 1, FORWARD, backClear);
            //correctedStraight(velocity, backStep, BACKWARD, stepTime * 4);
            raiseArm();
            Sleep(stepTime);
            straightenArm();
        }

        void alignToSteepRamp() {
            const float velocity = 10.0;
            const float turnToWall = PI / -2;

            turn(turnToWall);
            raiseArm();
            Sleep(WAIT_TIME);
            correctedAlign(velocity, BACKWARD);

            const float toRampDistance = 3.5;
            const float turnToRamp = PI / -2;

            correctedStraight(velocity, toRampDistance);
            turn(turnToRamp);
        }

        void driveSteepAndAlign() {
            const float velocity = 20.0;
            const float upRamp = 25.0;
            const float turnToWall = PI / 2;

            correctedStraight(velocity, upRamp);
            turn(turnToWall);
            correctedAlign(velocity / 2, BACKWARD);
        }

        void alignToLight() {
            const float velocity = 10.0;
            const float alignDistance = 23.0;
            const float turnToAlign = PI / 4 - (PI / 48);

            correctedStraight(velocity, alignDistance);
            turn(turnToAlign);
            correctedAlign(velocity, BACKWARD);

            const float toLight = 4.0;
            correctedStraight(velocity, toLight);
        }

        void alignToButton() {
            const float velocity = 10.0;
            const float forwardDistance = 7.0;
            const float turnAmount = PI / -4;

            correctedStraight(velocity, forwardDistance);
            turn(turnAmount);
        }

        void toBlueButton() {
            const float velocity = 10.0;
            const float distance = 0.0;
            const float timeout = 2.0;

            correctedStraight(velocity, distance, FORWARD, timeout);
        }

        void toRedButton() {
            const float velocity = 10.0;
            const float distance = 3.5;
            const float timeout = 2.0;

            correctedStraight(velocity, distance, FORWARD, timeout);
        }

        void pressButton() {
            const float velocity = 10.0;
            const float turnAmount = PI / -2;
            const float toWall = 15.0;
            const float timeout = 5.0;

            turn(turnAmount);
            correctedStraight(velocity, toWall, FORWARD, timeout);
            correctedStraight(velocity, toWall, BACKWARD);
            turn(turnAmount);
        }

        void alignPassport() {
            const float velocity = 10.0;

            correctedAlign(velocity, BACKWARD);
        }

        void flipPassport() {
            const float velocity = 5.0;
            const float turnToLever = (PI / 2) - (PI / 64);
            const float wallToLever = 10.5;
            const float leverDistance = 8.0;
            const float timeout = 5.0;
            const float extraStep = 0.75;

            correctedStraight(velocity, wallToLever);
            straightenArm();
            turn(turnToLever);
            correctedStraight(velocity, extraStep, FORWARD, TIMEOUT, true);
            raisePassport();
            correctedStraight(velocity, leverDistance, FORWARD, timeout, true);
            correctedStraight(velocity, leverDistance + extraStep, BACKWARD);
            raiseArm();
            turn(turnToLever * -1);
            correctedAlign(velocity, BACKWARD);
        }

        void finishCourse() {
            const float velocity = 10.0;
            const float getOffWall = 4.0;
            const float turnToWalls = PI / -2;
            const float downRamp = 25.0;
            const float alignToFinal = 15.0;
            const float turnToButton = PI / 4;

            correctedStraight(velocity, getOffWall);
            turn(turnToWalls);
            correctedStraight(velocity, downRamp);
            turn(turnToWalls * -1);
            correctedAlign(velocity, BACKWARD);
            correctedStraight(velocity, alignToFinal);
            turn(turnToButton);
            correctedAlign(velocity, BACKWARD);
        }
};

int main() {
    Robot robot;

    robot.hitStart();
    robot.depositLuggage();
    robot.alignForFuel();
    robot.driveToCorrectLever();
    robot.pressCorrectLever();
    robot.alignToSteepRamp();
    robot.driveSteepAndAlign();
    robot.alignToLight();

    Light recordedLight = robot.readLight(TIMEOUT);
    robot.alignToButton();
    if(recordedLight == BLUELIGHT) {
        robot.toBlueButton();
    } else {
        robot.toRedButton();
    }
    robot.pressButton();
    robot.alignPassport();
    robot.flipPassport();
    robot.finishCourse();
}