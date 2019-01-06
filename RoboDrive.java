/*
https://github.com/Rambotics/FTC-2016-2017-v2.4-pc/tree/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode
https://github.com/pmtischler/ftc_app/tree/master/SharedCode/src/main/java/com/github/pmtischler

Code Folding Expand/Collapse ALL => Control || Shift || +/-
Last Edit Location => Control + Shift + BackSpace
Add Bookmark => Control + F11 + number
Find Bookmark => Control + number
Show Bookmarks => Shift + F11
Jump to Declaration => Control + B


*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;

//===========================================
// ***** Section 1          *****
// ***** Change OpMode Name *****
//===========================================

@TeleOp(name="RoboDrive_MS", group="11697")
// @Disabled

public class RoboDrive extends LinearOpMode {

//===========================================
// ***** Section 2                *****
// ***** Declare global variables *****
//===========================================

    Hardware robot = new Hardware();

    // Mortors Settings [DO NOT CHANGE]
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder 1440, Andymark = 1120
    static final double     DRIVE_GEAR_REDUCTION    = 1.3 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH          = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    float                   SPEED_RATE           = 0.8f;       // Driving speed rate
    double                  curPosition;

    /* Declare Global Variables. */
    private Date today = new Date();
    private DateFormat myDateFormat = new SimpleDateFormat("yy/MM/dd HH:mm:ss");

    /* Insert more stuff as needed
    *  Look at https://tinyurl.com/yb2xco82 for examples */


/***** Main Code *****/

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        robot.frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        robot.frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        robot.rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        robot.rearRightMotor.setDirection(DcMotor.Direction.REVERSE);
        /* Insert initialization code */

        logMessage("Status", "Initialized v4.0a - " +  myDateFormat.format(today));
//
//        telemetry.addData("Status", "Position Reading: " + robot.stabbyBoi.getPosition());
//        telemetry.update();
//
//        robot.stabbyBoi.setPosition(0.8
//        );

        waitForStart(); // Wait for the game to start (driver presses PLAY)


        String                     PREVIOUS_MSG        = "";        // MesLog
        float lxValue = 0, lyValue = 0, rxValue = 0, ryValue = 0;

        while (opModeIsActive()) {

            // Drive the robot
            lxValue = getJoystickValue(gamepad1.left_stick_x);
            lyValue = getJoystickValue(gamepad1.left_stick_y);
            rxValue = getJoystickValue(gamepad1.right_stick_x);
            ryValue = getJoystickValue(gamepad1.right_stick_y);
            driveByJoystick(lxValue, lyValue, rxValue);

            /********************************************
             *************GAMEPAD 1**********************
             ********************************************/

            //Move Out
            if(gamepad1.a){   // extend the long arm
                robot.longMotor.setPower(1.0);


                telemetry.addData("Status", "Position Reading: " + robot.longMotor.getCurrentPosition());
                telemetry.update();
            }
            else if (gamepad1.b || (gamepad1.right_trigger > 0)) {  // pull back the long arm
                if(robot.longMotor.getCurrentPosition() > 420) {
                    robot.longMotor.setPower(-1.0);
                }

                if(robot.longMotor.getCurrentPosition() < 420) {
                    robot.longMotor.setPower(-0.9);
                }
                telemetry.addData("Status", "Position Reading: " + robot.longMotor.getCurrentPosition());
                telemetry.update();
            }
            else { 
            	robot.longMotor.setPower(0.0);
            }

            //Intake
            if(gamepad1.left_bumper) {
                robot.leftIn.setPosition(1.0);
                robot.rightIn.setPosition(-1.0);

                telemetry.addData("Status: ", "Left Reading: " + robot.leftIn.getPosition());
                telemetry.addData("Status: ", "Right Reading: " + robot.rightIn.getPosition());
                telemetry.update();
            }

            //Exhaust
            if(gamepad1.right_bumper) {
                robot.leftIn.setPosition(-1.0);
                robot.rightIn.setPosition(1.0);

                telemetry.addData("Status: ", "Left Reading: " + robot.leftIn.getPosition());
                telemetry.addData("Status: ", "Right Reading: " + robot.rightIn.getPosition());
                telemetry.update();
            }

            //Stop servos


/*   CONTROL WITH MOTORS (USE LATER)
            //Move Arm Up
            while(gamepad1.x) {
                robot.vertMotor.setPower(0.5);

                telemetry.addData("Status", "Position Reading: " + robot.vertMotor.getCurrentPosition());
                telemetry.update();
            }

            robot.vertMotor.setPower(0);

            //Move Arm Down
            while(gamepad1.y) {
                robot.vertMotor.setPower(-0.3);

                telemetry.addData("Status", "Position Reading: " + robot.vertMotor.getCurrentPosition());
                telemetry.update();
            }

            robot.vertMotor.setPower(0);
*/

/**********************************************************
// Modified the gamepad1.x and gamepad1.y as for loop below

            while(gamepad1.x) {
                robot.stabbyBoi.setPower(-0.4);

            }

            robot.stabbyBoi.setPower(0.0);

            while(gamepad1.y) {
                robot.stabbyBoi.setPower(0.4);

            }

            robot.stabbyBoi.setPower(0.0);
            
*************************************************************/

            if(gamepad1.x) {
                robot.stabbyBoi.setPower(-0.4);
            }
            else if(gamepad1.y) {
                robot.stabbyBoi.setPower(0.4);
            }
            else {
				robot.stabbyBoi.setPower(0.0);
            }

            //robot.stabbyBoi.setPosition(0.55);



            /********************************************
             *************GAMEPAD 2**********************
             ********************************************/

            //Control Up/Down Lift
            if(gamepad1.dpad_up){
                //robot.boostedMotor.setTargetPosition(2000);
                //robot.boostedMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.boostedMotor.setPower(0.8);

                telemetry.addData("Status", "Position Reading: " + robot.boostedMotor.getCurrentPosition());
                telemetry.update();
            }

            if (gamepad1.dpad_down) {
                //robot.boostedMotor.setTargetPosition(-2000);
                //robot.boostedMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.boostedMotor.setPower(-0.8);

                telemetry.addData("Status", "Position Reading: " + robot.boostedMotor.getCurrentPosition());
                telemetry.update();
            }

            if (gamepad1.dpad_left || gamepad1.dpad_right) {
                //robot.boostedMotor.setTargetPosition(-2000);
                //robot.boostedMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.boostedMotor.setPower(0.0);

                telemetry.addData("Status", "Position Reading: " + robot.boostedMotor.getCurrentPosition());
                telemetry.update();
            }

            if (gamepad1.left_trigger > 0) {

               // >>> 01/05/2019 Check the current long arm position, if it did not pull back, pull back it.
               // During the competition, you don't need to pull back the long arm all the way, It will pull back by this code.
                while(robot.longMotor.getCurrentPosition() > 10) {
                    robot.longMotor.setPower(-0.9);
                }
				robot.longMotor.setPower(0.0);
               // <<< 01/05/2019

                robot.pushyBoi.setPosition(1);
                sleep(500);
                robot.pushyBoi.setPosition(0);
                sleep(500);

                robot.vertMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.vertMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.vertMotor.setTargetPosition(-1980);  // Changed from -2000 to -1980. 12/28
                robot.vertMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.vertMotor.setPower(1);
                while (robot.vertMotor.isBusy() ) {
                    telemetry.addData("Path1",  "Running to " + robot.vertMotor.getCurrentPosition());
                    telemetry.update();
                }
                robot.dumpyBoi.setPosition(0.14);  // Changed from 0.10 to 0.14. 1/1/19
                sleep(1500);
				
                robot.dumpyBoi.setPosition(0.87);
                sleep(500);

                robot.vertMotor.setTargetPosition(0);
                robot.vertMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.vertMotor.setPower(1);
            }


            /*********
             if (gamepad1.left_trigger > 0 || gamepad1.right_trigger > 0) {
                //robot.boostedMotor.setTargetPosition(-2000);
                //robot.boostedMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                 robot.leftIn.setPosition(0.55);
                 robot.rightIn.setPosition(0.5);

                telemetry.addData("Status", "Position Reading: " + robot.boostedMotor.getCurrentPosition());
                telemetry.update();
            }
            **********/







        idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop

        }

        logMessage("Done", "Stopped!");
    }



//===========================================
// ***** Section 5              *****
// ***** User Defined Functions *****
//===========================================

    private void driveByJoystick(double lxValue, double lyValue, double rxValue) {
        double vD = Math.sqrt(Math.pow(lxValue, 2) + Math.pow(lyValue, 2));
        //double thetaD = Math.atan2(lyValue, lxValue);
        double thetaD = Math.atan2(lxValue, -lyValue);
        double vTheta = rxValue;

        // Convert desired motion to wheel power, with power clamping.
        Mecanum.Wheels wheels = Mecanum.motionToWheels(vD, thetaD, vTheta);
        robot.frontLeftMotor.setPower(wheels.frontLeft);
        robot.frontRightMotor.setPower(wheels.frontRight);
        robot.rearLeftMotor.setPower(wheels.backLeft);
        robot.rearRightMotor.setPower(wheels.backRight);

        //telemetry.addData("Status", "==> " + vD +", "+thetaD+", "+vTheta);
        //telemetry.update();
    }

    private void stopRobot() {
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.rearLeftMotor.setPower(0);
        robot.rearRightMotor.setPower(0);
    }

    private void logMessage (String myDescription, String myMessage) throws InterruptedException {

            telemetry.addData(myDescription, myMessage);
            telemetry.update();
            RobotLog.d("11697CW - " + myDescription + " : " + myMessage);


    }

    private float getJoystickValue (float rawValue) {
        //clip the power values so that it only goes from -1 to 1
        rawValue = Range.clip(rawValue, -1, 1);

        if (Math.abs(rawValue) < 0.1) {
            return 0;
        } else {
            return rawValue * SPEED_RATE;
        }
    }

//===========================================
// ***** Section 6              *****
// ***** Backup Un-used Code    *****
//===========================================

/********************************************

            if (gamepad1.dpad_up) {
                V_POSITION += INCREMENT ;
                if (V_POSITION >= robot.VArmHigh ) {
                    V_POSITION = robot.VArmHigh;
                }
                robot.vArm.setPosition(V_POSITION);
                telemetry.addData("UP >> Servo Position", "%5.2f", V_POSITION);
                telemetry.update();
                //sleep(CYCLE_MS);
                robot.waitForTick(CYCLE_MS);
            }
            if (gamepad1.dpad_down) {
                V_POSITION -= INCREMENT ;
                if (V_POSITION <= robot.VArmLow ) {
                    V_POSITION = robot.VArmLow;
                }
                robot.vArm.setPosition(V_POSITION);
                telemetry.addData("DOWN >> Servo Position", "%5.2f", V_POSITION);
                telemetry.update();
                //sleep(CYCLE_MS);
                robot.waitForTick(CYCLE_MS);
            }

********************************************/





}

