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

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.BitSet;
import java.util.Date;
import java.util.Locale;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import static java.lang.Thread.sleep;

@Autonomous(name="OldLong", group ="11697")
//BLUE Left has "public class BlueLeft extends LinearOpMode
public class OldLong extends LinearOpMode {
    Hardware robot = new Hardware();

    // Mortors Settings [DO NOT CHANGE]
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder 1440, Andymark = 1120
    static final double DRIVE_GEAR_REDUCTION = 1.3;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
            / (WHEEL_DIAMETER_INCHES * 3.1415);

    // Used by system functions
    private ElapsedTime moveTimer = new ElapsedTime();
    private Date today = new Date();
    private DateFormat myDateFormat = new SimpleDateFormat("yy/MM/dd HH:mm:ss");
    private String PREVIOUS_MSG = "";
    private String GoldPosition = "";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AWc1Bu//////AAABmY4cx70c2U11o94FzSDQGhpHGMy5d5FLzx/3zg+KOI61qY3ZmHIIlWvRGNv201o/XXdfMjjUgsO1nO1nbr40RrQYvttkFB4vMvS35sqvrvJk42LnYkcwKH2hU+fN8+oqm4lOU1EOtfIYo2We6/3xwDJdavPumuEUruK7ubhUjusYMkArzuqjomzGLDCjASXesOGTxhVa0J5Mm5HE9IWsFzqafI0PXiKBKs7xZ2RHXcnZVtTWJETqcN6Kk5vIcREjY7o74cfMzdebTIgeFh8FimMU8MxCB077/pyA7DxsP9TvY1hoqvGbaQs5iAzo81iJ45a0bnCvMamjed0SlkMJqnYgA3zuk7qa9w6TcqvvTEBT";
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

/* EDIT THIS PART LATER AS NEEDED
   // While loop variables
   static final double COLUMN_RIGHT = 8.7;
   static final double COLUMN_CENTER = 5.5;
   static final double COLUMN_LEFT = 2.1;
   private String TARGET_COLUMN = "EMPTY";
   private int MID_STOP = 9;
   private int iTurn = 1;
   private boolean findTarget = false;
   private double FINAL_STOP = -12.0;
   private double FINAL_MOVE = 0.0;
   private int LOOP_COUNTER = 0;
*/

    //Random values
    private boolean isDetected = false;

    //Variables for block detection
    private int goldMineralX = -1;
    private int silverMineralX = -1;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.boostedMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        }  else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        if (tfod != null) {
            tfod.activate();
        }

        sleep(2000);
        //  robot.waitForTick(2000);
        // Add more init statements later based on components
        // Make sure to add "waitForTick" statement breaks between each component


        //    telemetry.addData("Status: ", "Initialized v4.0b - " + myDateFormat.format(today));
        //    telemetry.update();


        //Scans blocks to determine GOLD location
        if(tfod != null) {
            //List for tensorflow recognition
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());

                if (updatedRecognitions.size() == 2) {

                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getTop();
                        } else {
                            silverMineralX = (int) recognition.getTop();
                        }
                    }

                    //Uses y-coordinates to determine block location
                    if (goldMineralX == -1) {
                        GoldPosition = "LEFT";
                    } else {
                        if (goldMineralX > silverMineralX) {
                            GoldPosition = "CENTER";
                        } else {
                            GoldPosition = "RIGHT";
                        }
                    }

                    telemetry.addData("Gold Mineral Position", GoldPosition);
                    //   telemetry.addData("goldMineralX", "VAL=" + goldMineralX);
                    //   telemetry.addData("silverMineralX", "VAL=" + silverMineralX);
                    telemetry.update();

                }
            }


        }

        waitForStart();

        goldMineralX = -1;
        silverMineralX = -1;

        sleep(250);

        if(tfod != null) {
            //List for tensorflow recognition
            List<Recognition> updatedRecognitions1 = tfod.getUpdatedRecognitions();

            if (updatedRecognitions1 != null) {
                telemetry.addData("# Object Detected", updatedRecognitions1.size());

                if (updatedRecognitions1.size() == 2) {

                    for (Recognition recognition : updatedRecognitions1) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getTop();
                        } else {
                            silverMineralX = (int) recognition.getTop();
                        }
                    }

                    //Uses y-coordinates to determine block location
                    if (goldMineralX == -1) {
                        GoldPosition = "LEFT";
                    } else {
                        if (goldMineralX > silverMineralX) {
                            GoldPosition = "CENTER";
                        } else {
                            GoldPosition = "RIGHT";
                        }
                    }

                    telemetry.addData("Gold Mineral Position", GoldPosition);
                    //   telemetry.addData("goldMineralX", "VAL=" + goldMineralX);
                    //   telemetry.addData("silverMineralX", "VAL=" + silverMineralX);
                    telemetry.update();

                }
            }


        }


        while (opModeIsActive()) {


            telemetry.addData("Status: ","Started");
            telemetry.update();

            robot.boostedMotor.setTargetPosition(9430);
            robot.boostedMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.boostedMotor.setPower(1.0);

            sleep(250);

            while (robot.boostedMotor.isBusy() ) {
                telemetry.addData("Status: ", "Position Reading: " + robot.boostedMotor.getCurrentPosition());
                telemetry.update();
            }

            //Drive to free from hook
            driveByEncoder(0.5, 0.0,  0.5);

            driveByEncoder(0.5, 1.3, 1.3);

            while (robot.boostedMotor.isBusy() ) {
                telemetry.addData("Status: ", "Position Reading: " + robot.boostedMotor.getCurrentPosition());
                telemetry.update();
            }



            //Adjust
//               driveByEncoder(0.5,0.5,0.5);

            //Shift left to be up against row of blocks (ORIGINAL 7.8)
            driveByEncoder(0.5, 8.0, 0.0);

            driveByEncoder(0.5, 7.6, -7.6);

            // driveByEncoder(0.5, 14.0, -14.0);



            switch(GoldPosition) {
                case "LEFT":


                    driveByEncoder(0.5, 8.2, 0);
                    driveByEncoder(0.5, 7.5,  7.5);
                    driveByEncoder(0.5, 0.0, 2.5);
                    driveByEncoder(0.5, 1.3,1.3);

                    break;


                case "CENTER":
                    //Knock off the center block
                    driveByEncoder(0.5, 1.0,  0.0);
                    driveByEncoder(0.5, 7.0,  7.0);
                    driveByEncoder(0.5, 2.6,-2.6);

                    break;


                case "RIGHT":
                    driveByEncoder(0.5, 0,6.2);
                    driveByEncoder(0.5, -1.3,1.3);
                    driveByEncoder(0.5, 7.5,  7.5);

                    break;

            }


            sleep(5000);
            break;


        }

    }



    /*
     **************************************************
     **************USER DEFINED FUNCTIONS:*************
     **************************************************
     */

    private void driveByEncoder (double speed, double leftInches, double rightInches) throws InterruptedException {

        /**********************************************************
         driveByEncoder(0.3, 10.0, 10.0);            // Forward
         driveByEncoder(0.3, -10.0, -10.0);          // Backward
         driveByEncoder(0.3, 0, 10.0);               // Shift Right
         driveByEncoder(0.3, 10.0, 0);               // Shift Left
         driveByEncoder(0.3, 22.0, -22.0);           // Turn Right
         driveByEncoder(0.3, -22.0, 22.0);           // Turn Left
         ***********************************************************/

        String robotAction = "";
        int newLeftTarget;
        int newRightTarget;

        if (leftInches < 0 && rightInches < 0) {
            robotAction = "BACKWARD";
        } else if (leftInches > 0 && rightInches > 0) {
            robotAction = "FORWARD";
        } else if (leftInches > 0 && rightInches == 0) {
            robotAction = "SHIFT_LEFT";
        } else if (leftInches == 0 && rightInches > 0) {
            robotAction = "SHIFT_RIGHT";
        } else if (leftInches < 0 && rightInches > 0) {
            robotAction = "TURN_LEFT";
        } else if (leftInches > 0 && rightInches < 0) {
            robotAction = "TURN_RIGHT";
        } else {
            return;
        }

        // Remember current motors direction, will reset in the end
        DcMotor.Direction dirFL = robot.frontLeftMotor.getDirection();
        DcMotor.Direction dirFR = robot.frontRightMotor.getDirection();
        DcMotor.Direction dirRL = robot.rearLeftMotor.getDirection();
        DcMotor.Direction dirRR = robot.rearRightMotor.getDirection();
        DcMotor.RunMode runModeFL = robot.frontLeftMotor.getMode();
        DcMotor.RunMode runModeFR = robot.frontRightMotor.getMode();
        DcMotor.RunMode runModeRL = robot.rearLeftMotor.getMode();
        DcMotor.RunMode runModeRR = robot.rearRightMotor.getMode();

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // power is removed from the motor, set the current encoder position to zero
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // All mortors will move forward
        robot.frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        robot.frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        robot.rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        robot.rearRightMotor.setDirection(DcMotor.Direction.FORWARD);
        //robot.frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        //robot.frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        //robot.rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        //robot.rearRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Determine new target position, and pass to motor controller
        newLeftTarget = (int)(leftInches * COUNTS_PER_INCH);
        newRightTarget = (int)(rightInches * COUNTS_PER_INCH);
        //logMessage("curFL,curFR",  robot.frontLeftMotor.getCurrentPosition() +", "+ robot.frontRightMotor.getCurrentPosition());

        switch(robotAction) {

            case "FORWARD":
                //logMessage("Moving Robot", "FORWARD");
                break;

            case "BACKWARD": // mortor direction aame as FORWAED, because encoder will be "-"
                //logMessage("Moving Robot", "BACKWARD");
                break;

            case "SHIFT_LEFT":
                //logMessage("Moving Robot", "SHIFT_LEFT");
                robot.frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);   //-
                robot.frontRightMotor.setDirection(DcMotor.Direction.FORWARD);  //+
                robot.rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);    //+
                robot.rearRightMotor.setDirection(DcMotor.Direction.REVERSE);   //-
                //robot.frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);   //-
                //robot.frontRightMotor.setDirection(DcMotor.Direction.REVERSE);  //+
                //robot.rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);    //+
                //robot.rearRightMotor.setDirection(DcMotor.Direction.FORWARD);   //-
                newRightTarget = newLeftTarget;
                break;

            case "SHIFT_RIGHT":
                //logMessage("Moving Robot", "SHIFT_RIGHT");
                robot.frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);   //+
                robot.frontRightMotor.setDirection(DcMotor.Direction.REVERSE);  //-
                robot.rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);    //-
                robot.rearRightMotor.setDirection(DcMotor.Direction.FORWARD);   //+
                //robot.frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);   //+
                //robot.frontRightMotor.setDirection(DcMotor.Direction.FORWARD);  //-
                //robot.rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);    //-
                //robot.rearRightMotor.setDirection(DcMotor.Direction.REVERSE);   //+
                newLeftTarget = newRightTarget;
                break;

            case "TURN_LEFT":
                //logMessage("Moving Robot", "TURN_LEFT");
                break;

            case "TURN_RIGHT":
                //logMessage("Moving Robot", "TURN_RIGHT");
                break;

        }

        robot.frontLeftMotor.setTargetPosition(newLeftTarget);
        robot.frontRightMotor.setTargetPosition(newRightTarget);
        robot.rearLeftMotor.setTargetPosition(newLeftTarget);
        robot.rearRightMotor.setTargetPosition(newRightTarget);
        //logMessage("newLeftTarget,newRightTarget",  newLeftTarget +", "+ newRightTarget);

        // Turn On RUN_TO_POSITION
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the motor speed and start motion.
        robot.frontLeftMotor.setPower(Math.abs(speed));
        robot.frontRightMotor.setPower(Math.abs(speed));
        robot.rearLeftMotor.setPower(Math.abs(speed));
        robot.rearRightMotor.setPower(Math.abs(speed));


        // keep looping while we are still active, and there is time left, and both motors are running.
        while (robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy() &&
                robot.rearLeftMotor.isBusy() && robot.rearRightMotor.isBusy()) {

            /*
            logMessage("Path1",  newLeftTarget +", "+ newRightTarget);
            logMessage("Path2",
                    robot.frontLeftMotor.getCurrentPosition() + ", " +
                    robot.frontRightMotor.getCurrentPosition() + ", " +
                            robot.rearLeftMotor.getCurrentPosition() + ", " +
                            robot.rearRightMotor.getCurrentPosition());
            */
        }


        //logMessage("FL,FR,RL,RR",
        //        robot.frontLeftMotor.getCurrentPosition() + ", " +
        //                robot.frontRightMotor.getCurrentPosition() + ", " +
        //                robot.rearLeftMotor.getCurrentPosition() + ", " +
        //                robot.rearRightMotor.getCurrentPosition());

        // Stop all motion;
        stopRobot();

        // Turn off RUN_TO_POSITION
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Reset back motors direction
        robot.frontLeftMotor.setDirection(dirFL);
        robot.frontRightMotor.setDirection(dirFR);
        robot.rearLeftMotor.setDirection(dirRL);
        robot.rearRightMotor.setDirection(dirRR);
        robot.frontLeftMotor.setMode(runModeFL);
        robot.frontRightMotor.setMode(runModeFR);
        robot.rearLeftMotor.setMode(runModeRL);
        robot.rearRightMotor.setMode(runModeRR);

    }

    private void stopRobot() {
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.rearLeftMotor.setPower(0);
        robot.rearRightMotor.setPower(0);
    }

    //Movement path for rightmost two blocks
    private void finishHim() throws InterruptedException{

        driveByEncoder(0.5, 11.5, -11.5);

        //Move up against the wall
        driveByEncoder(0.3, 0, 5);
        //Move away from wall
        driveByEncoder(0.3,1.5,0);


        //Drive to drop off team marker
        driveByEncoder(0.5, 18.0, 18.0);




        robot.markyBoi.setPosition(0.70);
        sleep(500);
        robot.markyBoi.setPosition(0.08);

        //Move up against the wall
        driveByEncoder(0.2, 0, 3.0);
        //Move away from wall
        driveByEncoder(0.3,1.5,0);
        driveByEncoder(0.3,-0.4,0.4);
        driveByEncoder(0.5, -12.0, -12.0);

        //Drive to crater
        driveByEncoder(0.2, 0, 3.0);
        driveByEncoder(0.3,1.5,0);
        driveByEncoder(0.3,-0.4,0.4);
        driveByEncoder(0.5, -13.0, -13.0);

        /*

        //*Insert code for team marker*

        //Turn around
        driveByEncoder(0.5, 15.25, -15.25);

        //Adjust
        driveByEncoder(0.5, -5.0, -5.0);

        //Drive to crater
        driveByEncoder(0.5, -24.0, -24.0);

        //Extend arm
        moveTimer.reset();
        while (moveTimer.seconds() < 2) {
            robot.longMotor.setPower(0.4);
            //idle();
        }

        ****/



/*
        //Drive to park in crater
        driveByEncoder(0.5, 24.0, 24.0);

        //Move away from wall
        driveByEncoder(0.3, 0,9.0);

        //Turn around
        driveByEncoder(0.3, 14, -14);
*/
    }

    //Movement path for the robot after the last block
    private void lastOne() throws InterruptedException {
        //Turn around for team marker
        driveByEncoder(0.3, 3.9, -3.9);

        //Move to drop off team marker
        driveByEncoder(0.3, 7.0,7.0);
        //Turn for team marker
        driveByEncoder(0.3, -8.5,8.5);

        sleep(250);

        //Back against wall
        driveByEncoder(0.5, 7.5, 0);

        //Adjust
        driveByEncoder(0.5, 0, 2.0);

        //*Insert code for team marker*

        //Turn around
        driveByEncoder(0.5, 15.5, -15.5);

        //Adjust
        driveByEncoder(0.5, -5.0, -5.0);

        //Drive to crater
        driveByEncoder(0.5, -24.0, -24.0);

        telemetry.addData("Status: ", "Got here");
        telemetry.update();

        //Extend arm
        moveTimer.reset();
        while (moveTimer.seconds() < 2) {
            robot.longMotor.setPower(0.4);
            //idle();
        }


        //robot.longMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //robot.longMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.longMotor.setTargetPosition(1000);
        //robot.longMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.longMotor.setPower(Math.abs(.4));

        //  robot.longMotor.setTargetPosition();
        //  robot.longMotor.

    }


    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

}

