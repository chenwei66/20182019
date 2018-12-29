package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;


/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 ***  Hardware Motor Controllers Settings ***
 RL Controller (AL025T7Z)
 0 - NeveRest 40 Gearmotor (FLMotor)
 RR Controller (AL025T80)
 1 - NeveRest 40 Gearmotor (FRMotor)
 FL Controller (A7008KTV)
 2 - NeveRest 40 Gearmotor (LRMotor)
 FR Controller (A7008KBB)
 3 - NeveRest 40 Gearmotor (RRMotor)
 */

public class Hardware
{
    //Define values for servos
    public static final double stabbyIniOpen = 0.00;
    public static final double stabbyopen = 0.00;
    public static final double stabbyClose = 0.00;

    /* Public OpMode members. */
    public DcMotor frontLeftMotor      = null;     // H2 channel 0     FLMotor
    public DcMotor frontRightMotor     = null;     // H2 channel 1     FRMotor
    public DcMotor rearLeftMotor       = null;     // H2 channel 2     RLMotor
    public DcMotor rearRightMotor      = null;     // H2 channel 3     RRMotor
    public DcMotor boostedMotor        = null;     // H3 channel 0
    public DcMotor longMotor           = null;     // H3 channel 2     longArm
    public DcMotor vertMotor           = null;     // H3 channel 1     vertArm
    public DcMotor stabbyBoi           = null;     // H3 Channel 3     stabbyBoi
    //stabbyBoi limits are -729 (down) to 0 (up)

    /* Public Servos */
    public Servo markyBoi              = null;     // H3 sero 1        markyBoi
    public Servo leftIn                = null;     // H3 servo 2       leftIn
    public Servo rightIn               = null;     // H3 servo 3       rightIn
    public Servo dumpyBoi              = null;     // H3 servo 3       rightIn
    public Servo pushyBoi              = null;

    /* local OpMode members. */
    HardwareMap hwMap             =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Hardware(){

    }


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hwMap = ahwMap;


        // Define and Initialize Motors
        frontLeftMotor  = hwMap.dcMotor.get("LF_Motor");
        frontRightMotor = hwMap.dcMotor.get("RF_Motor");
        rearLeftMotor   = hwMap.dcMotor.get("LR_Motor");
        rearRightMotor  = hwMap.dcMotor.get("RR_Motor");
        boostedMotor    = hwMap.dcMotor.get("Boosted");
        longMotor       = hwMap.dcMotor.get("longArm");
        vertMotor       = hwMap.dcMotor.get("vertArm");
        stabbyBoi       = hwMap.dcMotor.get("stabbyBoi");

        // Define and initialize servos
        markyBoi        = hwMap.get(Servo.class, "markyBoi");
        pushyBoi       = hwMap.get(Servo.class, "pushyBoi");
        dumpyBoi        = hwMap.get(Servo.class, "dumpyBoi");
        leftIn          = hwMap.get(Servo.class, "leftIn");
        rightIn         = hwMap.get(Servo.class, "rightIn");
/*
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotor.Direction.REVERSE);
*/
        //Initialize servo positions (DEFINE LATER)
        markyBoi.setPosition(0.08);
        pushyBoi.setPosition(0.0);
        dumpyBoi.setPosition(0.87);

       // leftIn.setPosition(0.0);
       // rightIn.setPosition(0.0);


Original:

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotor.Direction.FORWARD);

/*
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotor.Direction.REVERSE);
*/
        boostedMotor.setDirection(DcMotor.Direction.FORWARD);
        longMotor.setDirection(DcMotor.Direction.FORWARD);
        vertMotor.setDirection(DcMotor.Direction.FORWARD);
/*
// Fixing inverted controls:

        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotor.Direction.REVERSE);
  */
// End of fix

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        boostedMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        longMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vertMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        boostedMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        longMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vertMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);
        boostedMotor.setPower(0);
        longMotor.setPower(0);
        vertMotor.setPower(0);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    public void setArmStartPosition() {

    }


}

