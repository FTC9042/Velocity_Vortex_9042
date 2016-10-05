package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is the class that defines all hardware related allocations.
<<<<<<< HEAD
 *
 * Change this class whenever the robot changes to ensure compatibility of code
 *
 */
public class RobotHW {
    /* Public OpMode members. */
    public DcMotor leftMotor1 = null;
    public DcMotor rightMotor1 = null;
    public DcMotor leftMotor2 = null;
    public DcMotor rightMotor2 = null;

    /* Local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public RobotHW() {
    }

     /* Initialize standard Hardware interfaces
     *
     * @param ahwMap The hardwaremap included in the OpMode.
     */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        leftMotor1 = hwMap.dcMotor.get("l1");
        rightMotor1 = hwMap.dcMotor.get("r1");
        leftMotor2 = hwMap.dcMotor.get("l2");
        rightMotor2 = hwMap.dcMotor.get("r2");
        leftMotor1.setDirection(DcMotor.Direction.REVERSE);
        leftMotor2.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        leftMotor1.setPower(0);
        rightMotor1.setPower(0);
        leftMotor2.setPower(0);
        rightMotor2.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /***
<<<<<<< HEAD
     *
=======
>>>>>>> 5258817aa18d03b5d6fdf05944925c076c567a9a
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
<<<<<<< HEAD
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs)  throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    /**
     * Sets all motors to RUN_TO_POSITION mode
     */
    public void setToPositionMode() {
        leftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Resets all motor positions to 0
     */
    public void resetEncoder() {
        leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setToPositionMode();
    }

    /**
     * Sets all motors to RUN_WITHOUT_ENCODER mode
     */
    public void setToPowerMode() {
        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Sets all motors to RUN_USING_ENCODER mode
     */
    public void setToEncoderMode() {
        leftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Sets all motors to this runMode
     *
     * @param runMode runMode to be set
     */
    public void setMotorMode(DcMotor.RunMode runMode) {
        leftMotor1.setMode(runMode);
        leftMotor2.setMode(runMode);
        rightMotor1.setMode(runMode);
        rightMotor2.setMode(runMode);
    }

    /**
     * Drives the motors with the specified power levels
     * <p>
     * This method will not execute if the motors are in RUN_TO_POSITION mode
     *
     * @param left  Power of the left motors
     * @param right Power of the right motorss
     */
    public void drive(double left, double right) {
        if (leftMotor1.getMode() == DcMotor.RunMode.RUN_TO_POSITION)
            return; //Stops executing

        leftMotor1.setPower(left);
        leftMotor2.setPower(left);

        rightMotor1.setPower(right);
        rightMotor2.setPower(right);
    }

    /**
     * Sets target positions for the motors
     *
     * @param left  Absolute position in ticks of the left motor
     * @param right Absolute position in ticks of the right motor
     */
    public void setTarget(int left, int right) {
        if (leftMotor1.getMode() != DcMotor.RunMode.RUN_TO_POSITION)
            return;

        leftMotor1.setTargetPosition(left);
        leftMotor2.setTargetPosition(left);

        rightMotor1.setTargetPosition(right);
        rightMotor2.setTargetPosition(right);
    }

    /**
     * Sets the maximum speed of the motors in RUN_TO_POSITION mode
     *
     * @param speed Max speed in ticks per second
     */
    public void setMaxSpeed(int speed) {
        leftMotor1.setMaxSpeed(speed);
        leftMotor2.setMaxSpeed(speed);

        rightMotor1.setMaxSpeed(speed);
        rightMotor2.setMaxSpeed(speed);
    }

    /**
     * Sets the zeroPowerBehavior of the motors. Either coast or break
     * @param zeroPowerBehavior The behavior of motors with 0 as speed.
     */
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior){
        leftMotor1.setZeroPowerBehavior(zeroPowerBehavior);
        leftMotor2.setZeroPowerBehavior(zeroPowerBehavior);

        rightMotor1.setZeroPowerBehavior(zeroPowerBehavior);
        rightMotor2.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public DcMotor getLeftMotor1() {
        return leftMotor1;
    }

    public DcMotor getRightMotor1() {
        return rightMotor1;
    }

    public DcMotor getLeftMotor2() {
        return leftMotor2;
    }

    public DcMotor getRightMotor2() {
        return rightMotor2;
    }
}
