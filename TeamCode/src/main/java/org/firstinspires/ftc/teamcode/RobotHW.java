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
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;

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

        frontLeft = hwMap.dcMotor.get("l1");
        frontRight = hwMap.dcMotor.get("r1");
        backLeft = hwMap.dcMotor.get("l2");
        backRight = hwMap.dcMotor.get("r2");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /***
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
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Resets all motor positions to 0
     */
    public void resetEncoder() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setToPositionMode();
    }

    /**
     * Sets all motors to RUN_WITHOUT_ENCODER mode
     */
    public void setToPowerMode() {
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Sets all motors to RUN_USING_ENCODER mode
     */
    public void setToEncoderMode() {
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Sets all motors to this runMode
     *
     * @param runMode runMode to be set
     */
    public void setMotorMode(DcMotor.RunMode runMode) {
        frontLeft.setMode(runMode);
        backLeft.setMode(runMode);
        frontRight.setMode(runMode);
        backRight.setMode(runMode);
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
        if (frontLeft.getMode() == DcMotor.RunMode.RUN_TO_POSITION)
            return; //Stops executing

        frontLeft.setPower(left);
        backLeft.setPower(left);

        frontRight.setPower(right);
        backRight.setPower(right);
    }

    /**
     * Sets target positions for the motors
     *
     * @param left  Absolute position in ticks of the left motor
     * @param right Absolute position in ticks of the right motor
     */
    public void setTarget(int left, int right) {
        if (frontLeft.getMode() != DcMotor.RunMode.RUN_TO_POSITION)
            return;

        frontLeft.setTargetPosition(left);
        backLeft.setTargetPosition(left);

        frontRight.setTargetPosition(right);
        backRight.setTargetPosition(right);
    }

    /**
     * Sets the maximum speed of the motors in RUN_TO_POSITION mode
     *
     * @param speed Max speed in ticks per second
     */
    public void setMaxSpeed(int speed) {
        frontLeft.setMaxSpeed(speed);
        backLeft.setMaxSpeed(speed);

        frontRight.setMaxSpeed(speed);
        backRight.setMaxSpeed(speed);
    }

    /**
     * Sets the zeroPowerBehavior of the motors. Either coast or break
     * @param zeroPowerBehavior The behavior of motors with 0 as speed.
     */
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior){
        frontLeft.setZeroPowerBehavior(zeroPowerBehavior);
        backLeft.setZeroPowerBehavior(zeroPowerBehavior);

        frontRight.setZeroPowerBehavior(zeroPowerBehavior);
        backRight.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public DcMotor getfrontLeft() {
        return frontLeft;
    }

    public DcMotor getfrontRight() {
        return frontRight;
    }

    public DcMotor getbackLeft() {
        return backLeft;
    }

    public DcMotor getbackRight() {
        return backRight;
    }
}
