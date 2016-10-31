package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    //driving motors
    DcMotor backLeft, frontLeft, frontRight, backRight;

    GyroSensor gyro;

    //encoder targets
    private int rightTarget,
            leftTarget;


    //MOTOR RANGES
    private final double MOTOR_MAX = 1,
            MOTOR_MIN = -1;
    private final double INCHES_PER_DEGREE = Math.PI / 20;


    protected boolean on = true;

    //ENCODER CONSTANTS
    private final double CIRCUMFERENCE_INCHES = 4 * Math.PI,
            TICKS_PER_ROTATION = 1200 / 0.8522,
            TICKS_PER_INCH = TICKS_PER_ROTATION / CIRCUMFERENCE_INCHES,
            TOLERANCE = 40,
            ROBOT_WIDTH = 14.5;


    /* Local OpMode members. */
    HardwareMap hardwareMap = null;
    Telemetry telemetry = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public Robot() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, Telemetry atelemetry) {
        // save reference to HW Map
        hardwareMap = ahwMap;
        telemetry = atelemetry;

        backLeft = hardwareMap.dcMotor.get("l2");
        frontLeft = hardwareMap.dcMotor.get("l1");

        //right drive
        backRight = hardwareMap.dcMotor.get("r2");
        frontRight = hardwareMap.dcMotor.get("r1");

        gyro = hardwareMap.gyroSensor.get("gyro");

        backLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
        frontLeft.setPower(0);

        setToWOEncoderMode();
    }

    /***
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long remaining = periodMs - (long) period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    /**
     * Sets all motors to RUN_TO_POSITION mode
     */
    public void setToEncoderMode() {
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Resets the heading of the Gyro Sensor by calibrating
     */

    public void resetGyro() {
        gyro.calibrate();
    }

    /**
     * Sets all motors to RUN_WITHOUT_ENCODER mode
     */
    public void setToWOEncoderMode() {
        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Sets all motors to RUN_USING_ENCODER mode
     */
    public void resetEncoders() {
        DcMotor.RunMode prev = backLeft.getMode();
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorMode(prev);
    }

    /**
     * Sets the direction config of motors
     */
    public void setDirection() {
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
    }

    /**
     * Sets all motors to this runMode
     *
     * @param runMode runMode to be set
     */
    public void setMotorMode(DcMotor.RunMode runMode) {
        backLeft.setMode(runMode);
        backRight.setMode(runMode);
        frontRight.setMode(runMode);
        frontLeft.setMode(runMode);
    }

    /**
     * Drives the motors with the specified power levels
     * <p>
     * This method will not execute if the motors are in RUN_TO_POSITION mode
     *
     * @param leftpower  Power of the left motors
     * @param rightpower Power of the right motorss
     */
    public void setMotorPower(double leftpower, double rightpower) {
        backRight.setPower(rightpower);
        frontRight.setPower(rightpower);

        backLeft.setPower(leftpower);
        frontLeft.setPower(leftpower);
    }

    /**
     * Sets the maximum speed of the motors in RUN_TO_POSITION mode
     *
     * @param speed Max speed in ticks per second
     */
    public void setMaxSpeed(int speed) {
        backRight.setMaxSpeed(speed);
        backLeft.setMaxSpeed(speed);

        frontLeft.setMaxSpeed(speed);
        frontRight.setMaxSpeed(speed);
    }

    /**
     * Sets the encoder target positions of both the left and right motors from the global target variables
     */
    public void setTargetValueMotor() {
        frontLeft.setTargetPosition(leftTarget);
        backLeft.setTargetPosition(leftTarget);

        frontRight.setTargetPosition(rightTarget);
        backRight.setTargetPosition(rightTarget);
    }

    /**
     * Tests if the motors are close enough to their targets
     *
     * @return returns true if the robot is within tolerance of the target position
     */
    public boolean hasReached() {
        return (Math.abs(frontLeft.getCurrentPosition() - leftTarget) <= TOLERANCE &&
                Math.abs(backLeft.getCurrentPosition() - leftTarget) <= TOLERANCE &&
                Math.abs(frontRight.getCurrentPosition() - rightTarget) <= TOLERANCE &&
                Math.abs(backRight.getCurrentPosition() - rightTarget) <= TOLERANCE);
    }

    //ENCODER BASED MOVEMENT

    /**
     * This method sets the correct encoder ticks for moving a set distance. Run once per operation.
     *
     * @param distance_in_inches distance driven by the robot in inches
     */
    public void setDistance(double distance_in_inches) {
        resetEncoders();
        leftTarget = (int) (distance_in_inches * TICKS_PER_INCH);
        rightTarget = leftTarget;
        setToEncoderMode();
        setTargetValueMotor();
    }

    /**
     * This method is called repeatedly to move the robot
     *
     * @return if the loop should continue iterating or to break out of the loop
     */
    public boolean runStraight() {
        setMotorPower(.3, .3);
        if (!hasReached()) {
            // Display it for the driver.
            telemetry.addData("Back Right Motor", "Target %7d: Current Pos %7d", backRight.getTargetPosition(), backRight.getCurrentPosition());
            telemetry.addData("Front Right Motor", "Target %7d: Current Pos %7d", frontRight.getTargetPosition(), frontRight.getCurrentPosition());
            telemetry.addData("Back Left Motor", "Target %7d: Current Pos %7d", backLeft.getTargetPosition(), backLeft.getCurrentPosition());
            telemetry.addData("Front Left Motor", "Target %7d: Current Pos %7d", frontLeft.getTargetPosition(), frontLeft.getCurrentPosition());
            telemetry.update();
            return true;
        }
        setMotorPower(0, 0);
        resetEncoders();
        return false;
    }
}
