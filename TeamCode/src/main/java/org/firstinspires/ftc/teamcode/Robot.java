package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Robot
{
    //driving motors
    DcMotor backLeft, frontLeft, frontRight, backRight;

    //Manipulator Motors
    DcMotor roller;

    GyroSensor gyro;

    ColorSensor color;

    TouchSensor bumper;


    /* Local OpMode members. */
    HardwareMap hardwareMap  = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Robot() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hardwareMap = ahwMap;

        backLeft = hardwareMap.dcMotor.get("l2");
        frontLeft = hardwareMap.dcMotor.get("l1");

        //right drive
        backRight = hardwareMap.dcMotor.get("r2");
        frontRight = hardwareMap.dcMotor.get("r1");

        //Manipulators
        roller = hardwareMap.dcMotor.get("intake");

        //Sensors
        gyro = hardwareMap.gyroSensor.get("gyro");
        color = hardwareMap.colorSensor.get("color");
        bumper = hardwareMap.touchSensor.get("bumper");



        backLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
        frontLeft.setPower(0);

        setToWOEncoderMode();
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
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
    public void setToEncoderMode() {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Resets the heading of the Gyro Sensor by calibrating
     */

    public void resetGyro()  { gyro.calibrate(); }

    /**
     * Sets all motors to RUN_WITHOUT_ENCODER mode
     */
    public void setToWOEncoderMode() {
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Sets all motors to RUN_USING_ENCODER mode
     */
    public void resetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    /**
     * Sets the direction config of motors
     */
    public void setDirection() {
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        roller.setDirection(DcMotor.Direction.FORWARD);
    }

    /**
     * Sets all motors to this runMode
     *
     * @param runMode runMode to be set
     */
//    public void setMotorMode(DcMotor.RunMode runMode) {
//        backLeft.setMode(runMode);
//        backRight.setMode(runMode);
//        frontRight.setMode(runMode);
//        frontLeft.setMode(runMode);
//    }

    /**
     * Drives the motors with the specified power levels
     * <p>
     * This method will not execute if the motors are in RUN_TO_POSITION mode
     *
     * @param leftpower  Power of the left motors
     * @param rightpower Power of the right motorss
     */
    public void setMotorPower(double leftpower, double rightpower){
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

    public void checkPower(double rightSide, double leftSide){
        if (backLeft.getPower()!=leftSide){
            backLeft.setPower(leftSide);
        }
        if (frontLeft.getPower()!=leftSide){
            frontLeft.setPower(leftSide);
        }
        if (backRight.getPower()!=rightSide){
            backRight.setPower(rightSide);
        }
        if (frontRight.getPower()!=rightSide){
            frontRight.setPower(rightSide);
        }
    }

}
