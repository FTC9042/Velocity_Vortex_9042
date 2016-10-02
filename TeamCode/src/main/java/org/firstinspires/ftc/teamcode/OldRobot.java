package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class OldRobot
{
    //driving motors
    DcMotor frontLeft,
            backLeft;

    DcMotor frontRight,
            backRight;

    //arm motors
    DcMotor armMotor1,
            armMotor2,
            armPivot,
            propeller;

    //zipline servo
    Servo zipLiner,
            dropClimber;

    TouchSensor backBumper;

    GyroSensor Gyro;

    /* Local OpMode members. */
    HardwareMap hardwareMap  = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public OldRobot() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hardwareMap = ahwMap;

        frontLeft = hardwareMap.dcMotor.get("l1");
        backLeft = hardwareMap.dcMotor.get("l2");

        //right drive
        frontRight = hardwareMap.dcMotor.get("r1");
        backRight = hardwareMap.dcMotor.get("r2");

        //pivot motor
        armPivot = hardwareMap.dcMotor.get("arm");

        //tape measure arms
        armMotor1 = hardwareMap.dcMotor.get("tm1");
        armMotor2 = hardwareMap.dcMotor.get("tm2");
        propeller = hardwareMap.dcMotor.get("prop");

        //zipline servo
        zipLiner = hardwareMap.servo.get("zip");
        dropClimber = hardwareMap.servo.get("drop");

        Gyro = hardwareMap.gyroSensor.get("gyro");
        backBumper = hardwareMap.touchSensor.get("bumper");

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        propeller.setPower(0);
        armMotor1.setPower(0);
        armMotor2.setPower(0);
        armPivot.setPower(0);

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
}
