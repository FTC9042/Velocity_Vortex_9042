///*
//Copyright (c) 2016 Robert Atkinson
//
//All rights reserved.
//
//Redistribution and use in source and binary forms, with or without modification,
//are permitted (subject to the limitations in the disclaimer below) provided that
//the following conditions are met:
//
//Redistributions of source code must retain the above copyright notice, this list
//of conditions and the following disclaimer.
//
//Redistributions in binary form must reproduce the above copyright notice, this
//list of conditions and the following disclaimer in the documentation and/or
//other materials provided with the distribution.
//
//Neither the name of Robert Atkinson nor the names of his contributors may be used to
//endorse or promote products derived from this software without specific prior
//written permission.
//
//NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
//LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
//THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
//ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
//FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
//TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
//THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//*/
//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//
//import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
//
//@Autonomous(name="Encoder Drive", group="Tests")
//@Disabled
//public class DriveWithEncoderTemplate extends LinearOpMode{
//
//    OldRobot robot   = new OldRobot();
//    private ElapsedTime     runtime = new ElapsedTime();
//
//    //encoder targets
//    private int rightTarget,
//            leftTarget;
//
//    //SERVO CONSTANTS
//    private final double SERVO_MAX = .6,
//            SERVO_MIN = .2,
//            SERVO_NEUTRAL = 9.0 / 17;
//
//    private final int PROPELLER_RIGHT = -140,
//            PROPELLER_LEFT = 140;
//    //Stops the continuous servo
//
//    //MOTOR RANGES
//    private final double MOTOR_MAX = 1,
//            MOTOR_MIN = -1;
//    private final double INCHES_PER_DEGREE = Math.PI/20;
//
//
//    protected boolean on = true;
//
//    //ENCODER CONSTANTS
//    private final double CIRCUMFERENCE_INCHES = 4 * Math.PI,
//            TICKS_PER_ROTATION = 1200 / 1.05,
//            TICKS_PER_INCH = TICKS_PER_ROTATION / CIRCUMFERENCE_INCHES,
//            TOLERANCE = 40,
//            ROBOT_WIDTH = 14.5;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        robot.init(hardwareMap);
//
//        telemetry.addData("Status", "Resetting Encoders");    //
//        telemetry.update();
//        setDirection();
//        robot.Gyro.calibrate();
//        resetEncoders();
//        idle();
//        telemetry.addData("Status","Gyro Sensor Heading is "+robot.Gyro.getHeading());
//        telemetry.update();
//        waitForStart();
//
//        telemetry.addData("Status", "Forward 48 Inches");    //
//        telemetry.update();
//        runStraight(48, 10);  // S1: Forward 48 Inches with 5 Sec timeout
//        telemetry.addData("Status", " Turning Left 90 degrees");    //
//        telemetry.update();
//        resetEncoders();
//        turnRightWithGyro(90,10);  // S2: Turn Right 90 degrees with 10 Sec timeout
//        telemetry.addData("Status", " Moving Backwards 12 Inches");    //
//        telemetry.update();
//        resetEncoders();
//        runStraight(-12, 10);  // S3: Reverse 24 Inches with 4 Sec timeout
//        telemetry.addData("Path", "Complete");
//        telemetry.update();
//    }
//
//
//    public void setDirection() {
//        if (robot.frontLeft.getDirection() == DcMotor.Direction.FORWARD) {
//            robot.frontLeft.setDirection(DcMotor.Direction.REVERSE);
//        }
//
//        if (robot.backLeft.getDirection() == DcMotor.Direction.FORWARD) {
//            robot.backLeft.setDirection(DcMotor.Direction.REVERSE);
//        }
//
//        if (robot.frontRight.getDirection() == DcMotor.Direction.REVERSE) {
//            robot.frontRight.setDirection(DcMotor.Direction.FORWARD);
//        }
//
//        if (robot.backRight.getDirection() == DcMotor.Direction.REVERSE) {
//            robot.backRight.setDirection(DcMotor.Direction.FORWARD);
//        }
//
//        if (robot.armMotor1.getDirection() == DcMotor.Direction.FORWARD) {
//            robot.armMotor1.setDirection(DcMotor.Direction.REVERSE);
//        }
//    }
//
//
//    //ENCODER MANIPULATION
//    public void resetEncoders() {
//        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    }
//
//    public void setToEncoderMode() {
//
//        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }
//
//    public void setToWOEncoderMode() {
//
//        robot.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//    }
//
//
//    //ENCODER BASED MOVEMENT
//    public void runStraight(double distance_in_inches, int timeoutS) throws InterruptedException{
//        if (opModeIsActive()){
//        leftTarget = (int) (distance_in_inches * TICKS_PER_INCH);
//        rightTarget = leftTarget;
//        setToEncoderMode();
//        setTargetValueMotor();
//            runtime.reset();
//            setMotorPower(.5,.5);
//            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (!hasReached())) {
//                // Display it for the driver.
//                telemetry.addData("Path1",  "Running to %7d :%7d", rightTarget, leftTarget);
//                telemetry.addData("Path2",  "Running at %7d :%7d", robot.backLeft.getCurrentPosition(), robot.backRight.getCurrentPosition());
//                telemetry.update();
//
//                // Allow time for other processes to run.
//                idle();
//            }
//            setMotorPower(0,0);
//            setToWOEncoderMode();
//            sleep(250);
//        }
//    }
//    public  void turnRight(int angle, int timeoutS) throws InterruptedException{
//        if (opModeIsActive()){
//        leftTarget = (int)(angle*INCHES_PER_DEGREE*TICKS_PER_INCH);
//        rightTarget = -leftTarget;
//        setToEncoderMode();
//        setTargetValueMotor();
//            runtime.reset();
//            setMotorPower(.5,.5);
//            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (!hasReached())) {
//                // Display it for the driver.
//                telemetry.addData("Path1",  "Running to %7d :%7d", rightTarget, leftTarget);
//                telemetry.addData("Path2",  "Running at %7d :%7d", robot.backLeft.getCurrentPosition(), robot.backRight.getCurrentPosition());
//                telemetry.update();
//
//                // Allow time for other processes to run.
//                idle();
//            }
//            setMotorPower(0,0);
//            setToWOEncoderMode();
//            sleep(5000);
//        }
//    }
//    public  void turnRightWithGyro(int angle, int timeoutS) throws InterruptedException{
//        int initialAngle;
//        if (opModeIsActive()){
//            setToWOEncoderMode();
//            initialAngle = robot.Gyro.getHeading();
//            int targetAngle = initialAngle+angle;
//            setMotorPower(.4, -.4);
//            while (opModeIsActive() && (runtime.seconds() < timeoutS) && robot.Gyro.getHeading()<=targetAngle) {
//                setMotorPower(.4, -.4);
//                telemetry.addData("Update","Initial Angle is " + initialAngle);
//                telemetry.addData("Update","Current Angle is " + robot.Gyro.getHeading());
//                telemetry.addData("Update","Target Angle is " + targetAngle);
//                telemetry.update();
//                // Allow time for other processes to run.
//                idle();
//            }
//            setMotorPower(0,0);
//            sleep(1000);
//        }
//    }
//
//    public void setTargetValueMotor() {
//        robot.frontLeft.setTargetPosition(leftTarget);
//        robot.backLeft.setTargetPosition(leftTarget);
//
//        robot.frontRight.setTargetPosition(rightTarget);
//        robot.backRight.setTargetPosition(rightTarget);
//    }
//
//    public boolean hasReached() {
//        return (Math.abs(robot.frontLeft.getCurrentPosition() - leftTarget) <= TOLERANCE &&
//                Math.abs(robot.backLeft.getCurrentPosition() - leftTarget) <= TOLERANCE &&
//                Math.abs(robot.frontRight.getCurrentPosition() - rightTarget) <= TOLERANCE &&
//                Math.abs(robot.backRight.getCurrentPosition() - rightTarget) <= TOLERANCE);
//    }
//
//    public void setMotorPower(double leftPower, double rightPower) {
//        clipValues(leftPower, ComponentType.MOTOR);
//        clipValues(rightPower, ComponentType.MOTOR);
//
//        robot.frontLeft.setPower(leftPower);
//        robot.backLeft.setPower(leftPower);
//
//        robot.frontRight.setPower(rightPower);
//        robot.backRight.setPower(rightPower);
//    }
//    enum ComponentType {
//        NONE,
//        MOTOR,
//        SERVO
//    }
//
//    public double clipValues(double initialValue, ComponentType type) {
//        double finalval = 0;
//        if (type == ComponentType.MOTOR)
//            finalval = Range.clip(initialValue, MOTOR_MIN, MOTOR_MAX);
//        if (type == ComponentType.SERVO)
//            finalval = Range.clip(initialValue, SERVO_MIN, SERVO_MAX);
//        return finalval;
//    }
//}
