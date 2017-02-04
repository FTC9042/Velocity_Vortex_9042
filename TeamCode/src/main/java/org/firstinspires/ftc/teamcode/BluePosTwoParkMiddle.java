/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

//Back left wheel is on the second crack away from the corner vortex on driver side
//Flush against wall

@Autonomous(name="Blue Pos2: Park Partial Middle", group="Blue Position 2")
//@Disabled
public class BluePosTwoParkMiddle extends LinearOpMode{

    Robot robot   = new Robot();
    private ElapsedTime     runtime = new ElapsedTime();

    //encoder targets
    private int rightTarget,
            leftTarget;

    //ENCODER CONSTANTS
    private final double CIRCUMFERENCE_INCHES = 4 * Math.PI,
            TICKS_PER_ROTATION = 1200 / 0.8522,
            TICKS_PER_INCH = TICKS_PER_ROTATION / CIRCUMFERENCE_INCHES,
            TOLERANCE = 40,
            ROBOT_WIDTH = 14.5;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        robot.resetGyro();
        robot.setDirection();
        robot.resetEncoders();
        telemetry.addData("Status", "Resetting Encoders | Left:"+ robot.backLeft.getCurrentPosition()+" Right:"+robot.backRight.getCurrentPosition());
        while (robot.gyro.isCalibrating() && !opModeIsActive()){
            telemetry.addData("Status", "Gyro is Resetting. Currently at "+ robot.gyro.getHeading());
            telemetry.update();

            idle();
        }
        telemetry.addData("Status", "Gyro is done Calibrating.");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "Forward 40 Inches");
        telemetry.update();
        robot.shoot(.65);
        sleep(1000);
        robot.elevator.setPower(-.5);
        sleep(1000);
        robot.elevator.setPower(0);
        sleep(1000);
        robot.elevator.setPower(-.5);
        sleep(1000);
        robot.elevator.setPower(0);
        robot.stopShooter();
        runStraight(38, 10);
        turnRight(45,10);
        runStraight(10,10);

    }

    //ENCODER BASED MOVEMENT
    public void runStraight(double distance_in_inches, int timeoutS) throws InterruptedException{
        if (opModeIsActive()){
            leftTarget = (int) (distance_in_inches * TICKS_PER_INCH);
            rightTarget = leftTarget;
            robot.setToEncoderMode();
            setTargetValueMotor();
            runtime.reset();
            robot.setMotorPower(.4,.4);
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && !hasReached()) {
                basicTel();
                idle();
            }
            robot.setMotorPower(0,0);
            robot.resetEncoders();
        }
    }

    //Turning With Gyro's
    public void turnRight(int angle, int timeoutS) throws InterruptedException{
        if (opModeIsActive()){
            robot.setToWOEncoderMode();
            runtime.reset();
            robot.setMotorPower(.1,-.1);
            int targetAngle = robot.gyro.getHeading()+angle;
            if (targetAngle>=360){
                targetAngle-=360;
            }
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && Math.abs(robot.gyro.getHeading()-targetAngle)>=4) {
                basicTel();
                idle();
            }
            robot.setMotorPower(0,0);
            robot.resetEncoders();
        }
    }

    //Turning With Gyro's
    public void turnLeft(int angle, int timeoutS) throws InterruptedException{
        if (opModeIsActive()){
            robot.setToWOEncoderMode();
            runtime.reset();
            robot.setMotorPower(-.1,.1);
            int targetAngle = robot.gyro.getHeading()-angle;
            if (targetAngle<0){
                targetAngle += 360;
            }
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && Math.abs(robot.gyro.getHeading()-targetAngle)>=4) {
                basicTel();
                idle();
            }
            robot.setMotorPower(0,0);
            robot.resetEncoders();

        }
    }

    public void rollout(int timeoutS){
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < timeoutS){
            robot.roller.setPower(1);
        }
    }

    public void turnTowards(int angle, int timeoutS) throws InterruptedException{
        if (opModeIsActive()){
            runtime.reset();
            robot.setToWOEncoderMode();
            if (robot.gyro.getHeading()>angle){
                robot.setMotorPower(-.1,.1);
            }
            else{
                robot.setMotorPower(.1, -.1);
            }
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && Math.abs(robot.gyro.getHeading()-angle)>=3) {
                basicTel();
                idle();
            }
            robot.setMotorPower(0,0);
            robot.resetEncoders();
            sleep(500);
        }
    }

    public void setTargetValueMotor() {
        robot.frontLeft.setTargetPosition(leftTarget);
        robot.backLeft.setTargetPosition(leftTarget);

        robot.frontRight.setTargetPosition(rightTarget);
        robot.backRight.setTargetPosition(rightTarget);
    }

    public boolean hasReached() {
        return (Math.abs(robot.frontLeft.getCurrentPosition() - leftTarget) <= TOLERANCE &&
                Math.abs(robot.backLeft.getCurrentPosition() - leftTarget) <= TOLERANCE &&
                Math.abs(robot.frontRight.getCurrentPosition() - rightTarget) <= TOLERANCE &&
                Math.abs(robot.backRight.getCurrentPosition() - rightTarget) <= TOLERANCE);
    }

    public void basicTel(){
        telemetry.addData("Back Right Motor", "Target %7d: Current Pos %7d", robot.backRight.getTargetPosition(), robot.backRight.getCurrentPosition());
        telemetry.addData("Front Right Motor", "Target %7d: Current Pos %7d", robot.frontRight.getTargetPosition(), robot.frontRight.getCurrentPosition());
        telemetry.addData("Back Left Motor", "Target %7d: Current Pos %7d", robot.backLeft.getTargetPosition(), robot.backLeft.getCurrentPosition());
        telemetry.addData("Front Left Motor", "Target %7d: Current Pos %7d", robot.frontLeft.getTargetPosition(), robot.frontLeft.getCurrentPosition());
        telemetry.addData("Gyro", "Robot is facing %d",robot.gyro.getHeading());
        telemetry.addData("Colors","Red is %d and Blue is %d", robot.color.red(), robot.color.blue());
        telemetry.update();
    }

}
