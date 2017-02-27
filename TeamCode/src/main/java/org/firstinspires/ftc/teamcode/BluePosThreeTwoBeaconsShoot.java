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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//Back left wheel is on the second crack away from the corner vortex on driver side
//Flush against wall

@Autonomous(name="Blue Pos3: Both beacons and shoot! v13-14 ", group="Blue Position 3")
//@Disabled
public class BluePosThreeTwoBeaconsShoot extends LinearOpMode{

    Robot robot   = new Robot();
    private ElapsedTime     runtime = new ElapsedTime();
    private ElapsedTime     elapsed;

    private double travelDist;

    //encoder targets
    private int rightTarget,
            leftTarget;

    //ENCODER CONSTANTS
    private final double CIRCUMFERENCE_INCHES = 4 * Math.PI,
            TICKS_PER_ROTATION = 1200 / 0.8522,
            TICKS_PER_INCH = TICKS_PER_ROTATION / CIRCUMFERENCE_INCHES,
            TOLERANCE = 40,
            ROBOT_WIDTH = 14.5;

    private double targetRPM = 4100, currentRPM = 0, shooterSpeed = 0.47;
    private ElapsedTime RPMCycle;


    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        robot.color.enableLed(false);

        robot.setDirection();
        robot.resetEncoders();
        robot.setToBrake();
        if (robot.gyro.getHeading() != 0) {
            robot.gyro.calibrate();
            while (robot.gyro.isCalibrating() && !opModeIsActive()) {
                telemetry.addData("Status", "Gyro is Resetting. Currently at " + robot.gyro.getHeading());
                telemetry.update();

                idle();
            }
            telemetry.addData("Status", "Gyro is done Calibrating. Heading: "+robot.gyro.getHeading());
            telemetry.update();
        }
        else{
            telemetry.addData("Status", "Gyro is already Calibrated. Heading: "+robot.gyro.getHeading());
            telemetry.update();
        }

        waitForStart();
        elapsed = new ElapsedTime();

        runStraight(19, 10, .6);
        turnRight(30, 10);
        runStraight(48, 10, .7);
        turnRight(56, 5);
        turnTowards(90, 3);
        sleep(300);
        travelDist = robot.dist.getDistance(DistanceUnit.INCH) - 3;
        if(travelDist >= 30 || travelDist <= 0) {
            travelDist = 20;
        }
        runStraight(travelDist, 4, .6);
        telemetry.addData("Dist", "Distance Detected was "+robot.dist.getDistance(DistanceUnit.INCH));
        telemetry.log();
        telemetry.update();
        if (!isColorRed()){
            turnTowards(94, 3);
        }
        else{
            turnTowards(86, 3);
        }
        runStraight(5, 1, .3);
        runStraight(-3, 1, .3);
        turnTowards(90, 3);
        if (!isColorRed()){
            runStraight(-10, 3, .6);
            turnLeft(88, 4);
            turnTowards(0, 10);
            runStraight(44, 10, .8);
            turnRight(92, 4);
            turnTowards(90, 5);
            sleep(300);
            travelDist = robot.dist.getDistance(DistanceUnit.INCH) - 3;
            if(travelDist >= 30 || travelDist <= 0) {
                travelDist = 20;
            }
            runStraight(travelDist, 4, .6);
            telemetry.addData("Dist", "Distance Detected was "+robot.dist.getDistance(DistanceUnit.INCH));
            telemetry.log();
            telemetry.update();
            targetRPM = 4500;
            robot.shoot(shooterSpeed);
            if (!isColorRed()){
                turnTowards(94, 3);
            }
            else{
                turnTowards(86, 3);
            }
            runStraight(3, 1, .3);
            runStraight(-3, 1, .3);
            if (isColorRed() && elapsed.seconds()<25.5) {
                sleep(4400);
                runStraight(7, 3, .7);
            }
            else{
                turnLeft(47, 3);
                shoot();
            }

        }
        else {
            robot.shoot(shooterSpeed);
            sleep(4800);
            runStraight(5, 3, .4);
            runStraight(-3, 3, .5);
            shoot();
        }
    }
    //ENCODER BASED MOVEMENT
    public void runStraight(double distance_in_inches, int timeoutS, double speed) throws InterruptedException{
        if (opModeIsActive()){
            leftTarget = (int) (distance_in_inches * TICKS_PER_INCH);
            rightTarget = leftTarget;
            robot.setToEncoderMode();
            setTargetValueMotor();
            runtime.reset();
            robot.setMotorPower(speed,speed);
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && !hasReached()) {
                robot.checkPower(speed, speed);
                basicTel();
                idle();
            }
            robot.setMotorPower(0,0);
            robot.resetEncoders();
        }
    }

    public void shoot() {
        currentRPM = robot.getRPM();
        RPMCycle = new ElapsedTime();
        while (Math.abs(currentRPM-targetRPM)>100 && opModeIsActive() && elapsed.seconds()<28.5){
            if (RPMCycle.milliseconds()>=1000) {
                if (targetRPM > currentRPM) {
                    shooterSpeed += 0.01;
                } else if (targetRPM < currentRPM) {
                    shooterSpeed -= 0.01;
                }
                currentRPM = robot.getRPM();
                robot.shoot(shooterSpeed);
                RPMCycle.reset();
                telemetry.addData("Shooter Status", "Current RPM = "+currentRPM);
                telemetry.addData("Shooter Status", "Target RPM = "+targetRPM);
                telemetry.addData("Shooter Status", "Motor Power = "+robot.shooterLeft.getPower());
                telemetry.update();
                idle();
            }
            else{
                telemetry.addData("Shooter Status", "Current RPM = "+currentRPM);
                telemetry.addData("Shooter Status", "Target RPM = "+targetRPM);
                telemetry.addData("Shooter Status", "Motor Power = "+robot.shooterLeft.getPower());
                telemetry.update();
                idle();
            }

        }
        robot.elevator.setPower(-.9);
        sleep(2000);
        robot.elevator.setPower(0);
        robot.stopShooter();
    }

    public boolean isColorRed(){
        if (robot.color.red()>robot.color.blue() && robot.color.red()>=1){
            telemetry.addData("Colors","Red is %d and Blue is %d", robot.color.red(), robot.color.blue());
            telemetry.addData("Course","The color detected was red");
            telemetry.update();
            return true;
        }
        telemetry.addData("Colors","Red is %d and Blue is %d", robot.color.red(), robot.color.blue());
        telemetry.addData("Course","The color detected was blue");
        telemetry.update();
        return false;
    }

    //Turning With Gyro's
    public void turnRight(int angle, int timeoutS) throws InterruptedException{
        if (opModeIsActive()){
            robot.setToWOEncoderMode();
            runtime.reset();
            robot.setMotorPower(.15,-.15);
            int targetAngle = robot.gyro.getHeading()+angle;
            if (targetAngle>=360){
                targetAngle-=360;
            }
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && Math.abs(robot.gyro.getHeading()-targetAngle) >= 6) {
                robot.checkPower(.15, -.15);
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
            robot.setMotorPower(-.15 , .15);
            int targetAngle = robot.gyro.getHeading()-angle;
            if (targetAngle<0){
                targetAngle += 360;
            }
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && Math.abs(robot.gyro.getHeading()-targetAngle) >= 6) {
                robot.checkPower(-.15 , .15);
                basicTel();
                idle();
            }
            robot.setMotorPower(0 , 0);
            robot.resetEncoders();
        }
    }

    public void rollout(int timeoutS){
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < timeoutS){
            robot.roller.setPower(1);
            idle();
        }
    }

    public void turnTowards(int angle, int timeoutS) throws InterruptedException{
        if (opModeIsActive()){
            runtime.reset();
            robot.setToWOEncoderMode();
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && Math.abs(robot.gyro.getHeading()-angle)>=2) {
                if (robot.gyro.getHeading()>angle){
                    robot.setMotorPower(-.1, .1);
                }
                else{
                    robot.setMotorPower(.1, -.1);
                }
                basicTel();
            }
            robot.setMotorPower(0,0);
            robot.resetEncoders();
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
        telemetry.addData("Time", "Total Elapsed time is %.2f", elapsed.seconds());
        telemetry.update();
    }
}
