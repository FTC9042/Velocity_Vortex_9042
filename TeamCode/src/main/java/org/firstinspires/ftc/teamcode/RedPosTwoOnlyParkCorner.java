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

//Back left wheel is on the first crack away from the corner vortex on driver side
//Flush against wall

@Autonomous(name="Red Pos2: Only Park Corner", group="Red Position 2")
//@Disabled
public class RedPosTwoOnlyParkCorner extends LinearOpMode{

    Robot robot   = new Robot();
    private ElapsedTime     runtime = new ElapsedTime();

    //encoder targets
    private int rightTarget,
            leftTarget;


    //MOTOR RANGES
    private final double MOTOR_MAX = 1,
            MOTOR_MIN = -1;
    private final double INCHES_PER_DEGREE = Math.PI/20;


    protected boolean on = true;

    //ENCODER CONSTANTS
    private final double CIRCUMFERENCE_INCHES = 4 * Math.PI,
            TICKS_PER_ROTATION = 1200 / 0.8522,
            TICKS_PER_INCH = TICKS_PER_ROTATION / CIRCUMFERENCE_INCHES,
            TOLERANCE = 40,
            ROBOT_WIDTH = 14.5;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        telemetry.update();
        robot.resetGyro();
        robot.setDirection();
        robot.resetEncoders();
        telemetry.addData("Status", "Resetting Encoders | Left:"+ robot.backLeft.getCurrentPosition()+" Right:"+robot.backRight.getCurrentPosition());
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "Forward 48 Inches");
        telemetry.update();
        runStraight(30, 10);
        telemetry.addData("Status", "Forward 30 Inches");
        telemetry.update();
        turnRight(60, 10);
        telemetry.addData("Status", "Forward 45 Inches");
        telemetry.update();
        runStraight(-30,10);
        telemetry.addData("Status", "backwards 20 Inches");
        telemetry.update();
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
                // Display it for the driver.
                telemetry.addData("Back Right Motor", "Target %7d: Current Pos %7d", robot.backRight.getTargetPosition(), robot.backRight.getCurrentPosition());
                telemetry.addData("Front Right Motor", "Target %7d: Current Pos %7d", robot.frontRight.getTargetPosition(), robot.frontRight.getCurrentPosition());
                telemetry.addData("Back Left Motor", "Target %7d: Current Pos %7d", robot.backLeft.getTargetPosition(), robot.backLeft.getCurrentPosition());
                telemetry.addData("Front Left Motor", "Target %7d: Current Pos %7d", robot.frontLeft.getTargetPosition(), robot.frontLeft.getCurrentPosition());
                telemetry.addData("Gyro", "Robot is facing %d",robot.gyro.getHeading());
                telemetry.update();

                // Allow time for other processes to run.
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
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && robot.gyro.getHeading()<=targetAngle-3) {
                // Display it for the driver.
                telemetry.addData("Gyro", "Target is %d and Current is %d",angle, robot.gyro.getHeading() );
                telemetry.update();

                // Allow time for other processes to run.
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
            int currentAngle;
            int targetAngle = robot.gyro.getHeading()-angle;
            if (targetAngle<0){
                targetAngle = targetAngle+360;
            }
            if (robot.gyro.getHeading()<=5) {
                currentAngle = robot.gyro.getHeading() + 360;
            }
            else{
                currentAngle = robot.gyro.getHeading();
            }
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && currentAngle-targetAngle>=3) {
                // Display it for the driver.
                if (robot.gyro.getHeading() <= 5) {
                    currentAngle = robot.gyro.getHeading() + 360;
                } else {
                    currentAngle = robot.gyro.getHeading();
                }
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

}
