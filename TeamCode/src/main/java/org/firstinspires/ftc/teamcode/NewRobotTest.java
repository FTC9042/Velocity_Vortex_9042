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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="New Robot Drive", group="Tests")
//@Disabled
public class NewRobotTest extends LinearOpMode{

    Robot robot   = new Robot();
    private ElapsedTime     runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, telemetry);

        telemetry.update();
        robot.resetGyro();
        robot.setDirection();
        robot.resetEncoders();
        telemetry.addData("Status", "Resetting Encoders | Left:"+ robot.backLeft.getCurrentPosition()+" Right:"+robot.backRight.getCurrentPosition());
        telemetry.update();

        waitForStart();


        telemetry.addData("Status", "Forward 48 Inches");
        telemetry.update();
        robot.setDistance(12);
        while(opModeIsActive()&&robot.runStraight()) {  // S1: Forward 70 inches
            idle();     //Yield resources
        }
//        telemetry.addData("Status", "Turning Right 90 degrees");
//        telemetry.update();
//        turnRight(90,10);
    }




    //Turning with Encoders
//    public void turnRight(int angle, int timeoutS) throws InterruptedException{
//        if (opModeIsActive()){
//        leftTarget = (int)(angle*INCHES_PER_DEGREE*TICKS_PER_INCH);
//        rightTarget = -leftTarget;
//        robot.setToEncoderMode();
//        setTargetValueMotor();
//            runtime.reset();
//            robot.setMotorPower(.3,.3);
//            while (opModeIsActive() && (runtime.seconds() < timeoutS) && !hasReached()) {
//                // Display it for the driver.
//                telemetry.addData("Right Motor", "Target %7d: Current Pos %7d: Speed is %7f", leftTarget, robot.backRight.getCurrentPosition(), robot.backRight.getPower());
//                telemetry.addData("Left Motor", "Target %7d: Current Pos %7d: Speed is %7f", leftTarget, robot.backLeft.getCurrentPosition(), robot.backLeft.getPower());
//                telemetry.update();
//
//                // Allow time for other processes to run.
//                idle();
//            }
//            robot.setMotorPower(0,0);
//            robot.resetEncoders();
//            robot.setToWOEncoderMode();
//            sleep(1000);
//        }
//    }

    //Turning With Gyro's
    public void turnRight(int angle, int timeoutS) throws InterruptedException{
        if (opModeIsActive()){
            robot.setToWOEncoderMode();
            runtime.reset();
            robot.setMotorPower(.1,-.1);
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && robot.gyro.getHeading()<=angle-3) {
                // Display it for the driver.
                telemetry.addData("Gyro", "Target is %d and Current is %d",angle, robot.gyro.getHeading() );
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }
            robot.resetGyro();
            robot.setMotorPower(0,0);
            robot.resetEncoders();
            sleep(500);
        }
    }



}
