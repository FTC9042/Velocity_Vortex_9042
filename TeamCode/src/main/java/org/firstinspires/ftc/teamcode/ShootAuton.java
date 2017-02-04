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

/*Position one is with the back left wheel touching the wall at the first crack between two mats
Parallel to the midway line
*/

@Autonomous(name="JUST SHOOT CLOSE! :))", group="Position 1")
//@Disabled
public class ShootAuton extends LinearOpMode{

    Robot robot   = new Robot();
    private ElapsedTime     runtime = new ElapsedTime();

    //encoder targets
    private int rightTarget,
            leftTarget;
    private double targetRPM = 4100, currentRPM = 0, shooterSpeed = 0.47;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.setDirection();


        waitForStart();

        robot.shoot(shooterSpeed);
        sleep(2000);
        currentRPM = robot.getRPM();
        while (Math.abs(currentRPM-targetRPM)>100 && opModeIsActive()){
            if (targetRPM>currentRPM){
                shooterSpeed+=0.02;
                sleep(100);
            }
            else if (targetRPM<currentRPM){
                shooterSpeed-=0.02;
                sleep(100);
            }
            sleep(900);
            currentRPM = robot.getRPM();
            robot.shoot(shooterSpeed);
            telemetry.addData("Shooter Status", "Current RPM = "+currentRPM);
            telemetry.addData("Shooter Status", "Target RPM = "+targetRPM);
            telemetry.addData("Shooter Status", "Motor Power = "+robot.shooterLeft.getPower());
            telemetry.update();
            idle();
        }
        robot.elevator.setPower(-.9);
        sleep(2000);
        robot.elevator.setPower(0);
        robot.stopShooter();
    }

}
