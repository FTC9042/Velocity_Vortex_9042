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

@Autonomous(name="Red Pos2: BEACON(S)!", group="Red Position 2")
//@Disabled
public class RedPosTwoBeacons extends LinearOpMode{

    Robot robot   = new Robot();
    private ElapsedTime     runtime = new ElapsedTime();
    private ElapsedTime     elapsed;

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
        robot.color.enableLed(false);

        robot.setDirection();
        robot.resetEncoders();
        telemetry.addData("Status", "Resetting Encoders | Left:"+ robot.backLeft.getCurrentPosition()+" Right:"+robot.backRight.getCurrentPosition());
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

        runStraight(20, 10, .6);
        turnLeft(45, 10);
        runStraight(58, 10, .6);
        turnLeft(45, 3);
        turnTowards(270, 5);
        runStraight(13, 4, .6);
        runStraight(4, 1, .3);
        runStraight(-2, 1, .6);
        sleep(250);
        if (!isColorRed()){
            sleep(4700);
            runStraight(3, 1, .6);
        }
        runStraight(-10, 3, .6);

        if (elapsed.seconds()<17) {
            turnRight(90,5);
            turnTowards(0, 2);
            runStraight(44, 5, .7);
            turnLeft(90, 5);
            turnTowards(270, 5);
            if (elapsed.seconds() < 24) {
                runStraight(10, 2, .7);
                runStraight(2, 1, .3);
                runStraight(-2, 1, .6);
                sleep(250);
                if (!isColorRed()) {
                    sleep(4700);
                    runStraight(3, 1, .3);
                }
            }
        }
        else{
            turnRight(45, 5);
            runStraight(-24, 4, .6);
            turnLeft(90, 2);
            turnTowards(230, 5);
            runStraight(29, 4, .7);
            robot.roller.setPower(1);
            runStraight(4, 5, .6);
            rollout(10);
        }

    }


    /**
     *This method, runStraight, is our main method for driving straight.
     *It utilizes the encoders and RunToPosition mode of the encoders to
     *drive an accurate distance based on the distance input.
     * @param distance_in_inches is converted into encoder ticks using the private
     * final doubles at the beginning of each class.
     * @param timeoutS utilizes the ElapsedTime() object to detect
     * how much time has passed from the beginning of the method. If it has been too long
     * in the method, that means something has gone wrong, and the robot moves on.
     * @param speed allows us to change the speed based upon
     * time sensitivity and the distance being travelled.
     *
     */

    /*
    This method, runStraight, is our main method for driving straight.
    It utilizes the encoders and RunToPosition mode of the encoders to
    drive an accurate distance based on the distance input. The double distance_in_inches
    converted into encoder ticks using the private final doubles at the beginning of
    each class. The speed parameter allows us to change the speed based upon
    time sensitivity and the distance being travelled. The timeout parameter is constant
    throughout most of our methods. It utilizes the ElapsedTime() object to detect
    how much time has passed from the beginning of the method. If it has been too long
    in the method, that means something has gone wrong, and the robot moves on.
     */

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
            sleep(250);
        }
    }

    /*
    This method, isColorRed(), utilizes the color sensor. It reads the color
    sensor's red and blue input when pointed at the beacon to determine which
    color is more dominant. It returns true when the color is red along with a
    telemetry update, or false when the color is blue.
     */
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

    /*
    This method, turnRight(), is used to turn right a certain number of degrees.
    The angle parameter it takes in is used to determine the angle it should
    turn left. The robot makes sure that the target angle stays within the
    0-359 degree range that the gyro senses. The tolerance of the turn is
    give or take 3 degrees. This movement method does not use encoders,
    but it uses the gyro to constantly calculate the angle that the robot is facing.
     */
    public void turnRight(int angle, int timeoutS) throws InterruptedException{
        if (opModeIsActive()){
            robot.setToWOEncoderMode();
            runtime.reset();
            robot.setMotorPower(.13,-.13);
            int targetAngle = robot.gyro.getHeading()+angle;
            if (targetAngle>=360){
                targetAngle-=360;
            }
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && Math.abs(robot.gyro.getHeading()-targetAngle)>=4) {
                robot.checkPower(.13, -.13);
                basicTel();
                idle();
            }
            robot.setMotorPower(0,0);
            robot.resetEncoders();
        }
    }

    /*
    This method, turnLeft(), is used to turn left a certain number of degrees.
    The angle parameter it takes in is used to determine the angle it should
    turn left. The robot makes sure that the target angle stays within the
    0-359 degree range that the gyro senses. The tolerance of the turn is
    give or take 3 degrees. This movement method does not use encoders,
    but it uses the gyro to constantly calculate the angle that the robot is facing.
    */
    public void turnLeft(int angle, int timeoutS) throws InterruptedException{
        if (opModeIsActive()){
            robot.setToWOEncoderMode();
            runtime.reset();
            robot.setMotorPower(-.13,.13);
            int targetAngle = robot.gyro.getHeading()-angle;
            if (targetAngle<0){
                targetAngle += 360;
            }
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && Math.abs(robot.gyro.getHeading()-targetAngle)>=4) {
                robot.checkPower(-.13, .13);
                basicTel();
                idle();
            }
            robot.setMotorPower(0,0);
            robot.resetEncoders();
        }
    }

    /*
    This method, turnTowards(), is used for precision turning. Its turns
    are significantly slower than turnLeft() or turnRight(), but are much
    more precise. The angle parameter is the angle the robot should turn
    TO FACE towards between 0-359. It then calculates the fastest way to turn
    towards that angle by either turning right or left. This movement method does
    not use encoders, but it uses the gyro to calculate the angle that the robot is facing.
     */
    public void turnTowards(int angle, int timeoutS) throws InterruptedException{
        if (opModeIsActive()){
            runtime.reset();
            robot.setToWOEncoderMode();
            if (robot.gyro.getHeading()>angle){
                robot.setMotorPower(-.1,.1);
                while (opModeIsActive() && (runtime.seconds() < timeoutS) && Math.abs(robot.gyro.getHeading()-angle)>=3) {
                    robot.checkPower(-.1, .1);
                    basicTel();
                    idle();
                }
            }
            else{
                robot.setMotorPower(.1, -.1);
                while (opModeIsActive() && (runtime.seconds() < timeoutS) && Math.abs(robot.gyro.getHeading()-angle)>=3) {
                    robot.checkPower(.1, -.1);
                    basicTel();
                    idle();
                }
            }
            robot.setMotorPower(0,0);
            robot.resetEncoders();
        }
    }

    /*
    This function, setTargetValueMotor(), takes the public target variables
    and sets each motor to their respective target. This method is called
    after runStraight() converts the inches input to encoder ticks and sets the
    motors to RunToPosition mode.
     */
    public void setTargetValueMotor() {
        robot.frontLeft.setTargetPosition(leftTarget);
        robot.backLeft.setTargetPosition(leftTarget);

        robot.frontRight.setTargetPosition(rightTarget);
        robot.backRight.setTargetPosition(rightTarget);
    }

    /*
    This simple function, rollout(), sets the power of our intake roller to 1 in order to
    output balls into the corner vortex. It then waits for an amount of time
    which is one of the parameters before setting the power back to 0.
     */
    public void rollout(int timeoutS){
        runtime.reset();
        robot.roller.setPower(1);
        while (opModeIsActive() && runtime.seconds() < timeoutS){
            idle();
        }
        robot.roller.setPower(0);
    }

    /*
    This method, hasReached(), checks on the position of all of our motors. It checks their
    current position and if it is within the range of the target position.
    The range is determined by tolerance which is defined at the beginning of each program.
    It returns true only if ALL of the motors have achieved their target positions.
    Otherwise, it always returns false.
    */
    public boolean hasReached() {

        return (Math.abs(robot.frontLeft.getCurrentPosition() - leftTarget) <= TOLERANCE &&
                Math.abs(robot.backLeft.getCurrentPosition() - leftTarget) <= TOLERANCE &&
                Math.abs(robot.frontRight.getCurrentPosition() - rightTarget) <= TOLERANCE &&
                Math.abs(robot.backRight.getCurrentPosition() - rightTarget) <= TOLERANCE);
    }

    /*
    This method, basicTel(), includes all the possible telemetry data we may require
    at a given time for troubleshooting purposes. It includes the target position,
    current position, and power for all the motors, the angle that the robot is facing,
    the color the color sensor is detecting, and the time elapsed in the autonomous period.
     */
    public void basicTel(){

        telemetry.addData("Back Right Motor", "Target %7d: Current Pos %7d: Power Taken %7f", robot.backRight.getTargetPosition(), robot.backRight.getCurrentPosition(), robot.backRight.getPower());
        telemetry.addData("Front Right Motor", "Target %7d: Current Pos %7d: Power Taken %7f", robot.frontRight.getTargetPosition(), robot.frontRight.getCurrentPosition(), robot.frontRight.getPower());
        telemetry.addData("Back Left Motor", "Target %7d: Current Pos %7d: Power Taken %7f", robot.backLeft.getTargetPosition(), robot.backLeft.getCurrentPosition(), robot.backLeft.getPower());
        telemetry.addData("Front Left Motor", "Target %7d: Current Pos %7d: Power Taken %7f", robot.frontLeft.getTargetPosition(), robot.frontLeft.getCurrentPosition(), robot.frontLeft.getPower());
        telemetry.addData("Gyro", "Robot is facing %d",robot.gyro.getHeading());
        telemetry.addData("Colors","Red is %d and Blue is %d", robot.color.red(), robot.color.blue());
        telemetry.addData("Time", "Total Elapsed time is %.2f", elapsed.seconds());
        telemetry.update();
    }
}
