package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.SimpleDateFormat;
import java.util.Date;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
public class CompetitionTeleOp extends OpMode {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();


    /*
     * Code to run ONCE when the driver hits INIT
     */
    Robot robot = new Robot();
    boolean on = false;
    boolean turningRight = false;
    boolean turningLeft = false;
    boolean faceZero = false;
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap);

        robot.resetEncoders();
        robot.setToWOEncoderMode();
        robot.setDirection();
        robot.color.enableLed(false);

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());
                 //Gamepad 1 (Driver) controls
            if (gamepad1.right_bumper) {
                robot.setMotorPower(-.3 * gamepad1.left_stick_y, -.3 * gamepad1.right_stick_y);
            } else if (gamepad1.y) {
                robot.setMotorPower(.3, .3);
            } else if (gamepad1.a) {
                robot.setMotorPower(-.3, -.3);
            } else if (on){
                if (runtime.seconds()>15){
                    runtime.reset();
                }
                if (runtime.seconds()<10) {
                    robot.setMotorPower(-.1, -.1);
                    robot.trollMode.setPower(.8);
                }
                else{
                    on = false;
                    robot.trollMode.setPower(0);
                }
            } else if(turningRight){
                runtime.reset();
                if (turnTowards(90)){
                    robot.setMotorPower(0,0);
                    turningRight = false;
                }
            } else if (turningLeft){
                runtime.reset();
                if (turnTowards(270)){
                    robot.setMotorPower(0,0);
                    turningLeft = false;
                }
            } else if (faceZero){
                runtime.reset();
                if (turnTowards(0)){
                    robot.setMotorPower(0,0);
                    faceZero = false;
                }
            }
            else{
                robot.setMotorPower(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
            }

            //Automatic Turning
            if (gamepad1.dpad_left){
                turningLeft = true;
            }
            else if (gamepad1.dpad_right){
                turningRight = true;
            }
            else if (gamepad1.dpad_up){
                faceZero = true;
            }

            //Gamepad 2 (Operator) Controls
            //Intake mechanism Manipulation
            if (gamepad2.y) {
                robot.roller.setPower(1);
            } else if (gamepad2.a) {
                robot.roller.setPower(-.5);
            } else if (gamepad2.x) {
                robot.roller.setPower(.5);
            } else {
                robot.roller.setPower(0);
            }

            //Spin Center Vortex Controls
            if (gamepad2.right_bumper){
                on = true;
            }
            else if (gamepad2.left_bumper){
                on = false;
            }
            else if (gamepad2.dpad_down){
                robot.trollMode.setPower(-1);
            }
            else if (gamepad2.dpad_up){
                robot.trollMode.setPower(1);
            }
            else if (gamepad2.dpad_right || gamepad2.dpad_left){
                robot.trollMode.setPower(0);
            }

            //Master Break key to reset everything
            if (gamepad2.right_trigger>0 && gamepad2.left_trigger>0){
                turningRight=false;
                turningLeft=false;
                faceZero=false;
                on = false;
            }


        //Return Encoder Values
        telemetry.addData("Left Back Encoder", "Value is "+robot.backLeft.getCurrentPosition());
        telemetry.addData("Left Front Encoder", "Value is "+robot.frontLeft.getCurrentPosition());
        telemetry.addData("Right Back Encoder", "Value is "+robot.backRight.getCurrentPosition());
        telemetry.addData("Right Front Encoder", "Value is "+robot.frontRight.getCurrentPosition());
        telemetry.addData("Robot Angle","The robot is facing "+robot.gyro.getHeading());
        telemetry.addData("Colors", "Red %d Green %d Blue %d", robot.color.red(), robot.color.green(), robot.color.blue());
        telemetry.update();
    }

    public boolean turnTowards(int angle){
        if (robot.backLeft.getPower()!=-.1 || robot.backLeft.getPower()!=.1) {
            if (robot.gyro.getHeading() > angle) {
                robot.setMotorPower(-.15, .15);
            } else {
                robot.setMotorPower(.1, -.1);
            }
        }
        if (Math.abs(robot.gyro.getHeading()-angle)>=3){
            return false;
        } else{
            return true;
        }

    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {robot.setMotorPower(0,0);
    }
}