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

@TeleOp(name="TeleOp", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
public class BasicTeleOp extends OpMode {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();


    /*
     * Code to run ONCE when the driver hits INIT
     */
    Robot robot = new Robot();
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap);

        robot.resetGyro();
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
        if (gamepad1.right_trigger>0) {
            setMotorPower(-.3*gamepad1.left_stick_y, -.3*gamepad1.right_stick_y);

        }
        else if (gamepad1.y){
            setMotorPower(.3,.3);
        }
        else if (gamepad1.a){
            setMotorPower(-.3,-.3);
        }
        else {
            setMotorPower(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
        }

        //Gamepad 2 (Operator) Controls
        //Intake mechanism Manipulation
        if (gamepad2.y) {
            robot.roller.setPower(1);
        }
        else if (gamepad2.a){
            robot.roller.setPower(-.5);
        }
        else if (gamepad2.x){
            robot.roller.setPower(.5);
        }
        else{
            robot.roller.setPower(0);
        }

        //Beacon Booper Manipulation
        if (gamepad2.dpad_left){
            robot.booper.setPosition(1);
        }
        else if (gamepad2.dpad_right){
            robot.booper.setPosition(0);
        }
        else if (gamepad2.b){
            robot.booper.setPosition(.5);
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

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        setMotorPower(0,0);
    }
    public void setMotorPower(double leftPower, double rightPower){
        robot.backLeft.setPower(leftPower);
        robot.frontLeft.setPower(leftPower);
        robot.backRight.setPower(rightPower);
        robot.frontRight.setPower(rightPower);
    }
}