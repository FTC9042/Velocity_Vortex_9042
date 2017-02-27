package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.text.SimpleDateFormat;
import java.util.Date;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Test OP", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
public class TestTeleOp extends OpMode {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    //driving motors
    DcMotor backLeft, frontLeft, frontRight, backRight;

    //Manipulator Motors
    DcMotor roller,
            elevator,
            shooterRight,
            shooterLeft;



    GyroSensor gyro;

    ColorSensor color;

    TouchSensor bumper;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    Robot robot = new Robot();
    double shooterPower = .5;
    double currentRPM = 0;
    boolean isShooting;
    ElapsedTime clicker = new ElapsedTime();
    double speed = 0;
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        backLeft = hardwareMap.dcMotor.get("l2");
        frontLeft = hardwareMap.dcMotor.get("l1");

        //right drive
        backRight = hardwareMap.dcMotor.get("r2");
        frontRight = hardwareMap.dcMotor.get("r1");

        //Manipulators
        roller = hardwareMap.dcMotor.get("intake");
        elevator = hardwareMap.dcMotor.get("ele");

        shooterLeft = hardwareMap.dcMotor.get("left");
        shooterRight = hardwareMap.dcMotor.get("right");

        //Sensors
        gyro = hardwareMap.gyroSensor.get("gyro");
        color = hardwareMap.colorSensor.get("color");
        bumper = hardwareMap.touchSensor.get("bumper");


        robot.resetEncoders();
        robot.setToWOEncoderMode();
        robot.setDirection();

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

        if (gamepad1.y) {
            robot.setMotorPower(.3, .3);
        } else if (gamepad1.a) {
            robot.setMotorPower(-.3, -.3);
        } else if (gamepad1.right_trigger>0 && gamepad1.left_trigger>1){
            robot.setMotorPower(.2, .2);
        }
        else {
            robot.setMotorPower(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
        }

        //Gamepad 2 (Operator) Controls
        //Intake mechanism Manipulation
        if (gamepad2.y) {
            robot.roller.setPower(1);
        } else if (gamepad2.a) {
            robot.roller.setPower(-1);
        } else if (gamepad2.x) {
            robot.roller.setPower(.5);
        } else {
            robot.roller.setPower(0);
        }

        //purpose of testing the stuff
        if (gamepad2.dpad_up) {
            robot.elevator.setPower(-.5);
        }
        else if (gamepad2.dpad_down){
            robot.elevator.setPower(.5);
        }
        else {
            robot.elevator.setPower(0);
        }

        if (gamepad2.dpad_left && clicker.seconds()>0.5){
            shooterPower -= 0.03;
            clicker.reset();
        }
        else if (gamepad2.dpad_right && clicker.seconds()>0.5){
            shooterPower += 0.03;
            clicker.reset();
        }

        if (shooterPower >=.95){
            shooterPower = .9 ;
        }
        else if (shooterPower<=.3){
            shooterPower = .4;
        }

        if (gamepad2.right_trigger>0) {
            robot.shoot(shooterPower);
        }
        else if (gamepad2.right_bumper && clicker.seconds()>=0.500){
            if (currentRPM >= 4100){
                shooterPower -= 0.01;
            }
            else if (currentRPM <= 4100){
                shooterPower += 0.01;
            }
            robot.shoot(shooterPower);
            clicker.reset();
        }
        else if (gamepad2.left_bumper && clicker.seconds()>=0.500){
            if (currentRPM >= 6100){
                shooterPower -= 0.01;
            }
            else if (currentRPM <= 6000){
                shooterPower += 0.01;
            }
            robot.shoot(shooterPower);
            clicker.reset();
        }
        else if (gamepad2.right_stick_button||gamepad2.left_stick_button){
            robot.stopShooter();
        }

        if (runtime.seconds()>1.000){
            runtime.reset();
            currentRPM = robot.getRPM();
        }


        //Return Encoder Values
        telemetry.addData("Left Back Encoder", "Value is "+robot.backLeft.getCurrentPosition());
        telemetry.addData("Left Front Encoder", "Value is "+robot.frontLeft.getCurrentPosition());
        telemetry.addData("Right Back Encoder", "Value is "+robot.backRight.getCurrentPosition());
        telemetry.addData("Right Front Encoder", "Value is "+robot.frontRight.getCurrentPosition());
        telemetry.addData("Robot Angle","The robot is facing "+robot.gyro.getHeading());
        telemetry.addData("Gyro Status", ""+robot.gyro.status());
        telemetry.addData("Gyro Calimbrating?", "Gyro Calibrating?"+robot.gyro.isCalibrating());
        telemetry.addData("Colors", "Red %d Green %d Blue %d", robot.color.red(), robot.color.green(), robot.color.blue());
        if (robot.bumper.isPressed()){
            telemetry.addData("Shooter Status", "Locked and Loaded and Ready to Go");
        }
        telemetry.addData("Shooter Speed", "RPM = "+currentRPM);
        telemetry.addData("Shooter Power", "Power = "+Math.abs(robot.shooterLeft.getPower())+" | Target = "+ shooterPower);
        telemetry.addData("Shooter Stuff", "Shooter Left Position is "+Math.abs(robot.shooterLeft.getCurrentPosition()));
        telemetry.addData("Shooter Stuff", "Shooter Right Position is "+Math.abs(robot.shooterRight.getCurrentPosition()));
        telemetry.update();
        robot.color.enableLed(true);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.setMotorPower(0,0);
        robot.stopShooter();
        robot.elevator.setPower(0);
        robot.roller.setPower(0);
    }
}