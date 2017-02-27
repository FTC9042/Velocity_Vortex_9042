package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Gyro Test", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
public class TestGyro extends OpMode {
    /* Declare OpMode members. */

    GyroSensor gyro;

    @Override
    public void init() {
        gyro = hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();
        while (gyro.isCalibrating()){
            telemetry.addData("Gyro", "Gyro is calibrating");
            telemetry.update();
        }
        telemetry.addData("Gyro", "Gyro is done calibrating");
        telemetry.update();
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
    public void start(){}

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Gyro Status", "Gyro is facing "+gyro.getHeading());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {}
}