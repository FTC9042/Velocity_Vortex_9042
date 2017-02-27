package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;

@TeleOp(name="Color Test", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
public class TestColor extends OpMode {
    /* Declare OpMode members. */

    ColorSensor color;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        color = hardwareMap.colorSensor.get("color");

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
        color.enableLed(true);
        telemetry.addData("Red", "Red: "+color.red());
        telemetry.addData("Blue", "Blue: "+color.blue());
        telemetry.addData("Green", "Green: "+color.green());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {}
}