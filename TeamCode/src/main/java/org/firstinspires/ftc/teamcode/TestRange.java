package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Distance Test", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
public class TestRange extends OpMode {
    /* Declare OpMode members. */

    ModernRoboticsI2cRangeSensor distance;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        distance = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "distance");

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
        telemetry.addData("Distance Status", "Distance is "+distance.getDistance(DistanceUnit.INCH));
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {}
}