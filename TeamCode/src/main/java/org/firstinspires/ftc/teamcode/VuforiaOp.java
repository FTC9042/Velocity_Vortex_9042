package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Tim on 9/30/2016.
 */
@TeleOp(name="Vuforia Test", group ="Tests")
//@Disabled

public class VuforiaOp extends LinearOpMode {

    public void runOpMode() throws InterruptedException{
        VuforiaLocalizer.Parameters para = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        para.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        para.vuforiaLicenseKey = "AU+Uvo3/////AAAAGdjYkc8TVkvGi3gbOyzI/78U4cJXgtsDrKQQzVxuphNb8X7dpRSTS8cbzdTtVsdwde7l+kFjJnNEOm1NFffYgG/rf8Z3+gLanvBF7rpIHNPwFsbLR1GxJJQth2kFif2tYIOXSTJ7oQQZDcUojld9UR70afiVxqCqQHbuueadf7KQBOgihBye/RLvQc2dkSDQt3o/J++1NpwEhuguCsJVlBX/C576EQ23DkmR+vmVJPiW78U7I9yEQLsRRtYOe4LhkwV6GdmkweJshkO99nzNjF28+BeODZakEImq6lJQDTHsKvlYXvIolqIzoUNSMppMB35iFGc6qhgoo9O1yqpsum2clDulBglIVpjD3MLRalK8";
        para.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(para);
//        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Lego");
        beacons.get(3).setName("Gears");

        waitForStart();

        while(opModeIsActive()){
            for(VuforiaTrackable beacon : beacons){
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beacon.getListener()).getPose();
                telemetry.addData(beacon.getName() + "-Visible", ((VuforiaTrackableDefaultListener) beacon.getListener()).isVisible() ? "Visible" : "Not Visible");

                if(pose!=null){
                    VectorF translation = pose.getTranslation();
                    telemetry.addData(beacon.getName() + "-Visible", ((VuforiaTrackableDefaultListener) beacon.getListener()).isVisible() ? "Visible" : "Not Visible");
                    telemetry.addData(beacon.getName() + "-Translation", translation);

                    double degreeToTurn = Math.toDegrees(Math.atan2(translation.get(1), translation.get(2)));

                    telemetry.addData(beacon.getName()+"-Degrees", degreeToTurn);
                }
            }

            telemetry.addData("Time", this.getRuntime());
            telemetry.update();
        }
    }

}
