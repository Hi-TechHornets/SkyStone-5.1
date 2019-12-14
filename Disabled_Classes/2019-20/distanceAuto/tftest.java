package org.firstinspires.ftc.teamcode.Enabled_Classes.distanceAuto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Enabled_Classes.robotControl;
import org.firstinspires.ftc.teamcode.R;

import java.util.concurrent.TimeUnit;

@Autonomous
@Disabled
public class tftest extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final String VUFORIA_KEY =
            "AQgDy6j/////AAABmXISaVgPmUMmugAGhFFaTDBJsE9lRd9NNkRieex8wD8vq5MTahYK2UeRXAODvLE6NsZ/89ckvXlkMgAgRqlP/se9OTW0er/VNUu0M/eCAGXWgs0ji6it3/MAVY5tC4T5h/Inzn1NZQDa0XYMzCE2z5ionyuw/Vwk6rcqOsVqpxOYsLGMvcE++a64J8WJBoQHR9zSnStw++AarPQgqL0z9bQFbpopm2bw09FyY47tp4YvKl1kyYmPt0zbToit+D+r+WWQF0/1k1ikMaUt4SOhTRXHz/ZrzRW1jtnJOkYiCJ1qsxH8dH+2sS3dtrX1hzIZ+mKu4o+x1jsYClXc0P9p5x7bR1HGOH/cnsKLiBBly2QW";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
    public robotControl hth3;
    public FtcDashboard dashboard = FtcDashboard.getInstance();

    private String position;

    @Override
    //45 Degree Angle = 1000 ms at 0.5 power
    //90 Degree Angle = 2000 ms at 0.5 power

    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        hth3 = new robotControl();
        hth3.init(hardwareMap);
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            position = hth3.sampleSkyStone(telemetry, tfod);
//            telemetry.addData("Position", position);
//            telemetry.update();
        }

    }
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        dashboard.startCameraStream(vuforia, 0);
        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
