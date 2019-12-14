package org.firstinspires.ftc.teamcode.Enabled_Classes.distanceAuto;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Enabled_Classes.robotControl;
import org.firstinspires.ftc.teamcode.R;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Disabled
@Autonomous(name="Test Auto")
//@Config
public class distanceTest extends LinearOpMode {
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
    
    public static double lockMin = 0.5;
    public static double lockMax = 0.15;
    public static double confidence = 0.75;

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
            telemetry.addData("Sorry?", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }
        hth3.lock.setPosition(lockMax);

        dashboard.startCameraStream(vuforia, 0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        //Start of auto period

        Thread.sleep(500);
        //Goes to row of stones
//        forward();
        //Thread.sleep(400);
        strafeRight(0.8);
        Thread.sleep(1000);
        halt();
        hth3.lock.setPosition(lockMin);
        //Scan here
        Thread.sleep(1000);

        hth3.resetTimer();
        while(hth3.timer.time(TimeUnit.SECONDS) < 3 && running()) {
            position = hth3.sampleSkyStone(telemetry, tfod);
//            telemetry.addData("Position", position);
//            telemetry.update();
        }
        
        switch(position) {
            case "right":
                backward(0.6);
                Thread.sleep(700);
//                strafeRight();
//                Thread.sleep(700);
//                intake();
//                forward(0.5);
//                Thread.sleep(1000);
//                halt();
//                Thread.sleep(400);
//                stopIntake();
                
            case "center":
                
            case "left":
                
            default:
                
        }


        //Begins to scan first three stones
        /*
        strafeLeft();
        Thread.sleep(700);
        halt();
        //Scan here
        Thread.sleep(2000);
        strafeLeft();
        Thread.sleep(700);
        halt();
        //Scan here
        Thread.sleep(2000);
        */


        //Turns and moves towards foundation
        Thread.sleep(20000);
        forward(0.5);
        Thread.sleep(800);
        //forward();
        //Thread.sleep(200);
        halt();
        Thread.sleep(1500);
        hth3.rightWheel.setPower(0);
        hth3.leftWheel.setPower(0);
        backward(0.5);
        Thread.sleep(500);
        right(0.5);
        Thread.sleep(1500);

        forward(0.5);
        Thread.sleep(1500);
        halt();

        hth3.rightWheel.setPower(-0.4);
        hth3.leftWheel.setPower(0.4);
        Thread.sleep(1000);
        hth3.rightWheel.setPower(0);
        hth3.leftWheel.setPower(0);

        backward(0.5);
        Thread.sleep(500);
        halt();
        Thread.sleep(5000);


        telemetry.addData("Status","Complete!");

        if (tfod != null) {
            tfod.shutdown();
        }
    }
    /*
    public boolean running() {
        return opModeIsActive() && !isStopRequested();
    }

    public void wait(int time) {
        hth3.resetTimer();
        while(hth3.getTime() < time && running()) {
        }
    }
    */

    public void forward(double speed)
    {
        hth3.moveDriveMotors(speed);
    }
    public void backward(double speed)
    {
        hth3.moveDriveMotors(-speed);
    }
    public void strafeRight(double speed)
    {
        hth3.moveDriveMotors(-speed, speed, speed, -speed);
    }
    public void strafeLeft(double speed)
    {
        hth3.moveDriveMotors(speed, -speed, -speed, speed);
    }
    public void right(double speed)
    {
        hth3.moveDriveMotors(speed, speed, -speed, -speed);
    }
    public void left(double speed)
    {
        hth3.moveDriveMotors(-speed, -speed, speed, speed);
    }
    public void halt()
    {
        hth3.halt();
    }
    public void intake() {
        hth3.rightWheel.setPower(0.95);
        hth3.leftWheel.setPower(-0.95);
    }
    public void outtake() {
        hth3.rightWheel.setPower(-0.4);
        hth3.leftWheel.setPower(0.4);
    }
    public void stopIntake() {
        hth3.rightWheel.setPower(0.0);
        hth3.leftWheel.setPower(0.0);
    }
    /**
     * Initialize the Vuforia localization engine.
     */
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
        tfodParameters.minimumConfidence = confidence;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
    /*
    Vuforia Stone Setup
     */
    public void detectStone() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                }
                telemetry.update();
                if (updatedRecognitions.size() == 1) {
                    System.out.println("Detected!");
                }
            }
        }
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    public boolean running() {
        return opModeIsActive() && !isStopRequested();
    }
}
