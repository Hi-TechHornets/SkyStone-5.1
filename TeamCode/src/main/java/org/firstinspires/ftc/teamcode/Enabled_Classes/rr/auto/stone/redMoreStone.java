package org.firstinspires.ftc.teamcode.Enabled_Classes.rr.auto.stone;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Enabled_Classes.robotControl;
import org.firstinspires.ftc.teamcode.Enabled_Classes.rr.drive.mecanumDriveBase;
import org.firstinspires.ftc.teamcode.Enabled_Classes.rr.drive.mecanumDriveREV;
import org.firstinspires.ftc.teamcode.R;

@Autonomous(group = "stone")
//@Disabled
public class redMoreStone extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private static final String VUFORIA_KEY =
            "AQgDy6j/////AAABmXISaVgPmUMmugAGhFFaTDBJsE9lRd9NNkRieex8wD8vq5MTahYK2UeRXAODvLE6NsZ/89ckvXlkMgAgRqlP/se9OTW0er/VNUu0M/eCAGXWgs0ji6it3/MAVY5tC4T5h/Inzn1NZQDa0XYMzCE2z5ionyuw/Vwk6rcqOsVqpxOYsLGMvcE++a64J8WJBoQHR9zSnStw++AarPQgqL0z9bQFbpopm2bw09FyY47tp4YvKl1kyYmPt0zbToit+D+r+WWQF0/1k1ikMaUt4SOhTRXHz/ZrzRW1jtnJOkYiCJ1qsxH8dH+2sS3dtrX1hzIZ+mKu4o+x1jsYClXc0P9p5x7bR1HGOH/cnsKLiBBly2QW";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double stoneUp = 0.85;
    public static double stoneDown = 0.2;

    @Override
    public void runOpMode() {
        mecanumDriveBase drive = new mecanumDriveREV(hardwareMap);
        drive.resetEncoders();
        drive.setStone(stoneUp);
        
        initVuforia();
        if(dashboard != null) {
            telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        }

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        if (tfod != null) {
            tfod.activate();
        }

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (running()) {
            String result = "";
            drive.setPoseEstimate(new Pose2d(-39.5, -63, Math.toRadians(180.0)));
            Trajectory toStone = drive.trajectoryBuilder()
                                      .strafeTo(new Vector2d(-33 + 2, -46))
                                      .build();
            drive.followTrajectorySync(toStone);
            sleep(100);
            drive.turnSync(Math.toRadians(-5));

            if (tfod != null) {
                while(running()) {
                    result = robotControl.sampleSkyStoneRed(tfod);
                    if (!result.isEmpty()) {
                        break;
                    }
                }
            }

            telemetry.addData("result", result);
            telemetry.update();

            if (tfod != null) {
                tfod.shutdown();
            }

            Trajectory toLeft = drive.trajectoryBuilder()
                    .strafeTo(new Vector2d(-46.5 + 4, -33 + 1))
                    .build();

            Trajectory toCenter = drive.trajectoryBuilder()
                    .strafeTo(new Vector2d(-40 + 4, -33))
                    .build();

            Trajectory toRight = drive.trajectoryBuilder()
                    .strafeTo(new Vector2d(-33.5 + 5, -33))
                    .build();






            switch(result) {
                case "center":
                    Trajectory outCenter = drive.trajectoryBuilder()
                            .strafeLeft(5)
                            .back(50 + 8 - 11)
                            .build();
                    Trajectory backCenter = drive.trajectoryBuilder()
                            .forward(50 + 8 + 19 - 11)
                            .build();
                    Trajectory strafeInCenter = drive.trajectoryBuilder()
                            .strafeRight(6)
                            .build();
                    Trajectory strafeInFurtherCenter = drive.trajectoryBuilder()
                            .strafeRight(8)
                            .build();
                    Trajectory strafeOutCenter = drive.trajectoryBuilder()
                            .strafeLeft(6.75)
                            .build();
                    Trajectory strafeOutFurtherCenter = drive.trajectoryBuilder()
                            .strafeLeft(8.5)
                            .build();
                    Trajectory out2Center = drive.trajectoryBuilder()
                            .back(50 + 8 + 19 - 11)
                            .build();
                    Trajectory backCenterStoneThree = drive.trajectoryBuilder()
                            .forward(52 - 17 - 6)
                            .build();
                    Trajectory out3Center = drive.trajectoryBuilder()
                            .back(52 - 17 - 6)
                            .build();
                    Trajectory parkCenter = drive.trajectoryBuilder()
                            .forward(20 - 11)
                            .build();

                    // Move to stone
                    drive.followTrajectorySync(toCenter);
                    drive.setStone(stoneDown);
                    sleep(200);
//                    // Pull stone out and move it to the other side
                    drive.followTrajectorySync(outCenter);
                    drive.setStone(stoneUp);
//                    sleep(300);
//                    // Go back to the blocks (next set)
                    drive.followTrajectorySync(backCenter);
                    sleep(100);
                    drive.turnSync(Math.toRadians(-4));
                    sleep(100);
                    drive.followTrajectorySync(strafeInFurtherCenter);
                    drive.setStone(stoneDown);
                    sleep(200);
//                    // Move block to other side
                    drive.followTrajectorySync(strafeOutFurtherCenter);
                    drive.followTrajectorySync(out2Center);
                    drive.setStone(stoneUp);
                    sleep(200);
                    drive.followTrajectorySync(backCenterStoneThree);
                    drive.followTrajectorySync(strafeInFurtherCenter);
                    drive.setStone(stoneDown);
                    sleep(200);
                    drive.followTrajectorySync(strafeOutFurtherCenter);
                    drive.followTrajectorySync(out3Center);
                    drive.setStone(stoneUp);
                    sleep(200);
//                    // Park
                    drive.followTrajectorySync(parkCenter);
                    break;
                case "right":
                    // Get block
                    Trajectory outRight = drive.trajectoryBuilder()
                            .strafeLeft(5)
                            .back(50 - 11)
                            .build();
                    // Drop block
                    Trajectory backRight = drive.trajectoryBuilder()
                            .forward(52 + 19 - 12)
                            .build();
                    Trajectory strafeInRight = drive.trajectoryBuilder()
                            .strafeRight(6)
                            .build();
                    Trajectory strafeInFurtherRight = drive.trajectoryBuilder()
                            .strafeRight(8.5)
                            .build();
                    // Get block
                    Trajectory strafeOutRight = drive.trajectoryBuilder()
                            .strafeLeft(6.5)
                            .build();
                    Trajectory strafeOutFurtherRight = drive.trajectoryBuilder()
                            .strafeLeft(9)
                            .build();
                    Trajectory out2Right = drive.trajectoryBuilder()
                            .back(53 + 19 - 12.5)
                            .build();
                    Trajectory backRightStoneThree = drive.trajectoryBuilder()
                            .forward(52 - 10)
                            .build();
                    Trajectory out3Right = drive.trajectoryBuilder()
                            .back(52 - 9)
                            .build();
                    // Drop block
                    Trajectory parkRight = drive.trajectoryBuilder()
                            .forward(20 - 11)
                            .build();


                    drive.followTrajectorySync(toRight);
                    drive.setStone(stoneDown);
                    sleep(200);
                    drive.followTrajectorySync(outRight);
                    drive.setStone(stoneUp);
                    sleep(200);
                    drive.followTrajectorySync(backRight);
                    sleep(100);
                    drive.turnSync(Math.toRadians(-4));
                    sleep(100);
                    drive.followTrajectorySync(strafeInFurtherRight);
                    drive.setStone(stoneDown);
                    sleep(200);
                    drive.followTrajectorySync(strafeOutFurtherRight);
                    drive.followTrajectorySync(out2Right);
                    drive.setStone(stoneUp);
                    sleep(200);
                    drive.followTrajectorySync(backRightStoneThree);
                    sleep(100);
                    drive.turnSync(Math.toRadians(-4));
                    sleep(100);
                    drive.followTrajectorySync(strafeInFurtherRight);
                    drive.setStone(stoneDown);
                    sleep(200);
                    drive.followTrajectorySync(strafeOutFurtherRight);
                    drive.followTrajectorySync(out3Right);
                    drive.setStone(stoneUp);
                    drive.followTrajectorySync(parkRight);
                    break;
                case "left":
                    // Get block
                    Trajectory outLeft = drive.trajectoryBuilder()
                            .strafeLeft(5)
                            .back(50 + 16 - 11)
                            .build();
                    // Drop block
                    Trajectory backLeft = drive.trajectoryBuilder()
                            .forward(52 - 9 - 11)
                            .build();
                    Trajectory strafeInLeft = drive.trajectoryBuilder()
                            .strafeRight(6.3)
                            .build();
                    Trajectory strafeInFurtherLeft = drive.trajectoryBuilder()
                            .strafeRight(8.5)
                            .build();
                    // Get block
                    Trajectory strafeOutLeft = drive.trajectoryBuilder()
                            .strafeLeft(6.5)
                            .build();
                    Trajectory strafeOutFurtherLeft = drive.trajectoryBuilder()
                            .strafeLeft(8.7)
                            .build();
                    Trajectory out2Left = drive.trajectoryBuilder()
                            .back(52 - 9 - 11)
                            .build();
                    Trajectory backLeftStoneThree = drive.trajectoryBuilder()
                            .forward(52 - 9 - 3)
                            .build();
                    Trajectory out3Left = drive.trajectoryBuilder()
                            .back(52 - 9 - 4)
                            .build();
                    // Drop block
                    Trajectory parkLeft = drive.trajectoryBuilder()
                            .forward(20 - 11)
                            .build();


                    // Move to stone
                    drive.followTrajectorySync(toLeft);
                    drive.setStone(stoneDown);
                    sleep(200);
                    // Pull stone out and move it to the other side
                    drive.followTrajectorySync(outLeft);
                    drive.setStone(stoneUp);
                    sleep(300);
                    // Go back to the blocks (next set)
                    drive.followTrajectorySync(backLeft);
                    sleep(100);
                    drive.turnSync(Math.toRadians(-4));
                    sleep(100);
                    drive.followTrajectorySync(strafeInFurtherLeft);
                    drive.setStone(stoneDown);
                    sleep(200);
//                    // Move block to other side
                    drive.followTrajectorySync(strafeOutFurtherLeft);
                    drive.followTrajectorySync(out2Left);
                    drive.setStone(stoneUp);
                    sleep(200);
                    drive.followTrajectorySync(backLeftStoneThree);
                    sleep(100);
                    drive.turnSync(Math.toRadians(-4));
                    sleep(100);
                    drive.followTrajectorySync(strafeInFurtherLeft);
                    drive.setStone(stoneDown);
                    sleep(200);
                    drive.followTrajectorySync(strafeOutFurtherLeft);
                    drive.followTrajectorySync(out3Left);
                    drive.setStone(stoneUp);
//                    // Park
                    drive.followTrajectorySync(parkLeft);
                    break;
                default:
                    break;
            }
//            sleep(5000);
        }


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
        if(dashboard != null) {
            dashboard.startCameraStream(vuforia, 0);
        }
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
//        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        tfod.loadModelFromFile("/sdcard/FIRST/Skystone.tflite", LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public boolean running() {
        return opModeIsActive() && !isStopRequested();
    }
}