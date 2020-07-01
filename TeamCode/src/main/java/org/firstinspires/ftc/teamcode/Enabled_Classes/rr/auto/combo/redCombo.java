package org.firstinspires.ftc.teamcode.Enabled_Classes.rr.auto.combo;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
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

@Autonomous(group = "combo")
//@Disabled
public class redCombo extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private static final String VUFORIA_KEY =
            "AQgDy6j/////AAABmXISaVgPmUMmugAGhFFaTDBJsE9lRd9NNkRieex8wD8vq5MTahYK2UeRXAODvLE6NsZ/89ckvXlkMgAgRqlP/se9OTW0er/VNUu0M/eCAGXWgs0ji6it3/MAVY5tC4T5h/Inzn1NZQDa0XYMzCE2z5ionyuw/Vwk6rcqOsVqpxOYsLGMvcE++a64J8WJBoQHR9zSnStw++AarPQgqL0z9bQFbpopm2bw09FyY47tp4YvKl1kyYmPt0zbToit+D+r+WWQF0/1k1ikMaUt4SOhTRXHz/ZrzRW1jtnJOkYiCJ1qsxH8dH+2sS3dtrX1hzIZ+mKu4o+x1jsYClXc0P9p5x7bR1HGOH/cnsKLiBBly2QW";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double foundationHeight = 55.0;

    public static double foundationUp = 0.2;
    public static double foundationDown = 1;

    public static double stoneUp = 0.85;
    public static double stoneDown = 0.2;

    @Override
    public void runOpMode() {
        mecanumDriveREV drive = new mecanumDriveREV(hardwareMap);
        drive.resetEncoders();
        drive.setStone(stoneUp);
        drive.setFoundation(foundationUp);
        
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
                    .strafeTo(new Vector2d(-46.5 + 5, -33 + 1))
                    .build();

            Trajectory toCenter = drive.trajectoryBuilder()
                    .strafeTo(new Vector2d(-40 + 5, -33))
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
                            .forward(50 + 8 + 19 - 10)
                            .build();
                    Trajectory strafeInCenter = drive.trajectoryBuilder()
                            .strafeRight(7)
                            .build();
                    Trajectory strafeOutCenter = drive.trajectoryBuilder()
                            .strafeLeft(7.75)
                            .build();
                    Trajectory out2Center = drive.trajectoryBuilder()
                            .back(50 + 8 + 19 - 11)
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
                    drive.followTrajectorySync(strafeInCenter);
                    drive.setStone(stoneDown);
                    sleep(200);
//                    // Move block to other side
                    drive.followTrajectorySync(strafeOutCenter);
                    drive.followTrajectorySync(out2Center);
                    drive.setStone(stoneUp);
                    sleep(200);
//                    // Park
//                    drive.followTrajectorySync(parkCenter);
                    break;
                case "right":
                    // Get block
                    Trajectory outRight = drive.trajectoryBuilder()
                            .strafeLeft(5)
                            .back(50 - 11)
                            .build();
                    // Drop block
                    Trajectory backRight = drive.trajectoryBuilder()
                            .forward(52 + 19 - 11)
                            .build();
                    Trajectory strafeInRight = drive.trajectoryBuilder()
                            .strafeRight(6)
                            .build();
                    // Get block
                    Trajectory strafeOutRight = drive.trajectoryBuilder()
                            .strafeLeft(6.5)
                            .build();
                    Trajectory out2Right = drive.trajectoryBuilder()
                            .back(53 + 19 - 11)
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
                    drive.followTrajectorySync(strafeInRight);
                    drive.setStone(stoneDown);
                    sleep(200);
                    drive.followTrajectorySync(strafeOutRight);
                    drive.followTrajectorySync(out2Right);
                    drive.setStone(stoneUp);
                    sleep(200);
//                    drive.followTrajectorySync(parkRight);
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
                    // Get block
                    Trajectory strafeOutLeft = drive.trajectoryBuilder()
                            .strafeLeft(6.5)
                            .build();
                    Trajectory out2Left = drive.trajectoryBuilder()
                            .back(52 - 9 - 11)
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
                    drive.followTrajectorySync(strafeInLeft);
                    drive.setStone(stoneDown);
                    sleep(200);
//                    // Move block to other side
                    drive.followTrajectorySync(strafeOutLeft);
                    drive.followTrajectorySync(out2Left);
                    drive.setStone(stoneUp);
                    sleep(200);
//                    // Park
//                    drive.followTrajectorySync(parkLeft);
                    break;
                default:
                    break;
            }
//            sleep(5000);
            drive.turnSync(Math.toRadians(180) + drive.getRawExternalHeading());
            drive.turnSync(Math.toRadians(180) - drive.getRawExternalHeading());
            drive.turnSync(Math.toRadians(180) - drive.getRawExternalHeading());

            Trajectory toFoundation = drive.trajectoryBuilder()
                    .strafeTo(new Vector2d(foundationHeight, -33))
                    .strafeLeft(8)
                    .build();

            drive.followTrajectorySync(toFoundation);



            drive.turnSync(Math.toRadians(180) - drive.getRawExternalHeading());
            drive.turnSync(Math.toRadians(180) - drive.getRawExternalHeading());

            drive.setFoundation(foundationDown);
            drive.setFoundationRange(foundationDown - 0.01, foundationDown);
            sleep(300);

            drive.setConstraints(new DriveConstraints(20.0, 20.0, 0.0,
                    Math.toRadians(180.0), Math.toRadians(180.0), 0.0));

            //Pulls the foundation back
            drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Math.toRadians(180)));
            Trajectory pullFound = drive.trajectoryBuilder()
                    .splineTo(new Pose2d(foundationHeight + 2, -78, Math.toRadians(-90)), new ConstantInterpolator(Math.toRadians(9)))
                    .build();
            drive.followTrajectorySync(pullFound);

            drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Math.toRadians(0)));
            drive.resetConstraints();

            drive.setFoundationRange(foundationUp, foundationDown);
            drive.setFoundation(foundationUp);

            drive.turnSync(Math.toRadians(180) + drive.getRawExternalHeading());
            drive.turnSync(Math.toRadians(180) + drive.getRawExternalHeading());

            //Parks robot on tape
            Trajectory park1 = drive.trajectoryBuilder()
                    .strafeLeft(3)
                    .back(30)
                    .build();
            drive.followTrajectorySync(park1);


            Trajectory park2 = drive.trajectoryBuilder()
                    .strafeLeft(18)
                    .forward(22)
                    .build();
            drive.followTrajectorySync(park2);

            Trajectory park3 = drive.trajectoryBuilder()
                    .strafeRight(2)
                    .back(22)
                    .build();
            drive.followTrajectorySync(park3);
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