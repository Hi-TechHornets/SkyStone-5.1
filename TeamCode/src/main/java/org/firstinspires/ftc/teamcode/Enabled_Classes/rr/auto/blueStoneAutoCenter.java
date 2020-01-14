package org.firstinspires.ftc.teamcode.Enabled_Classes.rr.auto;

/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Enabled_Classes.robotControl;
import org.firstinspires.ftc.teamcode.Enabled_Classes.rr.drive.mecanumDriveBase;
import org.firstinspires.ftc.teamcode.Enabled_Classes.rr.drive.mecanumDriveREV;
import org.firstinspires.ftc.teamcode.R;

/**
 * This 2019-2020 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Skystone game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(group = "drive")
//@Disabled
public class blueStoneAutoCenter extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
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

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double stoneUp = 0.85;
    public static double stoneDown = 0.2;

    @Override
    public void runOpMode() {
        mecanumDriveBase drive = new mecanumDriveREV(hardwareMap);
        drive.resetEncoders();
        drive.setStone(stoneUp);
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

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

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (running()) {
            String result = "";
            drive.setPoseEstimate(new Pose2d(-39.5, 63));
            Trajectory toStone = drive.trajectoryBuilder()
                                      .strafeTo(new Vector2d(-30, 46))
                                      .build();
            drive.followTrajectorySync(toStone);
            sleep(100);
            drive.turnSync(Math.toRadians(-7));

            if (tfod != null) {
                while(running()) {
                    result = robotControl.sampleSkyStone(tfod);
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
                    .strafeTo(new Vector2d(-25, 33))
                    .build();

            Trajectory toCenter = drive.trajectoryBuilder()
                    .strafeTo(new Vector2d(-34, 33))
                    .build();

            Trajectory toRight = drive.trajectoryBuilder()
                    .strafeTo(new Vector2d(-42, 33))
                    .build();







            switch(result) {
                case "left":
                    // Get block
                    Trajectory outLeft = drive.trajectoryBuilder()
                            .strafeLeft(5)
                            .forward(50)
                            .build();
                    // Drop block
                    Trajectory backLeft = drive.trajectoryBuilder()
                            .back(53 + 19)
                            .build();
                    Trajectory strafeInLeft = drive.trajectoryBuilder()
                            .strafeRight(5)
                            .build();
                    // Get block
                    Trajectory strafeOutLeft = drive.trajectoryBuilder()
                            .strafeLeft(5)
                            .build();
                    Trajectory out2Left = drive.trajectoryBuilder()
                            .forward(53 + 19)
                            .build();
                    // Drop block
                    Trajectory parkLeft = drive.trajectoryBuilder()
                            .back(25)
                            .build();


                    drive.followTrajectorySync(toLeft);
                    drive.setStone(stoneDown);
                    sleep(300);
                    drive.followTrajectorySync(outLeft);
                    drive.setStone(stoneUp);
                    sleep(300);
                    drive.followTrajectorySync(backLeft);
                    drive.followTrajectorySync(strafeInLeft);
                    drive.setStone(stoneDown);
                    sleep(300);
                    drive.followTrajectorySync(strafeOutLeft);
                    drive.followTrajectorySync(out2Left);
                    drive.setStone(stoneUp);
                    sleep(300);
                    drive.followTrajectorySync(parkLeft);
                    break;
                case "center":
                    Trajectory outCenter = drive.trajectoryBuilder()
                            .strafeLeft(5)
                            .forward(55)
                            .build();
                    Trajectory backCenter = drive.trajectoryBuilder()
                            .back(58 + 19)
                            .strafeRight(15)
                            .build();
                    Trajectory out2Center = drive.trajectoryBuilder()
                            .strafeLeft(6)
                            .forward(55 + 19 + 19)
                            .build();
                    Trajectory parkCenter = drive.trajectoryBuilder()
                            .back(28)
                            .build();

                    // Move to stone
                    drive.followTrajectorySync(toCenter);
                    drive.setStone(stoneDown);
                    sleep(300);
                    // Pull stone out and move it to the other side
                    drive.followTrajectorySync(outCenter);
                    drive.setStone(stoneUp);
                    sleep(300);
                    // Go back to the blocks (next set)
                    drive.followTrajectorySync(backCenter);
                    drive.setStone(stoneDown);
                    sleep(300);
                    // Move block to other side
                    drive.followTrajectorySync(out2Center);
                    drive.setStone(stoneUp);
                    sleep(300);
                    // Park
                    drive.followTrajectorySync(parkCenter);
                    break;
                case "right":
                    drive.followTrajectorySync(toRight);
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
//        dashboard.startCameraStream(vuforia, 0);
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