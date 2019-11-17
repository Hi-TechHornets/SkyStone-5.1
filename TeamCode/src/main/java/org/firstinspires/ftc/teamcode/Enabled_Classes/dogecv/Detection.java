package org.firstinspires.ftc.teamcode.Enabled_Classes.dogecv;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.disnodeteam.dogecv.scoring.PerfectAreaScorer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Enabled_Classes.robotControl;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Locale;

@Autonomous
public class Detection extends LinearOpMode {
    private robotControl hth3;
    private FtcDashboard dashboard;
    private OpenCvCamera webcam;
    private SkystoneDetector skyStoneDetector;

    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(dashboard.getTelemetry(), telemetry);

        hth3 = new robotControl();
        hth3.init(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();

        skyStoneDetector = new SkystoneDetector();
//        skyStoneDetector.addScorer(new RatioScorer(1.25, 3));
//        skyStoneDetector.addScorer(new MaxAreaScorer(0.01));
        skyStoneDetector.addScorer(new PerfectAreaScorer(5000, 0.05));

        webcam.setPipeline(skyStoneDetector);

        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        if(running()) {
            hth3.moveDriveMotors(-0.2);
            while(running() && skyStoneDetector.foundRectangle().width < 300) {
                doTelemetry();
            }
            hth3.halt();
        }
    }

    private boolean running() {
        return opModeIsActive() && !isStopRequested();
    }

    private void doTelemetry() {
        telemetry.addData("Stone Position X", skyStoneDetector.getScreenPosition().x);
        telemetry.addData("Stone Position Y", skyStoneDetector.getScreenPosition().y);
        telemetry.addData("Stone Width", skyStoneDetector.foundRectangle().width);
        telemetry.addData("Stone Height", skyStoneDetector.foundRectangle().height);
        telemetry.addData("Frame Count", webcam.getFrameCount());
        telemetry.addData("FPS", String.format(Locale.US, "%.2f", webcam.getFps()));
        telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
        telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
        telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
        telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
        telemetry.update();
    }
}
