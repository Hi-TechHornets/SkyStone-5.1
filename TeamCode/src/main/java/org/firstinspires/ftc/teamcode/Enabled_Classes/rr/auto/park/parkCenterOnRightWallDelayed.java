package org.firstinspires.ftc.teamcode.Enabled_Classes.rr.auto.park;

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
import org.firstinspires.ftc.teamcode.Enabled_Classes.rr.drive.mecanumDriveBase;
import org.firstinspires.ftc.teamcode.Enabled_Classes.rr.drive.mecanumDriveREV;
import org.firstinspires.ftc.teamcode.R;

@Autonomous
public class parkCenterOnRightWallDelayed extends LinearOpMode {
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double stoneUp = 0.85;
    public static double stoneDown = 0.2;

    @Override
    public void runOpMode() {
        mecanumDriveBase drive = new mecanumDriveREV(hardwareMap);
        drive.resetEncoders();
        drive.setStone(stoneUp);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        if (running()) {
            drive.setPoseEstimate(new Pose2d(-39.5, 63));
            Trajectory forward = drive.trajectoryBuilder()
                    .forward(32)
                    .build();
            Trajectory strafe = drive.trajectoryBuilder()
                    .strafeLeft(26)
                    .build();
            sleep(20);
            drive.followTrajectorySync(strafe);
            drive.followTrajectorySync(forward);
        }
    }

    public boolean running() {
        return opModeIsActive() && !isStopRequested();
    }
}
