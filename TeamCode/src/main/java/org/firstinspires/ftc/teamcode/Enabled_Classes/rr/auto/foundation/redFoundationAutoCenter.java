package org.firstinspires.ftc.teamcode.Enabled_Classes.rr.auto.foundation;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.PathBuilder;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryConfig;
import com.acmerobotics.roadrunner.trajectory.TrajectoryLoader;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Enabled_Classes.rr.drive.mecanumDriveBase;
import org.firstinspires.ftc.teamcode.Enabled_Classes.rr.drive.mecanumDriveREV;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;

@Autonomous(group="drive")
@Config
public class redFoundationAutoCenter extends LinearOpMode {
    public static double foundationUp = 0.3;
    public static double foundationDown = 1;

    public static double foundationHeight = 50.0;
    public static double foundationPullOut = -70.0;

    @Override
    public void runOpMode() throws InterruptedException {
        mecanumDriveBase drive = new mecanumDriveREV(hardwareMap);
        drive.resetEncoders();
        drive.setFoundation(foundationUp);

        waitForStart();

        if (isStopRequested()) return;

//        try {
//            trajectory = load("strafing");
//            drive.followTrajectorySync(trajectory);
//        } catch (IOException ignored) {}
        drive.setPoseEstimate(new Pose2d(35, -61));
        Trajectory traj1 = drive.trajectoryBuilder()
                                .strafeTo(new Vector2d(foundationHeight, -33))
                                .strafeLeft(4)
                                .build();

        drive.followTrajectorySync(traj1);

        drive.setFoundation(foundationDown);
        sleep(500);

        Trajectory traj2 = drive.trajectoryBuilder()
                                .strafeTo(new Vector2d(foundationHeight, foundationPullOut))
                                .build();
        drive.followTrajectorySync(traj2);

        drive.setFoundation(foundationUp);
        sleep(500);

        Trajectory reset = drive.trajectoryBuilder()
                                .strafeTo(new Vector2d(80, -80))
                                .build();
        drive.followTrajectorySync(reset);

        drive.setPoseEstimate(new Pose2d(61, -61));

        Trajectory push = drive.trajectoryBuilder().back(41).strafeTo(new Vector2d(25, -45)).forward(15).build();
//        Trajectory backup = drive.trajectoryBuilder().back(33).strafeTo(new Vector2d(18, -32)).back(18).build();
        drive.followTrajectorySync(push);

        drive.setPoseEstimate(new Pose2d(28.5, -45));
        Trajectory backup = drive.trajectoryBuilder().strafeLeft(7).back(28.5).build();
        drive.followTrajectorySync(backup);




//        drive.followTrajectorySync(
//                drive.trajectoryBuilder()
//                        .splineTo(new Pose2d(30, 30, 0))
//                        .build()
//        );
    }
}
