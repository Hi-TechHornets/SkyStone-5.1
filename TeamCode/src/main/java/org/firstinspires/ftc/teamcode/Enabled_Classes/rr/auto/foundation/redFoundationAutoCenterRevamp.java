package org.firstinspires.ftc.teamcode.Enabled_Classes.rr.auto.foundation;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Enabled_Classes.rr.drive.mecanumDriveBase;
import org.firstinspires.ftc.teamcode.Enabled_Classes.rr.drive.mecanumDriveREV;

@Autonomous(group="drive")
@Config
@Disabled
public class redFoundationAutoCenterRevamp extends LinearOpMode {
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

        drive.setPoseEstimate(new Pose2d(35, -61, Math.PI + Math.PI));

        TrajectoryBuilder trajectoryBuilder = drive.trajectoryBuilder();


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

        Trajectory backup = drive.trajectoryBuilder().back(33).strafeTo(new Vector2d(18, -32)).back(18).build();
        drive.followTrajectorySync(backup);



//        drive.followTrajectorySync(
//                drive.trajectoryBuilder()
//                        .splineTo(new Pose2d(30, 30, 0))
//                        .build()
//        );
    }
}
