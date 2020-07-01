package org.firstinspires.ftc.teamcode.Enabled_Classes.rr.auto.foundation;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Enabled_Classes.rr.drive.mecanumDriveBase;
import org.firstinspires.ftc.teamcode.Enabled_Classes.rr.drive.mecanumDriveREV;

@Autonomous(group="foundation")
@Config
public class redFoundationCenter extends LinearOpMode {
    public static double foundationUp = 0.2;
    public static double foundationDown = 1;

    public static double foundationHeight = 55.0;
    public static double foundationPullOut = 70.0;

    @Override
    public void runOpMode() throws InterruptedException {
        mecanumDriveREV drive = new mecanumDriveREV(hardwareMap);
//        drive.invertMotors();
        drive.resetEncoders();
        drive.setFoundationRange(foundationUp, foundationDown);
        drive.setFoundation(foundationUp);
        drive.setStone(0.9);

        waitForStart();

        if (isStopRequested()) return;

        drive.setPoseEstimate(new Pose2d(35, -61, Math.toRadians(0)));

        //Goes to foundation
        Trajectory toFound = drive.trajectoryBuilder()
                .strafeTo(new Vector2d(foundationHeight, -33))
                .strafeLeft(4)
                .build();

        drive.followTrajectorySync(toFound);

        drive.turnSync(-(drive.getRawExternalHeading()));
        drive.turnSync(-(drive.getRawExternalHeading()));

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

        drive.turnSync(-(drive.getRawExternalHeading()));
        drive.turnSync(-(drive.getRawExternalHeading()));

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
