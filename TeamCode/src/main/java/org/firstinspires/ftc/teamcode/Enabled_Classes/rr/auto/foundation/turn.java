package org.firstinspires.ftc.teamcode.Enabled_Classes.rr.auto.foundation;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Enabled_Classes.rr.drive.mecanumDriveBase;
import org.firstinspires.ftc.teamcode.Enabled_Classes.rr.drive.mecanumDriveREV;

@Autonomous(group="drive")
@Config
@Disabled
public class turn extends LinearOpMode {
    public static double foundationUp = 0.2;
    public static double foundationDown = 1;

    public static double foundationHeight = 53.0;
    public static double foundationPullOut = 70.0;

    @Override
    public void runOpMode() throws InterruptedException {
        mecanumDriveREV drive = new mecanumDriveREV(hardwareMap);
//        drive.invertMotors();
        drive.resetEncoders();

        waitForStart();

        if (isStopRequested()) return;

        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

        sleep(5000);

        drive.turnSync(-(drive.getRawExternalHeading()));
        drive.turnSync(-(drive.getRawExternalHeading()));
        drive.turnSync(-(drive.getRawExternalHeading()));
        drive.turnSync(-(drive.getRawExternalHeading()));
        drive.turnSync(-(drive.getRawExternalHeading()));


    }
}
