package org.firstinspires.ftc.teamcode.Enabled_Classes.rr.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Enabled_Classes.rr.drive.mecanumDriveBase;
import org.firstinspires.ftc.teamcode.Enabled_Classes.rr.drive.mecanumDriveREV;

@Config
@Autonomous(group = "drive")
//@Disabled
public class StraightTest extends LinearOpMode {
    public static double DISTANCE = 30;

    @Override
    public void runOpMode() throws InterruptedException {
        mecanumDriveBase drive = new mecanumDriveREV(hardwareMap);

        Trajectory trajectory = drive.trajectoryBuilder()
                .forward(DISTANCE)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(trajectory);
    }
}
