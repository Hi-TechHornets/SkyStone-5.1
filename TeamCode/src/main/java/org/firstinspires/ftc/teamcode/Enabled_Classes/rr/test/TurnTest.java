package org.firstinspires.ftc.teamcode.Enabled_Classes.rr.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Enabled_Classes.rr.drive.mecanumDriveBase;
import org.firstinspires.ftc.teamcode.Enabled_Classes.rr.drive.mecanumDriveREV;
import org.firstinspires.ftc.teamcode.Enabled_Classes.rr.drive.mecanumDriveREVOptimized;

@Config
@Autonomous(group="drive")
@Disabled
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 90;

    @Override
    public void runOpMode() throws InterruptedException {
        mecanumDriveBase drive = new mecanumDriveREV(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.turnSync(Math.toRadians(ANGLE));
    }
}
