/*
package org.firstinspires.ftc.teamcode.Enabled_Classes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@Autonomous
@Config
public class testAuto extends LinearOpMode {
    public robotControl robot;
    public FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double stop = 0.5;
    public static double forward = 1.0;
    public static double backward = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        robot = new robotControl();
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if(running()) robot.cr.setPosition(stop);
        wait(5000);
        if(running()) robot.cr.setPosition(forward);
        wait(5000);
        if(running()) robot.cr.setPosition(backward);
        wait(5000);
        if(running()) robot.cr.setPosition(stop);
        wait(5000);
    }

    public boolean running() {
        return opModeIsActive() && !isStopRequested();
    }

    public void wait(int time) {
        robot.resetTimer();
        while(robot.getTime() < time && running()) {
        }
    }
}
*/