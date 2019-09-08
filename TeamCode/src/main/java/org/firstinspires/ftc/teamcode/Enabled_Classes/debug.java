package org.firstinspires.ftc.teamcode.Enabled_Classes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.concurrent.TimeUnit;

@Autonomous
@Disabled
public class debug extends LinearOpMode {
    private robotControl charlie;

    @Override
    public void runOpMode() {
        charlie= new robotControl();
        charlie.init(hardwareMap);
        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            charlie.resetTimer();
            driveTime(2000, 1.0);
            //charlie.leftRear.setPower(1.0);
        }
    }

    private boolean passCheck() {
        return opModeIsActive() && !isStopRequested();
        // It is on AND not off
    }

    private long getTime() {
        return charlie.moveTimer.time(TimeUnit.MILLISECONDS);
    }

    private void driveTime(int time, double pwr) {
        charlie.resetTimer();
        while (getTime() < time && passCheck()) {
            charlie.moveDriveMotors(pwr);
            //charlie.leftRear.setPower(1.0);
            telemetry.addData("rightFront", charlie.rightFront.getPower());
            telemetry.addData("rightRear", charlie.rightRear.getPower());
            telemetry.addData("leftFront", charlie.leftFront.getPower());
            telemetry.addData("leftRear", charlie.leftRear.getPower());
            telemetry.update();
        }
        charlie.halt();
    }
}
