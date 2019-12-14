package org.firstinspires.ftc.teamcode.Enabled_Classes.distanceAuto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Enabled_Classes.robotControl;

@Autonomous(name="Tape Auto")
//@Config
@Disabled
public class tapeAuto extends LinearOpMode{
    public robotControl hth3;
    public FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double forward = -1.0;
    public static double backward = 1.0;
    public static double lockMin = 0.5;
    public static double lockMax = 0.15;

    @Override
    //45 Degree Angle = 1000 ms at 0.5 power
    //90 Degree Angle = 2000 ms at 0.5 power

    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        hth3 = new robotControl();
        hth3.init(hardwareMap);

        hth3.lock.setPosition(lockMax);

        telemetry.addData("Status", "Initialized");

        telemetry.update();

        waitForStart();

        //Start of auto period
        //Move robot over to tape
        forward();
        Thread.sleep(1500);
        stopRobot();
        hth3.lock.setPosition(lockMin);
        Thread.sleep(10000);

        telemetry.addData("Status", "Complete!");

    }

    /*
    public boolean running() {
        return opModeIsActive() && !isStopRequested();
    }

    public void wait(int time) {
        hth3.resetTimer();
        while(hth3.getTime() < time && running()) {
        }
    }
    */

    public void forward() {
        hth3.rightFront.setPower(forward);
        hth3.leftRear.setPower(forward);
        hth3.leftFront.setPower(forward);
        hth3.rightRear.setPower(forward);
    }
    public void backward() {
        hth3.rightFront.setPower(backward);
        hth3.leftRear.setPower(backward);
        hth3.leftFront.setPower(backward);
        hth3.rightRear.setPower(backward);
    }

    public void strafeRight() {
        hth3.rightFront.setPower(backward / 2);
        hth3.leftRear.setPower(backward / 2);
        hth3.leftFront.setPower(forward / 2);
        hth3.rightRear.setPower(forward / 2);
    }

    public void strafeLeft() {
        hth3.rightFront.setPower(forward / 2);
        hth3.leftRear.setPower(forward / 2);
        hth3.leftFront.setPower(backward / 2);
        hth3.rightRear.setPower(backward / 2);
    }

    public void right() {
        hth3.rightFront.setPower(backward / 2);
        hth3.leftRear.setPower(forward / 2);
        hth3.leftFront.setPower(forward / 2);
        hth3.rightRear.setPower(backward / 2);
    }

    public void left() {
        hth3.rightFront.setPower(forward / 2);
        hth3.leftRear.setPower(backward / 2);
        hth3.leftFront.setPower(backward / 2);
        hth3.rightRear.setPower(forward / 2);
    }

    public void stopRobot() {
        hth3.rightFront.setPower(0);
        hth3.leftRear.setPower(0);
        hth3.leftFront.setPower(0);
        hth3.rightRear.setPower(0);
    }
}
