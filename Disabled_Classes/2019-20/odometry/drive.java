package org.firstinspires.ftc.teamcode.Enabled_Classes.odometry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "EncoderDrive")
@Disabled
public class drive extends LinearOpMode {
    static final double threshold = 0.3;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private ElapsedTime runtime = new ElapsedTime();
    private double rightx = 0;
    private double leftx = 0;
    private double lefty = 0;
    private robot bb = new robot();
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        bb.init(hardwareMap);
        telemetry.update();
        runtime.reset();
        bb.resetTicks();
        telemetry.addData("status", "initialized");
        telemetry.update();
        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData("Left ticks", bb.getLeftTicks());
            telemetry.addData("Left distance", (bb.getLeftTicks() / bb.oneRotationTicks) * 2.0 * Math.PI * bb.wheelRadius);
            telemetry.addData("Center ticks", bb.getCenterTicks());
            telemetry.addData("Right ticks", bb.getRightTicks());
            telemetry.addData("X value", bb.getX());
            telemetry.addData("Y value", bb.getY());
            telemetry.addData("Theta value", bb.getTheta());
            telemetry.update();
            lefty = Range.clip(gamepad1.left_stick_y, -1, 1);
            rightx = Range.clip(gamepad1.right_stick_x, -1, 1);
            leftx = Range.clip(gamepad1.left_stick_x, -1, 1);
            if (Math.abs(gamepad1.left_stick_y) > threshold) {
                bb.drive(-lefty, -lefty, -lefty, -lefty);
            } else if (Math.abs(gamepad1.left_stick_x) > threshold) {
                bb.drive(-leftx, leftx, leftx, -leftx);
            }  else if (Math.abs(gamepad1.right_stick_x) > threshold) {
                bb.drive(-rightx, -rightx, rightx, rightx);
            } else {
                bb.drive(0, 0, 0, 0);
            }
            idle();
        }
        bb.drive(0, 0, 0, 0);
    }
    public double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
}
