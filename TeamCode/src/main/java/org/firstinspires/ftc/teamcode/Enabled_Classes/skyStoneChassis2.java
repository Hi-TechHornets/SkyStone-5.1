package org.firstinspires.ftc.teamcode.Enabled_Classes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp
//@Disabled
public class skyStoneChassis2 extends LinearOpMode {
    private robotControl hth3;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double GRIPPER_MIN = 20.0;
    public static double GRIPPER_MAX = 120.0;

    public static double motorSpeed = 0.4;

    public float x, y, z, w, pwr;
    public static double deadzone = 0.2;

    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        hth3 = new robotControl();

        hth3.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Status", "Running");

            getJoyVals();

            pwr = y;

            hth3.rightFront.setPower(Range.clip(pwr - x + z, -1, 1));
            hth3.leftRear.setPower(Range.clip(pwr - x - z, -1, 1));
            hth3.leftFront.setPower(Range.clip(pwr + x - z, -1, 1));
            hth3.rightRear.setPower(Range.clip(pwr + x + z, -1, 1));

            hth3.gripperRotate.setPower((gamepad1.left_trigger>0.5) ? -motorSpeed : (gamepad1.left_bumper) ? motorSpeed : 0);

            if(gamepad1.right_bumper) hth3.gripperServo.setPosition(GRIPPER_MIN / 255.0);
            if(gamepad1.right_trigger>0.5) hth3.gripperServo.setPosition(GRIPPER_MAX / 255.0);
            telemetry.addData("servo", hth3.gripperServo.getPosition());
            telemetry.update();
        }
    }

    private void getJoyVals() {
        y = gamepad1.left_stick_y;
        x = -gamepad1.left_stick_x;
        z = gamepad1.right_stick_x;
        w = gamepad1.right_stick_y;

        if(Math.abs(-x) < deadzone) x = 0;
        if(Math.abs(y) < deadzone) y = 0;
        if(Math.abs(z) < deadzone) z = 0;
        if(Math.abs(w) < 0.9) w = 0;
    }
}
