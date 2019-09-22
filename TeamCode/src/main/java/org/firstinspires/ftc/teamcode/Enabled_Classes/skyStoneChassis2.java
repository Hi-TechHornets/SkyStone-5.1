package org.firstinspires.ftc.teamcode.Enabled_Classes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp

public class skyStoneChassis2 extends LinearOpMode {
    private robotControl hth3;

    public float x, y, z, w, pwr;
    public static double deadzone = 0.2;

    public void runOpMode() {
        hth3 = new robotControl();

        hth3.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();

            getJoyVals();

            pwr = y;

            hth3.rightFront.setPower(Range.clip(pwr - x + z, -1, 1));
            hth3.leftRear.setPower(Range.clip(pwr - x - z, -1, 1));
            hth3.leftFront.setPower(Range.clip(pwr + x - z, -1, 1));
            hth3.rightRear.setPower(Range.clip(pwr + x + z, -1, 1));
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
