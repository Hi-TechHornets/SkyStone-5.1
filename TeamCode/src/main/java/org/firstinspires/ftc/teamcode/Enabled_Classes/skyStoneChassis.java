package org.firstinspires.ftc.teamcode.Enabled_Classes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Disabled
public class skyStoneChassis extends LinearOpMode {
    private robotControl hth3;

    public void runOpMode() {
        hth3 = new robotControl();

        hth3.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();

            hth3.moveDriveMotors(gamepad1.left_stick_y, gamepad1.right_stick_y);

            if(gamepad1.dpad_right) {
                hth3.moveDriveMotors(1.0, -1.0, -1.0, 1.0);
            }

            if(gamepad1.dpad_left) {
                hth3.moveDriveMotors(-1.0, 1.0, 1.0, -1.0);
            }
        }
    }
}
