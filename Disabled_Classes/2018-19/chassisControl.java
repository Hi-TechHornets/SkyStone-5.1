package org.firstinspires.ftc.teamcode.Enabled_Classes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Disabled
public class chassisControl extends LinearOpMode {
    private robotControl charlie;

    public void runOpMode() {
        charlie = new robotControl();
        charlie.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Running");

            //charlie.moveDriveMotors(gamepad1.left_stick_y,gamepad1.right_stick_y);

            if(gamepad1.dpad_right) {
                charlie.moveDriveMotors(-1,1,-1,1);
            }
            if(gamepad1.dpad_left) {
                charlie.moveDriveMotors(1,-1,1,-1);
            }
            else
            {
                charlie.moveDriveMotors(gamepad1.right_stick_y+gamepad1.right_stick_x,gamepad1.right_stick_y-gamepad1.right_stick_x);
            }

            telemetry.update();

        }
    }
}
