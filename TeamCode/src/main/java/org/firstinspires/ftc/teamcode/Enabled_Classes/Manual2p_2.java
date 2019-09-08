package org.firstinspires.ftc.teamcode.Enabled_Classes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static java.lang.Math.abs;
@Disabled
@TeleOp(name = "Rover Ruckus: DriveTeam OpMode 2", group = "Drive")

public class Manual2p_2 extends LinearOpMode {
    private robotControl charlie;
    private int mode;
    private int MUMBOJUMBO;
    private boolean swit;
    private boolean armSwit;
    private boolean armMode=false;
    public void runOpMode() {
        //ToDo #2
        mode = 1;
        charlie=new robotControl();
        charlie.init(hardwareMap);

        MUMBOJUMBO = 0;
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        swit=false;


        int sr=4;
        double right;
        double left;
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
            if (abs(gamepad1.left_stick_y) > 0.1) {
                charlie.moveDriveMotors(-gamepad1.left_stick_y);
            }
            else if (abs(gamepad1.right_stick_x) > 0.1) {
                charlie.moveDriveMotors(gamepad1.right_stick_x, -gamepad1.right_stick_x);
            }
            else {
                charlie.halt();
            }
        }
    }
    private void liftMagnet() {
        if (charlie.magnet.getState() && opModeIsActive()) {
            charlie.rack.setPower(-.5);
        }

    }
    private void changeModeWheel()
    {
        if(gamepad1.a) {
            swit=true;
        }
        else if(swit)
        {
            if(mode == 1)
            {
                mode = 2;
            }
            else {
                mode = 1;
            }
            swit=false;
        }

    }
    private void changeModeArm() {
        if (gamepad2.y) {
            armSwit = true;
        } else if (armSwit) {
            armMode=!armMode;
            armSwit = false;
        }
    }
    private void driveSwitch()
    {
        switch(mode)
        {
            case 1:
                charlie.moveDriveMotors(-gamepad1.left_stick_y,-gamepad1.right_stick_y);
                break;
            case 2:
                charlie.moveDriveMotors(gamepad1.right_stick_y,  gamepad1.left_stick_y);
        }
    }

}