package org.firstinspires.ftc.teamcode.Enabled_Classes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Rover Ruckus: DriveTeam OpMode", group = "Drive")

public class Manual2p extends LinearOpMode {
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
            driveSwitch();
            right = gamepad1.right_bumper ? 1 : 0;
            left = gamepad1.left_bumper ? 1 : 0;
            charlie.markerLock.setPosition(charlie.markerLock.getPosition() + (right / 100.0));
            charlie.markerLock.setPosition(charlie.markerLock.getPosition() - (left / 100.0));
            //charlie.jaw.setPosition((-gamepad2.right_stick_y + 1) / 2); // gamepad2.right_bumper ? -0.5 : gamepad2.left_bumper ? 0.5 : 0.0
            telemetry.addData("rightBumper", gamepad2.right_bumper);
            telemetry.addData("leftBumper", gamepad2.left_bumper);
            telemetry.addData("stick", (-gamepad2.right_stick_y + 1) / 2);
            telemetry.addData("jaw", gamepad2.right_bumper ? -0.5 : gamepad2.left_bumper ? 0.5 : 0.0);
            if(armMode)
            {
                charlie.armExtender.setPower(gamepad2.left_stick_x*.5);
                charlie.armBase.setPower(-gamepad2.right_stick_x*.3);

            }
            else
            {
                if(gamepad2.a)
                {
                    liftMagnet();
                }
                else
                {
                    charlie.rack.setPower(gamepad2.left_stick_y);
                }

            }
            changeModeArm();
            changeModeWheel();

            telemetry.addData("mode", mode);
            telemetry.addData("lock", charlie.markerLock.getPosition());
            //telemetry.addData("left", left);
            //telemetry.addData("right", right);
            telemetry.addData("magnet", charlie.magnet.getState());
            //telemetry.addData("Marker", charlie.teamMarker.getPosition());

            telemetry.update();
        }
    }
    private void liftMagnet() {
        while (charlie.magnet.getState() && opModeIsActive()) {
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