package org.firstinspires.ftc.teamcode.Enabled_Classes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Enabled_Classes.util.ToggleBoolean;
import org.firstinspires.ftc.teamcode.Enabled_Classes.util.ToggleInt;

@Config
@TeleOp
//@Disabled
public class skyStoneChassis2 extends LinearOpMode {
    robotControl hth3;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double foundationUp = 0.3;
    public static double foundationDown = 1.0;

    public static double gripperClose = 0.0;
    public static double gripperOpen = 0.4;

    public static double rotateIn = 0.0;
    public static double rotateOut = 1.0;

    public static double liftPower = 0.6;

    public static double stoneUp = 0.85;
    public static double stoneDown = 0.2;

    public float x, y, z, w, pwr;
    public static double deadzone = 0.2;

    public ToggleBoolean in;
    public ToggleBoolean out;
    public ToggleBoolean mode;
    public ToggleBoolean speedMode;
    public ToggleBoolean rotateState;
    public ToggleBoolean gripperState;

    public void runOpMode() {
        in = new ToggleBoolean();
        out = new ToggleBoolean();
        mode = new ToggleBoolean();
        speedMode = new ToggleBoolean();
        rotateState = new ToggleBoolean();
        gripperState = new ToggleBoolean(); //I like to progmrmam

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        hth3 = new robotControl();

        hth3.init(hardwareMap);

        hth3.gripper.setPosition(gripperClose);

        telemetry.addData("Status", "Initialized");

        telemetry.update();

        waitForStart();

//        magnetThread.start();

        while(opModeIsActive()) {
            telemetry.addData("Status", "Running");

            getJoyVals();

            pwr = y;
            // Main drive
            if(!mode.output()) {
                if(speedMode.output()) {
                    hth3.rightFront.setPower(0.5 * Range.clip(pwr - x + z, -1, 1));
                    hth3.leftRear.setPower(0.5 * Range.clip(pwr - x - z, -1, 1));
                    hth3.leftFront.setPower(0.5 * Range.clip(pwr + x - z, -1, 1));
                    hth3.rightRear.setPower(0.5 * Range.clip(pwr + x + z, -1, 1));
                }
                else {
                    hth3.rightFront.setPower(Range.clip(pwr - x + z, -1, 1));
                    hth3.leftRear.setPower(Range.clip(pwr - x - z, -1, 1));
                    hth3.leftFront.setPower(Range.clip(pwr + x - z, -1, 1));
                    hth3.rightRear.setPower(Range.clip(pwr + x + z, -1, 1));
                }
            }
            else {
                if(speedMode.output()) {
                    hth3.rightFront.setPower(0.5 * Range.clip( pwr + x + z, -1, 1));
                    hth3.leftRear.setPower(0.5 * Range.clip(pwr + x - z, -1, 1));
                    hth3.leftFront.setPower(0.5 * Range.clip(pwr - x - z, -1, 1));
                    hth3.rightRear.setPower(0.5 * Range.clip(pwr - x + z, -1, 1));
                }
                else {
                    hth3.rightFront.setPower(Range.clip(-pwr + x + z, -1, 1));
                    hth3.leftRear.setPower(Range.clip(-pwr + x - z, -1, 1));
                    hth3.leftFront.setPower(Range.clip(-pwr - x - z, -1, 1));
                    hth3.rightRear.setPower(Range.clip(-pwr - x + z, -1, 1));
                }
            }

            // Manual lift control
            if(gamepad2.left_bumper) {
                hth3.lift.setPower(-liftPower);
            }
            else if(gamepad2.right_bumper) {
                hth3.lift.setPower(liftPower);
            }
            else {
                hth3.lift.setPower(0);
            }

            if(rotateState.output()) {
                hth3.rotate.setPosition(rotateOut);
            }
            else {
                hth3.rotate.setPosition(rotateIn);
            }

            if(gripperState.output()) {
                hth3.gripper.setPosition(gripperOpen);
            }
            else {
                hth3.gripper.setPosition(gripperClose);
            }

            // Manual foundation mover control
            if(gamepad2.right_trigger > 0.4) {
                hth3.foundation.setPosition(foundationUp);
            }
            else if(gamepad2.left_trigger > 0.4) {
                hth3.foundation.setPosition(foundationDown);
            }

            if(gamepad2.dpad_down) {
                hth3.stone.setPosition(stoneDown);
            }
            else if(gamepad2.dpad_up) {
                hth3.stone.setPosition(stoneUp);
            }

            // Manual lock control

//            if(gamepad1.left_bumper) {
//                hth3.lock.setPosition(lockMax);
//            }
//            else if(gamepad1.right_bumper) {
//                hth3.lock.setPosition(lockMin);
//            }



//            telemetry.addData("in", in.output());
//            telemetry.addData("out", out.output());
            telemetry.addData("mode", mode.output());
            telemetry.addData("speedmode", speedMode.output());
            telemetry.addData("gripperState", gripperState.output());
            telemetry.addData("rotateState", rotateState.output());
            //telemetry.addData("magnet", !hth3.magnet.getState());


//            telemetry.addData("leftX", gamepad1.left_stick_x);
//            telemetry.addData("leftY", gamepad1.left_stick_y);
//            telemetry.addData("rightX", gamepad1.right_stick_x);
            telemetry.addData("rightFront", hth3.rightFront.getCurrentPosition());
            telemetry.addData("leftFront", hth3.leftFront.getCurrentPosition());
            telemetry.addData("rightRear", hth3.rightRear.getCurrentPosition());
            telemetry.addData("leftRear", hth3.leftRear.getCurrentPosition());
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

        out.input(gamepad2.b);
        in.input(gamepad2.a);

        mode.input(gamepad1.x);
        speedMode.input(gamepad1.y);

        rotateState.input(gamepad2.y);
        gripperState.input(gamepad2.x);
    }

    public void wt(int time) {
        hth3.resetTimer();
        while(hth3.getTime() < time && running()) {
        }
    }

    public boolean running() {
        return opModeIsActive() && !isStopRequested();
    }
}

// old variables
//    public static double lockMin = 0.5;
//    public static double lockMax = 0.15;
//
//    public static double inSpeed = 0.95;
//    public static double outSpeed = 0.4;
//
//    public static double stop = 0.49;
//    public static double gripperClose = 0.0;
//    public static double gripperOpen = 1.0;
//
//    public static double rotateIn = 0.0;
//    public static double rotateOut = 1.0;
//
//    public static double liftPower = 0.6;
