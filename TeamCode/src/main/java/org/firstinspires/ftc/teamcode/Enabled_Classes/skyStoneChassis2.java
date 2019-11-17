package org.firstinspires.ftc.teamcode.Enabled_Classes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import edu.spa.ftclib.internal.state.ToggleBoolean;

@Config
@TeleOp
//@Disabled
public class skyStoneChassis2 extends LinearOpMode {
    robotControl hth3;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double lockMin = 0.5;
    public static double lockMax = 0.15;

    public static double motorSpeed = 0.4;

    public static double stop = 0.49;

    public static double foundationMin = 0.0;
    public static double foundationMax = 0.7;

    public static double liftPower = 0.6;

    public float x, y, z, w, pwr;
    public static double deadzone = 0.2;

    public ToggleBoolean in;
    public ToggleBoolean out;
    public ToggleBoolean mode;
    public ToggleBoolean speedMode;

    public void runOpMode() {
        in = new ToggleBoolean();
        out = new ToggleBoolean();
        mode = new ToggleBoolean();
        speedMode = new ToggleBoolean();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        hth3 = new robotControl();

        hth3.init(hardwareMap);

        magnetThread magnetThread = new magnetThread();

        telemetry.addData("Status", "Initialized");

        telemetry.update();

        waitForStart();

        magnetThread.start();

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
                    hth3.rightFront.setPower(0.5 * Range.clip(-(0.2 * pwr) + x + z, -1, 1));
                    hth3.leftRear.setPower(0.5 * Range.clip(-(0.2 * pwr) + x - z, -1, 1));
                    hth3.leftFront.setPower(0.5 * Range.clip(-(0.2 * pwr) - x - z, -1, 1));
                    hth3.rightRear.setPower(0.5 * Range.clip(-(0.2 * pwr) - x + z, -1, 1));
                }
                else {
                    hth3.rightFront.setPower(Range.clip(-pwr + x + z, -1, 1));
                    hth3.leftRear.setPower(Range.clip(-pwr + x - z, -1, 1));
                    hth3.leftFront.setPower(Range.clip(-pwr - x - z, -1, 1));
                    hth3.rightRear.setPower(Range.clip(-pwr - x + z, -1, 1));
                }
            }
            // Manual lift control
            if(gamepad1.left_bumper) {
                hth3.lift.setPower(-liftPower);
            }
            else if(gamepad1.right_bumper) {
                hth3.lift.setPower(liftPower);
            }
            else {
                hth3.lift.setPower(0);
            }
            // Manual foundation mover control
            if(gamepad1.right_trigger > 0.4) {
                hth3.foundation.setPosition(foundationMin);
            }
            else if(gamepad1.left_trigger > 0.4) {
                hth3.foundation.setPosition(foundationMax);
            }
            // Manual intake control
            if(in.output()) {
                hth3.rightWheel.setPower(0.95);
                hth3.leftWheel.setPower(-0.95);
            }
            else if(out.output()) {
                hth3.rightWheel.setPower(-0.4);
                hth3.leftWheel.setPower(0.4);
            }
            else {
                hth3.rightWheel.setPower(0);
                hth3.leftWheel.setPower(0);
            }

            // Manual lock control
            if(gamepad2.left_bumper) {
                hth3.lock.setPosition(lockMax);
            }
            else if(gamepad2.right_bumper) {
                hth3.lock.setPosition(lockMin);
            }

            telemetry.addData("in", in.output());
            telemetry.addData("out", out.output());
            telemetry.addData("mode", mode.output());
            telemetry.addData("speedmode", speedMode.output());
            telemetry.addData("magnet", !hth3.magnet.getState());
            switch(magnetThread.getLocation()) {
                case 0: telemetry.addData("Location", "bottom"); break;
                case 1: telemetry.addData("Location", "1st block"); break;
                case 2: telemetry.addData("Location", "2nd block"); break;
                case 3: telemetry.addData("Location", "top"); break;
            }
            telemetry.update();
        }
        magnetThread.interrupt();
    }

    private class magnetThread extends Thread {
        public int location = 0;
        private static final int max = 1;
        private static final int min = 0;

        public magnetThread() {
            this.setName("magnetThread");
        }

        public void run() {
            while (hth3.magnet.getState()) {
                hth3.lift.setPower(0.4);
            }
            while(!isInterrupted()) {
                if(gamepad1.dpad_up && location >= min && location < max) {
                    if(!hth3.magnet.getState()) {
                        while(!hth3.magnet.getState()) {
                            hth3.lift.setPower(-0.4);
                        }
                    }
                    while(hth3.magnet.getState()) {
                        hth3.lift.setPower(-0.4);
                    }
                    hth3.lift.setPower(0.0);
                    location += 1;
                }
                if(gamepad1.dpad_down && location > min && location <= max) {
                    if(!hth3.magnet.getState()) {
                        while(!hth3.magnet.getState()) {
                            hth3.lift.setPower(0.4);
                        }
                    }
                    while (hth3.magnet.getState()) {
                        hth3.lift.setPower(0.4);
                    }
                    hth3.lift.setPower(0.0);
                    location -= 1;
                }
            }
        }

        public int getLocation() {
            return this.location;
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

        out.input(gamepad1.b);
        in.input(gamepad1.a);

        mode.input(gamepad1.x);
        speedMode.input(gamepad1.y);
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
