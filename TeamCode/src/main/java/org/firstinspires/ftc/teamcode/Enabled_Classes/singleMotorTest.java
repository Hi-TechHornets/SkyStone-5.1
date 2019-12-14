package org.firstinspires.ftc.teamcode.Enabled_Classes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import edu.spa.ftclib.internal.state.Button;
import edu.spa.ftclib.internal.state.ToggleBoolean;

@TeleOp
public class singleMotorTest extends LinearOpMode {
    robotControl hth3;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private Button up;
    private Button down;
    private Button left;
    private Button right;

    private Button a;
    private ToggleBoolean b;

    private int lr = 0;
    private int ud = 0;

    private DcMotor currentMotor;
    private String motorName = "";

    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        hth3 = new robotControl();
        hth3.init(hardwareMap);

        up = new Button();
        down = new Button();
        left = new Button();
        right = new Button();
        a = new Button();
        b = new ToggleBoolean();

        currentMotor = hth3.leftFront;

        telemetry.addData("Status", "Initialized");

        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            getInputs();

            if(up.onPress() && ud == 1) ud = 0;
            if(down.onPress() && ud == 0) ud = 1;
            if(left.onPress() && lr == 1) lr = 0;
            if(right.onPress() && lr == 0) lr = 1;

            if(ud == 0 && lr == 0) {currentMotor = hth3.leftFront; motorName = "leftFront";}
            else if(ud == 0 && lr == 1) {currentMotor = hth3.rightFront; motorName = "rightFront";}
            else if(ud == 1 && lr == 0) {currentMotor = hth3.leftRear; motorName = "leftRear";}
            else {currentMotor = hth3.rightRear; motorName = "rightRear";}

            if(a.isPressed() && opModeIsActive()) {
                currentMotor.setPower(gamepad1.left_stick_y);

                telemetry.addData("Status", "Running");
                telemetry.addData("Current motor", motorName);
                telemetry.addData("Position", currentMotor.getCurrentPosition());
                telemetry.addData("Power", gamepad1.left_stick_y);
                telemetry.addData("a", a.isPressed());
                telemetry.update();
            }

            telemetry.addData("Status", "Running");
            telemetry.addData("Current motor", motorName);
            telemetry.addData("a", a.isPressed());
            telemetry.update();
        }
    }

    public void getInputs() {
        up.input(gamepad1.dpad_up);
        down.input(gamepad1.dpad_down);
        left.input(gamepad1.dpad_left);
        right.input(gamepad1.dpad_right);
        a.input(gamepad1.a);
        b.input(gamepad1.b);
    }
}
