package org.firstinspires.ftc.teamcode.Enabled_Classes.purepursuit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Master extends LinearOpMode {
    public void runOpMode() {
        Robot robot = new Robot();
        OpMode opMode = new MyOpMode(hardwareMap, telemetry, robot);
        opMode.init();

        waitForStart();

        while(opModeIsActive()) {
            opMode.loop();
            robot.update();
        }
    }
}
