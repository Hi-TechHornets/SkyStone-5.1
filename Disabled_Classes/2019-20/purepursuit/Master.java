package org.firstinspires.ftc.teamcode.Enabled_Classes.purepursuit;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Disabled
public class Master extends LinearOpMode {
    public void runOpMode() {
        Robot robot = new Robot();
        OpMode opMode = new MyOpMode(hardwareMap, telemetry);
        opMode.init();

        waitForStart();
//        opMode.loop();
        while(opModeIsActive()) {
            robot.update();
            telemetry.addData("Status", "Running");
            telemetry.addData("WorldX", robot.getXPos());
            telemetry.addData("WorldY", robot.getYPos());
            telemetry.addData("Rotation", robot.getWorldAngle_rad());
            telemetry.addData("robot", robot);
            telemetry.addData("xSpeed", Robot.xSpeed);
            telemetry.addData("ySpeed", Robot.ySpeed);
            telemetry.addData("turnSpeed", Robot.turnSpeed);
            telemetry.update();
//            if(robot.getXPos() > 90 && robot.getYPos() > 90) {
//                MyOpMode.halt();
//                break;
//            }
        }
    }
}
