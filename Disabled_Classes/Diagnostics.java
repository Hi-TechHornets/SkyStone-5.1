package org.firstinspires.ftc.teamcode.Enabled_Classes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "Diagnostic Check OpMode", group = "Drive")
@Disabled
public class Diagnostics extends LinearOpMode {
    private robotControl charlie;
    private int mode;
    @Override
    public void runOpMode() {
        mode = 1;
        charlie = new robotControl();
        charlie.diagnosticStartUpMotors(hardwareMap);

        waitForStart();
        double right;
        double left;
        while(true) {
            telemetry.addData("Wheel Checking: ", "testing");
            telemetry.addData("Lift Checking: ", "inactive");
            telemetry.addData("Collection Checking: ", "inactive");
            if(gamepad1.a) {
                if(mode == 1)
                {
                    mode = -1;
                }
                else {
                    mode = 1;
                }
            }
            if(mode == 1){
                charlie.rightFront.setPower(-gamepad1.right_stick_y);
                charlie.rightRear.setPower(-gamepad1.right_stick_y);
                charlie.leftFront.setPower(-gamepad1.left_stick_y);
                charlie.leftRear.setPower(-gamepad1.left_stick_y);
            }
            else {
                charlie.rightFront.setPower(gamepad1.left_stick_y);
                charlie.rightRear.setPower(gamepad1.left_stick_y);
                charlie.leftFront.setPower(gamepad1.right_stick_y);
                charlie.leftRear.setPower(gamepad1.right_stick_y);
            }
            telemetry.update();
            break;
        }
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            if(gamepad1.a) {
                if(mode == 1)
                {
                    mode = -1;
                }
                else {
                    mode = 1;
                }
            }
            if(mode == 1){
                charlie.rightFront.setPower(-gamepad1.right_stick_y);
                charlie.rightRear.setPower(-gamepad1.right_stick_y);
                charlie.leftFront.setPower(-gamepad1.left_stick_y);
                charlie.leftRear.setPower(-gamepad1.left_stick_y);
            }
            else {
                charlie.rightFront.setPower(gamepad1.left_stick_y);
                charlie.rightRear.setPower(gamepad1.left_stick_y);
                charlie.leftFront.setPower(gamepad1.right_stick_y);
                charlie.leftRear.setPower(gamepad1.right_stick_y);
            }


            /*if(gamepad1.right_bumper)
            {
                leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }*/
            right = gamepad1.right_bumper ? 1 : 0;
            left = gamepad1.left_bumper ? 1 : 0;
            charlie.rightTilt.setPosition(charlie.rightTilt.getPosition() + (right / 100.0));
            charlie.leftTilt.setPosition(charlie.leftTilt.getPosition() - (right / 100.0));
            charlie.rightTilt.setPosition(charlie.rightTilt.getPosition() - (left / 100.0));
            charlie.leftTilt.setPosition(charlie.leftTilt.getPosition() + (left / 100.0));
            //charlie.leftCollection.setPower(0.6 * gamepad1.left_stick_y);
            //charlie.rightCollection.setPower(0.6 * -gamepad1.left_stick_y);


            charlie.rack.setPower(gamepad2.left_stick_y);
            //charlie.spool.setPower(gamepad1.right_trigger);
            //charlie.spool.setPower(-gamepad1.left_trigger);
            //linearmotion.setPower(gamepad2.right_stick_y / 5);
            if(left>0.008){
                charlie.teamMarker.setPosition(charlie.teamMarker.getPosition()+left*0.05);
            }
            else if(right>0.008)
            {
                charlie.teamMarker.setPosition(charlie.teamMarker.getPosition()-right*0.05);
            }

            telemetry.addData("mode", mode);
            /*
            telemetry.addData("left", left);
            telemetry.addData("right", right);
            telemetry.addData("magnet", charlie.magnet.getState());
            telemetry.addData("Marker", charlie.teamMarker.getPosition());
            */
            telemetry.update();
        }
    }
}
