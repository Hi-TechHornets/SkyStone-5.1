package org.firstinspires.ftc.teamcode.Enabled_Classes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Manual Drive: One Con"/*, group = "Debug"*/)
@Disabled
public class Manual extends LinearOpMode {
    private Gyroscope imu;
    private Gyroscope imu_1;
     
    private DcMotor rightFront;
    private DcMotor rightRear;
    private DcMotor leftFront;
    private DcMotor leftRear;
    //private DcMotor pully;
    //private DcMotor linearmotion;

    private DcMotor rack;

    private Blinker expansion_Hub_1;
    private Blinker expansion_Hub_2;
    private ColorSensor color;
    
    private Servo srone;

    private DistanceSensor rightDistance;
    //private DistanceSensor leftDistance;
    private DistanceSensor rightFrontDistance;
    private DistanceSensor rightCDistance;

    
    private double right;
    private double left;

    private DigitalChannel magnet;

    private int mode;
    public void runOpMode() {
        int vole=0;
        mode = 1;
        rightFront= hardwareMap.dcMotor.get("RightFront");
        rightRear= hardwareMap.dcMotor.get("RightRear");
        leftFront= hardwareMap.dcMotor.get("LeftFront");
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear= hardwareMap.dcMotor.get("LeftRear");
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rack = hardwareMap.dcMotor.get("rack");
        rightDistance = hardwareMap.get(DistanceSensor.class, "rightRearDistance");
        //leftDistance = hardwareMap.get(DistanceSensor.class, "leftDistance");
        rightFrontDistance = hardwareMap.get(DistanceSensor.class, "rightFrontDistance");
        rightCDistance=hardwareMap.get(DistanceSensor.class,"rightRearDistance");
        
        //pully = hardwareMap.dcMotor.get("pully");
        //linearmotion = hardwareMap.dcMotor.get("linearmotion");
        //srone = hardwareMap.servo.get("srone");
        magnet = hardwareMap.digitalChannel.get("magnet");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        int sr=4;
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.addData("rightRearDistance",rightDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("rightFrontDistance",rightFrontDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("rightDistance",rightCDistance.getDistance(DistanceUnit.CM));
            //winch.setPower(1.0);
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
                rightFront.setPower(-gamepad1.right_stick_y);
                rightRear.setPower(-gamepad1.right_stick_y);
                leftFront.setPower(-gamepad1.left_stick_y);
                leftRear.setPower(-gamepad1.left_stick_y);
            }
            else {
                rightFront.setPower(gamepad1.left_stick_y);
                rightRear.setPower(gamepad1.left_stick_y);
                leftFront.setPower(gamepad1.right_stick_y);
                leftRear.setPower(gamepad1.right_stick_y);
            }


            /*if(gamepad1.right_bumper)
            {
                leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }*/
            left = (gamepad2.left_trigger );
            right = (gamepad2.right_trigger );
            
            telemetry.addData("left", left);
            telemetry.addData("right", right);
            telemetry.addData("mode", mode);
            telemetry.addData("magnet", magnet.getState());
            rack.setPower(gamepad2.left_stick_y);
            //linearmotion.setPower(gamepad2.right_stick_y / 5);
            if(left>0.008){
                //srone.setPosition(srone.getPosition()+left*0.05);
            }
            else if(right>0.008)
            {
                //srone.setPosition(srone.getPosition()-right*0.05);
            }
            //telemetry.addData("bucket", srone.getPosition());
            telemetry.update();
        }
    }
}