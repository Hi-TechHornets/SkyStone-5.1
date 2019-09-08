package org.firstinspires.ftc.teamcode.Enabled_Classes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import java.io.File;
import java.util.Collection;
@TeleOp(name = "Collector Test", group = "Debug")
@Disabled
public class collector extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private Gyroscope imu;
    private File[] webcamConfig;
    //private DcMotor winch;     -----spool
    private DcMotor rightFront;
    private DcMotor rightRear;
    private DcMotor leftFront;
    private DcMotor leftRear;
    //private DcMotor rack;      -----lyft
    private DcMotor rightCollection;
    private DcMotor leftCollection;
    private Blinker expansion_Hub_1;
    private Blinker expansion_Hub_2;
    private Servo teamMarker;
    private DcMotor lyft;
    private DistanceSensor rightDistance;
    private DistanceSensor centerDistance;
    private DistanceSensor leftDistance;
    private DigitalChannel magnet;
    private int turnTime;
    private Servo leftTilt;
    private Servo rightTilt;
    private DcMotor spool;
    private CRServo intake;
    private int right;
    private int left;
    private int MUMBOJUMBO;
    public void runOpMode(){
        leftCollection=hardwareMap.dcMotor.get("leftCollection");
        rightCollection=hardwareMap.dcMotor.get("rightCollection");
        rightFront = hardwareMap.dcMotor.get("RightFront");
        rightRear = hardwareMap.dcMotor.get("RightRear");
        leftFront = hardwareMap.dcMotor.get("LeftFront");
        leftRear = hardwareMap.dcMotor.get("LeftRear");
        rightTilt = hardwareMap.servo.get("rightTilt");
        leftTilt = hardwareMap.servo.get("leftTilt");
        intake = hardwareMap.crservo.get("intake");
        spool = hardwareMap.dcMotor.get("spool");
        lyft = hardwareMap.dcMotor.get("rack");
        MUMBOJUMBO = 0;
        waitForStart();
        while(opModeIsActive())
        {
            leftCollection.setPower(0.6 * gamepad1.left_stick_y);
            rightCollection.setPower(0.6 * -gamepad1.left_stick_y);
            rightFront.setPower(gamepad2.right_stick_y);
            rightRear.setPower(gamepad2.right_stick_y);
            leftFront.setPower(-gamepad2.left_stick_y);
            leftRear.setPower(-gamepad2.left_stick_y);
            right = gamepad1.right_bumper ? 1 : 0;
            left = gamepad1.left_bumper ? 1 : 0;
            rightTilt.setPosition(rightTilt.getPosition() + (right / 100.0));
            leftTilt.setPosition(leftTilt.getPosition() - (right / 100.0));
            rightTilt.setPosition(rightTilt.getPosition() - (left / 100.0));
            leftTilt.setPosition(leftTilt.getPosition() + (left / 100.0));
            spool.setPower(gamepad1.right_trigger);
            spool.setPower(-gamepad1.left_trigger);
            lyft.setPower(gamepad1.right_stick_y);
            telemetry.addData("rightTilt", rightTilt.getPosition());
            telemetry.addData("leftTilt", leftTilt.getPosition());
            telemetry.addData("right", right);
            telemetry.addData("left", left);
            telemetry.addData("up", lyft.getPower());
            telemetry.update();
            if(gamepad1.a) {
                MUMBOJUMBO = MUMBOJUMBO + 1;
                MUMBOJUMBO = MUMBOJUMBO % 2;
            }
            intake.setPower(MUMBOJUMBO);
        }
    }
}
