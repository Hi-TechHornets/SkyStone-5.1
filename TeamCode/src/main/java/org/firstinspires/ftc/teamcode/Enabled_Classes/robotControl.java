package org.firstinspires.ftc.teamcode.Enabled_Classes;

import android.util.Log;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.concurrent.TimeUnit;

//@Disabled
public class robotControl {
    public DcMotor leftFront;
    public DcMotor leftRear;
    public DcMotor rightFront;
    public DcMotor rightRear;

//    public DcMotor gripperRotate;

    public DcMotor rightWheel;
    public DcMotor leftWheel;

    public DcMotor lift;

//    public Servo gripperServo;
//    public Servo cr;
    public Servo lock;
    public Servo foundation;

    public BNO055IMUImpl imu;

    public DigitalChannel magnet;

    public ElapsedTime timer = new ElapsedTime();

    private HardwareMap hardwareMap = null;

    public void init(HardwareMap ahwMap) {
        hardwareMap = ahwMap;
        leftFront = hardwareMap.dcMotor.get("0leftFront");
        leftRear = hardwareMap.dcMotor.get("1leftRear");
        rightFront = hardwareMap.dcMotor.get("2rightFront");
        rightRear = hardwareMap.dcMotor.get("3rightRear");

        rightWheel = hardwareMap.dcMotor.get("rightWheel");
        leftWheel = hardwareMap.dcMotor.get("leftWheel");

        lift = hardwareMap.dcMotor.get("lift");

        lock = hardwareMap.servo.get("lock");
        foundation = hardwareMap.servo.get("foundation");

        magnet = hardwareMap.digitalChannel.get("magnet");

//        gripperRotate = hardwareMap.dcMotor.get("flipMotor");
//        gripperServo = hardwareMap.servo.get("gripperServo");
//        cr = hardwareMap.servo.get("cr");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMUImpl.class, "imu");
        imu.initialize(parameters);
    }

    public void moveDriveMotors(double power) {
        moveDriveMotors(power, power);
    }

    public void moveDriveMotors(double leftPower, double rightPower) {
        leftFront.setPower(leftPower);
        leftRear.setPower(leftPower);
        rightFront.setPower(rightPower);
        rightRear.setPower(rightPower);
    }

    public void moveDriveMotors(double leftFrontPower, double leftRearPower, double rightFrontPower, double rightRearPower) {
        leftFront.setPower(leftFrontPower);
        leftRear.setPower(leftRearPower);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower);
    }

    public void halt() {
        moveDriveMotors(0);
    }

    public void encoderMode(DcMotor.RunMode mode) {
        rightFront.setMode(mode);
        rightRear.setMode(mode);
        leftFront.setMode(mode);
        leftRear.setMode(mode);
    }

    public void resetTimer() {
        timer.reset();
    }

    public long getTime() {
        return timer.time(TimeUnit.MILLISECONDS);
    }

    public String sampleSkyStone(Telemetry telemetry, TFObjectDetector tfod) {
        String position = "";
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                if(updatedRecognitions.size() == 2) {
                    double stone1X = -1;
                    double stone2X = -1;
                    double skyStoneX = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals("Skystone")) {
                            skyStoneX = (int) recognition.getLeft();
                        }
                        else if(stone1X == -1) {
                            stone1X = (int) recognition.getLeft();
                        }
                        else {
                            stone2X = (int) recognition.getLeft();
                        }
                    }
                    if(skyStoneX == -1 && stone1X != -1 && stone2X != -1) {
                        telemetry.addData("Position", "left");
                        position = "left";
                    }
                    else if(skyStoneX != -1 && stone1X != 1) {
                        if(skyStoneX > stone1X) {
                            telemetry.addData("Position", "right");
                            position = "right";
                        }
                        else {
                            telemetry.addData("Position", "center");
                            position = "center";
                        }
                    }
//                    telemetry.addData("Position", position);
//                    telemetry.update();
                }
            }
        }
        telemetry.update();
        return position;
    }
}
