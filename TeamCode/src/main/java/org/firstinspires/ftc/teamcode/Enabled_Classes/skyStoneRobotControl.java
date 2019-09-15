package org.firstinspires.ftc.teamcode.Enabled_Classes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class skyStoneRobotControl {
    public DcMotor leftFront;
    public DcMotor leftRear;
    public DcMotor rightFront;
    public DcMotor rightRear;

    private HardwareMap hardwareMap = null;

    void init(HardwareMap ahwMap) {
        hardwareMap = ahwMap;
        leftFront = hardwareMap.dcMotor.get("0leftFront");
        leftRear = hardwareMap.dcMotor.get("1leftRear");
        rightFront = hardwareMap.dcMotor.get("2rightFront");
        rightRear = hardwareMap.dcMotor.get("3rightRear");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    void moveDriveMotors(double power) {
        moveDriveMotors(power, power);
    }

    void moveDriveMotors(double leftPower, double rightPower) {
        leftFront.setPower(leftPower);
        leftRear.setPower(leftPower);
        rightFront.setPower(rightPower);
        rightRear.setPower(rightPower);
    }

    void moveDriveMotors(double leftFrontPower, double leftRearPower, double rightFrontPower, double rightRearPower) {
        leftFront.setPower(leftFrontPower);
        leftRear.setPower(leftRearPower);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower);
    }

    void halt() {
        moveDriveMotors(0);
    }
}
