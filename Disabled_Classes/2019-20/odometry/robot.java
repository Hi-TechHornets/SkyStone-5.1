package org.firstinspires.ftc.teamcode.Enabled_Classes.odometry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.motors.NeveRest40Gearmotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
@Disabled
public class robot {
    static final DcMotor.ZeroPowerBehavior ZERO_POWER_BEHAVIOR = DcMotor.ZeroPowerBehavior.BRAKE;
    private static final MotorConfigurationType MOTOR_CONFIG = MotorConfigurationType.getMotorType(NeveRest40Gearmotor.class);
    private static final double TICKS_PER_REV = MOTOR_CONFIG.getTicksPerRev();
    HardwareMap hwMap;
    ElapsedTime clock = new ElapsedTime();
    BNO055IMU imu;
    BNO055IMU.Parameters parameters;
    Orientation angles;
    static final double oneRotationTicks = TICKS_PER_REV;
    static final double wheelRadius = 0.0508; // in meters
    static final double wheelDistanceApart = 0.09144 + .016 * 2.0; // in meters
    private int leftEncoderPos = 0;
    private int centerEncoderPos = 0;
    private int rightEncoderPos = 0;
    private double deltaLeftDistance = 0;
    private double deltaRightDistance = 0;
    private double deltaCenterDistance = 0;
    private double x = 0;
    private double y = 0;
    private double theta = 0;

    private DcMotor FR = null;
    private DcMotor FL = null;
    private DcMotor BR = null;
    private DcMotor BL = null;
    private DcMotor leftEncoderMotor = null;
    private DcMotor rightEncoderMotor = null;
    private DcMotor centerEncoderMotor = null;

    public void init(HardwareMap ahwMap, boolean initSensors) {
        hwMap = ahwMap;
        FR = hwMap.get(DcMotor.class, "2rightFront");
        FL = hwMap.get(DcMotor.class, "0leftFront");
        BR = hwMap.get(DcMotor.class, "3rightRear");
        BL = hwMap.get(DcMotor.class, "1leftRear");
        leftEncoderMotor = hwMap.get(DcMotor.class, "0leftFront");
        rightEncoderMotor = hwMap.get(DcMotor.class, "2rightFront");
        centerEncoderMotor = hwMap.get(DcMotor.class, "1leftRear");
        leftEncoderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightEncoderMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        centerEncoderMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        FL.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        BR.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        BL.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        if (initSensors) {
            imu = hwMap.get(BNO055IMU.class, "imu");
            parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            imu.initialize(parameters);
        }
    }
    public void init(HardwareMap ahwMap) {
        init(ahwMap, true);
    }
    public void resetTicks() {
        resetLeftTicks();
        resetCenterTicks();
        resetRightTicks();
    }
    public void resetLeftTicks() {
        leftEncoderPos = leftEncoderMotor.getCurrentPosition();
    }
    public int getLeftTicks() {
        return leftEncoderMotor.getCurrentPosition() - leftEncoderPos;
    }
    public void resetRightTicks() {
        rightEncoderPos = rightEncoderMotor.getCurrentPosition();
    }
    public int getRightTicks() {
        return rightEncoderMotor.getCurrentPosition() - rightEncoderPos;
    }
    public void resetCenterTicks() {
        centerEncoderPos = centerEncoderMotor.getCurrentPosition();
    }
    public int getCenterTicks() {
        return centerEncoderMotor.getCurrentPosition() - centerEncoderPos;
    }
    public void drive(double fl, double bl, double fr, double br) {
        FL.setPower(fl);
        BL.setPower(bl);
        FR.setPower(fr);
        BR.setPower(br);
    }
    public void updatePosition() {
        deltaLeftDistance = (getLeftTicks() / oneRotationTicks) * 2.0 * Math.PI * wheelRadius;
        deltaRightDistance = (getRightTicks() / oneRotationTicks) * 2.0 * Math.PI * wheelRadius;
        deltaCenterDistance = (getCenterTicks() / oneRotationTicks) * 2.0 * Math.PI * wheelRadius;
        x  += (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.cos(theta);
        y  += (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.sin(theta);
        theta  += (deltaLeftDistance - deltaRightDistance) / wheelDistanceApart;
        resetTicks();
    }
    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }
    public double getTheta() {
        return theta;
    }
    public void setX(double _x) {
        x = _x;
    }
    public void setY(double _y) {
        y = _y;
    }
    public void setTheta(double _theta) {
        theta = _theta;
    }
    public double angle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
    }
}
