package org.firstinspires.ftc.teamcode.Enabled_Classes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import edu.spa.ftclib.internal.controller.ErrorTimeThresholdFinishingAlgorithm;
import edu.spa.ftclib.internal.controller.FinishableIntegratedController;
import edu.spa.ftclib.internal.controller.PIDController;
import edu.spa.ftclib.internal.drivetrain.HeadingableMecanumDrivetrain;
import edu.spa.ftclib.internal.sensor.IntegratingGyroscopeSensor;
import edu.spa.ftclib.util.PIDTuner;

@TeleOp

public class tuner extends LinearOpMode {
    private robotControl hth3;
    private HeadingableMecanumDrivetrain robot;
    private FinishableIntegratedController controller;
    private ErrorTimeThresholdFinishingAlgorithm algorithm;

    public static double p = 2.2;
    public static double i = 0.05;
    public static double d = 0.0;
    public static double pidError = 0.002;

    public static double etError = 50;
    public static double etTimeThresh = 1.0;

    public static double setHeadingCo = 1;
    public static double setHeadingDiv = 2;

    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        hth3 = new robotControl();
        hth3.init(hardwareMap);

        hth3.encoderMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(!hth3.imu.isGyroCalibrated());

        edu.spa.ftclib.internal.controller.PIDController pid = new PIDController(p, i, d);
        pid.setMaxErrorForIntegral(pidError);

        DcMotor[] motors = {hth3.leftRear, hth3.leftFront, hth3.rightRear, hth3.rightFront};

        algorithm = new ErrorTimeThresholdFinishingAlgorithm(Math.PI / etError, etTimeThresh);

        controller = new FinishableIntegratedController(new IntegratingGyroscopeSensor(hth3.imu), pid, algorithm);

        robot = new HeadingableMecanumDrivetrain(motors, controller);

        robot.motors[2].setDirection(DcMotor.Direction.FORWARD);
        robot.motors[3].setDirection(DcMotor.Direction.FORWARD);

        PIDTuner tuner = new PIDTuner(robot, pid, gamepad1, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            tuner.update();
        }
    }
}
