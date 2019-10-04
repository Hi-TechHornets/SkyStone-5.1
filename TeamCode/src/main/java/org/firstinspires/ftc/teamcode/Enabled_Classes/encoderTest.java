package org.firstinspires.ftc.teamcode.Enabled_Classes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import edu.spa.ftclib.internal.controller.ErrorTimeThresholdFinishingAlgorithm;
import edu.spa.ftclib.internal.controller.FinishableIntegratedController;
import edu.spa.ftclib.internal.drivetrain.HeadingableMecanumDrivetrain;
import edu.spa.ftclib.internal.sensor.IntegratingGyroscopeSensor;
import edu.spa.ftclib.internal.controller.PIDController;

@Config
@Autonomous
@Disabled

public class encoderTest extends LinearOpMode {
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

        PIDController pid = new PIDController(p, i, d);
        pid.setMaxErrorForIntegral(pidError);

        DcMotor[] motors = {hth3.leftRear, hth3.leftFront, hth3.rightRear, hth3.rightFront};

        algorithm = new ErrorTimeThresholdFinishingAlgorithm(Math.PI / etError, etTimeThresh);

        controller = new FinishableIntegratedController(new IntegratingGyroscopeSensor(hth3.imu), pid, algorithm);

        robot = new HeadingableMecanumDrivetrain(motors, controller);

        robot.motors[2].setDirection(DcMotor.Direction.FORWARD);
        robot.motors[3].setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if(opModeIsActive() && !isStopRequested()) {
            robot.setTargetHeading((setHeadingCo * Math.PI) / setHeadingDiv);
            while (robot.isRotating() && opModeIsActive() && !isStopRequested()) {
                robot.updateHeading();
                doTelemetry();
            }

            telemetry.addData("Status", "Finished rotation");
            telemetry.update();
            sleep(3000);

//            sleep(1000);
//
//            robot.setTargetHeading(-Math.PI / 2);
//            while (robot.isRotating()) {
//                robot.updateHeading();
//                telemetry.addData("Heading", robot.getCurrentHeading());
//                telemetry.update();
//            }
//
//            sleep(1000);
//
//            robot.setTargetHeading(0);
//            while (opModeIsActive()) robot.updateHeading();
        }
    }

    void doTelemetry() {
        PIDController pid = (PIDController) robot.controller.algorithm;
        telemetry.addData("heading", robot.controller.getSensorValue());
        telemetry.addData("target", pid.getTarget());
        telemetry.addData("KP", pid.getKP());
        telemetry.addData("KI", pid.getKI());
        telemetry.addData("KD", pid.getKD());
        telemetry.addData("error", pid.getError());
        telemetry.addData("integral", pid.getIntegral());
        telemetry.addData("derivative", pid.getDerivative());
        telemetry.addData("Status", "Rotating");
        telemetry.update();
    }
}
