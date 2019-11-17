package org.firstinspires.ftc.teamcode.Enabled_Classes.purepursuit;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Enabled_Classes.robotControl;

import java.text.DecimalFormat;
import java.util.ArrayList;

@TeleOp
public class MyOpMode extends OpMode {
    robotControl hth3;
    private static DecimalFormat df = new DecimalFormat("#.00");

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    public ArrayList<CurvePoint> allPoints = new ArrayList<>();

    public HardwareMap hwm;
    public Telemetry parentTelemetry;
    public Robot r;

    public MyOpMode(HardwareMap hwm, Telemetry t, Robot r) {
        this.hwm = hwm;
        this.parentTelemetry = t;
        this.r = r;
    }

    public void init() {
        hth3 = new robotControl();
        hth3.init(hwm);

        telemetry = new MultipleTelemetry(parentTelemetry, dashboard.getTelemetry());

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        allPoints.add(new CurvePoint(50, 50, 1.0, 1.0, 50, Math.toRadians(50), 1.0));
        allPoints.add(new CurvePoint(50, 100, 1.0, 1.0, 50, Math.toRadians(50), 1.0));
        allPoints.add(new CurvePoint(100, 100, 1.0, 1.0, 50, Math.toRadians(50), 1.0));
    }

    public void loop() {
        telemetry.addData("Status", "Running");
        RobotMovement.followCurve(allPoints, Math.toRadians(90), hth3);
        telemetry.addData("WorldX", r.getXPos());
        telemetry.addData("WorldY", r.getYPos());
        telemetry.addData("Rotation", r.getWorldAngle_rad());
        telemetry.update();
    }
}
