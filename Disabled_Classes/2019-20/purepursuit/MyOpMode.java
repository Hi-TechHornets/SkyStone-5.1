package org.firstinspires.ftc.teamcode.Enabled_Classes.purepursuit;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Enabled_Classes.robotControl;

import java.text.DecimalFormat;
import java.util.ArrayList;

@TeleOp
@Disabled
public class MyOpMode extends OpMode {
    public static robotControl hth3;
    private static DecimalFormat df = new DecimalFormat("#.00");

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    public ArrayList<CurvePoint> allPoints = new ArrayList<>();

    public HardwareMap hwm;
    public Telemetry parentTelemetry;

    public MyOpMode(HardwareMap hwm, Telemetry t) {
        this.hwm = hwm;
        this.parentTelemetry = t;
    }

    public void init() {
        hth3 = new robotControl();
        hth3.init(hwm);

        telemetry = new MultipleTelemetry(parentTelemetry, dashboard.getTelemetry());

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        allPoints.add(new CurvePoint(50, 50, 1.0, 1.0, 50, Math.toRadians(50), 1.0));
        allPoints.add(new CurvePoint(50, 230, 1.0, 1.0, 40, Math.toRadians(50), 1.0));
        allPoints.add(new CurvePoint(200, 230, 1.0, 1.0, 40, Math.toRadians(50), 1.0));
        allPoints.add(new CurvePoint(200, 100, 1.0, 1.0, 50, Math.toRadians(50), 1.0));
    }

    public void loop() {
//        RobotMovement.followCurve(allPoints, Math.toRadians(90), hth3);


    }

    public static void leg1() {
        RobotMovement.goToPosition(49, 100, 1.0, Math.toRadians(90), 1.0, hth3);
    }

    public static void leg2() {
        RobotMovement.goToPosition(100, 100, 1.0, Math.toRadians(90), 1.0, hth3);
    }

    public static void halt() {
        hth3.halt();
    }
}
