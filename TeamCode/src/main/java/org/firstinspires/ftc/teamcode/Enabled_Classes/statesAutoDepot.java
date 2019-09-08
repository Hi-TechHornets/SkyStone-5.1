package org.firstinspires.ftc.teamcode.Enabled_Classes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.concurrent.TimeUnit;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaRoverRuckus;
import org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus;

@Autonomous(name = "statesAutoDepotGold", group = "Auto")

public class statesAutoDepot extends LinearOpMode {
    private VuforiaRoverRuckus vuforiaRoverRuckus;
    private TfodRoverRuckus tfodRoverRuckus;
    private robotControl charlie;

    @Override
    public void runOpMode() {
        charlie = new robotControl();
        charlie.init(hardwareMap);
        /*
        List<Recognition> recognitions;
        double goldMineralX;
        double silverMineral1X;
        double silverMineral2X;
        */
        String position = "";
        vuforiaRoverRuckus = new VuforiaRoverRuckus();
        tfodRoverRuckus = new TfodRoverRuckus();
        vuforiaRoverRuckus.initialize("", hardwareMap.get(WebcamName.class, "Webcam 1"), "teamwebcamcalibrations.xml",
                true, true, VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES,
                0, 0, 0, 0, 0, 0, true);
        tfodRoverRuckus.initialize(vuforiaRoverRuckus, (float) 0.30, true, true);
        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            dropFromLander();
            if (tfodRoverRuckus != null) {
                tfodRoverRuckus.activate();
            }
            charlie.resetTimer();
            while (charlie.moveTimer.time(TimeUnit.SECONDS) <3 && passCheck()&&position.equals("")) {
                charlie.rack.setPower(0.3);
                position = charlie.sampling(telemetry, tfodRoverRuckus);
            }
            charlie.rack.setPower(0.0);
        }

        telemetry.addData("Gem", position);
        telemetry.addData("RackPos",charlie.rack.getCurrentPosition());
        telemetry.update();

        wait(500);
        driveTime(300, 0.4);
        charlie.halt();
        switch (position) {
            case "Left":
                driveTime(350, -0.8, 0.8); // -0.4, 0.4
                driveTime(1500, 0.7); // 0.5
                driveTime(700, 0.8, -0.8);
                driveTime(1000, 0.8, 0.8);
                break;
            case "Right":
                driveTime(450, 0.8, -0.8); // 0.4, -0.4
                driveTime(1300, 0.7); // 0.5
                driveTime(700, -0.8, 0.8);
                driveTime(1000, 0.8, 0.8);
                break;
            case "Center":
                driveTime(1600, 0.8); // 0.5
                break;
            default:
                driveTime(1600, 0.8);
        }
        charlie.markerLock.setPosition(1.0);
        wait(100);
        charlie.teamMarker.setPosition(1.0); // drop team marker
        wait(1000);
        driveTime(1700, -0.8, -0.8);
        /*
        driveTime(300, 0.4); // 0.4  forwards slightly
        wait(200);
        driveTime(1200, -0.55, 0.55); // -0.4, 0.4  turn towards wall
        driveTime(500, 0.8); // 0.4  move to wall
        driveDistanceFront(20, 0.6); // 0.4  inch towards wall w/distance
        driveTime(800, -0.8, 0.8); // -0.4, 0.4  turn towards depot
        driveDistanceSFT(1300, 8, 0.8);
        driveDistanceSF(65, 4, 0.75); // 30, 11, 0.3  drive along wall
        charlie.halt(); // stop
        charlie.markerLock.setPosition(1.0);
        wait(100);
        charlie.teamMarker.setPosition(1.0); // drop team marker
        wait(1000); // wait for it to drop

        driveDistanceSB(3000, 8, -0.53); // 2000, 8, -0.4  reverse to crater
        */

        charlie.halt();
        vuforiaRoverRuckus.close();
        tfodRoverRuckus.close();
    }
    private boolean passCheck() {
        return opModeIsActive() && !isStopRequested();
    }
    private long getTime() {
        return charlie.moveTimer.time(TimeUnit.MILLISECONDS);
    }
    private void moveRackMotor(double pwr) {
        charlie.rack.setPower(pwr);
    }
    private void moveRackMotor() {
        charlie.rack.setPower(0);
    }
    private void wait(int time) {
        charlie.resetTimer();
        while(getTime() < time && passCheck()) {
        }
    }
    private void driveTime(int time, double pwrLeft, double pwrRight) {
        charlie.resetTimer();
        while (getTime() < time && passCheck()) {
            charlie.moveDriveMotors(pwrLeft, pwrRight);

        }
        charlie.halt();
    }
    private void driveTime(int time, double pwr) {
        charlie.resetTimer();
        while (getTime() < time && passCheck()) {
            charlie.moveDriveMotors(pwr);
        }
        charlie.halt();
    }
    private void liftByTime(int time, double pwr) {
        charlie.resetTimer();
        while (getTime() < time && passCheck()) {
            charlie.liftControl(pwr);
        }
        charlie.liftControl();
    }
    private void liftMagnet(double pwr) {
        while (charlie.magnet.getState() && passCheck()) {
            charlie.liftControl(pwr);
        }
        charlie.liftControl();
    }
    private void driveDistanceSF(int dist, int distS, double pwr) {
        double pwrR=pwr;
        double pwrL=pwr;
        while (charlie.rightFrontDistance.getDistance(DistanceUnit.CM) > dist && passCheck()) {
            if (charlie.rightDistance.getDistance(DistanceUnit.CM) < distS)
            {
                pwrL=pwrL*0.8;
                pwrR=pwrR*1.2;
            }
            else if(charlie.rightRearDistance.getDistance(DistanceUnit.CM)<distS)
            {
                pwrL=pwrL*1.2;
                pwrR=pwrR*.8;
            }
            else {
                pwrL=pwr;
                pwrR=pwr;
            }
            telemetry.addData("pwrL", pwrL);
            telemetry.addData("pwrR", pwrR);
            telemetry.addData("Distance Forward", charlie.rightFrontDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Distance Right Rear", charlie.rightRearDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Distance Right", charlie.rightDistance.getDistance(DistanceUnit.CM));
            telemetry.update();
            charlie.moveDriveMotors(pwrL, pwrR);
        }
        charlie.halt();
    }
    private void driveDistanceSFT(int time, int distS, double pwr) {
        double pwrR=pwr;
        double pwrL=pwr;
        int thing;
        charlie.resetTimer();
        while (getTime() < time && passCheck()) {
            if (charlie.rightDistance.getDistance(DistanceUnit.CM) < distS)
            {
                pwrL=pwrL*0.8;
                pwrR=pwrR*1.2;
            }
            else {
                pwrL=pwr;
                pwrR=pwr;
            }
            telemetry.addData("pwrL", pwrL);
            telemetry.addData("pwrR", pwrR);
            telemetry.addData("Distance Forward", charlie.rightFrontDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Distance Right", charlie.rightDistance.getDistance(DistanceUnit.CM));
            telemetry.update();
            charlie.moveDriveMotors(pwrL, pwrR);
        }
        charlie.halt();
    }
    private void driveDistanceSB(int time, int distS, double pwr) {
        double pwrR=pwr;
        double pwrL=pwr;
        charlie.resetTimer();
        while (getTime() < time && passCheck()) {
            if (charlie.rightDistance.getDistance(DistanceUnit.CM) < distS)
            {
                pwrL=pwrL*0.8;
                pwrR=pwrR*1.2;
            }
            else if(charlie.rightRearDistance.getDistance(DistanceUnit.CM)<distS)
            {
                pwrL=pwrL*1.2;
                pwrR=pwrR*.8;
            }
            else
            {
                pwrR=pwr;
                pwrL=pwr;
            }
            telemetry.addData("pwrL", pwrL);
            telemetry.addData("pwrR", pwrR);
            telemetry.addData("Distance Forward", charlie.rightFrontDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Distance Right Rear", charlie.rightDistance.getDistance(DistanceUnit.CM));
            telemetry.update();
            charlie.moveDriveMotors(pwrL, pwrR);
        }
        charlie.halt();
    }
    private void driveDistanceCenter(int dist, double pwr) {
        while (charlie.rightFrontDistance.getDistance(DistanceUnit.CM) > dist && passCheck()) {
            charlie.moveDriveMotors(pwr);
            telemetry.addData("dist", charlie.rightFrontDistance.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
    private void driveDistanceFront(int dist, double pwr) {
        while (charlie.rightFrontDistance.getDistance(DistanceUnit.CM) > dist && passCheck()) {
            charlie.moveDriveMotors(pwr);
            telemetry.addData("dist", charlie.rightFrontDistance.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
    private void dropFromLander() {
        charlie.teamMarker.setPosition(0.0);
        charlie.markerLock.setPosition(0.0);
        liftMagnet(-0.7);
        wait(200);
        driveTime(80, -0.4);
        driveTime(900, 0, 0.4);
        liftByTime(900, .75);
        charlie.rack.setPower(0);
        wait(200);
        driveTime(910, 0, -0.4);
    }
}
