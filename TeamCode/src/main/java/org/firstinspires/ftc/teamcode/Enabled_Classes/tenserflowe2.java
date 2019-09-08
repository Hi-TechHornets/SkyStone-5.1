package org.firstinspires.ftc.teamcode.Enabled_Classes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaRoverRuckus;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "TenserFlowDevBuild", group = "Debug")
public class tenserflowe2 extends LinearOpMode {

    private VuforiaRoverRuckus vuforiaRoverRuckus;
    private TfodRoverRuckus tfodRoverRuckus;
    private ElapsedTime moveTimer = new ElapsedTime(System.nanoTime());
    //private DcMotor rightFront;
    //private DcMotor rightRear;
    //private DcMotor leftFront;
    //private DcMotor leftRear;
    //private DcMotor rightCollection;
    //private DcMotor leftCollection;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        List<Recognition> recognitions;
        double goldMineralX;
        double silverMineral1X;
        double silverMineral2X;
        //rightFront = hardwareMap.dcMotor.get("RightFront");
        //rightFront.setDirection(DcMotor.Direction.REVERSE);
        //rightRear = hardwareMap.dcMotor.get("RightRear");
        //rightRear.setDirection(DcMotor.Direction.REVERSE);
        //leftFront = hardwareMap.dcMotor.get("LeftFront");
        //leftRear = hardwareMap.dcMotor.get("LeftRear");
        //rightCollection = hardwareMap.dcMotor.get("rightCollection");
        //leftCollection = hardwareMap.dcMotor.get("leftCollection");
        vuforiaRoverRuckus = new VuforiaRoverRuckus();
        tfodRoverRuckus = new TfodRoverRuckus();
        String position = "";
        // Put initialization blocks here.
        vuforiaRoverRuckus.initialize("", hardwareMap.get(WebcamName.class, "Webcam 1"), "teamwebcamcalibrations.xml",
                true, true, VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES,
                0, 0, 0, 0, 0, 0, true);tfodRoverRuckus.initialize(vuforiaRoverRuckus, (float)0.55, true, true);
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {

            if (tfodRoverRuckus != null) {
                tfodRoverRuckus.activate();
            }
            // Put run blocks here.
            resetTimer();
            while(passCheck()) {
                if (tfodRoverRuckus != null) {
                    recognitions = tfodRoverRuckus.getRecognitions();
                    telemetry.addData("# Objects Recognized", recognitions.size());
                    if (recognitions.size() == 2) {
                        goldMineralX = -1;
                        silverMineral1X = -1;
                        silverMineral2X = -1;
                        // fixed
                        for (Recognition recognition : recognitions) {
                            if (recognition.getLabel().equals("Gold Mineral")) {
                                goldMineralX = recognition.getLeft();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = recognition.getLeft();
                            } else {
                                silverMineral2X = recognition.getLeft();
                            }
                        }
                        // Make sure we found one gold mineral and two silver minerals.
                        // If there is no gold and two silvers visible, then it is on the right
                        if (goldMineralX == -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                            telemetry.addData("Gold Mineral Position", "Right");
                            position = "Right";
                        }
                        // If it can see one gold and one silver...
                        else if (goldMineralX != -1 && silverMineral1X != -1) {
                            if (goldMineralX > silverMineral1X) {
                                // ... if the gold is to the right, then it is in the center
                                telemetry.addData("Gold Mineral Position", "Center");
                                position = "Center";
                            } else {
                                // ... if the gold is to the left, then it is on the left
                                telemetry.addData("Gold Mineral Position", "Left");
                                position = "Left";
                            }
                        }
                        telemetry.update();
                        /*
                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Left");
                                position = "Left";
                            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Right");
                                position = "Right";
                            } else {
                                telemetry.addData("Gold Mineral Position", "Center");
                                position = "Center";
                            }
                        }*/
                    }
                }
                telemetry.update();
            }
        }
        vuforiaRoverRuckus.close();
        tfodRoverRuckus.close();
        /*
        telemetry.addData("Gem", position);
        telemetry.update();
        resetTimer();
        while (moveTimer.time(TimeUnit.SECONDS) < 3 && passCheck()) {

        }
        switch (position) {
            case "Left":
                resetTimer();
                while (getTime() < 490 && passCheck()) {
                    moveMotors(-0.4, 0.4);
                }
                moveMotors(0, 0);
                resetTimer();
                while (getTime() < 500 && passCheck()) {
                    moveMotors(0, 0);
                }
                resetTimer();
                while (getTime() < 1500 && passCheck()) {
                    moveMotors(0.75, 0.65);
                }
                resetTimer();
                while (getTime() < 500 && passCheck()) {
                    moveMotors(0, 0);
                }
                resetTimer();
                while (getTime() < 1500 && passCheck()) {
                    moveMotors(-0.75, -0.65);
                }
                moveMotors(0, 0);
                break;
            case "Center":
                resetTimer();
                while (getTime() < 2000 && passCheck()) {
                    moveMotors(0.8, 0.7);
                }
                moveMotors(0, 0);
                break;
            case "Right":
                resetTimer();
                while (getTime() < 490 && passCheck()) {
                    moveMotors(0.4, -0.4);
                }
                moveMotors(0, 0);
                resetTimer();
                while (getTime() < 500 && passCheck()) {
                    moveMotors(0, 0);
                }
                resetTimer();
                while (getTime() < 1500 && passCheck()) {
                    moveMotors(0.75, 0.65);
                }
                resetTimer();
                while (getTime() < 500 && passCheck()) {
                    moveMotors(0, 0);
                }
                resetTimer();
                while (getTime() < 1500 && passCheck()) {
                    moveMotors(-0.75, -0.65);
                }
                moveMotors(0, 0);
                break;
            default:
                resetTimer();
                while (getTime() < 2000 && passCheck()) {
                    moveMotors(0.8, 0.7);
                }
                moveMotors(0, 0);
                break;
        }
        moveMotors(0, 0);
        */
    }

    private void resetTimer() {
        moveTimer.reset();
    }
    private boolean passCheck() {
        return opModeIsActive() && !isStopRequested();
    }
    private long getTime() {
        return moveTimer.time(TimeUnit.MILLISECONDS);
    }
    /*
    private void moveMotors(double power) {
        rightFront.setPower(power);
        rightRear.setPower(power);
        leftFront.setPower(power);
        leftFront.setPower(power);
    }
    private void moveMotors() {
        rightFront.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        leftFront.setPower(0);
    }
    private void moveMotors(double powerLeft, double powerRight) {
        rightFront.setPower(powerRight);
        rightRear.setPower(powerRight);
        leftFront.setPower(powerLeft);
        leftFront.setPower(powerLeft);
    }
    */
}
