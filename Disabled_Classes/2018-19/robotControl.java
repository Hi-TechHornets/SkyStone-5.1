
package org.firstinspires.ftc.teamcode.Enabled_Classes;

import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus;

import java.io.File;
import java.util.Collection;
import java.util.List;

public class robotControl{

    private Gyroscope imu;
    private File[] webcamConfig;

    //ToDo: add webcam

    public DcMotor rightFront;
    public DcMotor rightRear;
    public DcMotor leftFront;
    public DcMotor leftRear;

    public DcMotor armBase;
    public DcMotor armExtender;

    public DistanceSensor rightRearDistance;
    //public DistanceSensor leftDistance;
    public DistanceSensor rightFrontDistance;
    public DistanceSensor rightDistance;

    public DigitalChannel magnet;
    public DcMotor rack;

    //public DcMotor rightCollection;
    //public DcMotor leftCollection;
    //public DcMotor spool;
    //public CRServo intake;
    public Servo markerLock;
    public Servo teamMarker;
    public Servo jaw;


    public Servo rightTilt;
    public Servo leftTilt;

    private HardwareMap hardwareMap=  null;

    public ElapsedTime moveTimer = new ElapsedTime();

    private Collection<Blinker.Step> forward;
    private Collection<Blinker.Step> raise;

    public robotControl()
    {

    }
    void init(HardwareMap ahwMap) {
        hardwareMap= ahwMap;
        rightFront = hardwareMap.dcMotor.get("RightFront");
        rightRear = hardwareMap.dcMotor.get("RightRear");
        leftFront= hardwareMap.dcMotor.get("LeftFront");
        leftRear = hardwareMap.dcMotor.get("LeftRear");

        armBase= hardwareMap.dcMotor.get("armBase");
        armExtender = hardwareMap.dcMotor.get("armExtend");

        rightRearDistance = hardwareMap.get(DistanceSensor.class, "rightRearDistance");
        //leftDistance = hardwareMap.get(DistanceSensor.class, "leftDistance");
        rightFrontDistance = hardwareMap.get(DistanceSensor.class, "rightFrontDistance");
        rightDistance =hardwareMap.get(DistanceSensor.class,"rightDistance");
        magnet = hardwareMap.get(DigitalChannel.class, "magnet");
        rack= hardwareMap.dcMotor.get("rack");
        jaw = hardwareMap.servo.get("jaw");

        //rightCollection = hardwareMap.dcMotor.get("rightCollection");
        //leftCollection = hardwareMap.dcMotor.get("leftCollection");

        //spool = hardwareMap.dcMotor.get("spool");
        //intake = hardwareMap.crservo.get("intake");

        teamMarker = hardwareMap.servo.get("teamMarker");

        markerLock = hardwareMap.servo.get("markerLock");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rack.setDirection(DcMotor.Direction.REVERSE);


        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightTilt = hardwareMap.servo.get("rightTilt");
        //leftTilt = hardwareMap.servo.get("leftTilt");
    }
    void diagnosticStartUpMotors(HardwareMap ahwMap) {
        //ToDo: make a step by step start up sequence
        hardwareMap= ahwMap;
        rightFront = hardwareMap.dcMotor.get("RightFront");
        rightRear = hardwareMap.dcMotor.get("RightRear");
        leftFront= hardwareMap.dcMotor.get("LeftFront");
        leftRear = hardwareMap.dcMotor.get("LeftRear");
    }

    void moveDriveMotors(double power) {
        moveDriveMotors(power*1.1, power);

        // Sets all motors to same power level
    }
    //ToDo: Add in a balance adjustment
    void halt() {
        moveDriveMotors(0, 0);
        //Sets all motors to 0 power
    }
    void moveDriveMotors()
    {
        moveDriveMotors(0);
    }

    void moveDriveMotors(double powerLeft, double powerRight) {
        //ToDo: #3 *1.15
        rightFront.setPower(powerRight );
        rightRear.setPower(powerRight );
        leftFront.setPower(powerLeft );
        leftRear.setPower(powerLeft );
    }

    void moveDriveMotors(double pwrLeftFront, double pwrLeftRear, double pwrRightFront, double pwrRightRear) {
        rightFront.setPower(pwrRightFront * 1.15);
        rightRear.setPower(pwrRightRear * 1.15);
        leftFront.setPower(pwrLeftFront * 1.15);
        leftRear.setPower(pwrLeftRear * 1.15);
    }

    public void resetEncoders()
    {
        //ToDo: Reset Encoders
    }

    public void liftControl(double pwr) {
        rack.setPower(pwr);
    }
    public void liftControl() {
        rack.setPower(0);
    }
    public void resetTimer() {
        moveTimer.reset();
    }

    public String sampling(Telemetry telemetry, TfodRoverRuckus tfodRoverRuckus/*, List<Recognition> recognitions*/) {
        List<Recognition> recognitions;
        double goldMineralX;
        double silverMineral1X;
        double silverMineral2X;
        String position = "";
        if (tfodRoverRuckus != null) {
            recognitions = tfodRoverRuckus.getRecognitions();
            telemetry.addData("# Objects Recognized", recognitions.size());
            // Changed from 3 to 2

            if (recognitions.size() == 2)
            {
                goldMineralX = -1;
                silverMineral1X = -1;
                silverMineral2X = -1;
                // This just records values, and is unchanged
                for (Recognition recognition : recognitions)
                {
                    if (recognition.getLabel().equals("Gold Mineral"))
                    {
                        goldMineralX = (int) recognition.getLeft();
                    }
                    else if (silverMineral1X == -1)
                    {
                        silverMineral1X = (int) recognition.getLeft();
                    }
                    else
                    {
                        silverMineral2X = (int) recognition.getLeft();
                    }
                }
                // If there is no gold (-1) and there two silvers (not -1) the gold
                // is not visible, and must be on the right
                if (goldMineralX == -1 && silverMineral1X != -1 && silverMineral2X != -1)
                {
                    telemetry.addData("Gold Mineral Position", "Right");
                    position = "Right";
                }
                // If you can see one gold and one silver ...
                else if (goldMineralX != -1 && silverMineral1X != -1)
                {
                    // ... if the gold is to the right of the silver, the gold is in the center ...
                    if (goldMineralX > silverMineral1X)
                    {
                        telemetry.addData("Gold Mineral Position", "Center");
                        position = "Center";
                    }
                    // ... otherwise it is on the left
                    else
                    {
                        telemetry.addData("Gold Mineral Position", "Left");
                        position = "Left";
                    }
                }
            }
        }
        telemetry.update();
        return position;
    }
}


