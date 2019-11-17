package org.firstinspires.ftc.teamcode.Enabled_Classes.purepursuit;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Enabled_Classes.robotControl;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.Enabled_Classes.purepursuit.MovementVars.*;
import static org.firstinspires.ftc.teamcode.Enabled_Classes.purepursuit.Robot.*;
import static org.firstinspires.ftc.teamcode.Enabled_Classes.purepursuit.MathFunctions.AngleWrap;
import static org.firstinspires.ftc.teamcode.Enabled_Classes.purepursuit.MathFunctions.lineCircleIntersection;

public class RobotMovement {
    public static void followCurve(ArrayList<CurvePoint> allPoints, double followAngle, robotControl robot) {
//        for(int i = 0; i < allPoints.size() - 1; i++) {
//            ComputerDebugging.sendLine(new Point(allPoints.get(i).x,allPoints.get(i).y), new Point(allPoints.get(i + 1).x, allPoints.get(i + 1).y));
//        }

        CurvePoint followMe = getFollowPointPath(allPoints, new Point(worldXPosition, worldYPosition), allPoints.get(0).followDistance);
//        ComputerDebugging.sendKeyPoint(new Point(followMe.x, followMe.y));

        goToPosition(followMe.x, followMe.y, followMe.moveSpeed, followAngle, followMe.turnSpeed, robot);
    }

    public static CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Point robotPosition, double followRadius) {
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));

        for(int i = 0; i < pathPoints.size() - 1; i++) {
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i + 1);

            ArrayList<Point> intersections = lineCircleIntersection(robotPosition, followRadius, startLine.toPoint(), endLine.toPoint());

            double closestAngle = 10000000;
            for(Point thisIntersection : intersections) {
                double angle = Math.atan2(thisIntersection.y - worldYPosition, thisIntersection.x - worldXPosition);
                double deltaAngle = Math.abs(MathFunctions.AngleWrap(angle - worldAngle_rad));

                if(deltaAngle < closestAngle) {
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                }
            }
        }
        return followMe;
    }


    public static void goToPosition(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed, robotControl robot) {
        double distanceToTarget = Math.hypot(x-worldXPosition, y - worldYPosition);

        double absoluteAngleToTarget = Math.atan2(y-worldYPosition, x-worldXPosition);

        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (worldAngle_rad - Math.toRadians(90)));

        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;


        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        movement_x = movementXPower * movementSpeed;
        movement_y = movementYPower * movementSpeed;

        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;
        movement_turn = Range.clip(relativeTurnAngle/Math.toRadians(30), -1, 1) * turnSpeed;

        if(distanceToTarget < 10) {
            movement_turn = 0;
        }

        robot.rightFront.setPower(Range.clip(movement_y - movement_x + movement_turn, -1, 1));
        robot.leftRear.setPower(Range.clip(movement_y - movement_x - movement_turn, -1, 1));
        robot.leftFront.setPower(Range.clip(movement_y + movement_x - movement_turn, -1, 1));
        robot.rightRear.setPower(Range.clip(movement_y + movement_x + movement_turn, -1, 1));
    }
}
