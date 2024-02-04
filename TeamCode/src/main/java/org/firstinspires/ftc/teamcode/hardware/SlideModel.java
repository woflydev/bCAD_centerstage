package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SlideModel {
    //Distances all in mm

    public static int MAXENCODER = 1250;
    public static double SPOOLDIAMETER = 125.66;
    public static double PPR = 145.1;
    public static double MAXEXTENSION = (MAXENCODER/PPR) * SPOOLDIAMETER;
    public static double ARMLENGTH = 125.50; //distance from pivot to pivot
    public static double SLIDEANGLE = Math.toRadians(30); //degrees
    public static double WRISTLENGTH = 90; //mm
    public static double MAXWRISTANGLE = Math.toRadians(270); //degrees
    public static double MINWRISTANGLE = Math.toRadians(0); //degrees
    public static double WRISTANGLE = Math.toRadians(-30); //degrees

    public static double BUFFER = 0.05; // Amount of room at top of touchpad
    public static double MAXPOSITION = MAXEXTENSION + ARMLENGTH;


    public double distanceToEncoder(double distance) {
        return (PPR * (distance / SPOOLDIAMETER));
    }

    public double radiansToServo(double angle){
        double servoRange = MAXWRISTANGLE - MINWRISTANGLE;
        // Calculate the normalized position of the angle in the servo range
        double normalizedPosition = (angle - MINWRISTANGLE) / servoRange;
        // Convert the normalized position into a servo position between 1 and 0
        double servoPosition = 1 - normalizedPosition;
        // Return the servo position
        return servoPosition;
    }

    /**
     * @param targetX
     * @param targetY
     * @return
     */
    public double[] inverseKinematics(int targetX, int targetY) {
        int tx = targetX - (int)(Math.cos(WRISTANGLE)*WRISTLENGTH); // Adjusts for static wrist angle (essentially removes wrist from equation)
        double ty = targetY - (int)(Math.sin(WRISTANGLE)*WRISTLENGTH);
        if(Math.atan2(ty, tx) > SLIDEANGLE) return null;
        double minDistance = minDistance(-Math.tan(SLIDEANGLE), 1, 0, tx,ty);
        if(minDistance > ARMLENGTH) return null;

        double[] point = closestPoint(-Math.tan(SLIDEANGLE), 1, 0, tx,ty);

        double parallelDist = Math.sqrt((ARMLENGTH*ARMLENGTH) - (minDistance*minDistance));
        double armAngle;
        double tempY1 = point[1] - (Math.sin(SLIDEANGLE)*parallelDist);
        double tempY2 = point[1] - (Math.sin(SLIDEANGLE+Math.toRadians(180))*parallelDist);
        double slideDistanceSolution1 = Math.hypot(point[0] - (Math.cos(SLIDEANGLE)*parallelDist), tempY1);
        //double slideDistanceSolution2 = Math.hypot(point[0] - (Math.cos(SLIDEANGLE+Math.toRadians(180))*parallelDist), tempY2);

        if(slideDistanceSolution1 < MAXEXTENSION && tempY1 > 0){
            armAngle = Math.atan2(parallelDist, minDistance)- (Math.toRadians(90)-SLIDEANGLE);
            return new double[]{slideDistanceSolution1, armAngle};
        }   else return null;

    }

    public double minDistance(double a, double b, double c, double x0, double y0) {
        // Use the formula d = |ax0 + by0 + c| / sqrt(a^2 + b^2)
        double numerator = Math.abs(a * x0 + b * y0 + c);
        double denominator = Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2));
        return numerator / denominator;
    }

    public double[] closestPoint(double a, double b, double c, double x0, double y0) {
        // Use the formula x = (b(bx0 - ay0) - ac) / (a^2 + b^2) and y = (a(-bx0 + ay0) - bc) / (a^2 + b^2)
        double x = (b * (b * x0 - a * y0) - a * c) / (Math.pow(a, 2) + Math.pow(b, 2));
        double y = (a * (-b * x0 + a * y0) - b * c) / (Math.pow(a, 2) + Math.pow(b, 2));
        return new double[] {x, y};
    }

    public int[] convertInputToLocation(double x, double y, Telemetry telemetry) {
        /*
        Mapping:
            Touchpad [-1, -1]
            IK [0, 0]

            Touchpad [1, 1]
            IK [MAXPOSITION * Math.sin(SLIDEANGLE), MAXPOSITION * Math.cos(SLIDEANGLE)]
         */

        x = Range.clip(x, -1 ,1 ); // Occasionally touchpad returns unusual value
        y = Range.clip(y, -1 ,1 );

        double newX = (x+1)/2 * MAXPOSITION * Math.cos(SLIDEANGLE);
        double newY = (y+1)/2 * MAXPOSITION * Math.sin(SLIDEANGLE);

        telemetry.addData("Real X Coordinate: ", newX);
        telemetry.addData("Real Y Coordinate: ", newY);

        return new int[] {(int) newX, (int) newY};
    }
}
