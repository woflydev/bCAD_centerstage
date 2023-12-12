package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveSubsystem extends SubsystemBase {

    // Declare drivebase motor variables
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx rearLeft;
    public DcMotorEx rearRight;

    // Declare the drivebase
    MecanumDrive drivebase;

    // Declare variables
    public double speedModifier;

    int wheelDiameter = 98;                  // Wheel diameter in millimetres
    double ticksPerRotation = 751.8;         // Also called encoder resolution
    double gearRatio = 2.5;                  // Gear ratio from input to output. For example, if your motor spins one time and the wheel spins twice, your ratio ratio would be 2

    double wheelCircumference = 2 * Math.PI * (wheelDiameter/2);
    double distancePerRotation = (wheelCircumference/10) * gearRatio;        // in cm
    double distancePerTick = distancePerRotation/ticksPerRotation;           // distance per tick in cm

    double START_DRIVE_SLOWDOWN_AT_CM = 50;
    double START_DRIVE_SLOWDOWN_AT_DEGREES = 15;

    public DriveSubsystem(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
        rearLeft = hardwareMap.get(DcMotorEx.class, "RearLeft");
        rearRight = hardwareMap.get(DcMotorEx.class, "RearRight");

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        rearLeft.setDirection(DcMotorEx.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    /*
    public void forward(int distanceCM, int heading, double power, Telemetry telemetry, double botHeading) {
        double requiredTicks = distanceCM/distancePerTick;
        double currentTicks = 0;
        double distancedTravelled;
        double originalPower = power;

        if(currentTicks < requiredTicks) {
            rotate(heading, telemetry, botHeading, 1);
            currentTicks = frontLeft.getCurrentPosition();
            distancedTravelled = currentTicks * distancePerTick;
            telemetry.addData("Distance Left", distanceCM - distancedTravelled);
            telemetry.update();

            if (distanceCM - distancedTravelled < START_DRIVE_SLOWDOWN_AT_CM) {
                power = RobotUtil.scaleVal(distanceCM - distancedTravelled, 0, START_DRIVE_SLOWDOWN_AT_CM, 0.2, originalPower);
            }

            frontLeft.setPower(power);
            frontRight.setPower(power);
            rearLeft.setPower(power);
            rearRight.setPower(power);
        } else {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            rearLeft.setPower(0);
            rearRight.setPower(0);
        }


    }

    public void backwards(int distanceCM, int heading, double power, Telemetry telemetry, double botHeading) {
        double requiredTicks = distanceCM/distancePerTick;
        double currentTicks = 0;
        double distancedTravelled;
        double originalPower = power;

        rotate(heading, telemetry, botHeading, 1);

        while(currentTicks > requiredTicks) {
            currentTicks = frontLeft.getCurrentPosition();
            distancedTravelled = currentTicks * distancePerTick;
            telemetry.addData("Distance Left", distanceCM - distancedTravelled);
            telemetry.update();

            if (distancedTravelled - distanceCM < START_DRIVE_SLOWDOWN_AT_CM) {
                power = RobotUtil.scaleVal(distanceCM - distancedTravelled, 0, START_DRIVE_SLOWDOWN_AT_CM, 0, originalPower/2);
            }

            frontLeft.setPower(power);
            frontRight.setPower(power);
            rearLeft.setPower(power);
            rearRight.setPower(power);
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
    }

    public boolean rotate(double rotation, Telemetry telemetry, double botHeading, double power) {
        telemetry.addData("Bot Heading: ", botHeading);
        double originalPower = power;

        if(withinUncertainty(botHeading, rotation, 2)) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            rearLeft.setPower(0);
            rearRight.setPower(0);
           return true;

        } else if(botHeading < rotation) {
            if (rotation - botHeading < START_DRIVE_SLOWDOWN_AT_DEGREES) {
                power = RobotUtil.scaleVal(rotation - botHeading, 0, START_DRIVE_SLOWDOWN_AT_CM, 0, originalPower);
            }

            frontLeft.setPower(-power);
            frontRight.setPower(power);
            rearLeft.setPower(-power);
            rearRight.setPower(power);

        } else if(botHeading > rotation) {
            if (botHeading - rotation < START_DRIVE_SLOWDOWN_AT_DEGREES) {
                power = RobotUtil.scaleVal(rotation - botHeading, 0, START_DRIVE_SLOWDOWN_AT_CM, 0.1, originalPower);
            }

            frontLeft.setPower(power);
            frontRight.setPower(-power);
            rearLeft.setPower(power);
            rearRight.setPower(-power);

        }

        return false;
    }

    public double getDistanceTravelled() {
        return(frontLeft.getCurrentPosition() * distancePerTick);
    }

    public void resetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

     */

    public void userControlledDrive(Gamepad gamepad1, double botHeading) {
        double speedModifier = 0.7; //Used to be 0.7

        if (gamepad1.left_bumper) {
            speedModifier = 0.3;
        } else if (gamepad1.right_bumper) {
            speedModifier = 1;
        }

//        double y = -gamepad1.right_stick_y; // Remember, Y stick value is reversed
//        double x = gamepad1.right_stick_x;
//        double rx = gamepad1.left_stick_x;

        double y = -gamepad1.right_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.right_stick_x;
        double rx = gamepad1.left_stick_x;

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeft.setPower(frontLeftPower * speedModifier);
        rearLeft.setPower(backLeftPower * speedModifier);
        frontRight.setPower(frontRightPower * speedModifier);
        rearRight.setPower(backRightPower * speedModifier);
    }

    public boolean withinUncertainty(double currentPos, double wantedPos, double range) {
        if((currentPos < wantedPos + range) && currentPos > wantedPos - range) {
            return true;
        } else {
            return false;
        }
    }
}
