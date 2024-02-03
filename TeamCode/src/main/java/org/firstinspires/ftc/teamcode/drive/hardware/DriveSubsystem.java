package org.firstinspires.ftc.teamcode.drive.hardware;

import static org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.RobotAutoConstants.BONK_X_TOLERANCE;
import static org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.RobotAutoConstants.BONK_Y_TOLERANCE;
import static org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.RobotConstants.*;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.RobotAutoConstants;
import org.firstinspires.ftc.teamcode.drive.rr.bCADMecanumDrive;

public class DriveSubsystem extends SubsystemBase {
    public DcMotorEx frontLM;
    public DcMotorEx frontRM;
    public DcMotorEx backLM;
    public DcMotorEx backRM;
    public double speedModifier = 0.7;

    private final int wheelDiameter = 98;                  // Wheel diameter in millimetres
    private final double ticksPerRotation = 751.8;         // Also called encoder resolution
    private final double gearRatio = 2.5;                  // Gear ratio from input to output.

    private final double wheelCircumference = 2 * Math.PI * (wheelDiameter / 2);
    private final double distancePerRotation = (wheelCircumference / 10) * gearRatio;        // in cm
    private final double distancePerTick = distancePerRotation / ticksPerRotation;           // distance per tick in cm

    public DriveSubsystem(HardwareMap hardwareMap) {
        frontLM = hardwareMap.get(DcMotorEx.class, FRONT_LEFT);
        frontRM = hardwareMap.get(DcMotorEx.class, FRONT_RIGHT);
        backLM = hardwareMap.get(DcMotorEx.class, BACK_LEFT);
        backRM = hardwareMap.get(DcMotorEx.class, BACK_RIGHT);

        frontLM.setDirection(DcMotorEx.Direction.REVERSE);
        backLM.setDirection(DcMotorEx.Direction.REVERSE);

        frontLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void Mecanum(Gamepad gamepad1, double botHeading) {
        if (gamepad1.right_trigger >= 0.3) {
            speedModifier = gamepad1.right_trigger + 1.6;
        } else {
            speedModifier = 1;
        }

        double y = -gamepad1.left_stick_y; // y stick reversed
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLM.setPower(frontLeftPower / speedModifier);
        backLM.setPower(backLeftPower / speedModifier);
        frontRM.setPower(frontRightPower / speedModifier);
        backRM.setPower(backRightPower / speedModifier);
    }

    public boolean withinUncertainty(double currentPos, double wantedPos, double range) {
        return (currentPos < wantedPos + range) && currentPos > wantedPos - range;
    }

    public void ResetEncoders(boolean runWithoutEncodersOnReset) {
        frontLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (runWithoutEncodersOnReset) {
            frontLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else {
            frontLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
