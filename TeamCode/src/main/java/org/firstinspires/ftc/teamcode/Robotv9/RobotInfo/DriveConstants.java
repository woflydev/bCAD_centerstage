package org.firstinspires.ftc.teamcode.Robotv9.RobotInfo;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class DriveConstants {
    public static final double RR_HEADING_PID_kP = 8; // old: 9
    public static final double RR_TRANSLATIONAL_PID_kP = 6;

    public static final double RR_LATERAL_MULTI = 1;

    public static final double TICKS_PER_REV = 751.8;
    public static final double MAX_RPM = 223;

    public static final boolean RUN_USING_ENCODER = false;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
            getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

    public static double WHEEL_RADIUS = 1.8898; // in
    public static double GEAR_RATIO = 2;
    public static double TRACK_WIDTH = 11; // old: 9.32555

    public static double kV = 0.015429759176254825;
    public static double kA = 0.003; //old 0.005
    public static double kStatic = 0.05;

    public static double MAX_VEL = 75.02365997; // old 70
    public static double MAX_ACCEL = 65; // old 75

    public static double MAX_ANG_VEL = Math.toRadians(270); //old: 250
    public static double MAX_ANG_ACCEL = Math.toRadians(270);

    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.UP;

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }
}