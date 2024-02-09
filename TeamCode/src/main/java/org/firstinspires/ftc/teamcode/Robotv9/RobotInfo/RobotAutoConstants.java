package org.firstinspires.ftc.teamcode.Robotv9.RobotInfo;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class RobotAutoConstants {
    // note: -------------------------------------------------------------- NUMBER CONFIG

    public static final double BONK_X_TOLERANCE = 5;
    public static final double BONK_Y_TOLERANCE = 5;
    public static final double BONK_ROT_TOLERANCE = 10;
    public static final double MAX_INTAKE_CURRENT_DRAW = 2.6; // in amps

    // note: -------------------------------------------------------------- POSE CONFIG
    public static final Pose2d[] RED_STARTING_POSES = {
            new Pose2d(8, -62.5, Math.toRadians(270)),
            new Pose2d(-35, -60, Math.toRadians(270)),
    };
    public static final Pose2d[] BLUE_STARTING_POSES = {
            new Pose2d(8, 62.5, Math.toRadians(90)),
            new Pose2d(-35, 60, Math.toRadians(90)),
    };
    public static final Pose2d[] RED_PARKING_POSES = {
            // note: inner is first
            new Pose2d(47, -15, Math.toRadians(90)),
            new Pose2d(46, -60, Math.toRadians(90)),
    };
    public static final Pose2d[] BLUE_PARKING_POSES = {
            new Pose2d(47, 15, Math.toRadians(270)),
            new Pose2d(46, 60, Math.toRadians(270)),
    };
    public static final Pose2d[] RED_YELLOW_PIXEL_BACKDROP_POSES = {
            // note: starts with LOC_1
            new Pose2d(43, -31, Math.toRadians(180)),
            new Pose2d(43, -34.5, Math.toRadians(180)),
            new Pose2d(43, -43, Math.toRadians(180)),
    };
    public static final Pose2d[] BLUE_YELLOW_PIXEL_BACKDROP_POSES = {
            new Pose2d(RED_YELLOW_PIXEL_BACKDROP_POSES[0].getX(), -RED_YELLOW_PIXEL_BACKDROP_POSES[2].getY(), Math.toRadians(180)),
            new Pose2d(RED_YELLOW_PIXEL_BACKDROP_POSES[1].getX(), -RED_YELLOW_PIXEL_BACKDROP_POSES[1].getY(), Math.toRadians(180)),
            new Pose2d(RED_YELLOW_PIXEL_BACKDROP_POSES[2].getX(), -RED_YELLOW_PIXEL_BACKDROP_POSES[0].getY(), Math.toRadians(180)),
    };

    // how many units to get audience spikemark from backdrop
    public static final double AUDIENCE_OFFSET_AMOUNT = 48;
    public static final double AUDIENCE_HEADING_VARIATION = 0;
    // note: backdrop, from Loc 1 to Loc 3.
    public static final Pose2d[] RED_PURPLE_PIXEL_SPIKEMARK_BACKDROP = {
            new Pose2d(10, -30, Math.toRadians(0)),
            new Pose2d(22, -28, Math.toRadians(0)),
            new Pose2d(34, -28, Math.toRadians(0))
    };
    public static final Pose2d[] RED_PURPLE_SPIKEMARK_AUDIENCE = {
            new Pose2d(-38, -38, Math.toRadians(315)),
            new Pose2d(-36, -35, Math.toRadians(270)),
            new Pose2d(-34, -38, Math.toRadians(225)),
    };
    public static final Pose2d[] RED_PURPLE_CHECKPOINTS = { // note: to avoid bumping purple when transiting
            new Pose2d(-60, -48, Math.toRadians(90)),
            new Pose2d(-58, -10, Math.toRadians(180)),
    };

    // BLUE AUDIENCE VARS
    public static final Pose2d[] BLUE_PURPLE_SPIKEMARK_BACKDROP = {
            new Pose2d(RED_PURPLE_PIXEL_SPIKEMARK_BACKDROP[2].getX(), -RED_PURPLE_PIXEL_SPIKEMARK_BACKDROP[0].getY(), Math.toRadians(0)),
            new Pose2d(RED_PURPLE_PIXEL_SPIKEMARK_BACKDROP[1].getX(), -RED_PURPLE_PIXEL_SPIKEMARK_BACKDROP[1].getY(), Math.toRadians(0)),
            new Pose2d(RED_PURPLE_PIXEL_SPIKEMARK_BACKDROP[0].getX(), -RED_PURPLE_PIXEL_SPIKEMARK_BACKDROP[2].getY(), Math.toRadians(0))
    };
    public static final Pose2d[] BLUE_PURPLE_SPIKEMARK_AUDIENCE = {
            new Pose2d(RED_PURPLE_SPIKEMARK_AUDIENCE[2].getX(), -RED_PURPLE_SPIKEMARK_AUDIENCE[0].getY(), Math.toRadians(135)),
            new Pose2d(RED_PURPLE_SPIKEMARK_AUDIENCE[1].getX(), -RED_PURPLE_SPIKEMARK_AUDIENCE[1].getY(), Math.toRadians(90)),
            new Pose2d(RED_PURPLE_SPIKEMARK_AUDIENCE[0].getX(), -RED_PURPLE_SPIKEMARK_AUDIENCE[2].getY(), Math.toRadians(45)),
    };
    public static final Pose2d[] BLUE_PURPLE_CHECKPOINTS = {
            new Pose2d(RED_PURPLE_CHECKPOINTS[0].getX(), -RED_PURPLE_CHECKPOINTS[0].getY(), Math.toRadians(270)),
            new Pose2d(RED_PURPLE_CHECKPOINTS[0].getX(), -RED_PURPLE_CHECKPOINTS[1].getY(), Math.toRadians(180)),
    };

    // CYCLING VARS
    public static final Pose2d[] BACKDROP_CYCLE_DROPOFF_POSES = {
            // note: red first
            new Pose2d(35, -35, Math.toRadians(180)),
            new Pose2d(35, 35, Math.toRadians(180)),
    };
    public static final Pose2d[] CYCLING_STACK_KNOCK_POSES = {
            new Pose2d(-55, -2, Math.toRadians(180)),
            new Pose2d(-55, 2, Math.toRadians(180))
    };
    public static final Pose2d[] CYCLING_STACK_INNER_POSES = {
            // note: again, red is first
            new Pose2d(-60, -14, Math.toRadians(200)),
            new Pose2d(-60, 14, Math.toRadians(160)),
    };
    public static final Pose2d[] STAGE_DOOR_POSES = {
            new Pose2d(16, -4, Math.toRadians(180.0)), // note: old values - (28, 8)
            new Pose2d(16, 4, Math.toRadians(180.0)),
    };
    public static final Pose2d[] CYCLE_RETURN_POSES = {
            new Pose2d(20, -5, Math.toRadians(180.0)),
            new Pose2d(20, 5, Math.toRadians(180.0)),
    };
    public static final Pose2d[] RED_CYCLE_CHECKPOINTS = {
            new Pose2d(-7, -36, Math.toRadians(181.94)),
            new Pose2d(-56, -36, Math.toRadians(180)),
            new Pose2d(-58, -36, Math.toRadians(180)),

            // note: used for recentering the robot to not bonk into the truss
            new Pose2d(-45, -36, Math.toRadians(180)),

    };
    public static final Pose2d[] BLUE_CYCLE_CHECKPOINTS = {
            new Pose2d(RED_CYCLE_CHECKPOINTS[0].getX(), -RED_CYCLE_CHECKPOINTS[0].getY(), Math.toRadians(178.06)),
            new Pose2d(RED_CYCLE_CHECKPOINTS[1].getX(), -RED_CYCLE_CHECKPOINTS[1].getY(), Math.toRadians(260)),
            new Pose2d(RED_CYCLE_CHECKPOINTS[2].getX(), -RED_CYCLE_CHECKPOINTS[2].getY(), Math.toRadians(180)),

            new Pose2d(RED_CYCLE_CHECKPOINTS[3].getX(), -RED_CYCLE_CHECKPOINTS[3].getY(), Math.toRadians(180))
    };

    public static final double CAUTION_SPEED = 14;
}
