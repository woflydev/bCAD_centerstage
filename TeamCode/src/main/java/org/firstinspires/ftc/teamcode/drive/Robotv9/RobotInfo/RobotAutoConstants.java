package org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class RobotAutoConstants {
    // -------------------------------------------------------------- AUTO CONFIG
    public static final Pose2d[] RED_STARTING_POSES = {
            new Pose2d(11, -60, Math.toRadians(90)),
            new Pose2d(-35, -60, Math.toRadians(90)),
    };
    public static final Pose2d[] BLUE_STARTING_POSES = {
            new Pose2d(11, 60, Math.toRadians(270)),
            new Pose2d(-35, 60, Math.toRadians(270)),
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
    public static final Pose2d[] BACKDROP_CENTER_POSES = {
            new Pose2d(43, -35.00, Math.toRadians(180)),
            new Pose2d(43, 35.00, Math.toRadians(180)),
    };
    public static final Pose2d[] RED_YELLOW_PIXEL_BACKDROP_POSES = {
            // note: starts with LOC_1
            new Pose2d(42, -28, Math.toRadians(180)),
            new Pose2d(42, -36, Math.toRadians(180)),
            new Pose2d(42, -42, Math.toRadians(180)),
    };
    public static final Pose2d[] BLUE_YELLOW_PIXEL_BACKDROP_POSES = {
            new Pose2d(42, 42, Math.toRadians(180)),
            new Pose2d(42, 36, Math.toRadians(180)),
            new Pose2d(42, 28, Math.toRadians(180)),
    };
    // how many units to get audience spikemark from backdrop
    public static final double AUDIENCE_OFFSET_AMOUNT = 48;
    public static final Pose2d[] RED_PURPLE_PIXEL_SPIKEMARK_POSES = {
            // note: backdrop, from Loc 1 to Loc 3. is modified with AUDIENCE_OFFSET_AMOUNT in runtime.
            new Pose2d(10, -42, Math.toRadians(135)),
            new Pose2d(12, -38, Math.toRadians(90)),
            new Pose2d(14, -42, Math.toRadians(45))
    };
    public static final Pose2d[] BLUE_PURPLE_PIXEL_SPIKEMARK_POSES = {
            new Pose2d(14, 42, Math.toRadians(315)),
            new Pose2d(12, 38, Math.toRadians(270)),
            new Pose2d(10, 42, Math.toRadians(225))
    };

    // note: below vars used for cycling
    public static final Pose2d[] CYCLING_STACK_KNOCK_POSES = {
            new Pose2d(new Vector2d(-55, -2), Math.toRadians(180)),
            new Pose2d(new Vector2d(-55, 2), Math.toRadians(180))
    };
    public static final Pose2d[] CYCLING_STACK_INNER_POSES = {
            // note: again, red is first
            new Pose2d(new Vector2d(-57.5, -12), Math.toRadians(180.00)),
            new Pose2d(new Vector2d(-57.5, 12), Math.toRadians(180.00)),
    };
    public static final Pose2d[] STAGE_DOOR_POSES = {
            new Pose2d(new Vector2d(16, -4), Math.toRadians(180.0)), // note: old values - (28, 8)
            new Pose2d(new Vector2d(16, 4), Math.toRadians(180.0)),
    };
    public static final Pose2d[] CYCLE_RETURN_POSES = {
            new Pose2d(new Vector2d(30, -5), Math.toRadians(180.0)),
            new Pose2d(new Vector2d(30, 5), Math.toRadians(180.0)),
    };

    public static final double CAUTION_SPEED = 14;
    public static final double DEPOSIT_YELLOW_TO_BACKDROP_TRANSIT_RED = 1.55;
    public static final double DEPOSIT_YELLOW_TO_BACKDROP_TRANSIT_BLUE = 1.45;

    public static final double CYCLE_STACK_APPROACH_AMOUNT = 0.05;
    public static final double CYCLE_STACK_REVERSE_AMOUNT = CYCLE_STACK_APPROACH_AMOUNT - 0.22;
    public static final double CYCLE_BACKDROP_APPROACH_AMOUNT = 0.45;

    public static final double AUDIENCE_PURPLE_APPROACH_SPEED = 20;
    public static final double AUDIENCE_YELLOW_BACKDROP_APPROACH_AMOUNT = 0.485;
}
