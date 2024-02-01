package org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {
    // -------------------------------------------------------------- ROBOT CONFIG

    public static final String FRONT_LEFT = "frontL";
    public static final String FRONT_RIGHT = "frontR";
    public static final String BACK_LEFT = "backL";
    public static final String BACK_RIGHT = "backR";
    public static final String LIFT_R = "liftR";
    public static final String LIFT_L = "liftL";
    public static final String INTAKE_MOTOR = "intake";
    public static final String SERVO_FLAP = "flap";
    public static final String SERVO_CLAW = "claw";
    public static final String SERVO_WRIST = "wrist";
    public static final String SERVO_SPIN = "spin";
    public static final String SERVO_ELBOW = "elbow";
    public static final String SERVO_PLANE = "plane";
    public static final String HUB_IMU = "imu";
    public static final String FRONT_CAMERA = "Webcam 1";
    public static final String BACK_CAMERA = "Webcam 2";
    public static final String SERVO_HANG_R = "hangR";
    public static final String SERVO_HANG_L = "hangL";

    public static final double MAX_LIFT_SPEED = 0.8;
    public static final int LIFT_RESET_TIMEOUT = 4; // note: how many seconds the lift should be active for before brought down automatically
    public static final int MAX_OUTTAKE_HEIGHT = 3200;
    public static final int MIN_OUTTAKE_HEIGHT = 0;

    public static final double FLAP_CLOSE = 200;
    public static final double FLAP_OPEN = 114;
    public static final double CLAW_CLOSE = 149;
    public static final double CLAW_AUTO_CLOSE = 100;
    public static final double CLAW_OPEN = 98;
    public static final double CLAW_DEPOSIT = 50;
    public static final double ELBOW_HOME = 235;
    public static final double ELBOW_PICKUP = 273;
    public static final double ELBOW_GRABBED_STANDBY = 225;
    public static final double ELBOW_ACTIVE = 130;
    public static final double ELBOW_AUTO_ACTIVE = 10;
    public static final double ELBOW_CYCLE_ACTIVE = ELBOW_AUTO_ACTIVE - 10;
    public static final double WRIST_PICKUP = 260;
    public static final double WRIST_HOME = 210; // old: 170
    public static final double WRIST_ACTIVE = 145;
    public static final double WRIST_AUTO_ACTIVE = 230;
    public static final double WRIST_CYCLE_ACTIVE = WRIST_AUTO_ACTIVE + 10;

    public static final double PLANE_HOME = 30;
    public static final double PLANE_ACTIVE = 0;
    public static final double INTAKE_SPEED = 0.8;
    public static final double INTAKE_REVERSE_SPEED = 0.8;
    public static final int SPIN_HOME = 25;
    public static final int SPIN_DEPOSIT = SPIN_HOME + 90;

    // -------------------------------------------------------------- JUNCTION PRESETS

    public static final int JUNCTION_AUTO_YELLOW = 495;
    public static final int JUNCTION_AUTO_WHITE = 637;
    public static final int JUNCTION_LOW = 300;
    public static final int JUNCTION_MID = 600;
    public static final int JUNCTION_HIGH = 900;

    // -------------------------------------------------------------- VISION

    public static final boolean USE_CAMERA_STREAM = true;

    public static final boolean USE_LIVE_VIEW = true;
    public static final boolean USE_DRIVE = true;
    public static final boolean USE_BACK = true;

    public static final double FIELD_LENGTH = 3.58;
    public static final double CAMERA_HEIGHT = 0.313;
    public static final double WALL_TAG_X = 1.005;
    public static final double SMALL_WALL_TAG_X = 0.9;

    public static final double BACKDROP_DEPTH = 1.55;
    public static final double TAG_HEIGHT = 0.12;

    public static final double PIXEL_SPACE = 0.05;

    public static final double ROAD_RUNNER_SCALE = 72 / (FIELD_LENGTH / 2);

    public static final double INTAKE_POWER = 0.5;
    public static final double INTAKE_TIME = 1;

    public static final double TRUSS_WIDTH = 0.9;

    public static final Double[] PATH_Y = {-1.46, -0.83, 0d, 0.83, 1.46};

    public static final double HEADING = Math.PI / 2;

    public static final double BACKDROP_ANGLE = - Math.PI / 2;

    public static final double TAG_WALL_ANGLE = Math.PI / 2;

    public static final double ELBOW_DROPOFF = 0.8;

    public static final double INTAKE_OUTPUT = 0.5;
    public static final double INTAKE_OUTPUT_TIME = 1000;

    public static final int INITIAL_HEIGHT = 15;
    public static final double INITIAL_FORWARD = 0.3;
}
