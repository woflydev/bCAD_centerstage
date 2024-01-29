package org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo;

public class ASubsystemState {
    public enum Drivetrain {
        MANUAL,
        ALIGNING_WITH_BACKDROP,
        ALIGNING_WITH_OUTER_WALL,
    }

    public enum Outtake {
        IDLE,
        ACTIVATED,
        FLAP_OPENING,
        WRIST_PICKING,
        ELBOW_PICKING,
        CLAW_CLOSING,
        PENDING_GRABBED_AND_READY,
        GRABBED_AND_READY,
        PRIMED_FOR_DEPOSIT,
        SERVO_SPINNING,
        PENDING_DEPOSIT,
        CLAW_OPENING,
        OUTTAKE_RESET,
        AWAITING_OUTTAKE_RESET,
        OUTTAKE_RESET_HARD,
    }

    public enum PlaneLauncher {
        IDLE,
        ACTIVE,
    }
}

