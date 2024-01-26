package org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo;

public class AutoState {
    public enum RootAutoState {
        BA_PLAY,
        BA_SCANNING,
        BA_MOVING_TO_SPIKEMARK,
        BA_DEPOSIT_PURPLE,
        BA_MOVING_TO_BACKDROP,
        A_ALIGNING_WITH_YELLOW_TRANSIT_TRAJECTORY,
        A_ALIGNING_WITH_BACKDROP_FOR_DEPOSIT,
        BA_DEPOSIT_YELLOW,
        BA_MOVING_TO_CYCLE,
        BA_INTAKE_PIXELS_FROM_STACK,
        BA_MOVING_BACK_FROM_CYCLE,
        BA_DEPOSIT_WHITE,

        // note: handle finish
        BA_MOVING_TO_PARKING,
        BA_PARKED,
    }

    public enum RobotAlliance {
        BLUE,
        RED,
        NONE,
    }

    public enum RobotStartingPosition {
        BACKDROP,
        AUDIENCE,
    }

    public enum RobotParkingLocation {
        INNER,
        OUTER,
    }

    public enum RobotTaskFinishBehaviour {
        DO_NOT_CYCLE,
        CYCLE,
        CYCLE_TWICE_NONONONONO,
    }

    public enum RobotLocMode {
        CAM,
        MEC,
    }
}

