package org.firstinspires.ftc.teamcode.Robotv9.RobotInfo;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.AAutoState.*;

public class GlobalStorage {
    public static Pose2d switchoverPose = new Pose2d(0,0,0);
    public static RobotAlliance alliance = RobotAlliance.RED;
    public static RobotStartingPosition startingPosition = RobotStartingPosition.BACKDROP;
}