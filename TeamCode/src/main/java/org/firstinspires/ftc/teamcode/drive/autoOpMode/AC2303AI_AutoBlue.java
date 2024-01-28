package org.firstinspires.ftc.teamcode.drive.autoOpMode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.Robotv9.Auto_Fullstack_Base;
import org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.AAutoState.RobotAlliance;
import org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.AAutoState.RobotParkingLocation;
import org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.AAutoState.RobotStartingPosition;
import org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.AAutoState.RobotTaskFinishBehaviour;
import org.opencv.core.Point;

@Config
@Autonomous(name="BlueAudienceAuto_INNER", group="BLUE")
public class AC2303AI_AutoBlue extends Auto_Fullstack_Base {
    public static double r1x = 25;
    public static double r1y = 90;
    public static double r2x = 150;
    public static double r2y = 60;
    public static double r3x = 260;
    public static double r3y = 90;

    public AC2303AI_AutoBlue() {
        super(
                RobotAlliance.BLUE,
                RobotStartingPosition.AUDIENCE,
                RobotParkingLocation.INNER,
                RobotTaskFinishBehaviour.DO_NOT_CYCLE,
                new Point(r1x, r1y),
                new Point(r2x, r2y),
                new Point(r3x, r3y)
        );
    }
}