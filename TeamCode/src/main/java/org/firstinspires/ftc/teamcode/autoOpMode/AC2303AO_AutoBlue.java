package org.firstinspires.ftc.teamcode.autoOpMode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robotv9.Auto_Fullstack_Base;
import org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.AAutoState.RobotAlliance;
import org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.AAutoState.RobotParkingLocation;
import org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.AAutoState.RobotStartingPosition;
import org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.AAutoState.RobotTaskFinishBehaviour;
import org.opencv.core.Point;

@Config
@Autonomous(name="BlueAudienceAuto_OUTER", group="BLUE")
public class AC2303AO_AutoBlue extends Auto_Fullstack_Base {
    public static double r1x = 0;
    public static double r1y = 140; //old 90
    public static double r2x = 120;
    public static double r2y = 120;
    public static double r3x = 260;
    public static double r3y = 140;

    public AC2303AO_AutoBlue() {
        super(
                RobotAlliance.BLUE,
                RobotStartingPosition.AUDIENCE,
                RobotParkingLocation.OUTER,
                RobotTaskFinishBehaviour.DO_NOT_CYCLE,
                new Point(r1x, r1y),
                new Point(r2x, r2y),
                new Point(r3x, r3y)
        );
    }
}