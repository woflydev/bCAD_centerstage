package org.firstinspires.ftc.teamcode.drive.autoOpMode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.Robotv9.Auto_Fullstack_Base_Old;
import org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.FSM_Auto_State.RobotAlliance;
import org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.FSM_Auto_State.RobotParkingLocation;
import org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.FSM_Auto_State.RobotStartingPosition;
import org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.FSM_Auto_State.RobotTaskFinishBehaviour;
import org.opencv.core.Point;

@Config
@Autonomous(name="NAT_RedAudienceAuto_INNER", group="Final")
public class AC2303AI_AutoRed extends Auto_Fullstack_Base_Old {
    public static double r1x = 25;
    public static double r1y = 90;
    public static double r2x = 150;
    public static double r2y = 60;
    public static double r3x = 260;
    public static double r3y = 90;

    public AC2303AI_AutoRed() {
        super(
                RobotAlliance.RED,
                RobotStartingPosition.AUDIENCE,
                RobotParkingLocation.INNER,
                RobotTaskFinishBehaviour.DO_NOT_CYCLE,
                new Point(r1x, r1y),
                new Point(r2x, r2y),
                new Point(r3x, r3y)
        );
    }
}