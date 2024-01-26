package org.firstinspires.ftc.teamcode.drive.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.Robotv9.Auto_Fullstack_Base;
import org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.FSM_Auto_State.RobotAlliance;
import org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.FSM_Auto_State.RobotParkingLocation;
import org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.FSM_Auto_State.RobotStartingPosition;
import org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.FSM_Auto_State.RobotTaskFinishBehaviour;
import org.opencv.core.Point;

@Config
@Autonomous(name="NAT_BlueBackdropAuto_OUTER", group="Final")
public class AC2303BO_AutoBlue extends Auto_Fullstack_Base {
    public static double r1x = 25;
    public static double r1y = 90;
    public static double r2x = 150;
    public static double r2y = 60;
    public static double r3x = 260;
    public static double r3y = 90;

    public AC2303BO_AutoBlue() {
        super(
                RobotAlliance.BLUE,
                RobotStartingPosition.BACKDROP,
                RobotParkingLocation.OUTER,
                RobotTaskFinishBehaviour.DO_NOT_CYCLE,
                new Point(r1x, r1y),
                new Point(r2x, r2y),
                new Point(r3x, r3y)
        );
    }
}