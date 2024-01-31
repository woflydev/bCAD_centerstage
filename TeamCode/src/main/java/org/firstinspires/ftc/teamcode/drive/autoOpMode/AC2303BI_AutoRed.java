package org.firstinspires.ftc.teamcode.drive.autoOpMode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.Robotv9.Auto_Fullstack_Base;
import org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.AAutoState.*;
import org.opencv.core.Point;

@Config
@Autonomous(name="RedBackdropAuto_INNER", group="RED")
public class AC2303BI_AutoRed extends Auto_Fullstack_Base {
    public static double r1x = 0;
    public static double r1y = 140; //old 90
    public static double r2x = 120;
    public static double r2y = 120;
    public static double r3x = 260;
    public static double r3y = 140;

    public AC2303BI_AutoRed() {
        super(
                RobotAlliance.RED,
                RobotStartingPosition.BACKDROP,
                RobotParkingLocation.INNER,
                RobotTaskFinishBehaviour.DO_NOT_CYCLE,
                new Point(r1x, r1y),
                new Point(r2x, r2y),
                new Point(r3x, r3y)
        );
    }
}