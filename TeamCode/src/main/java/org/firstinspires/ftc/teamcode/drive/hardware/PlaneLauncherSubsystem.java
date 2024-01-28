package org.firstinspires.ftc.teamcode.drive.hardware;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import static org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.RobotConstants.*;

public class PlaneLauncherSubsystem extends SubsystemBase {
    public ServoEx planeLauncher;

    public PlaneLauncherSubsystem(HardwareMap hardwareMap) {
        planeLauncher = new SimpleServo(hardwareMap, SERVO_PLANE, 0, 260, AngleUnit.DEGREES);
        planeLauncher.turnToAngle(PLANE_HOME);
    }

    public void launch() { planeLauncher.turnToAngle(PLANE_ACTIVE); }
    public void reset() { planeLauncher.turnToAngle(PLANE_HOME); }
}
