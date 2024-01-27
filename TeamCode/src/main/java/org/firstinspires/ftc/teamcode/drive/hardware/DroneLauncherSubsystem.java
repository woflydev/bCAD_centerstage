package org.firstinspires.ftc.teamcode.drive.hardware;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.RobotConstants;

public class DroneLauncherSubsystem extends SubsystemBase {
    public ServoEx planeLauncher;

    public DroneLauncherSubsystem(HardwareMap hardwareMap) {
        planeLauncher = new SimpleServo(hardwareMap, RobotConstants.SERVO_PLANE, 0, 260, AngleUnit.DEGREES);
        planeLauncher.turnToAngle(30);
    }

    public void launch() { planeLauncher.turnToAngle(0); }
    public void reset() { planeLauncher.turnToAngle(30); }
}
