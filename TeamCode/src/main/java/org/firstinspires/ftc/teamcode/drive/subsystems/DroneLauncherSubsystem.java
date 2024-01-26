package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DroneLauncherSubsystem extends SubsystemBase {
    public ServoEx Shoot; // Deposit Slide 1

    public DroneLauncherSubsystem(HardwareMap hardwareMap) {
        // Assign variables here with parameters
        Shoot = new SimpleServo(hardwareMap, "Shoot", 0, 260, AngleUnit.DEGREES);

        Shoot.turnToAngle(30);
    }

    public void shoot() { Shoot.turnToAngle(0); }

    public void reset() { Shoot.turnToAngle(30); }

}
