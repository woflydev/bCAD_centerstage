package org.firstinspires.ftc.teamcode.fullstack;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.drive.Fullstack_Base;

@TeleOp(name="TeleOp", group="!RC-1.0.0")
public class Fullstack_TeleOp extends Fullstack_Base {
    public void MainLoop() {
        // note: intake, manual controls
        RuntimeConfig();

        // note: FSMs
        DrivetrainSubsystem();
        OuttakeSubsystem();
        PlaneLauncherSubsystem();

        // note: redundancies
        PassiveArmResetCheck();

        // note: telemetry handled in parent
    }
}
