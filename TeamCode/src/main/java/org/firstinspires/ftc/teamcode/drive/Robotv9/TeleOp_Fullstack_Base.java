package org.firstinspires.ftc.teamcode.drive.Robotv9;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.commands.HomeCommand;
import org.firstinspires.ftc.teamcode.drive.commands.LeftTriggerReader;
import org.firstinspires.ftc.teamcode.drive.commands.OpModeTemplate;
import org.firstinspires.ftc.teamcode.drive.commands.RightTriggerReader;
import org.firstinspires.ftc.teamcode.drive.commands.ScoreCommand;
import org.firstinspires.ftc.teamcode.drive.commands.TransferCommand;

public class TeleOp_Fullstack_Base extends OpModeTemplate {
    ScoreCommand scoreCommand;

    @Override
    public void initialize() {
        initHardware(false);
        scoreCommand = new ScoreCommand(deposit, lift);

        new GamepadButton(toolOp, GamepadKeys.Button.START).toggleWhenPressed(
                () -> intake.openCover(),
                () -> intake.closeCover()
        );

        new GamepadButton(toolOp, GamepadKeys.Button.A).toggleWhenPressed(() -> deposit.grab(), () -> deposit.release());
        new GamepadButton(toolOp, GamepadKeys.Button.BACK).toggleWhenPressed(() -> shooter.shoot(),  () -> shooter.reset());

        new GamepadButton(toolOp, GamepadKeys.Button.X).whenPressed(scoreCommand);
        new GamepadButton(toolOp, GamepadKeys.Button.Y).whenPressed(new HomeCommand(deposit, lift));
        new GamepadButton(toolOp, GamepadKeys.Button.B).whenPressed(new TransferCommand(deposit, lift, intake));

        new GamepadButton(toolOp, GamepadKeys.Button.LEFT_BUMPER).whenPressed(() -> deposit.mosaicSpin(1, telemetry)).whenReleased(() -> deposit.mosaicSpin(0, telemetry));
        new GamepadButton(toolOp, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(() -> deposit.mosaicSpin(-1, telemetry)).whenReleased(() -> deposit.mosaicSpin(0, telemetry));
        new GamepadButton(toolOp, GamepadKeys.Button.DPAD_UP).whenPressed(() -> lift.liftOffset += 10);
        new GamepadButton(toolOp, GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> lift.liftOffset += -10);

        new GamepadButton(toolOp, GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> deposit.manualWristControl(1, telemetry));
        new GamepadButton(toolOp, GamepadKeys.Button.DPAD_RIGHT).whenPressed(() -> deposit.manualWristControl(-1, telemetry));

        new RightTriggerReader(toolOp, driveOp).whenActive(intake::spin).whenInactive(intake::stop);
        new LeftTriggerReader(toolOp, driveOp).whenActive(intake::Rspin).whenInactive(intake::stop);

        new GamepadButton(driveOp, GamepadKeys.Button.BACK).whenPressed(() -> imu.resetYaw());

    }

    @Override
    public void run() {
        super.run();
        MainLoop();
    }

    public void MainLoop() {}
}