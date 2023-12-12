package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// Import local FTCLib hardware classes
import org.firstinspires.ftc.teamcode.commands.HomeCommand;
import org.firstinspires.ftc.teamcode.commands.LeftTriggerReader;
import org.firstinspires.ftc.teamcode.commands.OpModeTemplate;
import org.firstinspires.ftc.teamcode.commands.ScoreCommand;
import org.firstinspires.ftc.teamcode.commands.TransferCommand;
import org.firstinspires.ftc.teamcode.commands.RightTriggerReader;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Competition TeleOp", group = "TeleOp")

public class TeleOp extends OpModeTemplate {
    ScoreCommand scoreCommand;

    @Override
    public void initialize() {
        initHardware(false);
        scoreCommand = new ScoreCommand(deposit, lift);

        new GamepadButton(toolOp, GamepadKeys.Button.START).toggleWhenPressed(() -> intake.openCover(), () -> intake.closeCover());
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
        drivebase.userControlledDrive(gamepad1, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        if(!deposit.currentlyPlacing && !deposit.currentlyHoming && !deposit.currentlyTransfering) {
            lift.run(toolOp.getLeftY());

        }

        hang.run(gamepad2);

        deposit.manualV4BControl(toolOp.getRightY(), telemetry);
        // TODO: Delete Later
//        deposit.manualWristControl(toolOp.getLeftY(), telemetry);

        telemetry.addData("Wrist Position", deposit.Wrist.getAngle());

        telemetry.addData("Slide position", lift.leftMotor.motor.getCurrentPosition());

        telemetry.update();
    }
}