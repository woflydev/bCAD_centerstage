package org.firstinspires.ftc.teamcode.drive.Robotv9;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.commands.HomeCommand;
import org.firstinspires.ftc.teamcode.drive.commands.LeftTriggerReader;
import org.firstinspires.ftc.teamcode.drive.commands.OpModeTemplate;
import org.firstinspires.ftc.teamcode.drive.commands.RightTriggerReader;
import org.firstinspires.ftc.teamcode.drive.commands.ScoreCommand;
import org.firstinspires.ftc.teamcode.drive.commands.TransferCommand;

public class TeleOp_Fullstack_Base extends OpModeTemplate {
    @Override
    public void initialize() {
        InitBlock();
        new GamepadButton(gamepad2Ex, GamepadKeys.Button.START).toggleWhenPressed(
                () -> intake.openCover(),
                () -> intake.closeCover()
        );

        new GamepadButton(gamepad2Ex, GamepadKeys.Button.A).toggleWhenPressed(
                () -> deposit.grab(),
                () -> deposit.release()
        );

        new GamepadButton(gamepad2Ex, GamepadKeys.Button.BACK).toggleWhenPressed(
                () -> shooter.shoot(),
                () -> shooter.reset()
        );

        // note: will not be scheduled unless button becomes INACTIVE -> ACTIVE again
        new GamepadButton(gamepad2Ex, GamepadKeys.Button.X).whenPressed(new ScoreCommand(deposit, lift));
        new GamepadButton(gamepad2Ex, GamepadKeys.Button.Y).whenPressed(new HomeCommand(deposit, lift));
        new GamepadButton(gamepad2Ex, GamepadKeys.Button.B).whenPressed(new TransferCommand(deposit, lift, intake));

        // note: will not be scheduled unless button becomes INACTIVE -> ACTIVE again
        new GamepadButton(gamepad2Ex, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> deposit.mosaicSpin(1, telemetry))
                .whenReleased(() -> deposit.mosaicSpin(0, telemetry));

        new GamepadButton(gamepad2Ex, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> deposit.mosaicSpin(-1, telemetry))
                .whenReleased(() -> deposit.mosaicSpin(0, telemetry));

        new GamepadButton(gamepad2Ex, GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> lift.liftOffset += 10);
        new GamepadButton(gamepad2Ex, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> lift.liftOffset -= 10);

        new GamepadButton(gamepad2Ex, GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(() -> deposit.manualWristControl(1, telemetry));
        new GamepadButton(gamepad2Ex, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(() -> deposit.manualWristControl(-1, telemetry));

        new RightTriggerReader(gamepad2Ex, gamepad1Ex)
                .whenActive(intake::spin)
                .whenInactive(intake::stop);
        new LeftTriggerReader(gamepad2Ex, gamepad1Ex)
                .whenActive(intake::reverseSpin)
                .whenInactive(intake::stop);

        new GamepadButton(gamepad1Ex, GamepadKeys.Button.BACK)
                .whenPressed(() -> imu.resetYaw());
    }

    @Override
    public void run() {
        super.run();
        MainLoop();
        RuntimeConfiguration();
        StatusTelemetry();
    }

    public void RuntimeConfiguration() {
        if (!deposit.outtakeBusy) {
            // note: allows for manual lift operation by driver2
            lift.run(gamepad2Ex.getLeftY());
        }

        // note: drivetrain
        drivebase.userControlledDrive(gamepad1, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // note: elbow
        deposit.manualElbowControl(gamepad2Ex.getRightY(), telemetry);

        // note: manual hanging control
        hang.run(gamepad2);
    }

    public void StatusTelemetry() {
        telemetry.addData("Wrist Position", deposit.wrist.getAngle());
        telemetry.addData("Slide position", lift.liftLM.motor.getCurrentPosition());
        telemetry.update();
    }

    public void MainLoop() {}
}