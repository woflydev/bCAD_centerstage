package org.firstinspires.ftc.teamcode.drive.Robotv9;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.ASubsystemState.Outtake;
import org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.commands.teleopCommands.HomeCommand;
import org.firstinspires.ftc.teamcode.drive.commands.teleopCommands.subcommands.LeftTriggerReader;
import org.firstinspires.ftc.teamcode.drive.commands.OpModeTemplate;
import org.firstinspires.ftc.teamcode.drive.commands.teleopCommands.RaiseAndPrimeCommand;
import org.firstinspires.ftc.teamcode.drive.commands.teleopCommands.subcommands.RightTriggerReader;
import org.firstinspires.ftc.teamcode.drive.commands.teleopCommands.DepositAndResetCommand;
import org.firstinspires.ftc.teamcode.drive.commands.teleopCommands.TransferAndStandbyCommand;

public class TeleOp_Fullstack_Base extends OpModeTemplate {
    @Override
    public void initialize() {
        InitBlock();

        // note: ------------------------driver 1------------------------------------------------------------
        new GamepadButton(gamepad1Ex, GamepadKeys.Button.BACK)
                .whenPressed(() -> imu.resetYaw());

        // note: ------------------------driver 2------------------------------------------------------------
        new GamepadButton(gamepad2Ex, GamepadKeys.Button.START).toggleWhenPressed(
                () -> intake.openFlap(),
                () -> intake.closeFlap()
        );

        new GamepadButton(gamepad2Ex, GamepadKeys.Button.A).toggleWhenPressed(
                () -> deposit.clawGrab(),
                () -> deposit.clawDeposit()
        );

        new GamepadButton(gamepad2Ex, GamepadKeys.Button.BACK).toggleWhenPressed(
                () -> shooter.launch(),
                () -> shooter.reset()
        );

        // note: will not be scheduled unless button becomes INACTIVE -> ACTIVE again
        new GamepadButton(gamepad2Ex, GamepadKeys.Button.X).whenPressed(new DepositAndResetCommand(deposit, lift));
        new GamepadButton(gamepad2Ex, GamepadKeys.Button.Y).whenPressed(new HomeCommand(deposit, lift));
        new GamepadButton(gamepad2Ex, GamepadKeys.Button.B).whenPressed(new TransferAndStandbyCommand(deposit, lift, intake));

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
                .whenActive(intake::spinAndCloseFlap)
                .whenInactive(intake::stop);
        new LeftTriggerReader(gamepad2Ex, gamepad1Ex)
                .whenActive(intake::reverseSpin)
                .whenInactive(intake::stop);
    }

    @Override
    public void run() {
        super.run();
        MainLoop();
        RuntimeConfiguration();
        StatusTelemetry();
    }

    public void RuntimeConfiguration() {
        // note: single driver pixel control
        if (gamepad1.left_bumper && deposit.outtakeState == Outtake.IDLE) {
            new TransferAndStandbyCommand(deposit, lift, intake);
        } else if (deposit.outtakeState == Outtake.GRABBED_AND_READY) { // note: will only be GRABBED_AND_READY after transfer
                 if (gamepad1.a) new RaiseAndPrimeCommand(deposit, lift, intake, RobotConstants.JUNCTION_LOW);
            else if (gamepad1.b) new RaiseAndPrimeCommand(deposit, lift, intake, RobotConstants.JUNCTION_MID);
            else if (gamepad1.y) new RaiseAndPrimeCommand(deposit, lift, intake, RobotConstants.JUNCTION_HIGH);
        } else if (deposit.outtakeState == Outtake.PENDING_DEPOSIT) {
            if (gamepad1.a || gamepad1.b || gamepad1.y) new DepositAndResetCommand(deposit, lift);
        }

        if (!deposit.outtakeBusy) {
            // note: allows for manual lift operation by driver2
            lift.run(gamepad2Ex.getLeftY());
        }

        drivebase.Mecanum(gamepad1, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        deposit.manualElbowControl(gamepad2Ex.getRightY(), telemetry);
        hang.run(gamepad2);

        Delay(60); // note: input debouncing
    }

    public void StatusTelemetry() {
        telemetry.addData("Wrist Position", deposit.wrist.getAngle());
        telemetry.addData("Slide position", lift.liftLM.motor.getCurrentPosition());
        telemetry.update();
    }

    public void MainLoop() {}

    private void Delay(double time) { try { Thread.sleep((long)time); } catch (Exception e) { System.out.println("Exception!"); } }
}