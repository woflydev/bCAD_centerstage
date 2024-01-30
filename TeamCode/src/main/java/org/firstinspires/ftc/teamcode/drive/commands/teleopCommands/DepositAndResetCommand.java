package org.firstinspires.ftc.teamcode.drive.commands.teleopCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.ASubsystemState;
import org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.hardware.DepositSubsystem;
import org.firstinspires.ftc.teamcode.drive.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.drive.hardware.LiftSubsystem;

public class DepositAndResetCommand extends CommandBase {
    private final DepositSubsystem deposit;
    private final LiftSubsystem lift;
    private final IntakeSubsystem intake;
    private final ElapsedTime timer = new ElapsedTime();

    private HomeCommand hmc;

    public DepositAndResetCommand(DepositSubsystem deposit, LiftSubsystem lift, IntakeSubsystem intake) {
        this.deposit = deposit;
        this.lift = lift;
        this.intake = intake;
        addRequirements(deposit, lift);
    }

    @Override
    public void initialize() {
        deposit.outtakeBusy = true;
    }

    @Override
    public void execute() {
        switch (deposit.outtakeState) {
            // note: continued from RaiseAndPrimeCommand
            case GRABBED_AND_READY:
                deposit.outtakeState = ASubsystemState.Outtake.PENDING_DEPOSIT;
                break;
            case PENDING_DEPOSIT:
                deposit.elbow.turnToAngle(RobotConstants.ELBOW_ACTIVE);
                deposit.wrist.turnToAngle(RobotConstants.WRIST_ACTIVE);

                timer.reset();
                deposit.outtakeState = ASubsystemState.Outtake.CLAW_OPENING;
                break;
            case CLAW_OPENING:
                if (timer.milliseconds() > 400) {
                    deposit.wrist.turnToAngle(RobotConstants.WRIST_ACTIVE - 20);
                    deposit.clawDeposit();

                    timer.reset();
                    deposit.outtakeState = ASubsystemState.Outtake.OUTTAKE_RESET;
                }
                break;
            case OUTTAKE_RESET:
                if (timer.milliseconds() >= 200) {
                    intake.closeFlap();
                    // note: this doesn't run it immediately. waits until this command ends then runs.
                    new HomeCommand(deposit, lift, intake).schedule();

                    timer.reset();
                    deposit.outtakeState = ASubsystemState.Outtake.AWAITING_OUTTAKE_RESET;
                }
            case AWAITING_OUTTAKE_RESET:
                break;
        }
    }

    @Override
    public void end(boolean interrupted) { deposit.outtakeBusy = false; }

    @Override
    public boolean isFinished() { return deposit.outtakeState == ASubsystemState.Outtake.AWAITING_OUTTAKE_RESET; }
}