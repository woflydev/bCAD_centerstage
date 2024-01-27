package org.firstinspires.ftc.teamcode.drive.commands.teleopCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.ASubsystemState;
import org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.hardware.DepositSubsystem;
import org.firstinspires.ftc.teamcode.drive.hardware.LiftSubsystem;

public class DepositAndResetCommand extends CommandBase {
    private final DepositSubsystem deposit;
    private final LiftSubsystem lift;
    private final ElapsedTime timer = new ElapsedTime();

    public DepositAndResetCommand(DepositSubsystem deposit, LiftSubsystem lift) {
        this.deposit = deposit;
        this.lift = lift;
        addRequirements(deposit, lift);
    }

    @Override
    public void initialize() {
        deposit.outtakeBusy = true;
        deposit.elbow.turnToAngle(RobotConstants.ELBOW_ACTIVE);
    }

    @Override
    public void execute() {
        switch (deposit.outtakeState) {
            // note: continued from RaiseAndPrimeCommand
            case PENDING_DEPOSIT:
                deposit.outtakeState = ASubsystemState.Outtake.CLAW_OPENING;
                break;
            case CLAW_OPENING:
                deposit.clawDeposit();
                timer.reset();
                deposit.outtakeState = ASubsystemState.Outtake.OUTTAKE_RESET;
                break;
            case OUTTAKE_RESET:
                if (timer.milliseconds() >= 600) {
                    new HomeCommand(deposit, lift).schedule();

                    timer.reset();
                    deposit.outtakeState = ASubsystemState.Outtake.AWAITING_OUTTAKE_RESET;
                }
            case AWAITING_OUTTAKE_RESET:
                if (lift.liftLM.getCurrentPosition() < 10) {
                    deposit.outtakeState = ASubsystemState.Outtake.IDLE;
                }
        }
    }

    @Override
    public void end(boolean interrupted) { deposit.outtakeBusy = false; }

    @Override
    public boolean isFinished() {
        return deposit.outtakeState == ASubsystemState.Outtake.IDLE;
    }
}