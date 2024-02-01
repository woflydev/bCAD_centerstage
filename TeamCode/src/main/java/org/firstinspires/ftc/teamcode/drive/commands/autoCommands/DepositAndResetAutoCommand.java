package org.firstinspires.ftc.teamcode.drive.commands.autoCommands;

import static org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.RobotConstants.ELBOW_HOME;
import static org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.RobotConstants.SPIN_HOME;
import static org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.RobotConstants.WRIST_HOME;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.ASubsystemState;
import org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.commands.teleopCommands.HomeCommand;
import org.firstinspires.ftc.teamcode.drive.hardware.DepositSubsystem;
import org.firstinspires.ftc.teamcode.drive.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.drive.hardware.LiftSubsystem;

public class DepositAndResetAutoCommand extends CommandBase {
    private final DepositSubsystem deposit;
    private final LiftSubsystem lift;
    private final IntakeSubsystem intake;
    private final ElapsedTime timer = new ElapsedTime();

    private final int stateDuration = 300;

    public DepositAndResetAutoCommand(DepositSubsystem deposit, LiftSubsystem lift, IntakeSubsystem intake) {
        this.deposit = deposit;
        this.lift = lift;
        this.intake = intake;
        addRequirements(deposit, lift);
    }

    @Override
    public void initialize() {
        deposit.outtakeBusy = true;
        timer.reset();
    }

    @Override
    public void execute() {
        if (withinState(0)) {
            deposit.elbow.turnToAngle(RobotConstants.ELBOW_AUTO_ACTIVE);
            deposit.wrist.turnToAngle(RobotConstants.WRIST_AUTO_ACTIVE);
        } else if (withinState(1)) {
            deposit.wrist.turnToAngle(RobotConstants.WRIST_AUTO_ACTIVE - 10);
            deposit.clawDeposit();
        }
    }

    @Override
    public void end(boolean interrupted) {
        deposit.outtakeBusy = false;
    }

    @Override
    public boolean isFinished() { return (timer.milliseconds() >= stateDuration * 2); }

    private boolean withinState(double stateNumber) {
        return timer.milliseconds() >= (stateDuration * stateNumber) && timer.milliseconds() <= stateDuration * (stateNumber + 1);
    }
}