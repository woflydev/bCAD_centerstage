package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.drive.hardware.DepositSubsystem;
import org.firstinspires.ftc.teamcode.drive.hardware.LiftSubsystem;

public class ScoreCommand extends CommandBase {
    private final DepositSubsystem deposit;
    private final LiftSubsystem lift;

    SequentialCommandGroup commandGroup;

    public ScoreCommand(DepositSubsystem deposit, LiftSubsystem lift) {
        this.deposit = deposit;
        this.lift = lift;
        addRequirements(deposit, lift);
    }

    @Override
    public void initialize() {
        commandGroup = new SequentialCommandGroup(
                new PlaceCommand(deposit, lift),
                new FlattenCommand(deposit)
        );
        commandGroup.schedule();

    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}