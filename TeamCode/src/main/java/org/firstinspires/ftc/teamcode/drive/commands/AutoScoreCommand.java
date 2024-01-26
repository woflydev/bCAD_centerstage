package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.drive.hardware.DepositSubsystem;
import org.firstinspires.ftc.teamcode.drive.hardware.LiftSubsystem;

public class AutoScoreCommand extends CommandBase {
    private final DepositSubsystem deposit;
    private final LiftSubsystem lift;
    private SequentialCommandGroup commandGroup;
    private int elevation;

    public AutoScoreCommand(DepositSubsystem deposit, LiftSubsystem lift) {
        this.deposit = deposit;
        this.lift = lift;
        addRequirements(deposit, lift);
    }

    @Override
    public void initialize() {
        deposit.outtakeBusy = true;
        /*commandGroup = new SequentialCommandGroup(
                new PlaceCommand(deposit, lift),
                new FlattenCommand(deposit)
        );
        commandGroup.schedule();*/
    }

    @Override
    public void execute() {
        deposit.elbow.turnToAngle(110); AutoWait();
        lift.AutoRun();
        deposit.spin.turnToAngle(deposit.flatSpin); AutoWait();
        deposit.release(); AutoWait();
        lift.AutoHome();
    }

    public void AutoWait() {
        try { Thread.sleep(400); } catch (Exception e) { System.out.println("Exception occurred during delay!"); }
    }

    @Override
    public void end(boolean interrupted) { deposit.outtakeBusy = false; }

    @Override
    public boolean isFinished() {
        return true;
    }
}