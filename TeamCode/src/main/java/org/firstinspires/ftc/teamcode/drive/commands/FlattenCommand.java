package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.hardware.DepositSubsystem;

public class FlattenCommand extends CommandBase {
    private final DepositSubsystem deposit;
    private ElapsedTime timer;

    public FlattenCommand(DepositSubsystem deposit) {
        this.deposit = deposit;
        addRequirements(deposit);
    }

    @Override
    public void initialize() {
        timer = new ElapsedTime();
        timer.reset();


    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        deposit.Spin.turnToAngle(deposit.flatSpin);
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds()>1000;
    }
}