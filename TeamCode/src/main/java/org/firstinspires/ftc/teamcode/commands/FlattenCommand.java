package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.DepositSubsystem;
import org.firstinspires.ftc.teamcode.hardware.LiftSubsystem;

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