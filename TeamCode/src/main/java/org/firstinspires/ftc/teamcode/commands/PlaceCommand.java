package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.DepositSubsystem;
import org.firstinspires.ftc.teamcode.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.LiftSubsystem;

public class PlaceCommand extends CommandBase {
    private final DepositSubsystem deposit;
    private final LiftSubsystem lift;
    private ElapsedTime timer;
    int elevation;
    int delay;

    public PlaceCommand(DepositSubsystem deposit, LiftSubsystem lift) {
        this.deposit = deposit;
        this.lift = lift;
        addRequirements(deposit, lift);
    }

    @Override
    public void initialize() {
        deposit.currentlyPlacing = true;
        timer = new ElapsedTime();
        timer.reset();

        //deposit.V4B.turnToAngle(126);
        deposit.V4B.turnToAngle(110);
        deposit.outtaking = true;
    }

    @Override
    public void execute() {
        elevation = 100+ lift.liftOffset;

        lift.leftMotor.motor.setTargetPosition(elevation);
        lift.rightMotor.motor.setTargetPosition(elevation);

        lift.leftMotor.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.rightMotor.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lift.leftMotor.motor.setPower(1);
        lift.rightMotor.motor.setPower(1);
    }

    @Override
    public void end(boolean interrupted) {
        deposit.currentlyPlacing = false;
    }

    @Override
    public boolean isFinished() {
        if(lift.leftMotor.getCurrentPosition() > elevation-10) { return true; }
        else return false;
    }
}