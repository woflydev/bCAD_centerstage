package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.hardware.DepositSubsystem;
import org.firstinspires.ftc.teamcode.drive.hardware.LiftSubsystem;

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
        deposit.outtakeBusy = true;
        deposit.elbow.turnToAngle(110);
    }

    @Override
    public void execute() {
        elevation = 100 + lift.liftOffset;

        lift.liftLM.motor.setTargetPosition(elevation);
        lift.liftRM.motor.setTargetPosition(elevation);

        lift.liftLM.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.liftRM.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lift.liftLM.motor.setPower(1);
        lift.liftRM.motor.setPower(1);
    }

    @Override
    public void end(boolean interrupted) {
        deposit.outtakeBusy = false;
    }

    @Override
    public boolean isFinished() {
        return lift.liftLM.getCurrentPosition() > elevation - 10;
    }
}