package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.hardware.DepositSubsystem;
import org.firstinspires.ftc.teamcode.drive.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.drive.hardware.LiftSubsystem;

public class TransferCommand extends CommandBase {
    private final DepositSubsystem deposit;
    private final LiftSubsystem lift;
    private final IntakeSubsystem intake;
    private ElapsedTime timer;
    int delay;

    public TransferCommand(DepositSubsystem deposit, LiftSubsystem lift, IntakeSubsystem intake) {
        this.deposit = deposit;
        this.lift = lift;
        this.intake = intake;
        addRequirements(deposit, lift);
    }

    @Override
    public void initialize() {
        deposit.currentlyTransfering = true;
        timer = new ElapsedTime();
        timer.reset();

        deposit.Wrist.turnToAngle(260);
        deposit.Spin.turnToAngle(deposit.transferSpin);
        deposit.Gripper.turnToAngle(deposit.transferGrip);
    }

    @Override
    public void execute() {
        delay = 500;

        lift.leftMotor.motor.setTargetPosition(0+lift.liftOffset);
        lift.rightMotor.motor.setTargetPosition(0+lift.liftOffset);

        lift.leftMotor.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.rightMotor.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lift.leftMotor.motor.setPower(1);
        lift.rightMotor.motor.setPower(1);

        if(timer.milliseconds() > delay && timer.milliseconds() < delay*2) {
            deposit.V4B.turnToAngle(268);
        } else if (timer.milliseconds() > delay*2 && timer.milliseconds() < delay*3) {
            deposit.grab();
        } else if(timer.milliseconds() > delay*3) {
            intake.openCover();
        }
    }

    @Override
    public void end(boolean interrupted) {
        deposit.currentlyTransfering = false;
        deposit.V4B.turnToAngle(225);
        lift.leftMotor.motor.setTargetPosition(0);
        lift.rightMotor.motor.setTargetPosition(0);

        lift.leftMotor.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.rightMotor.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() > delay*4;
    }
}