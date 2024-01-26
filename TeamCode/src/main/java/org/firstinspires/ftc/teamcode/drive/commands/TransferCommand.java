package org.firstinspires.ftc.teamcode.drive.commands;

import static org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.RobotConstants.*;

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
        deposit.outtakeBusy = true;
        timer = new ElapsedTime();
        timer.reset();

        deposit.wrist.turnToAngle(260);
        deposit.spin.turnToAngle(deposit.transferSpin);
        deposit.claw.turnToAngle(CLAW_OPEN);
    }

    @Override
    public void execute() {
        delay = 500;

        lift.liftLM.motor.setTargetPosition(lift.liftOffset);
        lift.liftRM.motor.setTargetPosition(lift.liftOffset);

        lift.liftLM.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.liftRM.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lift.liftLM.motor.setPower(1);
        lift.liftRM.motor.setPower(1);

        if (timer.milliseconds() > delay && timer.milliseconds() < delay*2) {
            deposit.elbow.turnToAngle(268);
        } else if (timer.milliseconds() > delay * 2 && timer.milliseconds() < delay*3) {
            deposit.grab();
        } else if(timer.milliseconds() > delay * 3) {
            intake.openCover();
        }
    }

    @Override
    public void end(boolean interrupted) {
        deposit.outtakeBusy = false;

        deposit.elbow.turnToAngle(225);
        lift.liftLM.motor.setTargetPosition(0);
        lift.liftRM.motor.setTargetPosition(0);

        lift.liftLM.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.liftRM.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() > delay * 4;
    }
}