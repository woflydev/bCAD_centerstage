package org.firstinspires.ftc.teamcode.commands.autoCommands.pickup;

import static org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotConstants.ELBOW_GRABBED_STANDBY;
import static org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotConstants.ELBOW_HOME;
import static org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotConstants.ELBOW_PICKUP;
import static org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotConstants.SPIN_HOME;
import static org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotConstants.WRIST_HOME;
import static org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotConstants.WRIST_PICKUP;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.ASubsystemState.Outtake;
import org.firstinspires.ftc.teamcode.hardware.DepositSubsystem;
import org.firstinspires.ftc.teamcode.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.LiftSubsystem;

public class TransferAndStandbyAutoCommand extends CommandBase {
    private final DepositSubsystem deposit;
    private final LiftSubsystem lift;
    private final IntakeSubsystem intake;
    private final ElapsedTime timer = new ElapsedTime();

    private final int stateDuration = 300;

    public TransferAndStandbyAutoCommand(DepositSubsystem deposit, LiftSubsystem lift, IntakeSubsystem intake) {
        this.deposit = deposit;
        this.lift = lift;
        this.intake = intake;
        addRequirements(deposit, lift);
    }

    @Override
    public void initialize() {
        deposit.outtakeBusy = true;

        deposit.elbow.turnToAngle(ELBOW_HOME);
        deposit.wrist.turnToAngle(WRIST_HOME);
        deposit.spin.turnToAngle(SPIN_HOME);
        deposit.clawReset();

        lift.targetLiftPosition = lift.liftOffset;
        lift.UpdateLift(false, 0);

        deposit.outtakeState = Outtake.ACTIVATED;

        timer.reset();
    }

    @Override
    public void execute() {
        if (withinState(0, 1.5)) {
            intake.stop();
            intake.openFlap();
            deposit.clawReset();
        } else if (withinState(1.5, 1.5)) {
            deposit.wrist.turnToAngle(WRIST_PICKUP);
        } else if (withinState(3, 1)) {
            deposit.elbow.turnToAngle(ELBOW_PICKUP);
        } else if (withinState(4, 1)) {
            deposit.clawGrab();
        }
    }

    @Override
    public void end(boolean interrupted) {
        deposit.outtakeBusy = false;
        deposit.elbow.turnToAngle(ELBOW_GRABBED_STANDBY);
        lift.liftLM.motor.setTargetPosition(0);
        lift.liftRM.motor.setTargetPosition(0);
        lift.liftLM.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.liftRM.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() >= stateDuration * 5;
    }

    private boolean withinState(double stateNumber, double endTimeFactor) {
        return timer.milliseconds() >= (stateDuration * stateNumber) && timer.milliseconds() <= stateDuration * (stateNumber + endTimeFactor);
    }
}