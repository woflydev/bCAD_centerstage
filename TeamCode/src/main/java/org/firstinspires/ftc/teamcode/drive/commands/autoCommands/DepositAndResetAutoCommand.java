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
    }

    @Override
    public void execute() {
        if (withinState(0)) {
            deposit.elbow.turnToAngle(RobotConstants.ELBOW_ACTIVE);
            deposit.wrist.turnToAngle(RobotConstants.WRIST_ACTIVE);
        } else if (withinState(1)) {
            deposit.wrist.turnToAngle(RobotConstants.WRIST_ACTIVE - 10);
            deposit.clawDeposit();
        } else if (withinState(2)) {
            intake.closeFlap();
            deposit.elbow.turnToAngle(ELBOW_HOME);
            deposit.wrist.turnToAngle(WRIST_HOME);
            deposit.clawReset();
        } else if (withinState(3)) {
            lift.targetLiftPosition = 0;
            lift.UpdateLift(false, 0);

        }
    }

    private boolean withinState(double stateNumber) {
        return timer.milliseconds() >= (stateDuration * stateNumber) && timer.milliseconds() <= stateDuration * (stateNumber + 1);
    }

    @Override
    public void end(boolean interrupted) {
        deposit.outtakeBusy = false;
        intake.stop();

        Delay(60);
        intake.closeFlap();
        deposit.spin.turnToAngle(SPIN_HOME);
        deposit.outtakeState = ASubsystemState.Outtake.IDLE;
    }

    @Override
    public boolean isFinished() { return (timer.milliseconds() >= stateDuration * 4) && (lift.liftLM.getCurrentPosition() <= 5); }

    private void Delay(double time) { try { Thread.sleep((long)time); } catch (Exception e) { System.out.println("Exception!"); } }
}