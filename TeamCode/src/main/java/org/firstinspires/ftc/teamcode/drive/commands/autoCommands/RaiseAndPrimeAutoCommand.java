package org.firstinspires.ftc.teamcode.drive.commands.autoCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.hardware.DepositSubsystem;
import org.firstinspires.ftc.teamcode.drive.hardware.LiftSubsystem;

public class RaiseAndPrimeAutoCommand extends CommandBase {
    private final DepositSubsystem deposit;
    private final LiftSubsystem lift;
    private final boolean spin;
    private final boolean cycleValues;
    private final int stateDuration = 300;
    private final ElapsedTime timer = new ElapsedTime();
    private int targetHeight;
    private boolean finishTriggered;

    public RaiseAndPrimeAutoCommand(DepositSubsystem deposit,
                                    LiftSubsystem lift,
                                    int target,
                                    boolean spin,
                                    boolean cycleValues) {
        this.deposit = deposit;
        this.lift = lift;
        this.targetHeight = target;
        this.spin = spin;
        this.cycleValues = cycleValues;
        addRequirements(deposit, lift);
    }

    @Override
    public void initialize() {
        finishTriggered = false;
        deposit.outtakeBusy = true;
        timer.reset();
    }

    @Override
    public void execute() {
        targetHeight += lift.liftOffset;

        if (withinState(0)) {
            lift.targetLiftPosition = targetHeight;
            lift.UpdateLift(false, 0);
            deposit.elbow.turnToAngle(cycleValues ? RobotConstants.ELBOW_CYCLE_ACTIVE : RobotConstants.ELBOW_AUTO_ACTIVE);
            deposit.wrist.turnToAngle(cycleValues ? RobotConstants.WRIST_CYCLE_ACTIVE : RobotConstants.WRIST_AUTO_ACTIVE);
        } else if (withinState(1)) {
            if (spin) {
                deposit.spin.turnToAngle(RobotConstants.SPIN_DEPOSIT);
            }
            finishTriggered = true;
        }
    }

    private boolean withinState(double stateNumber) {
        return timer.milliseconds() >= (stateDuration * stateNumber) && timer.milliseconds() <= stateDuration * (stateNumber + 1);
    }

    private boolean interrupted() {
        return finishTriggered;
    }

    @Override
    public void end(boolean interrupted) {
        deposit.outtakeBusy = false;
    }

    @Override
    public boolean isFinished() { return (timer.milliseconds() >= stateDuration * 2) && interrupted(); }
}