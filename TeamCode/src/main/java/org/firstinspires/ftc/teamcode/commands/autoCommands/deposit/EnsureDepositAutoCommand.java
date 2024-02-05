package org.firstinspires.ftc.teamcode.commands.autoCommands.deposit;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.DriveConstants;
import org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotConstants;
import org.firstinspires.ftc.teamcode.hardware.DepositSubsystem;
import org.firstinspires.ftc.teamcode.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.LiftSubsystem;
import org.firstinspires.ftc.teamcode.rr.bCADMecanumDrive;

public class EnsureDepositAutoCommand extends CommandBase {
    private final DepositSubsystem deposit;
    private final LiftSubsystem lift;
    private final IntakeSubsystem intake;
    private final ElapsedTime timer = new ElapsedTime();

    private final Telemetry t;

    private final int stateDuration = 250;

    public EnsureDepositAutoCommand(DepositSubsystem deposit, LiftSubsystem lift, IntakeSubsystem intake, Telemetry t) {
        this.deposit = deposit;
        this.lift = lift;
        this.intake = intake;
        this.t = t;
        addRequirements(deposit, lift);
    }

    @Override
    public void initialize() {
        deposit.outtakeBusy = true;
        timer.reset();

        deposit.elbow.turnToAngle(RobotConstants.ELBOW_AUTO_ACTIVE);
        deposit.wrist.turnToAngle(RobotConstants.WRIST_AUTO_ACTIVE);
        deposit.wrist.turnToAngle(RobotConstants.WRIST_AUTO_ACTIVE);
        deposit.clawDeposit();
    }

    @Override
    public void execute() {
        if (withinState(0)) {

        }
    }

    @Override
    public void end(boolean interrupted) {
        deposit.outtakeBusy = false;
    }

    @Override
    public boolean isFinished() { return true; }

    private boolean withinState(double stateNumber) {
        return timer.milliseconds() >= (stateDuration * stateNumber) && timer.milliseconds() <= stateDuration * (stateNumber + 1);
    }
}