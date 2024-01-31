package org.firstinspires.ftc.teamcode.drive.commands.autoCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.commands.teleopCommands.HomeCommand;
import org.firstinspires.ftc.teamcode.drive.hardware.DepositSubsystem;
import org.firstinspires.ftc.teamcode.drive.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.drive.hardware.LiftSubsystem;

public class AutoScoreCommand extends CommandBase {
    private final DepositSubsystem deposit;
    private final LiftSubsystem lift;
    private final IntakeSubsystem intake;
    private SequentialCommandGroup commandGroup;
    private int elevation;

    public AutoScoreCommand(DepositSubsystem deposit, LiftSubsystem lift, IntakeSubsystem intake) {
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
        deposit.elbow.turnToAngle(RobotConstants.ELBOW_ACTIVE);
        deposit.wrist.turnToAngle(RobotConstants.WRIST_ACTIVE); AutoWait();
        lift.AutoRun();
        deposit.spin.turnToAngle(RobotConstants.SPIN_DEPOSIT); AutoWait();
        deposit.clawDeposit(); AutoWait();
        deposit.spin.turnToAngle(RobotConstants.SPIN_HOME); AutoWait();
        deposit.elbow.turnToAngle(RobotConstants.ELBOW_HOME);
        deposit.wrist.turnToAngle(RobotConstants.WRIST_HOME);AutoWait();
        lift.AutoHome();



        //new HomeCommand(deposit, lift, intake).schedule();
    }

    public void AutoWait() {
        try { Thread.sleep(400); } catch (Exception e) { System.out.println("Exception occurred during delay!"); }
    }

    @Override
    public void end(boolean interrupted) { deposit.outtakeBusy = false; }

    @Override
    public boolean isFinished() {
        return true;
    }
}