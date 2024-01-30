package org.firstinspires.ftc.teamcode.drive.commands.teleopCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.ASubsystemState;
import org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.hardware.DepositSubsystem;
import org.firstinspires.ftc.teamcode.drive.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.drive.hardware.LiftSubsystem;

public class RaiseAndPrimeCommand extends CommandBase {
    private final DepositSubsystem deposit;
    private final LiftSubsystem lift;
    private final IntakeSubsystem intake;
    private final ElapsedTime timer = new ElapsedTime();
    private int targetHeight;

    public RaiseAndPrimeCommand(DepositSubsystem deposit, LiftSubsystem lift, IntakeSubsystem intake, int target) {
        this.deposit = deposit;
        this.lift = lift;
        this.intake = intake;
        this.targetHeight = target;
        addRequirements(deposit, lift);
    }

    @Override
    public void initialize() {
        deposit.outtakeBusy = true;
    }

    @Override
    public void execute() {
        targetHeight += lift.liftOffset;
        switch (deposit.outtakeState) {
            // note: continued from TransferAndStandbyCommand
            case GRABBED_AND_READY:
                lift.targetLiftPosition = targetHeight;
                lift.UpdateLift(false, 0);

                timer.reset();
                deposit.outtakeState = ASubsystemState.Outtake.PRIMED_FOR_DEPOSIT;
                break;
            case PRIMED_FOR_DEPOSIT:
                if (timer.milliseconds() >= 100) {
                    deposit.elbow.turnToAngle(RobotConstants.ELBOW_ACTIVE);
                    deposit.wrist.turnToAngle(RobotConstants.WRIST_ACTIVE);

                    timer.reset();
                    deposit.outtakeState = ASubsystemState.Outtake.SERVO_SPINNING;
                }
                break;
            case SERVO_SPINNING:
                if (timer.milliseconds() >= 450) {
                    //deposit.spin.turnToAngle(RobotConstants.SPIN_DEPOSIT);
                    deposit.spinNew.setPosition(RobotConstants.SPIN_DEPOSIT);

                    timer.reset();
                    deposit.outtakeState = ASubsystemState.Outtake.PENDING_DEPOSIT;
                }
            case PENDING_DEPOSIT:
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        deposit.outtakeBusy = false;
    }

    @Override
    public boolean isFinished() { return deposit.outtakeState == ASubsystemState.Outtake.PENDING_DEPOSIT; }
}