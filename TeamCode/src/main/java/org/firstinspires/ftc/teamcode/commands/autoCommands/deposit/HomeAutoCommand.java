package org.firstinspires.ftc.teamcode.commands.autoCommands.deposit;

import static org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotConstants.ELBOW_HOME;
import static org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotConstants.SPIN_HOME;
import static org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotConstants.WRIST_HOME;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.ASubsystemState;
import org.firstinspires.ftc.teamcode.hardware.DepositSubsystem;
import org.firstinspires.ftc.teamcode.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.LiftSubsystem;

public class HomeAutoCommand extends CommandBase {
    private final DepositSubsystem deposit;
    private final LiftSubsystem lift;
    private final IntakeSubsystem intake;
    private final ElapsedTime timer = new ElapsedTime();

    private final int stateDuration = 200;

    public HomeAutoCommand(DepositSubsystem deposit, LiftSubsystem lift, IntakeSubsystem intake) {
        this.deposit = deposit;
        this.lift = lift;
        this.intake = intake;
        addRequirements(deposit, lift);
    }

    @Override
    public void initialize() {
        deposit.outtakeBusy = true;
        timer.reset();
    }

    @Override
    public void execute() {
        if (withinState(1)) {
            lift.targetLiftPosition = 0;
            lift.UpdateLift(false, 0);
        } else if (withinState(2)) {
            intake.closeFlap();
            deposit.clawGrab(); // note: just so it doesn't clip the flap
            deposit.elbow.turnToAngle(ELBOW_HOME);
            deposit.wrist.turnToAngle(WRIST_HOME);
            deposit.spin.turnToAngle(SPIN_HOME);
        }
    }

    @Override
    public void end(boolean interrupted) {
        deposit.outtakeBusy = false;
        intake.stop();
        intake.closeFlap();
        deposit.outtakeState = ASubsystemState.Outtake.IDLE;
    }

    @Override
    public boolean isFinished() { return (lift.liftLM.getCurrentPosition() <= 6); }

    private void Delay(double time) { try { Thread.sleep((long)time); } catch (Exception e) { System.out.println("Exception!"); } }

    private boolean withinState(double stateNumber) {
        return timer.milliseconds() >= (stateDuration * stateNumber) && timer.milliseconds() <= stateDuration * (stateNumber + 1);
    }
}