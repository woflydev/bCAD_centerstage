package org.firstinspires.ftc.teamcode.commands.teleopCommands;

import static org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotConstants.*;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.ASubsystemState.Outtake;
import org.firstinspires.ftc.teamcode.hardware.DepositSubsystem;
import org.firstinspires.ftc.teamcode.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.LiftSubsystem;

public class TransferAndStandbyCommand extends CommandBase {
    private final DepositSubsystem deposit;
    private final LiftSubsystem lift;
    private final IntakeSubsystem intake;
    private final ElapsedTime timer = new ElapsedTime();

    public TransferAndStandbyCommand(DepositSubsystem deposit, LiftSubsystem lift, IntakeSubsystem intake) {
        this.deposit = deposit;
        this.lift = lift;
        this.intake = intake;
        addRequirements(deposit, lift);
    }

    @Override
    public void initialize() {
        deposit.outtakeBusy = true;
        timer.reset();

        deposit.elbow.turnToAngle(ELBOW_HOME);
        deposit.wrist.turnToAngle(WRIST_HOME);
        deposit.spin.turnToAngle(SPIN_HOME);
        deposit.clawReset();

        deposit.outtakeState = Outtake.ACTIVATED;
    }

    @Override
    public void execute() {
        switch (deposit.outtakeState) {
            case IDLE:
            case GRABBED_AND_READY:
                break;
            case ACTIVATED:
                lift.targetLiftPosition = lift.liftOffset;
                lift.UpdateLift(false, 0);

                timer.reset();

                deposit.outtakeState = Outtake.FLAP_OPENING;
                break;
            case FLAP_OPENING:
                if (timer.milliseconds() >= 200 || lift.liftLM.getCurrentPosition() <= 8 + lift.liftOffset) {
                    intake.stop();
                    intake.openFlap();
                    deposit.clawReset();

                    timer.reset();
                    deposit.outtakeState = Outtake.WRIST_PICKING;
                }
                break;
            case WRIST_PICKING:
                if (timer.milliseconds() >= 200) {
                    deposit.wrist.turnToAngle(WRIST_PICKUP);

                    timer.reset();
                    deposit.outtakeState = Outtake.ELBOW_PICKING;
                }
                break;
            case ELBOW_PICKING:
                if (timer.milliseconds() >= 150) {
                    deposit.elbow.turnToAngle(ELBOW_PICKUP);

                    timer.reset();
                    deposit.outtakeState = Outtake.CLAW_CLOSING;
                }
                break;
            case CLAW_CLOSING:
                if (timer.milliseconds() >= 350) {
                    deposit.clawGrab();
                    deposit.outtakeState = Outtake.PENDING_GRABBED_AND_READY;
                }
                break;
            case PENDING_GRABBED_AND_READY:
                if (timer.milliseconds() >= 600) {
                    timer.reset();
                    deposit.outtakeState = Outtake.GRABBED_AND_READY;
                }

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
        return deposit.outtakeState == Outtake.GRABBED_AND_READY;
    }
}