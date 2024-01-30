package org.firstinspires.ftc.teamcode.drive.commands.teleopCommands;

import static org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.RobotConstants.*;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.ASubsystemState;
import org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.hardware.DepositSubsystem;
import org.firstinspires.ftc.teamcode.drive.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.drive.hardware.LiftSubsystem;

public class HomeCommand extends CommandBase {
    private final DepositSubsystem deposit;
    private final LiftSubsystem lift;
    private final IntakeSubsystem intake;

    public HomeCommand(DepositSubsystem deposit, LiftSubsystem lift, IntakeSubsystem intake) {
        this.deposit = deposit;
        this.lift = lift;
        this.intake = intake;
        addRequirements(deposit, lift, intake);
    }

    @Override
    public void initialize() {
        deposit.outtakeBusy = true;

        deposit.elbow.turnToAngle(ELBOW_HOME);
        deposit.wrist.turnToAngle(WRIST_HOME);
        deposit.clawReset();
    }

    @Override
    public void execute() {
        lift.liftLM.motor.setTargetPosition(0);
        lift.liftRM.motor.setTargetPosition(0);
        lift.liftLM.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.liftRM.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.liftLM.motor.setPower(MAX_LIFT_SPEED);
        lift.liftRM.motor.setPower(MAX_LIFT_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();

        Delay(60);
        intake.closeFlap();
        deposit.spin.turnToAngle(SPIN_HOME);
        deposit.outtakeBusy = false;
        deposit.outtakeState = ASubsystemState.Outtake.IDLE;
    }

    @Override
    public boolean isFinished() {
        return lift.liftLM.getCurrentPosition() <= 5;
    }

    private void Delay(double time) { try { Thread.sleep((long)time); } catch (Exception e) { System.out.println("Exception!"); } }
}