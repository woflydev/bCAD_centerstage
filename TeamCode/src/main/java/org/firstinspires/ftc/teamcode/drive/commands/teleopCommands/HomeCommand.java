package org.firstinspires.ftc.teamcode.drive.commands.teleopCommands;

import static org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.RobotConstants.*;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.hardware.DepositSubsystem;
import org.firstinspires.ftc.teamcode.drive.hardware.LiftSubsystem;

public class HomeCommand extends CommandBase {
    private final DepositSubsystem deposit;
    private final LiftSubsystem lift;

    public HomeCommand(DepositSubsystem deposit, LiftSubsystem lift) {
        this.deposit = deposit;
        this.lift = lift;
        addRequirements(deposit, lift);
    }

    @Override
    public void initialize() {
        deposit.outtakeBusy = true;

        deposit.elbow.turnToAngle(ELBOW_HOME);
        deposit.wrist.turnToAngle(WRIST_HOME);
        deposit.spin.turnToAngle(SPIN_HOME);
        deposit.clawReset();
    }

    @Override
    public void execute() {
        lift.liftLM.motor.setTargetPosition(0);
        lift.liftRM.motor.setTargetPosition(0);

        lift.liftLM.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.liftRM.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lift.liftLM.motor.setPower(1);
        lift.liftRM.motor.setPower(1);
    }

    @Override
    public void end(boolean interrupted) {
        deposit.outtakeBusy = false;
    }

    @Override
    public boolean isFinished() {
        return lift.liftLM.getCurrentPosition() < 10;
    }
}