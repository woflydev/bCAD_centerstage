package org.firstinspires.ftc.teamcode.drive.commands.autoCommands;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.hardware.DepositSubsystem;
import org.firstinspires.ftc.teamcode.drive.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.drive.hardware.LiftSubsystem;
import org.firstinspires.ftc.teamcode.drive.rr.bCADMecanumDrive;

public class DepositAutoCommand extends CommandBase {
    private final DepositSubsystem deposit;
    private final bCADMecanumDrive drive;
    private final LiftSubsystem lift;
    private final IntakeSubsystem intake;
    private final ElapsedTime timer = new ElapsedTime();

    private final Telemetry t;

    private final int stateDuration = 250;

    public DepositAutoCommand(bCADMecanumDrive drive, DepositSubsystem deposit, LiftSubsystem lift, IntakeSubsystem intake, Telemetry t) {
        this.drive = drive;
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
    }

    @Override
    public void execute() {
        if (withinState(0)) {
            deposit.elbow.turnToAngle(RobotConstants.ELBOW_AUTO_ACTIVE);
            deposit.wrist.turnToAngle(RobotConstants.WRIST_AUTO_ACTIVE);
        } else if (withinState(1)) {
            deposit.wrist.turnToAngle(RobotConstants.WRIST_AUTO_ACTIVE);
            deposit.clawDeposit();
        }
    }

    @Override
    public void end(boolean interrupted) {
        deposit.outtakeBusy = false;
    }

    @Override
    public boolean isFinished() { return (timer.milliseconds() >= stateDuration * 3) && !drive.isBusy(); }

    private boolean withinState(double stateNumber) {
        return timer.milliseconds() >= (stateDuration * stateNumber) && timer.milliseconds() <= stateDuration * (stateNumber + 1);
    }

    public Trajectory CalcKinematics(double inches, double speed) {
        double finalSpeed = speed == 0 ? DriveConstants.MAX_VEL : speed;
        return drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(inches,
                        bCADMecanumDrive.getVelocityConstraint(finalSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        bCADMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
    }
}