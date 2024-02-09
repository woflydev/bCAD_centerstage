package org.firstinspires.ftc.teamcode.commands.autoCommands.pickup;

import static org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotAutoConstants.BACKDROP_CYCLE_DROPOFF_POSES;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.AAutoState;
import org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.DriveConstants;
import org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotAutoConstants;
import org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotConstants;
import org.firstinspires.ftc.teamcode.hardware.DepositSubsystem;
import org.firstinspires.ftc.teamcode.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.LiftSubsystem;
import org.firstinspires.ftc.teamcode.rr.bCADMecanumDrive;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

public class PickupFromStacks extends CommandBase {
    private final DepositSubsystem deposit;
    private final bCADMecanumDrive drive;
    private final LiftSubsystem lift;
    private final IntakeSubsystem intake;
    private final AAutoState.RobotAlliance alliance;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime utilTimer = new ElapsedTime();

    private final int stateDuration = 500;

    private boolean finish = false;
    private boolean ejecting = false;

    private Pose2d[] wCyclingCheckpoints;

    public PickupFromStacks(bCADMecanumDrive drive,
                            DepositSubsystem deposit,
                            LiftSubsystem lift,
                            IntakeSubsystem intake,
                            AAutoState.RobotAlliance alliance) {
        this.drive = drive;
        this.deposit = deposit;
        this.lift = lift;
        this.intake = intake;
        this.alliance = alliance;
        this.wCyclingCheckpoints = alliance == AAutoState.RobotAlliance.RED ? RobotAutoConstants.RED_CYCLE_CHECKPOINTS : RobotAutoConstants.BLUE_CYCLE_CHECKPOINTS;

        addRequirements(deposit, lift);
    }

    @Override
    public void initialize() {
        deposit.outtakeBusy = false;
        finish = false;
        ejecting = false;
        intake.cautiousReverseSpin();
        stateTimer.reset();
        utilTimer.reset();

        drive.followTrajectorySequenceAsync(CalcKinematics(4, 0));
    }

    @Override
    public void execute() {
        drive.update();
        //drive.CheckForBonk();

        if (withinState(1, 1)) {
            intake.stop();
        } else if (withinState(2, 1)) {
            intake.spin();

            TrajectorySequence traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(wCyclingCheckpoints[2])
                    .build();

            drive.followTrajectorySequence(traj);
        } else if (withinState(4, 999)) {
            intake.stop();
            finish = true;
        }

        // todo: add colour sensor input when calvin fixes it
        /*if (intake.intakeM.motorEx.getPower() >= RobotConstants.INTAKE_SPEED - 0.05
            && intake.intakeM.motorEx.isOverCurrent() && !ejecting) {
            intake.reverseSpin();
            utilTimer.reset();
            ejecting = true;
        } else if (utilTimer.seconds() >= 3 && ejecting) {
            ejecting = false;
            intake.spin();
        }*/
    }

    @Override
    public void end(boolean interrupted) {
        deposit.outtakeBusy = false;
        intake.reverseSpin();
        drive.breakFollowing();
    }

    @Override
    public boolean isFinished() { return finishTriggered(); }

    private boolean withinState(double stateNumber, double endTimeFactor) {
        return stateTimer.milliseconds() >= (stateDuration * stateNumber) && stateTimer.milliseconds() <= stateDuration * (stateNumber + endTimeFactor);
    }

    private TrajectorySequence CalcKinematics(double inches, double speed) {
        double finalSpeed = speed == 0 ? DriveConstants.MAX_VEL : speed;
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(inches,
                        bCADMecanumDrive.getVelocityConstraint(finalSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        bCADMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
    }

    private boolean finishTriggered() {
        return finish;
    }
}