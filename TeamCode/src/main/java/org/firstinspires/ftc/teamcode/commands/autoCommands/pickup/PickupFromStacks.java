package org.firstinspires.ftc.teamcode.commands.autoCommands.pickup;

import static org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotAutoConstants.BACKDROP_CYCLE_DROPOFF_POSES;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
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
        stateTimer.reset();
        utilTimer.reset();

        // note: the below piece of code kinda works but not really
        /*intake.spin();
        TrajectorySequence traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(wCyclingCheckpoints[2])
                .lineToLinearHeading(wCyclingCheckpoints[4])
                .lineToLinearHeading(wCyclingCheckpoints[5])
                .build();

        drive.followTrajectorySequenceAsync(traj);*/

        TrajectorySequence traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(wCyclingCheckpoints[2])
                .build();

        intake.cautiousReverseSpin();
        timeout(0.5);
        intake.autoSpin();
        drive.followTrajectorySequence(CalcKinematics(0.001, 0));
        timeout(3);

        finish = true;
    }

    @Override
    public void execute() {
        //drive.update();

        if (withinState(0, 1)) {

        } else if (withinState(1, 999)) {

        }
/*
        intake.stop();
        finish = true;*/

/*        drive.update();
        drive.CheckForBonk();*/

        // todo: add colour sensor input
        if (intake.intakeM.motorEx.getPower() >= RobotConstants.INTAKE_SPEED - 0.05
            && intake.intakeM.motorEx.getCurrent(CurrentUnit.AMPS) >= RobotAutoConstants.MAX_INTAKE_CURRENT_DRAW && !ejecting) {
            intake.reverseSpin();
            utilTimer.reset();
            ejecting = true;
        } else if (utilTimer.seconds() >= 0.7 && ejecting) {
            ejecting = false;
            drive.followTrajectorySequence(CalcKinematics(2, 0));
            intake.spin();
        }
    }

    @Override
    public void end(boolean interrupted) {
        deposit.outtakeBusy = false;
        intake.reverseSpin();
        drive.breakFollowing();
    }

    @Override
    public boolean isFinished() { return finishTriggered() && !drive.isBusy(); }

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

    private void timeout(double time) {
        ElapsedTime wait = new ElapsedTime();
        wait.reset();
        while (wait.seconds() <= time) { short x; }
    }
}