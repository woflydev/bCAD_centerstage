package org.firstinspires.ftc.teamcode.commands.autoCommands.pickup;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
    private final ElapsedTime timer = new ElapsedTime();

    private final int stateDuration = 1000;

    private boolean finish = false;

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
        intake.spin();
        timer.reset();

        drive.followTrajectorySequenceAsync(CalcKinematics(6, 0));
    }

    @Override
    public void execute() {
        drive.update();
        drive.CheckForBonk();

        // todo: add colour sensor input when calvin fixes it
        /*if (intake.intakeM.motorEx.getPower() >= RobotConstants.INTAKE_SPEED - 0.05
            && intake.intakeM.getCorrectedVelocity() <= RobotAutoConstants.MIN_ALLOWABLE_INTAKE_VEL) {
            intake.cautiousReverseSpin();
        } else {
            intake.spin();
        }*/

        /*if (drive.bonked) {
            intake.cautiousReverseSpin();

            TrajectorySequence realign = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(wCyclingCheckpoints[1])
                    .build();

            drive.followTrajectorySequenceAsync(realign);
            intake.spin();

            drive.bonked = false;
        }*/
    }

    @Override
    public void end(boolean interrupted) {
        deposit.outtakeBusy = false;
        intake.stop();
    }

    @Override
    public boolean isFinished() { return (withinState(3, 99) && !drive.isBusy()) || finishTriggered(); }

    private boolean withinState(double stateNumber, double endTimeFactor) {
        return timer.milliseconds() >= (stateDuration * stateNumber) && timer.milliseconds() <= stateDuration * (stateNumber + endTimeFactor);
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