package org.firstinspires.ftc.teamcode.drive.commands.autoCommands;

import static org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.RobotAutoConstants.CYCLING_STACK_INNER_POSES;
import static org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.RobotAutoConstants.CYCLING_STACK_KNOCK_POSES;
import static org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.RobotAutoConstants.STAGE_DOOR_POSES;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.AAutoState.RobotAlliance;
import org.firstinspires.ftc.teamcode.drive.rr.bCADMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.rr.trajectorysequence.TrajectorySequence;

public class MoveToStacks extends CommandBase {
    private final bCADMecanumDrive drive;
    private final ElapsedTime timer = new ElapsedTime();

    private final int allianceIndex;

    public MoveToStacks(bCADMecanumDrive drive,
                        RobotAlliance alliance) {
        this.drive = drive;
        this.allianceIndex = alliance == RobotAlliance.RED ? 0 : 1;

        addRequirements();
    }

    @Override
    public void initialize() {
        TrajectorySequence cycleTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .splineToConstantHeading(STAGE_DOOR_POSES[allianceIndex].vec(), STAGE_DOOR_POSES[allianceIndex].getHeading())
                .waitSeconds(0.001)
                .splineToLinearHeading(CYCLING_STACK_KNOCK_POSES[allianceIndex], CYCLING_STACK_KNOCK_POSES[allianceIndex].getHeading())
                .waitSeconds(0.001)
                .lineToLinearHeading(CYCLING_STACK_INNER_POSES[allianceIndex])
                .build();
        drive.followTrajectorySequence(cycleTrajectory); // note: blocking
    }

    @Override
    public void execute() { }

    @Override
    public void end(boolean interrupted) {
        // uses blocking drive
    }

    @Override
    public boolean isFinished() { return !drive.isBusy(); }
}