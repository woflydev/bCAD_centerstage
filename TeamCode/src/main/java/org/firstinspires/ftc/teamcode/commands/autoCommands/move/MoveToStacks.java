package org.firstinspires.ftc.teamcode.commands.autoCommands.move;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.AAutoState.RobotAlliance;
import org.firstinspires.ftc.teamcode.rr.bCADMecanumDrive;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

public class MoveToStacks extends CommandBase {
    private final bCADMecanumDrive drive;
    private final ElapsedTime timer = new ElapsedTime();

    private final int allianceIndex;
    private final Pose2d[] wCyclingCheckpoints;

    public MoveToStacks(bCADMecanumDrive drive,
                        RobotAlliance alliance,
                        Pose2d[] wCyclingCheckpoints) {
        this.drive = drive;
        this.allianceIndex = alliance == RobotAlliance.RED ? 0 : 1;
        this.wCyclingCheckpoints = wCyclingCheckpoints;

        addRequirements();
    }

    @Override
    public void initialize() {
        /*TrajectorySequence cycleTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .splineToConstantHeading(STAGE_DOOR_POSES[allianceIndex].vec(), STAGE_DOOR_POSES[allianceIndex].getHeading())
                .splineToLinearHeading(CYCLING_STACK_KNOCK_POSES[allianceIndex], CYCLING_STACK_KNOCK_POSES[allianceIndex].getHeading())
                .lineToLinearHeading(CYCLING_STACK_INNER_POSES[allianceIndex])
                .build();*/
        /*TrajectorySequence cycleTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(STAGE_DOOR_POSES[allianceIndex])
                .splineTo(CYCLING_STACK_INNER_POSES[allianceIndex].vec(), CYCLING_STACK_INNER_POSES[allianceIndex].getHeading())
                .build();*/

        TrajectorySequence cycleTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .splineTo(wCyclingCheckpoints[0].vec(), wCyclingCheckpoints[0].getHeading())
                .splineTo(wCyclingCheckpoints[1].vec(), wCyclingCheckpoints[1].getHeading())
                //.splineTo(wCyclingCheckpoints[2].vec(), wCyclingCheckpoints[2].getHeading())
                .build();

        drive.followTrajectorySequenceAsync(cycleTrajectory); // note: blocking
    }

    @Override
    public void execute() { drive.update(); }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() { return !drive.isBusy(); }
}