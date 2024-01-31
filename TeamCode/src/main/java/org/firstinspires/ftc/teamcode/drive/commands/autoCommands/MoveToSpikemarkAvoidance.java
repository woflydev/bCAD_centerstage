package org.firstinspires.ftc.teamcode.drive.commands.autoCommands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.rr.bCADMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.drive.vision2.VisionPropPipeline;

public class MoveToSpikemarkAvoidance extends CommandBase {
    private final bCADMecanumDrive drive;
    private final ElapsedTime timer = new ElapsedTime();

    private final Pose2d[] wPurpleAvoidanceCheckpoints;

    public MoveToSpikemarkAvoidance(bCADMecanumDrive drive,
                                    Pose2d[] wPurpleAvoidanceCheckpoints) {
        this.drive = drive;
        this.wPurpleAvoidanceCheckpoints = wPurpleAvoidanceCheckpoints;

        addRequirements();
    }

    @Override
    public void initialize() {
        TrajectorySequence purpleAvoidanceTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(wPurpleAvoidanceCheckpoints[0])
                .waitSeconds(0.001)
                .splineToLinearHeading(wPurpleAvoidanceCheckpoints[1], wPurpleAvoidanceCheckpoints[1].getHeading())
                .build();
        drive.followTrajectorySequenceAsync(purpleAvoidanceTrajectory);
    }

    @Override
    public void execute() {
        drive.update();
    }

    @Override
    public void end(boolean interrupted) { }

    @Override
    public boolean isFinished() { return !drive.isBusy(); }
}