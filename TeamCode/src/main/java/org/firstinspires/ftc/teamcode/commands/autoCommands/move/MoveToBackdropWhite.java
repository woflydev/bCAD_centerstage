package org.firstinspires.ftc.teamcode.commands.autoCommands.move;

import static org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotAutoConstants.BACKDROP_CYCLE_DROPOFF_POSES;
import static org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotAutoConstants.CYCLE_RETURN_POSES;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.AAutoState.RobotAlliance;
import org.firstinspires.ftc.teamcode.rr.bCADMecanumDrive;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

public class MoveToBackdropWhite extends CommandBase {
    private final bCADMecanumDrive drive;
    private final ElapsedTime timer = new ElapsedTime();

    private final int allianceIndex;
    private final Pose2d[] wCyclingCheckpoints;

    public MoveToBackdropWhite(bCADMecanumDrive drive,
                               RobotAlliance alliance,
                               Pose2d[] wCyclingCheckpoints) {
        this.drive = drive;
        this.allianceIndex = alliance == RobotAlliance.RED ? 0 : 1;
        this.wCyclingCheckpoints = wCyclingCheckpoints;
        addRequirements();
    }

    @Override
    public void initialize() {
        TrajectorySequence toBackdropTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(wCyclingCheckpoints[3])
                .lineToLinearHeading(BACKDROP_CYCLE_DROPOFF_POSES[allianceIndex])
                .build();

        drive.followTrajectorySequenceAsync(toBackdropTrajectory);
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