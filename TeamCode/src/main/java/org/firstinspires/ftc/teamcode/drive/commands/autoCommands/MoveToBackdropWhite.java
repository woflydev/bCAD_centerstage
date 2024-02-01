package org.firstinspires.ftc.teamcode.drive.commands.autoCommands;

import static org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.RobotAutoConstants.BACKDROP_CYCLE_DROPOFF_POSES;
import static org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.RobotAutoConstants.CYCLE_RETURN_POSES;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.AAutoState.RobotAlliance;
import org.firstinspires.ftc.teamcode.drive.rr.bCADMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.rr.trajectorysequence.TrajectorySequence;

import java.lang.reflect.Executable;

public class MoveToBackdropWhite extends CommandBase {
    private final bCADMecanumDrive drive;
    private final ElapsedTime timer = new ElapsedTime();

    private final int allianceIndex;

    public MoveToBackdropWhite(bCADMecanumDrive drive, RobotAlliance alliance) {
        this.drive = drive;
        this.allianceIndex = alliance == RobotAlliance.RED ? 0 : 1;
        addRequirements();
    }

    @Override
    public void initialize() {
        TrajectorySequence toBackdropTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(CYCLE_RETURN_POSES[allianceIndex])
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