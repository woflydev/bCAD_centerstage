package org.firstinspires.ftc.teamcode.commands.autoCommands.move;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.rr.bCADMecanumDrive;
import org.firstinspires.ftc.teamcode.vision2.VisionPropPipeline;

public class MoveToSpikemark extends CommandBase {
    private final bCADMecanumDrive drive;
    private final ElapsedTime timer = new ElapsedTime();

    private final Pose2d[] wPurpleSpikemarkAlign;
    private final VisionPropPipeline.Randomization randomization;

    public MoveToSpikemark(bCADMecanumDrive drive,
                           VisionPropPipeline.Randomization randomization,
                           Pose2d[] wPurpleSpikemarkAlign) {
        this.drive = drive;
        this.randomization = randomization;
        this.wPurpleSpikemarkAlign = wPurpleSpikemarkAlign;

        addRequirements();
    }

    @Override
    public void initialize() {
        switch (randomization) {
            case LOCATION_1:
                drive.followTrajectoryAsync(GenerateLineTraj(wPurpleSpikemarkAlign[0], false));
                break;
            case LOCATION_2:
                drive.followTrajectoryAsync(GenerateLineTraj(wPurpleSpikemarkAlign[1], false));
                break;
            case LOCATION_3:
                drive.followTrajectoryAsync(GenerateLineTraj(wPurpleSpikemarkAlign[2], false));
                break;
        }
    }

    @Override
    public void execute() {
        drive.update();
    }

    @Override
    public void end(boolean interrupted) { }

    @Override
    public boolean isFinished() { return !drive.isBusy(); }

    private Trajectory GenerateLineTraj(Pose2d target, boolean constantHeading) {
        return constantHeading ?
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToConstantHeading(target.vec())
                        .build()
                :
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(target)
                        .build();
    }
}