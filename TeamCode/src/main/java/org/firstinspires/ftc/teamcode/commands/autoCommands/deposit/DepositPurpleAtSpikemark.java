package org.firstinspires.ftc.teamcode.commands.autoCommands.deposit;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.DriveConstants;
import org.firstinspires.ftc.teamcode.rr.bCADMecanumDrive;
import org.firstinspires.ftc.teamcode.vision2.VisionPropPipeline;

public class DepositPurpleAtSpikemark extends CommandBase {
    private final bCADMecanumDrive drive;
    private final ElapsedTime timer = new ElapsedTime();

    private VisionPropPipeline.Randomization randomization;

    public DepositPurpleAtSpikemark(bCADMecanumDrive drive) {
        this.drive = drive;

        // note: no subsystem requirements for this one
        addRequirements();
    }

    @Override
    public void initialize() {
        drive.followTrajectory(CalcKinematics(2, 0));
    }

    @Override
    public void execute() { drive.update(); }

    @Override
    public void end(boolean interrupted) { }

    @Override
    public boolean isFinished() { return !drive.isBusy(); }

    private Trajectory CalcKinematics(double inches, double speed) {
        double finalSpeed = speed == 0 ? DriveConstants.MAX_VEL : speed;
        return drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(inches,
                        bCADMecanumDrive.getVelocityConstraint(finalSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        bCADMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
    }
}