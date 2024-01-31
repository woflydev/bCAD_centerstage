package org.firstinspires.ftc.teamcode.drive.commands.autoCommands;

import static org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.RobotAutoConstants.CAUTION_SPEED;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.AAutoState.RobotAlliance;
import org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.rr.bCADMecanumDrive;

public class ApproachWhite extends CommandBase {
    private final bCADMecanumDrive drive;
    private final ElapsedTime timer = new ElapsedTime();

    public ApproachWhite(bCADMecanumDrive drive) {
        this.drive = drive;
        addRequirements();
    }

    @Override
    public void initialize() {
        drive.followTrajectory(CalcKinematics(3, CAUTION_SPEED));
    }

    @Override
    public void execute() { }

    @Override
    public void end(boolean interrupted) {
        // uses blocking drive
    }

    @Override
    public boolean isFinished() { return !drive.isBusy(); }

    public Trajectory CalcKinematics(double inches, double speed) {
        double finalSpeed = speed == 0 ? DriveConstants.MAX_VEL : speed;
        return drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(inches,
                        bCADMecanumDrive.getVelocityConstraint(finalSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        bCADMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
    }
}