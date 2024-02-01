package org.firstinspires.ftc.teamcode.drive.commands.autoCommands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.AAutoState;
import org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.AAutoState.RobotAlliance;
import org.firstinspires.ftc.teamcode.drive.hardware.DepositSubsystem;
import org.firstinspires.ftc.teamcode.drive.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.drive.hardware.LiftSubsystem;
import org.firstinspires.ftc.teamcode.drive.rr.bCADMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.vision2.VisionPropPipeline;

public class MoveToParking extends CommandBase {
    private final bCADMecanumDrive drive;
    private final ElapsedTime timer = new ElapsedTime();

    private final Pose2d PARKING_POSE;
    private final RobotAlliance alliance;

    public MoveToParking(bCADMecanumDrive drive,
                         RobotAlliance alliance,
                         Pose2d PARKING_POSE) {
        this.drive = drive;
        this.alliance = alliance;
        this.PARKING_POSE = PARKING_POSE;

        addRequirements();
    }

    @Override
    public void initialize() {
        Trajectory parking = drive
                .trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(PARKING_POSE).build();

        drive.followTrajectory(parking);
        ExecuteRotation(alliance == AAutoState.RobotAlliance.RED ? 90 : 270, true); // note: ensure field centric heading on finish
    }

    @Override
    public void execute() { drive.update(); }

    @Override
    public void end(boolean interrupted) { ExecuteRotation(alliance == AAutoState.RobotAlliance.RED ? 90 : 270, false); }

    @Override
    public boolean isFinished() { return !drive.isBusy(); }

    public void ExecuteRotation(double heading, boolean async) {
        double diff = heading - Math.toDegrees(drive.getPoseEstimate().getHeading());
        double amt = diff > 180 ? Math.toRadians(-(360 - diff)) : Math.toRadians(diff);
        if (async) {
            drive.turnAsync(amt);
        } else {
            drive.turn(amt);
        }
    }
}