package org.firstinspires.ftc.teamcode.commands.autoCommands.localizer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.AAutoState.RobotAlliance;
import org.firstinspires.ftc.teamcode.rr.bCADMecanumDrive;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision2.VisualLoc;

public class RelocalizeAtBackdrop extends CommandBase {
    private final bCADMecanumDrive drive;
    private final VisualLoc visualLoc;
    private final Telemetry t;
    private final ElapsedTime timer = new ElapsedTime();

    public RelocalizeAtBackdrop(bCADMecanumDrive drive,
                                VisualLoc visualLoc,
                                Telemetry t) {
        this.drive = drive;
        this.visualLoc = visualLoc;
        this.t = t;
        addRequirements();
    }

    @Override
    public void initialize() {
        drive.setPoseEstimate(visualLoc.WhereTheHellAmI());
        t.addData("Result of WTHAI", visualLoc.WhereTheHellAmI());
        t.update();
    }

    @Override
    public void execute() {  }

    @Override
    public void end(boolean interrupted) {
        visualLoc.stop();
    }

    @Override
    public boolean isFinished() { return true; }
}