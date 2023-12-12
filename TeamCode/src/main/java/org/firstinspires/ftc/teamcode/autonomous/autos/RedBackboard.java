package org.firstinspires.ftc.teamcode.autonomous.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomous.propLocationDetection.propLocationDetector;

// Import Custom Classes
import org.firstinspires.ftc.teamcode.commands.OpModeTemplate;
import org.firstinspires.ftc.teamcode.commands.ScoreCommand;
import org.firstinspires.ftc.teamcode.commands.TransferCommand;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Red Backboard Side", group = "Red Autos")

public class RedBackboard extends OpModeTemplate {

    // Create Prop Location-Related Variables
    private propLocationDetector.Locations propLocation = propLocationDetector.Locations.NOT_FOUND;

    private SampleMecanumDrive drive;
    TrajectorySequence rightPurple;
    TrajectorySequence leftPurple;
    TrajectorySequence middlePurple;
    TrajectorySequence rightPixelAvoidance;
    TrajectorySequence middlePixelAvoidance;
    TrajectorySequence leftPixelAvoidance;
    TrajectorySequence chosenSequence;
    TrajectorySequence chosenFollowUp;
    TrajectorySequence backboardToStack;
    TrajectorySequence stackToBackboard;
    TrajectorySequence intakeProne;

    private boolean autoAlreadyRun;

    public void initialize() {
        autoAlreadyRun = false;
        initHardware(false);

        deposit.V4B.turnToAngle(180);

        ElapsedTime wait = new ElapsedTime();
        wait.reset();
        while(wait.seconds() < 2) {
        }

        deposit.Gripper.turnToAngle(110);

        wait.reset();
        while(wait.seconds() < 2) {
        }

        deposit.V4B.turnToAngle(270);

        // Initiate the camera
        WebcamName propCamera = hardwareMap.get(WebcamName.class, "Webcam");
        // TODO: Change the alliance colour when swapping over <-----
        propLocationDetector propLocationDetector = new propLocationDetector(propCamera, "RED");

        drive = new SampleMecanumDrive(hardwareMap);

        // Generate the autos

        int offset = 8;

        middlePurple = drive.trajectorySequenceBuilder(new Pose2d(8.68, -64.14, Math.toRadians(270.00)))
                .lineTo(new Vector2d(8.53, -40.86))
                .build();

        middlePixelAvoidance = drive.trajectorySequenceBuilder(new Pose2d(8.38, -40.12, Math.toRadians(270.00)))
                .lineToLinearHeading(new Pose2d(48.00, -35.96, Math.toRadians(180.00)))
                .build();

        rightPurple = drive.trajectorySequenceBuilder(new Pose2d(8.68, -64.14, Math.toRadians(270.00)))
                .lineToLinearHeading(new Pose2d(8.00-offset, -43.00, Math.toRadians(290.00)))
                .build();

        rightPixelAvoidance = drive.trajectorySequenceBuilder(new Pose2d(7.00-offset, -60.00, Math.toRadians(290.00)))
                .lineToLinearHeading(new Pose2d(48.00-4, -29.29, Math.toRadians(180.00)))
                .build();

        leftPurple = drive.trajectorySequenceBuilder(new Pose2d(8.68, -64.14, Math.toRadians(270.00)))
                .lineToLinearHeading(new Pose2d(17.72-offset, -47.09, Math.toRadians(240.00)))
                .build();

        leftPixelAvoidance = drive.trajectorySequenceBuilder(new Pose2d(12.53-3, -60.00, Math.toRadians(290.00)))
                .lineToLinearHeading(new Pose2d(48.00-4, -45.31, Math.toRadians(180.00)))
                .build();


        // Detect the team prop's location
        while (opModeInInit()) {
            propLocation = propLocationDetector.detectPropLocation(telemetry, propLocation);
            telemetry.addData("Detected Side", propLocation);

            FtcDashboard dashboard = FtcDashboard.getInstance();
            Telemetry dashboardTelemetry = dashboard.getTelemetry();

            dashboardTelemetry.addData("Detected Side", propLocation);
            dashboardTelemetry.update();

            telemetry.update();
        }

        switch(propLocation) {
            case NOT_FOUND:
                chosenSequence = rightPurple;
                chosenFollowUp = rightPixelAvoidance;
                break;
            case FRONT:
                chosenSequence = middlePurple;
                chosenFollowUp = middlePixelAvoidance;
                break;
            case RIGHT:
                chosenSequence = leftPurple;
                chosenFollowUp = leftPixelAvoidance;
                break;
        }
    }

    public void autoScore() {
        if (propLocation == propLocationDetector.Locations.NOT_FOUND) {
            propLocation = propLocationDetector.Locations.LEFT;
        }
        new ScoreCommand(deposit, lift).schedule();

        ElapsedTime wait = new ElapsedTime();
        wait.reset();
        while(wait.seconds() < 1) {}

        lift.autoRun();
        deposit.release();

        wait.reset();
        while(wait.seconds() < 0.5) {}
        wait.reset();

        lift.autoHome();
    }

//    public void correctBackdropAngle(TrajectorySequence current) {
//        double turnValue = Math.toRadians(90 - Math.toDegrees(drive.getExternalHeading()));
//
//        if(turnValue > Math.abs(Math.toRadians(1))) {
//            TrajectorySequence corrective = drive.trajectorySequenceBuilder(current.end())
//                    .turn(turnValue)
//                    .build();
//
//            drive.followTrajectorySequence(corrective);
//        }
//    }

    public void timeout(double time) {
        ElapsedTime wait = new ElapsedTime();
        wait.reset();
        while(wait.seconds() < time) {}
    }

    public void cycle() {
        drive.followTrajectorySequence(backboardToStack);

        intake.spin();

        timeout(0.5);

        drive.followTrajectorySequence(intakeProne);

        timeout(1);

        new TransferCommand(deposit, lift, intake).schedule();

        timeout(0.5);

        intake.Rspin();

        drive.followTrajectorySequence(stackToBackboard);

        autoScore();
    }


    public void run() {
        if (!autoAlreadyRun) {
            autoAlreadyRun = true;

            // Show the detected side to the telemetry
            telemetry.addData("Detected Side", propLocation);
            telemetry.update();

            drive.setPoseEstimate(middlePurple.start());

            drive.followTrajectorySequence(chosenSequence);
            drive.followTrajectorySequence(chosenFollowUp);

            // increase i term

            autoScore();

//            cycle();

        }

    }


}