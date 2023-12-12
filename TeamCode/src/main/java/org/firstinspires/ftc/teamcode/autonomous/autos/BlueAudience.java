package org.firstinspires.ftc.teamcode.autonomous.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomous.propLocationDetection.propLocationDetector;

// Import Custom Classes
import org.firstinspires.ftc.teamcode.commands.OpModeTemplate;
import org.firstinspires.ftc.teamcode.commands.ScoreCommand;
import org.firstinspires.ftc.teamcode.commands.TransferCommand;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Blue Audience Side", group = "Blue Autos")

public class BlueAudience extends OpModeTemplate {

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
        propLocationDetector propLocationDetector = new propLocationDetector(propCamera, "BLUE");

        drive = new SampleMecanumDrive(hardwareMap);

        // Generate the autos
        rightPurple = drive.trajectorySequenceBuilder(new Pose2d(-36.70, 67.40, Math.toRadians(90.00)))
                .lineTo(new Vector2d(-43.82, 42.04))
                .build();
        leftPurple = drive.trajectorySequenceBuilder(new Pose2d(-36.70, 67.40, Math.toRadians(90.00)))
                .lineToSplineHeading(new Pose2d(-45.45, 51.39, Math.toRadians(130.00)))
                .lineToLinearHeading(new Pose2d(-28.84, 38.78, Math.toRadians(145.00)))
                .build();
        middlePurple = drive.trajectorySequenceBuilder(new Pose2d(-36.70, 67.40, Math.toRadians(90.00)))
                .lineTo(new Vector2d(-36.11, 39.82))
                .build();

        rightPixelAvoidance = drive.trajectorySequenceBuilder(rightPurple.start())
                .lineTo(new Vector2d(-37.05, 13.86))
                .lineTo(new Vector2d(14.00, 14.00))
                .lineToLinearHeading(new Pose2d(48.00, 32.00, Math.toRadians(180.00)))
                .build();

        middlePixelAvoidance = drive.trajectorySequenceBuilder(new Pose2d(-48.12, 49.90, Math.toRadians(180.00)))
                .lineTo(new Vector2d(-47.83, 12.53))
                .lineTo(new Vector2d(14.00, 14.00))
                .lineToLinearHeading(new Pose2d(48, 39, Math.toRadians(180.00)))
                .build();

        leftPixelAvoidance = drive.trajectorySequenceBuilder(new Pose2d(-36.45, 51.97, Math.toRadians(180.00)))
                .lineTo(new Vector2d(-36.60, 12.65))
                .lineTo(new Vector2d(14.00, 14.00))
                .lineToLinearHeading(new Pose2d(48.00, 46.00, Math.toRadians(180.00)))
                .build();

            backboardToStack = drive.trajectorySequenceBuilder(new Pose2d(41.18, 30.72, Math.toRadians(180.00)))
                    .lineTo(new Vector2d(6.45, 6.45))
                    .splineToLinearHeading(new Pose2d(-62.00, 14.00, Math.toRadians(180.00)), Math.toRadians(180.00))
                    .build();

            stackToBackboard = drive.trajectorySequenceBuilder(backboardToStack.end())
                    .lineToLinearHeading(new Pose2d(9.42, 3.93, Math.toRadians(180.00)))
                    .lineToLinearHeading(new Pose2d(48, 34.33, Math.toRadians(180.00)))
                    .build();

            intakeProne = drive.trajectorySequenceBuilder(backboardToStack.end())
                    .back(10)
                    .forward(10)
                    .build();

        // Detect the team prop's location
        while (opModeInInit()) {
            propLocation = propLocationDetector.detectPropLocation(telemetry, propLocation);
            telemetry.addData("Detected Side", propLocation);
            telemetry.update();
        }


        switch(propLocation) {
            case LEFT:
            case NOT_FOUND:
                chosenSequence = leftPurple;
                break;
            case FRONT:
                chosenSequence = middlePurple;
                break;
            case RIGHT:
                chosenSequence = rightPurple;
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

    public void correctBackdropAngle(TrajectorySequence current) {
        double turnValue = Math.toRadians(90 - Math.toDegrees(drive.getExternalHeading()));

        if(turnValue > Math.abs(Math.toRadians(1))) {
            TrajectorySequence corrective = drive.trajectorySequenceBuilder(current.end())
                    .turn(turnValue)
                    .build();

            drive.followTrajectorySequence(corrective);
        }
    }

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

            drive.setPoseEstimate(new Pose2d(-36.70, 67.40, Math.toRadians(90.00)));

            drive.followTrajectorySequence(chosenSequence);

            // increase i term

            if(chosenSequence == rightPurple) {
                drive.followTrajectorySequence(rightPixelAvoidance);

                ElapsedTime wait = new ElapsedTime();
                while (wait.seconds() < 1) {
                    correctBackdropAngle(rightPixelAvoidance);
                }

            } else if(chosenSequence == middlePurple) {
                drive.followTrajectorySequence(middlePixelAvoidance);

                ElapsedTime wait = new ElapsedTime();
                while (wait.seconds() < 1) {
                    correctBackdropAngle(middlePixelAvoidance);
                }
            } else if(chosenSequence == leftPurple) {
                drive.followTrajectorySequence(leftPixelAvoidance);

                ElapsedTime wait = new ElapsedTime();
                while (wait.seconds() < 1) {
                    correctBackdropAngle(leftPixelAvoidance);
                }
            }

            telemetry.addData("Remaining Difference: ", Math.toRadians(90 - Math.toDegrees(drive.getExternalHeading())));
            telemetry.update();

            autoScore();

            cycle();

        }

    }


}