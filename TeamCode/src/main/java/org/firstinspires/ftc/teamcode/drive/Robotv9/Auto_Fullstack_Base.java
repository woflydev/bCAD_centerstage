package org.firstinspires.ftc.teamcode.drive.Robotv9;

import static org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.RobotAutoConstants.*;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.AutoState.*;
import org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.commands.OpModeTemplate;
import org.firstinspires.ftc.teamcode.drive.commands.AutoScoreCommand;
import org.firstinspires.ftc.teamcode.drive.commands.TransferCommand;
import org.firstinspires.ftc.teamcode.drive.rr.bCADMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.drive.vision2.VisionPropPipeline;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "Blue Audience Side", group = "Blue Autos")

public class Auto_Fullstack_Base extends OpModeTemplate {
    private RootAutoState autoState = RootAutoState.BA_PLAY;
    private bCADMecanumDrive drive;
    public VisionPropPipeline.Randomization randomization;
    public RobotAlliance alliance;
    public RobotStartingPosition startingPosition;
    public RobotParkingLocation parkingLocation;
    public RobotTaskFinishBehaviour taskFinishBehaviour;
    public RobotLocMode locMode;
    private boolean autoAlreadyRun;
    public boolean taskFinishBehaviourSelected;
    public int cycleCounter = 0;
    public int allianceIndex;
    public int startingPositionIndex;
    public int parkingLocationIndex;
    public int dir;
    public Pose2d[] workingYellowBackdropAlign;
    public Pose2d[] workingPurpleSpikemarkAlign;
    public Point r1;
    public Point r2;
    public Point r3;

    private final ElapsedTime autoTimer = new ElapsedTime();

    private TrajectorySequence rightPurple;
    private TrajectorySequence leftPurple;
    private TrajectorySequence middlePurple;
    private TrajectorySequence rightPixelAvoidance;
    private TrajectorySequence middlePixelAvoidance;
    private TrajectorySequence leftPixelAvoidance;
    private TrajectorySequence chosenSequence;
    private TrajectorySequence backboardToStack;
    private TrajectorySequence stackToBackboard;
    private TrajectorySequence intakeProne;

    public Auto_Fullstack_Base(RobotAlliance alliance,
                               RobotStartingPosition startPos,
                               RobotParkingLocation parkLoc,
                               RobotTaskFinishBehaviour taskFinBehaviour,
                               Point r1, Point r2, Point r3) {
        this.alliance = alliance;
        this.startingPosition = startPos;
        this.parkingLocation = parkLoc;
        this.taskFinishBehaviour = taskFinBehaviour;
        this.dir = alliance == RobotAlliance.RED ? 1 : -1;
        this.r1 = r1;
        this.r2 = r2;
        this.r3 = r3;
    }

    public void initialize() {
        autoAlreadyRun = false;
        InitBlock();

        deposit.elbow.turnToAngle(180);
        timeout(2);
        deposit.claw.turnToAngle(110);
        timeout(2);
        deposit.elbow.turnToAngle(270);

        drive = new bCADMecanumDrive(hardwareMap, telemetry);
        workingYellowBackdropAlign = SortYellowBackdropAlign();
        workingPurpleSpikemarkAlign = SortPurpleSpikemarkAlign();
        BuildTrajectories();
        VisionPropDetection();
    }

    public void run() {
        if (!autoAlreadyRun) {
            drive.setPoseEstimate(new Pose2d(-36.70, 67.40, Math.toRadians(90.00)));
            autoAlreadyRun = true;

            while (!isStopRequested() && opModeIsActive()) {
                StatusTelemetry();
                HandlePurple();
                HandleYellow();

                drive.update();
            }
        }
    }

    private void HandlePurple() {
        if (startingPosition == RobotStartingPosition.BACKDROP) {
            switch (autoState) {
                case BA_PLAY:
                    switch (randomization) {
                        case LOCATION_1:
                            drive.followTrajectoryAsync(GenerateTraj(workingPurpleSpikemarkAlign[0], false));
                            break;
                        case LOCATION_2:
                            drive.followTrajectoryAsync(GenerateTraj(workingPurpleSpikemarkAlign[1], false));
                            break;
                        case LOCATION_3:
                            drive.followTrajectoryAsync(GenerateTraj(workingPurpleSpikemarkAlign[2], false));
                            break;
                    }
                    autoState = RootAutoState.BA_DEPOSIT_PURPLE;
                    break;
                case BA_DEPOSIT_PURPLE:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(CalcKinematics(6, 0));
                        autoState = RootAutoState.BA_MOVING_TO_BACKDROP;
                    }
                    break; // note: handoff to yellow
            }
        }
    }

    private void HandleYellow() {
        if (startingPosition == RobotStartingPosition.BACKDROP) {
            switch (autoState) {
                case BA_MOVING_TO_BACKDROP:
                    switch (randomization) {
                        case LOCATION_1:
                            drive.followTrajectoryAsync(GenerateTraj(workingYellowBackdropAlign[0], false));
                            break;
                        case LOCATION_2:
                            drive.followTrajectoryAsync(GenerateTraj(workingYellowBackdropAlign[1], false));
                            break;
                        case LOCATION_3:
                            drive.followTrajectoryAsync(GenerateTraj(workingYellowBackdropAlign[2], false));
                            break;
                    }
                    autoTimer.reset();
                    autoState = RootAutoState.BA_DEPOSIT_YELLOW;
                    break;
                case BA_DEPOSIT_YELLOW:
                    if (!drive.isBusy()) {
                        ExecuteRotation(180, false); // note: alignment should be blocking
                        Score();
                        autoTimer.reset();
                        if (taskFinishBehaviour == RobotTaskFinishBehaviour.DO_NOT_CYCLE) {
                            autoState = RootAutoState.BA_MOVING_TO_PARKING;
                        } else {
                            autoState = RootAutoState.BA_MOVING_TO_CYCLE;
                        }
                    }
                    break;
            }
        }
    }

    private void StatusTelemetry() {
        telemetry.addData("Autonomous Clock", autoTimer.seconds());
        telemetry.addData("Robot X", drive.getPoseEstimate().getX());
        telemetry.addData("Robot Y", drive.getPoseEstimate().getY());
        telemetry.addData("Robot Heading", Math.toDegrees(drive.getPoseEstimate().getHeading()));
        telemetry.addData("Target Location",
                randomization == VisionPropPipeline.Randomization.LOCATION_1 ? "LEFT (LOC_1)" :
                randomization == VisionPropPipeline.Randomization.LOCATION_2 ? "MIDDLE (LOC_2)" :
                randomization == VisionPropPipeline.Randomization.LOCATION_3 ? "RIGHT (LOC_3)" : "NONE"
        );
        telemetry.addLine("---------");
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Starting Position", startingPosition);
        telemetry.addData("Task Completion Behavior", taskFinishBehaviour);
        telemetry.addData("Parking Location", parkingLocation);
        telemetry.update();
    }

    private void VisionPropDetection() {
        OpenCvWebcam webcam;
        VisionPropPipeline pipeline = new VisionPropPipeline( alliance, r1, r2, r3 );

        @SuppressLint("DiscouragedApi")
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId",
                "id",
                hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(webcam, 20);
            }

            @Override
            public void onError(int errorCode) {
                // intentional noop
                telemetry.addLine("error, you stupid idiot");
            }
        });

        autoTimer.reset();
        while (opModeInInit()) {
            randomization = pipeline.getRandomization();
            telemetry.addData("TEAM_PROP_LOCATION", randomization);
            telemetry.update();
        }
        webcam.closeCameraDevice();

        switch(randomization) {
            case LOCATION_1:
                chosenSequence = leftPurple;
                break;
            default:
            case LOCATION_2:
                chosenSequence = middlePurple;
                break;
            case LOCATION_3:
                chosenSequence = rightPurple;
                break;
        }
    }

    private void BuildTrajectories() {
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
    }

    public void ExecuteRotation(double heading, boolean async) {
        double diff = heading - Math.toDegrees(drive.getPoseEstimate().getHeading());
        double amt = diff > 180 ? Math.toRadians(-(360 - diff)) : Math.toRadians(diff);
        if (async) {
            drive.turnAsync(amt);
        } else {
            drive.turn(amt);
        }
    }

    private Trajectory GenerateTraj(Pose2d target, boolean constantHeading) {
        return constantHeading ?
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToConstantHeading(target.vec())
                        .build()
                :
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(target)
                        .build();
    }

    public Trajectory CalcKinematics(double inches, double speed) {
        double finalSpeed = speed == 0 ? DriveConstants.MAX_VEL : speed;
        return drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(inches,
                        bCADMecanumDrive.getVelocityConstraint(finalSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        bCADMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
    }

    private Pose2d[] SortYellowBackdropAlign() {
        return alliance == RobotAlliance.RED ?
                RED_YELLOW_PIXEL_BACKDROP_POSES : BLUE_YELLOW_PIXEL_BACKDROP_POSES;
    }

    private Pose2d[] SortPurpleSpikemarkAlign() {
        if (alliance == RobotAlliance.RED) {
            return startingPosition == RobotStartingPosition.BACKDROP ? RED_PURPLE_PIXEL_SPIKEMARK_POSES : ApplyAudienceOffset(RED_PURPLE_PIXEL_SPIKEMARK_POSES);
        } else {
            return startingPosition == RobotStartingPosition.BACKDROP ? BLUE_PURPLE_PIXEL_SPIKEMARK_POSES : ApplyAudienceOffset(BLUE_PURPLE_PIXEL_SPIKEMARK_POSES);
        }
    }

    private Pose2d[] ApplyAudienceOffset(Pose2d[] input) {
        Pose2d[] returnArray = new Pose2d[input.length];
        for (int i = 0; i < input.length; i++) {
            Pose2d pose = input[i];
            returnArray[i] = new Pose2d(pose.getX() - AUDIENCE_OFFSET_AMOUNT, pose.getY(), pose.getHeading());
        }
        return returnArray;
    }

    public void Score() { new AutoScoreCommand(deposit, lift).schedule(); }

    public void timeout(double time) {
        ElapsedTime wait = new ElapsedTime();
        wait.reset();
        while (wait.seconds() < time) { short x; }
    }

    public void Cycle() {
        drive.followTrajectorySequence(backboardToStack);

        intake.spin();

        timeout(0.5);

        drive.followTrajectorySequence(intakeProne);

        timeout(1);

        new TransferCommand(deposit, lift, intake).schedule();

        timeout(0.5);

        intake.reverseSpin();

        drive.followTrajectorySequence(stackToBackboard);

        Score();
    }
}