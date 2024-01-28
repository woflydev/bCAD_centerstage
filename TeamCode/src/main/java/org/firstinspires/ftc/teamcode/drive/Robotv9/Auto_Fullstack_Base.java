package org.firstinspires.ftc.teamcode.drive.Robotv9;

import static org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.RobotAutoConstants.*;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.AAutoState.*;
import org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.commands.OpModeTemplate;
import org.firstinspires.ftc.teamcode.drive.commands.autoCommands.AutoScoreCommand;
import org.firstinspires.ftc.teamcode.drive.commands.teleopCommands.TransferAndStandbyCommand;
import org.firstinspires.ftc.teamcode.drive.rr.bCADMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.drive.vision2.VisionPropPipeline;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class Auto_Fullstack_Base extends OpModeTemplate {
    private RootAutoState autoState = RootAutoState.BA_PLAY;
    private bCADMecanumDrive drive;
    public VisionPropPipeline.Randomization randomization;
    public RobotAlliance alliance;
    public RobotStartingPosition startingPosition;
    public RobotParkingLocation parkingLocation;
    public RobotTaskFinishBehaviour taskFinishBehaviour;
    public boolean autoAlreadyRun;
    public boolean taskFinishBehaviourSelected = false;
    public int cycleCounter = 0;
    public int allianceIndex;
    public int dir;
    public Pose2d[] wYellowBackdropAlign;
    public Pose2d[] wPurpleSpikemarkAlign;
    public Pose2d[] wPurpleAvoidanceCheckpoints;
    public static Pose2d START_POSE = new Pose2d();
    public static Pose2d PARKING_POSE = new Pose2d();
    public Point r1;
    public Point r2;
    public Point r3;

    private final ElapsedTime autoTimer = new ElapsedTime();

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

        allianceIndex = this.alliance == RobotAlliance.RED ? 0 : 1;
    }

    public void initialize() {
        autoAlreadyRun = false;
        InitBlock();
        EnsureAttachmentNormalization();
        DetermineStartFinishPoses();

        taskFinishBehaviourSelected = false;
        wYellowBackdropAlign = SortPoseBasedOnAlliance(RED_YELLOW_PIXEL_BACKDROP_POSES, BLUE_YELLOW_PIXEL_BACKDROP_POSES);
        wPurpleAvoidanceCheckpoints = SortPoseBasedOnAlliance(RED_PURPLE_CHECKPOINTS, BLUE_PURPLE_CHECKPOINTS);
        wPurpleSpikemarkAlign = SortPurpleSpikemarkAlign();

        drive = new bCADMecanumDrive(hardwareMap, telemetry);
        drive.setPoseEstimate(START_POSE);

        SelectTaskFinishBehaviour();
        VisionPropDetection();
    }

    public void run() {
        if (!autoAlreadyRun) {
            autoAlreadyRun = true;

            while (!isStopRequested() && opModeIsActive()) {
                StatusTelemetry();
                HandlePurple();
                HandleYellow();
                HandleCycle();
                HandleFinish();

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
                            drive.followTrajectoryAsync(GenerateTraj(wPurpleSpikemarkAlign[0], false));
                            break;
                        case LOCATION_2:
                            drive.followTrajectoryAsync(GenerateTraj(wPurpleSpikemarkAlign[1], false));
                            break;
                        case LOCATION_3:
                            drive.followTrajectoryAsync(GenerateTraj(wPurpleSpikemarkAlign[2], false));
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
        } else {
            switch (autoState) {
                case BA_PLAY:
                    switch (randomization) {
                        case LOCATION_1:
                            drive.followTrajectoryAsync(GenerateTraj(wPurpleSpikemarkAlign[0], false));
                            break;
                        case LOCATION_2:
                            drive.followTrajectoryAsync(GenerateTraj(wPurpleSpikemarkAlign[1], false));
                            break;
                        case LOCATION_3:
                            drive.followTrajectoryAsync(GenerateTraj(wPurpleSpikemarkAlign[2], false));
                            break;
                    }
                    autoState = RootAutoState.BA_DEPOSIT_PURPLE;
                    break;
                case BA_DEPOSIT_PURPLE:
                    if (!drive.isBusy()) {
                        TrajectorySequence purpleAvoidanceTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(wPurpleAvoidanceCheckpoints[0])
                                .waitSeconds(0.001)
                                .splineToLinearHeading(wPurpleAvoidanceCheckpoints[1], wPurpleAvoidanceCheckpoints[1].getHeading())
                                .build();
                        drive.followTrajectorySequenceAsync(purpleAvoidanceTrajectory);
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
                            drive.followTrajectoryAsync(GenerateTraj(wYellowBackdropAlign[0], false));
                            break;
                        case LOCATION_2:
                            drive.followTrajectoryAsync(GenerateTraj(wYellowBackdropAlign[1], false));
                            break;
                        case LOCATION_3:
                            drive.followTrajectoryAsync(GenerateTraj(wYellowBackdropAlign[2], false));
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
                        autoState = (cycleCounter > 0) ? RootAutoState.BA_MOVING_TO_CYCLE : RootAutoState.BA_MOVING_TO_PARKING;
                    }
                    break;
            }
        }
    }

    private void HandleCycle() {
        if (startingPosition == RobotStartingPosition.BACKDROP) {
            switch (autoState) {
                case BA_MOVING_TO_CYCLE:
                    if (!drive.isBusy()) {
                        TrajectorySequence cycleTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .splineToConstantHeading(STAGE_DOOR_POSES[allianceIndex].vec(), STAGE_DOOR_POSES[allianceIndex].getHeading())
                                .waitSeconds(0.001)
                                .splineToLinearHeading(CYCLING_STACK_KNOCK_POSES[allianceIndex], CYCLING_STACK_KNOCK_POSES[allianceIndex].getHeading())
                                .waitSeconds(0.001)
                                .lineToLinearHeading(CYCLING_STACK_INNER_POSES[allianceIndex])
                                .build();
                        drive.followTrajectorySequence(cycleTrajectory); // note: blocking
                        ExecuteRotation(180, true);
                        autoState = RootAutoState.BA_INTAKE_PIXELS_FROM_STACK;
                    }
                    break;
                case BA_INTAKE_PIXELS_FROM_STACK:
                    if (!drive.isBusy()) {
                        intake.spinAndCloseFlap();
                        drive.followTrajectory(CalcKinematics(3, CAUTION_SPEED));
                        timeout(0.5);

                        TrajectorySequence toBackdropTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToConstantHeading(CYCLE_RETURN_POSES[allianceIndex].vec())
                                .waitSeconds(0.001)
                                .lineToConstantHeading(BACKDROP_CENTER_POSES[allianceIndex].vec())
                                .build();

                        drive.followTrajectorySequenceAsync(toBackdropTrajectory);

                        autoTimer.reset();
                        autoState = RootAutoState.BA_MOVING_BACK_FROM_CYCLE;
                    }
                    break;
                case BA_MOVING_BACK_FROM_CYCLE:
                    if (autoTimer.seconds() >= 2) {
                        intake.reverseSpin();
                        new TransferAndStandbyCommand(deposit, lift, intake).schedule();
                        autoState = RootAutoState.BA_DEPOSIT_WHITE;
                    }
                    break;
                case BA_DEPOSIT_WHITE:
                    if (!drive.isBusy()) {
                        intake.stop();
                        ExecuteRotation(180, false);
                        Score();

                        // note: depending on the number of cycles to do, moves to parking / initiates another cycle.
                        cycleCounter--;
                        autoState = (cycleCounter > 0) ? RootAutoState.BA_MOVING_TO_CYCLE : RootAutoState.BA_MOVING_TO_PARKING;
                    }
            }
        }
    }

    private void HandleFinish() {
        switch (autoState) {
            case BA_MOVING_TO_PARKING:
                if (!drive.isBusy()) {
                    Trajectory parking = drive
                            .trajectoryBuilder(drive.getPoseEstimate())
                            .splineToLinearHeading(PARKING_POSE, PARKING_POSE.getHeading()).build();

                    drive.followTrajectory(parking);
                    ExecuteRotation(alliance == RobotAlliance.RED ? 90 : 270, true); // note: ensure field centric heading on finish
                    autoState = RootAutoState.BA_PARKED;
                }
                break;
            case BA_PARKED:
                // todo: other custom logic if parked
                break;
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

    private void SelectTaskFinishBehaviour() {
        telemetry.addData("TEAM_PROP_LOCATION", randomization);
        telemetry.addData("SELECTED_ALLIANCE", alliance);
        telemetry.addLine("====================================");
        telemetry.addLine("PLEASE SELECT TASK FINISH BEHAVIOUR!");
        telemetry.addLine("X / SQUARE for DO_NOT_CYCLE.");
        telemetry.addLine("Y / TRIANGLE for CYCLE_ONCE.");
        telemetry.addLine("B / CIRCLE for CYCLE_TWICE.");
        telemetry.update();

        while (!isStopRequested() && opModeInInit() && !taskFinishBehaviourSelected) {
            if (gamepad1.x) {
                taskFinishBehaviour = RobotTaskFinishBehaviour.DO_NOT_CYCLE;
                cycleCounter = 0;
                taskFinishBehaviourSelected = true;
            } else if (gamepad1.y) {
                taskFinishBehaviour = RobotTaskFinishBehaviour.CYCLE;
                cycleCounter = 1;
                taskFinishBehaviourSelected = true;
            } else if (gamepad1.b) {
                taskFinishBehaviour = RobotTaskFinishBehaviour.CYCLE_TWICE_NONONONONO;
                cycleCounter = 2;
                taskFinishBehaviourSelected = true;
            }
        }

        // note: in the event that program is started before selection, defaults to DO_NOT_CYCLE.
        if (!taskFinishBehaviourSelected) taskFinishBehaviour = RobotTaskFinishBehaviour.DO_NOT_CYCLE;

        telemetry.addLine("INITIALIZATION COMPLETE! TASK FINISH BEHAVIOUR SELECTED!");
        telemetry.addData("Selected Behaviour", taskFinishBehaviour);
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
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, RobotConstants.FRONT_CAMERA), cameraMonitorViewId);
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

    private Pose2d[] SortPoseBasedOnAlliance(Pose2d[] posesForRed, Pose2d[] posesForBlue) {
        return alliance == RobotAlliance.RED ? posesForRed : posesForBlue;
    }

    private Pose2d[] SortPurpleSpikemarkAlign() {
        if (alliance == RobotAlliance.RED) {
            return startingPosition == RobotStartingPosition.BACKDROP ? RED_PURPLE_PIXEL_SPIKEMARK_BACKDROP : RED_PURPLE_SPIKEMARK_AUDIENCE;
        } else {
            return startingPosition == RobotStartingPosition.BACKDROP ? BLUE_PURPLE_SPIKEMARK_BACKDROP : BLUE_PURPLE_SPIKEMARK_AUDIENCE;
        }
    }

    private Pose2d[] ApplyAudienceOffset(Pose2d[] input) {
        Pose2d[] returnArray = new Pose2d[input.length];
        for (int i = 0; i < input.length; i++) {
            Pose2d pose = input[i];
            returnArray[i] = new Pose2d(pose.getX() - AUDIENCE_OFFSET_AMOUNT, pose.getY(), Math.toRadians(AUDIENCE_HEADING_VARIATION));
        }
        return returnArray;
    }

    private void DetermineStartFinishPoses() {
        START_POSE =
                this.startingPosition == RobotStartingPosition.BACKDROP
                    ? (this.alliance == RobotAlliance.RED
                        ? RED_STARTING_POSES[0]
                        : BLUE_STARTING_POSES[0])
                    : (this.alliance == RobotAlliance.RED
                        ? RED_STARTING_POSES[1]
                        : BLUE_STARTING_POSES[1]);

        PARKING_POSE =
                this.parkingLocation == RobotParkingLocation.INNER
                        ? (this.alliance == RobotAlliance.RED
                            ? RED_PARKING_POSES[0]
                            : BLUE_PARKING_POSES[0])
                        : (this.alliance == RobotAlliance.RED
                            ? RED_PARKING_POSES[1]
                            : BLUE_PARKING_POSES[1]);
    }

    private void EnsureAttachmentNormalization() {
        deposit.elbow.turnToAngle(180);
        timeout(2);
        deposit.claw.turnToAngle(110);
        timeout(2);
        deposit.elbow.turnToAngle(270);
    }

    private void Score() { new AutoScoreCommand(deposit, lift).schedule(); }

    public void timeout(double time) {
        ElapsedTime wait = new ElapsedTime();
        wait.reset();
        while (wait.seconds() < time) { short x; }
    }
}