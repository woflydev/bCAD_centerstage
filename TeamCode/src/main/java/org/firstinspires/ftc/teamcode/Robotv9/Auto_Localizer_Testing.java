package org.firstinspires.ftc.teamcode.Robotv9;

import static org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotAutoConstants.AUDIENCE_HEADING_VARIATION;
import static org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotAutoConstants.AUDIENCE_OFFSET_AMOUNT;
import static org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotAutoConstants.BLUE_CYCLE_CHECKPOINTS;
import static org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotAutoConstants.BLUE_PARKING_POSES;
import static org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotAutoConstants.BLUE_PURPLE_CHECKPOINTS;
import static org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotAutoConstants.BLUE_PURPLE_SPIKEMARK_AUDIENCE;
import static org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotAutoConstants.BLUE_PURPLE_SPIKEMARK_BACKDROP;
import static org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotAutoConstants.BLUE_STARTING_POSES;
import static org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotAutoConstants.BLUE_YELLOW_PIXEL_BACKDROP_POSES;
import static org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotAutoConstants.BONK_X_TOLERANCE;
import static org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotAutoConstants.BONK_Y_TOLERANCE;
import static org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotAutoConstants.RED_CYCLE_CHECKPOINTS;
import static org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotAutoConstants.RED_PARKING_POSES;
import static org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotAutoConstants.RED_PURPLE_CHECKPOINTS;
import static org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotAutoConstants.RED_PURPLE_PIXEL_SPIKEMARK_BACKDROP;
import static org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotAutoConstants.RED_PURPLE_SPIKEMARK_AUDIENCE;
import static org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotAutoConstants.RED_STARTING_POSES;
import static org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotAutoConstants.RED_YELLOW_PIXEL_BACKDROP_POSES;
import static org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotConstants.CLAW_AUTO_CLOSE;
import static org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotConstants.ELBOW_GRABBED_STANDBY;
import static org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotConstants.FRONT_CAMERA;
import static org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotConstants.JUNCTION_AUTO_WHITE;
import static org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotConstants.JUNCTION_AUTO_YELLOW;
import static org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotConstants.SPIN_HOME;
import static org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotConstants.WRIST_HOME;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.AAutoState.RobotAlliance;
import org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.AAutoState.RobotParkingLocation;
import org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.AAutoState.RobotStartingPosition;
import org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.AAutoState.RobotTaskFinishBehaviour;
import org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.ASubsystemState.Outtake;
import org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotConstants;
import org.firstinspires.ftc.teamcode.commands.OpModeTemplate;
import org.firstinspires.ftc.teamcode.commands.autoCommands.deposit.DepositPurpleAtSpikemark;
import org.firstinspires.ftc.teamcode.commands.autoCommands.deposit.EnsureDepositAutoCommand;
import org.firstinspires.ftc.teamcode.commands.autoCommands.deposit.HomeAutoCommand;
import org.firstinspires.ftc.teamcode.commands.autoCommands.localizer.RelocalizeAtBackdrop;
import org.firstinspires.ftc.teamcode.commands.autoCommands.move.MoveToBackdropWhite;
import org.firstinspires.ftc.teamcode.commands.autoCommands.move.MoveToBackdropYellow;
import org.firstinspires.ftc.teamcode.commands.autoCommands.move.MoveToParking;
import org.firstinspires.ftc.teamcode.commands.autoCommands.move.MoveToSpikemark;
import org.firstinspires.ftc.teamcode.commands.autoCommands.move.MoveToSpikemarkAvoidance;
import org.firstinspires.ftc.teamcode.commands.autoCommands.move.MoveToStacks;
import org.firstinspires.ftc.teamcode.commands.autoCommands.pickup.PickupFromStacks;
import org.firstinspires.ftc.teamcode.commands.autoCommands.pickup.RaiseAndPrimeAutoCommand;
import org.firstinspires.ftc.teamcode.commands.autoCommands.pickup.TransferAndStandbyAutoCommand;
import org.firstinspires.ftc.teamcode.rr.bCADMecanumDrive;
import org.firstinspires.ftc.teamcode.vision2.VisionPropPipeline;
import org.firstinspires.ftc.teamcode.vision2.VisualLoc;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.HashMap;

public class Auto_Localizer_Testing extends OpModeTemplate {
    private bCADMecanumDrive drive;
    public VisionPropPipeline.Randomization randomization;
    public RobotAlliance alliance;
    public RobotStartingPosition startingPosition;
    public RobotParkingLocation parkingLocation;
    public RobotTaskFinishBehaviour taskFinishBehaviour;
    public boolean autoAlreadyRun;
    public boolean taskFinishBehaviourSelected = false;
    public int allianceIndex;
    public int dir;
    public Pose2d[] wYellowBackdropAlign;
    public Pose2d[] wPurpleSpikemarkAlign;
    public Pose2d[] wPurpleAvoidanceCheckpoints;
    public Pose2d[] wCycleCheckpoints;
    public static Pose2d START_POSE = new Pose2d();
    public static Pose2d PARKING_POSE = new Pose2d();
    public Point r1;
    public Point r2;
    public Point r3;

    private VisualLoc visualLoc;

    private final ElapsedTime autoTimer = new ElapsedTime();

    public Auto_Localizer_Testing(RobotAlliance alliance,
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
        SelectTaskFinishBehaviour();
        EnsureAttachmentNormalization();
        DetermineStartFinishPoses();

        taskFinishBehaviourSelected = false;
        wYellowBackdropAlign = SortPoseBasedOnAlliance(RED_YELLOW_PIXEL_BACKDROP_POSES, BLUE_YELLOW_PIXEL_BACKDROP_POSES);
        wPurpleAvoidanceCheckpoints = SortPoseBasedOnAlliance(RED_PURPLE_CHECKPOINTS, BLUE_PURPLE_CHECKPOINTS);
        wCycleCheckpoints = SortPoseBasedOnAlliance(RED_CYCLE_CHECKPOINTS, BLUE_CYCLE_CHECKPOINTS);
        wPurpleSpikemarkAlign = SortPurpleSpikemarkAlign();

        visualLoc = new VisualLoc(hardwareMap, drive, START_POSE, FRONT_CAMERA, telemetry);
        drive = new bCADMecanumDrive(hardwareMap);
        drive.setPoseEstimate(START_POSE);

        randomization = VisionPropPipeline.Randomization.LOCATION_2;

        VisionPropDetection();
    }

    @Override
    public void run() {
        if (!autoAlreadyRun) {
            visualLoc.Make();
            BuildAutoSequence().schedule();
            autoTimer.reset();
            while (opModeIsActive() && !isStopRequested()) {
                super.run();
                StatusTelemetry();
                drive.CheckForBonk(); // todo: remove if still being special
            }
        }
    }

    private SequentialCommandGroup BuildAutoSequence() {
        return new SequentialCommandGroup(
                new MoveToSpikemark(drive, randomization, wPurpleSpikemarkAlign),
                new ConditionalCommand(
                        new DepositPurpleAtSpikemark(drive),
                        new MoveToSpikemarkAvoidance(drive, wPurpleAvoidanceCheckpoints),
                        () -> startingPosition == RobotStartingPosition.BACKDROP
                ),

                new MoveToBackdropYellow(drive, randomization, wYellowBackdropAlign),
                //new RelocalizeAtBackdrop(drive, visualLoc, telemetry),
                new WaitCommand(10000)
        );
    }

    // note: ------------------------------UTIL AND SYSTEM-------------------------------------------------------

    private void StatusTelemetry() {
        telemetry.addData("Bonk Or Not", drive.bonked);
        telemetry.addData("Pose Error", drive.getLastError());
        telemetry.addData("Pose Difference", drive.diffPosition);
        telemetry.addData("Visual Localizer Pose", visualLoc.WhereTheHellAmI());
        telemetry.addData("Intake Current Draw", intake.intakeM.motorEx.getCurrent(CurrentUnit.AMPS));

        telemetry.addLine("---------");
        telemetry.addData("Autonomous Clock", autoTimer.seconds());
        telemetry.addData("Robot Heading", Math.toDegrees(drive.getPoseEstimate().getHeading()));
        telemetry.addData("Target Location",
                randomization == VisionPropPipeline.Randomization.LOCATION_1 ? "LEFT (LOC_1)" :
                randomization == VisionPropPipeline.Randomization.LOCATION_2 ? "MIDDLE (LOC_2)" :
                randomization == VisionPropPipeline.Randomization.LOCATION_3 ? "RIGHT (LOC_3)" : "NONE"
        );

        telemetry.addLine("---------");
        telemetry.addData("Robot X", drive.getPoseEstimate().getX());
        telemetry.addData("Robot Y", drive.getPoseEstimate().getY());
        telemetry.addData("Robot Heading", Math.toDegrees(drive.getPoseEstimate().getHeading()));
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
                taskFinishBehaviourSelected = true;
            } else if (gamepad1.y) {
                taskFinishBehaviour = RobotTaskFinishBehaviour.CYCLE;
                taskFinishBehaviourSelected = true;
            } else if (gamepad1.b) {
                taskFinishBehaviour = RobotTaskFinishBehaviour.CYCLE_TWICE;
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
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, FRONT_CAMERA), cameraMonitorViewId);
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
        timeout(3);
        deposit.claw.turnToAngle(CLAW_AUTO_CLOSE);
        deposit.elbow.turnToAngle(ELBOW_GRABBED_STANDBY);
        deposit.wrist.turnToAngle(WRIST_HOME);
        deposit.spin.turnToAngle(SPIN_HOME);
        deposit.outtakeState = Outtake.GRABBED_AND_READY;
    }

    private void timeout(double time) {
        ElapsedTime wait = new ElapsedTime();
        wait.reset();
        while (wait.seconds() <= time) { short x; }
    }
}