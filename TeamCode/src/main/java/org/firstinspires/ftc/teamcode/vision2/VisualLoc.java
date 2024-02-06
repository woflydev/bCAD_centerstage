package org.firstinspires.ftc.teamcode.vision2;

import static java.lang.Thread.sleep;

import android.annotation.SuppressLint;
import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotConstants;
import org.firstinspires.ftc.teamcode.rr.bCADMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class VisualLoc implements Localizer {
    //fx, fy, cx, cy
    public static double[] NEW_CALIBRATION = {1805.11209646, 1805.11209646, 1020.05252149, 743.423990613};

    public Pose2d poseEstimate;
    public Pose2d poseVelocity;

    private ArrayList<Double> lastWheelPositions = new ArrayList<>();
    private Double lastExtHeading = Double.NaN;

    public HardwareMap hardwareMap;

    private static final int ACQUISITION_TIME = 10;
    private static final int SLEEP_TIME = 1;
    private static final float CORRECTION_FACTOR = 1;

    private int AVERAGE_LENGTH = 3;
    private boolean useExternalHeading = true;
    private ArrayList<Pose2d> previousPoses = new ArrayList<>();
    public List<AprilTagDetection> currentDetections;

    public boolean isBlind = false;
    public long blindTime = 0;

    private final String FRONT_CAMERA;
    private final ElapsedTime elapsedTime = new ElapsedTime();
    private final Telemetry t;

    private final bCADMecanumDrive drive;
    private AprilTagProcessor aprilTag;
    public VisionPortal visionPortal;

    // This assumes the april tag starts facing along the y-axis, may change later
    public static AprilTagMetadata[] tagArray = {
            new AprilTagMetadata(7, "Back 1", 0.127,
                    new VectorF((float) - RobotConstants.FIELD_LENGTH / 2, (float) -RobotConstants.WALL_TAG_X, (float) RobotConstants.CAMERA_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                    (float) Math.cos(RobotConstants.TAG_WALL_ANGLE / 2), 0, 0,
                    (float) Math.sin(RobotConstants.TAG_WALL_ANGLE / 2), ACQUISITION_TIME)),
            new AprilTagMetadata(9, "Back 2", 0.127,
                    new VectorF((float) - RobotConstants.FIELD_LENGTH / 2, (float) RobotConstants.WALL_TAG_X, (float) RobotConstants.CAMERA_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                    (float) Math.cos(RobotConstants.TAG_WALL_ANGLE / 2), 0, 0,
                    (float) Math.sin(RobotConstants.TAG_WALL_ANGLE / 2), ACQUISITION_TIME)
            ),
            new AprilTagMetadata(8, "Back 1a", 0.1,
                    new VectorF((float) - RobotConstants.FIELD_LENGTH / 2, (float)-RobotConstants.SMALL_WALL_TAG_X, (float) RobotConstants.CAMERA_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                    (float) Math.cos(RobotConstants.TAG_WALL_ANGLE / 2), 0, 0,
                    (float) Math.sin(RobotConstants.TAG_WALL_ANGLE / 2), ACQUISITION_TIME)
            ),
            new AprilTagMetadata(10, "Back 2a", 0.1,
                    new VectorF((float) - RobotConstants.FIELD_LENGTH / 2, (float)RobotConstants.SMALL_WALL_TAG_X, (float) RobotConstants.CAMERA_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                    (float) Math.cos(RobotConstants.TAG_WALL_ANGLE / 2), 0, 0,
                    (float) Math.sin(RobotConstants.TAG_WALL_ANGLE / 2), ACQUISITION_TIME)),
            new AprilTagMetadata(1, "Backdrop 1", 0.05,
                    new VectorF((float) RobotConstants.BACKDROP_DEPTH, (float) 1.003F, (float) RobotConstants.TAG_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                    (float) Math.cos(RobotConstants.BACKDROP_ANGLE / 2), 0, 0,
                    (float) Math.sin(RobotConstants.BACKDROP_ANGLE / 2), ACQUISITION_TIME)),
            new AprilTagMetadata(2, "Backdrop 2", 0.05,
                    new VectorF((float) RobotConstants.BACKDROP_DEPTH, 0.88F, (float) RobotConstants.TAG_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                    (float) Math.cos(RobotConstants.BACKDROP_ANGLE / 2), 0, 0,
                    (float) Math.sin(RobotConstants.BACKDROP_ANGLE / 2), ACQUISITION_TIME)),
            new AprilTagMetadata(3, "Backdrop 3", 0.05,
                    new VectorF((float) RobotConstants.BACKDROP_DEPTH, 0.74F, (float) RobotConstants.TAG_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                    (float) Math.cos(RobotConstants.BACKDROP_ANGLE / 2), 0, 0,
                    (float) Math.sin(RobotConstants.BACKDROP_ANGLE / 2), ACQUISITION_TIME)),
            new AprilTagMetadata(4, "Backdrop 4", 0.05,
                    new VectorF((float) RobotConstants.BACKDROP_DEPTH, -0.75F, (float) RobotConstants.TAG_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                    (float) Math.cos(RobotConstants.BACKDROP_ANGLE / 2), 0, 0,
                    (float) Math.sin(RobotConstants.BACKDROP_ANGLE / 2), ACQUISITION_TIME)),
            new AprilTagMetadata(5, "Backdrop 5", 0.05,
                    new VectorF((float) RobotConstants.BACKDROP_DEPTH, -0.9F, (float) RobotConstants.TAG_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                    (float) Math.cos(RobotConstants.BACKDROP_ANGLE / 2), 0, 0,
                    (float) Math.sin(RobotConstants.BACKDROP_ANGLE / 2), ACQUISITION_TIME)),
            new AprilTagMetadata(6, "Backdrop 6", 0.05,
                    new VectorF((float) RobotConstants.BACKDROP_DEPTH, -1.05F, (float) RobotConstants.TAG_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                    (float) Math.cos(RobotConstants.BACKDROP_ANGLE / 2), 0, 0,
                    (float) Math.sin(RobotConstants.BACKDROP_ANGLE / 2), ACQUISITION_TIME))
    };

    public VisualLoc(HardwareMap map, bCADMecanumDrive d, Pose2d currentPose, String frontCamera,  Telemetry t) {
        this.hardwareMap = map;
        this.poseEstimate = currentPose;
        this.poseVelocity = new Pose2d(0, 0, 0);
        this.t = t;
        this.FRONT_CAMERA = frontCamera;
        this.drive = d;
    }

    @NonNull
    public Pose2d getPoseEstimate() {
        return this.poseEstimate;
    }

    public void setPoseEstimate(Pose2d newPose) {
        this.poseEstimate = newPose;
    }

    public Pose2d getPoseVelocity() {
        return poseVelocity;
    }

    public void Make() {
        elapsedTime.reset();
        initPortal();
    }

    public Pose2d WhereTheHellAmI() {
        analyseDetections();
        return this.poseEstimate;
    }

    public void update() {
        // note: disabled for now to prevent auto update. instead, is called manually.
        /*int STARTUP_TIME = 200;
        if (elapsedTime.milliseconds() > STARTUP_TIME) {
            analyseDetections();
            Delay(SLEEP_TIME);
        }*/
    }

    public void stop() {
        analyseDetections(); // TODO: this is called here to stop while loops, hopefully
        visionPortal.close();
        t.addLine("Vision portal closed!");
    }

    public void Delay(double time) {
        try { sleep((long)time); } catch (Exception e) { System.out.println("interrupted"); }
    }

    public void initPortal() {
        AprilTagLibrary.Builder b = new AprilTagLibrary.Builder();
        for (AprilTagMetadata tag : tagArray) {
            b.addTag(tag);
        }

        AprilTagLibrary library = b.build();

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(library)
                .setOutputUnits(DistanceUnit.METER, AngleUnit.RADIANS)
                .setLensIntrinsics(NEW_CALIBRATION[0], NEW_CALIBRATION[1], NEW_CALIBRATION[2], NEW_CALIBRATION[3])
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, FRONT_CAMERA));
        builder.setCameraResolution(new Size(1280, 720));

        if (RobotConstants.USE_LIVE_VIEW) {
            builder.enableLiveView(true);
            builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
            builder.setAutoStopLiveView(false);
        } else {
            builder.enableLiveView(false);
        }

        builder.addProcessors(aprilTag);
        visionPortal = builder.build();
    }

    @SuppressLint("DefaultLocale")
    public void analyseDetections() {
        currentDetections = aprilTag.getDetections();

        double heading = 0;
        int notNullTags = 0;

        VectorF avgPos = new VectorF(0, 0, 0);

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                avgPos.add(vectorFromPose(detection, false));
                heading += yawFromPose(detection);
                notNullTags++;
            }
        }

        if (notNullTags > 0) {
            avgPos.multiply(1 / (float) notNullTags);
            heading /= notNullTags;
            Pose2d roughPose = new Pose2d(avgPos.get(0), avgPos.get(1), heading);

            Pose2d previousAvg = previousPoses.size() > 0 ? new Pose2d(0, 0, 0) : roughPose;
            for (Pose2d pose : previousPoses) {
                previousAvg = previousAvg.plus(pose.times(1 / (float) previousPoses.size()));
            }

            poseEstimate = roughPose.plus(previousAvg).times(0.5);
            previousPoses.add(poseEstimate);

            while (previousPoses.size() > AVERAGE_LENGTH) {
                previousPoses.remove(0);
            }

            poseVelocity = poseEstimate.minus(previousPoses.get(previousPoses.size() - 1)).div(SLEEP_TIME);
            isBlind = false;
        }
    }

    public VectorF vectorFromPose(AprilTagDetection detection, boolean normal) {
        AprilTagPoseFtc pose = detection.ftcPose;

        VectorF u = new VectorF(0, -1, 0);
        VectorF v = new VectorF(1, 0, 0);

        u = detection.metadata.fieldOrientation.applyToVector(u);
        v = detection.metadata.fieldOrientation.applyToVector(v);
        VectorF w = cross(u, v);

        VectorF newNormal = u;
        newNormal = rotationAboutAxis(pose.bearing - pose.yaw, w).applyToVector(newNormal);
        newNormal = rotationAboutAxis(pose.elevation, v).applyToVector(newNormal);
        if (normal) {
            return newNormal;
        } else {
            return newNormal.multiplied((float) pose.range * CORRECTION_FACTOR).added(detection.metadata.fieldPosition);
        }
    }

    // Assumes pitch and roll are negligible
    // Heading is clockwise
    public double yawFromPose(AprilTagDetection detection) {
        AprilTagPoseFtc pose = detection.ftcPose;
        return mod((float) Math.toDegrees(pose.bearing - pose.yaw - Math.acos(detection.metadata.fieldOrientation.w) * 2), 360);
    }

    public VectorF cross(VectorF a, VectorF b) {
        return new VectorF(a.get(1) * b.get(2) - a.get(2) * b.get(1),
                a.get(2) * b.get(0) - a.get(0) * b.get(2),
                a.get(0) * b.get(1) - a.get(1) * b.get(0));
    }

    public Quaternion rotationAboutAxis(double theta, VectorF axis) {
        VectorF normAxis = axis.multiplied(1 / axis.magnitude());
        return new Quaternion((float) Math.cos(theta / 2),
                (float) (Math.sin(theta / 2) * normAxis.get(0)),
                (float) (Math.sin(theta / 2) * normAxis.get(1)),
                (float) (Math.sin(theta / 2) * normAxis.get(2)),
                0);
    }

    // Return negatives as well, if only positive use Math.floor
    public float mod(float n, float m) {
        return (n - m * Math.round(n / m));
    }

    public ArrayList<Double> differences(ArrayList<Double> first, ArrayList<Double> second) {
        if (first.size() != second.size()) {
            throw new IllegalArgumentException();
        }

        ArrayList<Double> tmp = new ArrayList<>();
        for (int i = 0; i < first.size(); i++) {
            tmp.add(first.get(i) - second.get(i));
        }
        return tmp;
    }

    public VectorF vectorFromPose(AprilTagDetection detection) {
        AprilTagPoseFtc pose = detection.ftcPose;

        VectorF u = new VectorF(0, -1, 0);
        VectorF v = new VectorF(1, 0, 0);

        u = detection.metadata.fieldOrientation.applyToVector(u);
        v = detection.metadata.fieldOrientation.applyToVector(v);
        VectorF w = cross(u, v);

        VectorF newNormal = u;
        newNormal = rotationAboutAxis(pose.bearing - pose.yaw, w).applyToVector(newNormal);
        newNormal = rotationAboutAxis(pose.elevation, v).applyToVector(newNormal);

        return newNormal.multiplied((float) pose.range * CORRECTION_FACTOR).added(detection.metadata.fieldPosition);
    }
}
