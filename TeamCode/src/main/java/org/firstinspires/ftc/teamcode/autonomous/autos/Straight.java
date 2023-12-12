//package org.firstinspires.ftc.teamcode.autonomous;
//
//import static java.lang.Thread.sleep;
//
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.teamcode.autonomous.aprilTagDetection.AprilTagCameraDetection;
//import org.firstinspires.ftc.teamcode.hardware.Deposit;
//import org.firstinspires.ftc.teamcode.hardware.Drivebase;
//import org.firstinspires.ftc.teamcode.hardware.DroneLauncher;
//import org.firstinspires.ftc.teamcode.hardware.Hang;
//import org.firstinspires.ftc.teamcode.hardware.Intake;
//import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
//import org.openftc.easyopencv.OpenCvCamera;
//
//@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Straight Drive Auto")
//
//public class Straight extends OpMode {
//
//    // Create Prop Location-Related Variables
//    enum Locations {LEFT, FRONT, MIDDLE}
//    private Locations propLocation;
//
//    // Create AprilTag-Related Variables
//    AprilTagCameraDetection aprilTagDetector;
//    private OpenCvCamera camera;
//    private String aprilTagLocation;
//    private int[] tagsToSearchFor = new int[]{1, 2, 3};
//    public ElapsedTime Timer = new ElapsedTime();
//
//    Drivebase driveBase;
//    IMU imu;
//    GamepadEx driveOp;
//    GamepadEx toolOp;
//    SampleMecanumDrive drive;
//    TrajectorySequence autoJustParkMiddle;
//
//    // Declaring Commands
//    private Deposit deposit;
//    private Intake intake;
//    private Hang hang;
//    private DroneLauncher shooter;
//
//    public void init() {
//        driveBase = new Drivebase(hardwareMap);
//        // Initialise the imuGyro with the correct orientation
//        imu = hardwareMap.get(IMU.class, "imu");
//        imu.initialize(new IMU.Parameters(
//                new RevHubOrientationOnRobot(
//                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
//                        RevHubOrientationOnRobot.UsbFacingDirection.UP
//                )
//        ));
//        imu.resetYaw();
//        driveOp = new GamepadEx(gamepad1);
//        toolOp = new GamepadEx(gamepad2);
//        deposit = new Deposit(hardwareMap);
//        intake = new Intake(hardwareMap, driveOp);
//        hang = new Hang(hardwareMap);
//        shooter = new DroneLauncher(hardwareMap);
//        telemetry.addLine("initialization complete");
//        telemetry.update();
//
//        driveBase.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        driveBase.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        driveBase.rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        driveBase.rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        driveBase.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        driveBase.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        driveBase.rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        driveBase.rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//    }
//
//    public void start() {
//        int tileLength = 61;
//
//        deposit.place();
//
////        driveBase.drive(100, 0, 0.5, telemetry, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
//        driveBase.forward((tileLength*2+20), 0, 1, telemetry, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
//
//        Timer.reset();
//
//    }
//
//    public void loop() {
//        telemetry.addData("Distance Travelled: ", driveBase.getDistanceTravelled());
//
//
//        if(Timer.seconds() < 5) {
//            intake.Rspin();
//        } else {
//            intake.intakeSpinner.motor.setPower(0);
//        }
//
//        if(Timer.seconds() > 5) {
//            driveBase.rotate(-90, telemetry, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), 1);
//        }
//
//        telemetry.update();
//
//
//    }
//
//}
