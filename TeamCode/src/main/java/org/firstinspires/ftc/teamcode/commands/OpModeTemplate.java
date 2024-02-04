package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.AAutoState.*;
import org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotConstants;
import org.firstinspires.ftc.teamcode.hardware.DepositSubsystem;
import org.firstinspires.ftc.teamcode.hardware.DriveSubsystem;
import org.firstinspires.ftc.teamcode.hardware.PlaneLauncherSubsystem;
import org.firstinspires.ftc.teamcode.hardware.HangSubsystem;
import org.firstinspires.ftc.teamcode.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.LiftSubsystem;
import org.firstinspires.ftc.teamcode.rr.bCADMecanumDrive;

abstract public class OpModeTemplate extends CommandOpMode {
    protected RootAutoState autoState = RootAutoState.BA_PLAY;
    protected DriveSubsystem drivebase;
    protected DepositSubsystem deposit;
    protected IntakeSubsystem intake;
    protected HangSubsystem hang;
    protected PlaneLauncherSubsystem shooter;
    protected LiftSubsystem lift;

    protected bCADMecanumDrive drive;

    protected GamepadEx gamepad1Ex;
    protected GamepadEx gamepad2Ex;

    protected IMU imu;

    protected void InitBlock() {
        imu = hardwareMap.get(IMU.class, RobotConstants.HUB_IMU);
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));
        imu.resetYaw();

        gamepad1Ex = new GamepadEx(gamepad1);
        gamepad2Ex = new GamepadEx(gamepad2);

        drivebase = new DriveSubsystem(hardwareMap);
        deposit = new DepositSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap, telemetry, gamepad1, gamepad2);
        hang = new HangSubsystem(hardwareMap, gamepad2, telemetry);
        shooter = new PlaneLauncherSubsystem(hardwareMap);
        lift = new LiftSubsystem(hardwareMap, telemetry, gamepad2, deposit, lift);

        telemetry.addLine("initialization complete");
        telemetry.update();

        register(intake, drivebase, hang, shooter, deposit);
    }

    protected void RumbleGamepad() {
        gamepad1.rumble(500);
        gamepad2.rumble(500);
    }
}