package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.AutoState;
import org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.AutoState.*;
import org.firstinspires.ftc.teamcode.drive.hardware.DepositSubsystem;
import org.firstinspires.ftc.teamcode.drive.hardware.DriveSubsystem;
import org.firstinspires.ftc.teamcode.drive.hardware.DroneLauncherSubsystem;
import org.firstinspires.ftc.teamcode.drive.hardware.HangSubsystem;
import org.firstinspires.ftc.teamcode.drive.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.drive.hardware.LiftSubsystem;

abstract public class OpModeTemplate extends CommandOpMode {
    protected RootAutoState autoState = RootAutoState.BA_PLAY;
    protected DriveSubsystem drivebase;
    protected DepositSubsystem deposit;
    protected IntakeSubsystem intake;
    protected HangSubsystem hang;
    protected DroneLauncherSubsystem shooter;
    protected LiftSubsystem lift;

    protected GamepadEx gamepad1Ex;
    protected GamepadEx gamepad2Ex;

    protected IMU imu;

    protected void InitBlock() {
        imu = hardwareMap.get(IMU.class, "imu");
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
        shooter = new DroneLauncherSubsystem(hardwareMap);
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