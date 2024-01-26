package org.firstinspires.ftc.teamcode.drive;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.Robotv9.TeleOp_Fullstack_Base;
import org.firstinspires.ftc.teamcode.drive.commands.HomeCommand;
import org.firstinspires.ftc.teamcode.drive.commands.LeftTriggerReader;
import org.firstinspires.ftc.teamcode.drive.commands.OpModeTemplate;
import org.firstinspires.ftc.teamcode.drive.commands.RightTriggerReader;
import org.firstinspires.ftc.teamcode.drive.commands.ScoreCommand;
import org.firstinspires.ftc.teamcode.drive.commands.TransferCommand;

@TeleOp(name="TeleOp RC1", group="!RC")
public class TeleOp_ReleaseCandidate1 extends TeleOp_Fullstack_Base {
    @Override
    public void MainLoop() {
        drivebase.userControlledDrive(gamepad1, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        if(!deposit.currentlyPlacing && !deposit.currentlyHoming && !deposit.currentlyTransfering) {
            lift.run(toolOp.getLeftY());
        }

        hang.run(gamepad2);
        deposit.manualV4BControl(toolOp.getRightY(), telemetry);

        telemetry.addData("Wrist Position", deposit.Wrist.getAngle());
        telemetry.addData("Slide position", lift.leftMotor.motor.getCurrentPosition());
        telemetry.update();
    }
}