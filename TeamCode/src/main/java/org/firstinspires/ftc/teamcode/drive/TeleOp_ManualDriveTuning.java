package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Robotv9.TeleOp_Fullstack_Base;

@TeleOp(name="Manual Drive Tuning", group="~tuning")
public class TeleOp_ManualDriveTuning extends TeleOp_Fullstack_Base {
    @Override
    public void loop() {
        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x * 1.1;
        double rx = -gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontL = (y + x + rx) / denominator;
        double backL = (y - x + rx) / denominator;
        double frontR = (y - x - rx) / denominator;
        double backR = (y + x - rx) / denominator;

        frontLM.setPower(frontL);
        backLM.setPower(backL);
        frontRM.setPower(frontR);
        backRM.setPower(backR);
    }
}
