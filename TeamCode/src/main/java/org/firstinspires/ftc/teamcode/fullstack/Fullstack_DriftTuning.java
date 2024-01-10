package org.firstinspires.ftc.teamcode.fullstack;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.drive.Fullstack_Base;

@TeleOp(name="Manual Drive Tuning", group="~tuning")
public class Fullstack_DriftTuning extends Fullstack_Base {

    // note: textbook mecanum drive, thanks y'all
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
