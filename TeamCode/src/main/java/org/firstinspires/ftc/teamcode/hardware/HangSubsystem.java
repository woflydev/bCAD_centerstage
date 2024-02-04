package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HangSubsystem extends SubsystemBase {
    Telemetry telemetry;

    public MotorEx Hang;
    public CRServo HangPusher;
    Gamepad gamepad2;
    float targetPos;

    public HangSubsystem(HardwareMap hardwareMap, Gamepad gamepad2, Telemetry telemetry) {
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;
        // Assign variables here with parameters
        Hang = new MotorEx(hardwareMap, "hang");
        Hang.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        HangPusher = hardwareMap.crservo.get("HangPusher");

        targetPos = 0;
    }

    public void run(Gamepad gamepad2) {
        int scaling = 50;

        double power = Math.abs(gamepad2.touchpad_finger_1_y);

        telemetry.addData("Hang power", power);
        telemetry.addData("Hang target", targetPos);
        telemetry.addData("Hang current", Hang.motor.getCurrentPosition());

        if(gamepad2.touchpad_finger_1) {
            targetPos = targetPos + (gamepad2.touchpad_finger_1_y * scaling);

            if(gamepad2.touchpad_finger_1_y > 0.1) {
                Hang.motor.setTargetPosition((int) targetPos);
                Hang.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                Hang.motor.setPower(power);
                HangPusher.setPower(-1);

            } else if (gamepad2.touchpad_finger_1_y < -0.1) {
                Hang.motor.setTargetPosition((int) targetPos);
                Hang.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                Hang.motor.setPower(power);
                HangPusher.setPower(power/3);
            }
        } else {
            Hang.motor.setPower(0);
            HangPusher.setPower(0);
        }
    }
}
