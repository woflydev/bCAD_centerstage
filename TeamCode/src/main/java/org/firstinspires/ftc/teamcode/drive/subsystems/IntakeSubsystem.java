package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.Timing.Timer;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.concurrent.TimeUnit;

public class IntakeSubsystem extends SubsystemBase {
    Telemetry telemetry;
    public final Motor intakeSpinner;
    boolean driversAlerted;
    private Timer timer = new Timer(5, TimeUnit.SECONDS);
    public ServoEx IntakeCover;
    public ElapsedTime coverTimer = new ElapsedTime();
    ColorRangeSensor is;

    Gamepad gamepad1;
    Gamepad gamepad2;

    public IntakeSubsystem(HardwareMap hardwareMap, GamepadEx toolop, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        intakeSpinner = new Motor(hardwareMap, "Spinner");
        intakeSpinner.setRunMode(Motor.RunMode.RawPower);
        intakeSpinner.setInverted(false);

        //is = hardwareMap.get(ColorRangeSensor.class, "is");

        IntakeCover = new SimpleServo(hardwareMap, "I", 0, 260, AngleUnit.DEGREES);

        IntakeCover.turnToAngle(200);
        driversAlerted = false;

    }

    @Override
    public void periodic() {
//        telemetry.addData("Color Sensor Distance: ", is.getDistance(DistanceUnit.MM));
//        int threshold = 30;
//
//        if(is.getDistance(DistanceUnit.MM) < threshold && !driversAlerted) {
//            gamepad1.rumble(800);
//            gamepad2.rumble(800);
//            driversAlerted = true;
//        } else if (is.getDistance(DistanceUnit.MM) > threshold) {
//            driversAlerted = false;
//        }
    }

    public void spin() {
        intakeSpinner.set(-0.8);
        IntakeCover.turnToAngle(200);
    }

    public void Rspin() {
        intakeSpinner.set(0.8);
    }

    public void stop() {
        intakeSpinner.set(0);
    }


    public void manualCoverControl(double angle, Telemetry telemetry) {
        int scaling = 8;

        IntakeCover.rotateByAngle(angle  * scaling);

        telemetry.addData("Intake Cover: ", IntakeCover.getAngle());
    }

    public void openCover() {IntakeCover.turnToAngle(114);}
    public void closeCover() {IntakeCover.turnToAngle(200);}


//    @Override
//    public void periodic() {
//        if (intakeCurrentlySpinning) {
//            intakeSpinner.set(0.7);
//        } else {
//            intakeSpinner.set(0);
//        }
//
//    }
}
