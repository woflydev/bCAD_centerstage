package org.firstinspires.ftc.teamcode.drive.hardware;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeSubsystem extends SubsystemBase {
    Telemetry telemetry;
    public final Motor intakeM;
    public ServoEx servoFlap;

    // note: below vars are for rumble
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    private ColorRangeSensor is;
    private boolean driversAlerted;

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        intakeM = new Motor(hardwareMap, "Spinner");
        intakeM.setRunMode(Motor.RunMode.RawPower);
        intakeM.setInverted(false);
        servoFlap = new SimpleServo(hardwareMap, "I", 0, 260, AngleUnit.DEGREES);

        servoFlap.turnToAngle(200);
        driversAlerted = false;

        is = hardwareMap.get(ColorRangeSensor.class, "is"); // note: for rumble
    }

    public void spin() {
        intakeM.set(-0.8);
        servoFlap.turnToAngle(200);
    }

    public void reverseSpin() {
        intakeM.set(0.8);
    }

    public void stop() {
        intakeM.set(0);
    }

    public void openCover() {
        servoFlap.turnToAngle(114);}
    public void closeCover() {
        servoFlap.turnToAngle(200);}

    // note: rumble not being tested
    /*@Override
    public void periodic() {
        telemetry.addData("Color Sensor Distance: ", is.getDistance(DistanceUnit.MM));
        int threshold = 30;

        if (is.getDistance(DistanceUnit.MM) < threshold && !driversAlerted) {
            gamepad1.rumble(800);
            gamepad2.rumble(800);
            driversAlerted = true;
        } else if (is.getDistance(DistanceUnit.MM) > threshold) {
            driversAlerted = false;
        }
    }

    public void manualCoverControl(double angle, Telemetry telemetry) {
        int scaling = 8;
        servoFlap.rotateByAngle(angle  * scaling);
        telemetry.addData("Intake Cover: ", servoFlap.getAngle());
    }*/
}
