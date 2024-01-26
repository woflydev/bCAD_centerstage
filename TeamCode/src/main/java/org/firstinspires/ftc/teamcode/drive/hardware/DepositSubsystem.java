package org.firstinspires.ftc.teamcode.drive.hardware;

import static org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.RobotConstants.*;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DepositSubsystem extends SubsystemBase {
    // Servos
    public ServoEx wrist;
    public ServoEx claw;
    public SimpleServo elbow;
    public ServoEx spin;

    public int closedPosition = 140;
    public int openPosition = 30;
    public int transferGrip = 80;
    public int transferSpin = 27;
    public int flatSpin = transferSpin+90;
    public boolean outtaking;
    public boolean currentlyPlacing;
    public boolean currentlyHoming;
    public boolean currentlyTransfering;

    public DepositSubsystem(HardwareMap hardwareMap) {
        int MIN_ANGLE = 0;
        int MAX_ANGLE = 355;

        elbow = new SimpleServo(hardwareMap, SERVO_ELBOW, MIN_ANGLE, MAX_ANGLE, AngleUnit.DEGREES);

        elbow.setInverted(true);

        wrist = new SimpleServo(hardwareMap, SERVO_WRIST, 0, 260, AngleUnit.DEGREES);
        claw = new SimpleServo(hardwareMap, SERVO_CLAW, 0, 180, AngleUnit.DEGREES);
        spin = new SimpleServo(hardwareMap, SERVO_SPIN, 0, 180, AngleUnit.DEGREES);

        elbow.turnToAngle(240);
        wrist.turnToAngle(170);
        spin.turnToAngle(transferSpin);
        claw.turnToAngle(transferGrip);
        outtaking = false;
    }

    @Override
    public void periodic() {
        if (outtaking) {
            double difference = elbow.getAngle() - 60;
            wrist.turnToAngle(190 - difference);
        }
    }

    public void manualElbowControl(double angle, Telemetry telemetry) {
        int scaling = 8;
        elbow.rotateByAngle(angle  * scaling);
        telemetry.addData("V4B: ", elbow.getAngle());
    }

    public void manualWristControl(double angle, Telemetry telemetry) {
        int scaling = 10;
        wrist.rotateByAngle(-angle  * scaling);
        telemetry.addData("Wrist: ", wrist.getAngle());
    }

    public void manualSpinControl(double angle, Telemetry telemetry) {
        int scaling = 5;
        spin.rotateByAngle(-angle  * scaling);
        telemetry.addData("Spin: ", spin.getAngle());
    }

    public void mosaicSpin(double direction, Telemetry telemetry) {
        if(direction == 1) {
            spin.turnToAngle(flatSpin - 60);
        } else if (direction == -1){
            spin.turnToAngle(flatSpin + 60);
        } else {
            spin.turnToAngle(flatSpin);
        }
    }

    public void place() {
        elbow.turnToAngle(324);
        wrist.turnToAngle(165);
    }

    public void grab() { claw.turnToAngle(closedPosition); }
    public void release() { claw.turnToAngle(openPosition); }

    public boolean withinUncertainty(double currentPos, double wantedPos, double range) {
        return (currentPos < wantedPos + range) && currentPos > wantedPos - range;
    }
}
