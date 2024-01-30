package org.firstinspires.ftc.teamcode.drive.hardware;

import static org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.RobotConstants.*;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.ASubsystemState.Outtake;

public class DepositSubsystem extends SubsystemBase {
    public Outtake outtakeState = Outtake.IDLE;
    public ServoEx wrist;
    public ServoEx claw;
    public SimpleServo elbow;
    public ServoEx spin;
    public Servo spinNew;
    public boolean outtakeBusy;

    public DepositSubsystem(HardwareMap hardwareMap) {
        elbow = new SimpleServo(hardwareMap, SERVO_ELBOW, 0, 355, AngleUnit.DEGREES);
        wrist = new SimpleServo(hardwareMap, SERVO_WRIST, 0, 260, AngleUnit.DEGREES);
        claw = new SimpleServo(hardwareMap, SERVO_CLAW, 0, 180, AngleUnit.DEGREES);
        //spin = new SimpleServo(hardwareMap, SERVO_SPIN, 0, 180, AngleUnit.DEGREES);

        spinNew = hardwareMap.get(Servo.class, SERVO_SPIN);

        elbow.setInverted(true);
        elbow.turnToAngle(ELBOW_HOME); // todo: test 240 (og) vs 260
        wrist.turnToAngle(WRIST_HOME);
        //spin.turnToAngle(SPIN_HOME);
        spinNew.setPosition(SPIN_HOME);
        claw.turnToAngle(CLAW_OPEN);
        outtakeBusy = false;
    }

    @Override
    public void periodic() {
        if (outtakeBusy) {
            /*double difference = elbow.getAngle() - 60;
            wrist.turnToAngle(190 - difference);*/
        }
    }

    public void manualElbowControl(double angle, Telemetry telemetry) {
        int scaling = 8;
        elbow.rotateByAngle(angle  * scaling);
        telemetry.addData("Elbow", elbow.getAngle());
    }

    public void manualWristControl(double angle, Telemetry telemetry) {
        int scaling = 10;
        wrist.rotateByAngle(-angle  * scaling);
        telemetry.addData("Wrist", wrist.getAngle());
    }

    public void mosaicSpin(double direction, Telemetry telemetry) {
        if (direction == 1) {
            //spin.turnToAngle(SPIN_DEPOSIT - 60); // SPIN_DEPOSIT - 60
            spinNew.setPosition(SPIN_DEPOSIT - 0.3);
        } else if (direction == -1){
            //spin.turnToAngle(SPIN_DEPOSIT + 60); // plus 60
            spinNew.setPosition(SPIN_DEPOSIT + 0.3);
        } else {
            //spin.turnToAngle(SPIN_DEPOSIT);
            spinNew.setPosition(SPIN_DEPOSIT);
        }
    }

    public void clawGrab() { claw.turnToAngle(CLAW_CLOSE); }
    public void clawReset() { claw.turnToAngle(CLAW_OPEN); }
    public void clawDeposit() { claw.turnToAngle(CLAW_DEPOSIT); }

    public boolean withinUncertainty(double currentPos, double wantedPos, double range) {
        return (currentPos < wantedPos + range) && currentPos > wantedPos - range;
    }
}
