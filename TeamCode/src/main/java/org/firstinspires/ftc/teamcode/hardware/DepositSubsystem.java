package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DepositSubsystem extends SubsystemBase {

    // Servos
    public ServoEx Wrist;
    public ServoEx Gripper;
    public SimpleServo V4B;
    public AnalogInput V4B_Analog;
    public ServoEx Spin;

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
        // Assign variables here with parameters
//        GS = new SimpleServo(hardwareMap, "GS", 0, 360, AngleUnit.DEGREES);

        int MIN_ANGLE = 0;
        int MAX_ANGLE = 355;

        V4B = new SimpleServo(hardwareMap, "V4B", MIN_ANGLE, MAX_ANGLE, AngleUnit.DEGREES);

        V4B.setInverted(true);

        V4B_Analog = hardwareMap.get(AnalogInput.class, "VA");

        Wrist = new SimpleServo(hardwareMap, "W", 0, 260, AngleUnit.DEGREES);
        Gripper = new SimpleServo(hardwareMap, "G", 0, 180, AngleUnit.DEGREES);
        Spin = new SimpleServo(hardwareMap, "Spin", 0, 180, AngleUnit.DEGREES);

        V4B.turnToAngle(240);
        Wrist.turnToAngle(170);
        Spin.turnToAngle(transferSpin);
        Gripper.turnToAngle(transferGrip);
        outtaking = false;

    }

    @Override
    public void periodic() {
        if(outtaking) {
            double difference = V4B.getAngle() - 60;
            Wrist.turnToAngle(190-difference);
        }
    }


    public double getV4BPos() {
        return(V4B_Analog.getVoltage() / 3.3 * 360);
    }


    public void manualV4BControl(double angle, Telemetry telemetry) {
        int scaling = 8;

        V4B.rotateByAngle(angle  * scaling);

        telemetry.addData("V4B: ", V4B.getAngle());
    }

    public void manualWristControl(double angle, Telemetry telemetry) {
        int scaling = 10;

        Wrist.rotateByAngle(-angle  * scaling);

        telemetry.addData("Wrist: ", Wrist.getAngle());
    }

    public void manualSpinControl(double angle, Telemetry telemetry) {
        int scaling = 5;

        Spin.rotateByAngle(-angle  * scaling);

        telemetry.addData("Spin: ", Spin.getAngle());
    }

    public void mosaicSpin(double direction, Telemetry telemetry) {
        if(direction == 1) {
            Spin.turnToAngle(flatSpin - 60);
        } else if (direction == -1){
            Spin.turnToAngle(flatSpin + 60);
        } else {
            Spin.turnToAngle(flatSpin);
        }
    }


    public void place() {
        V4B.turnToAngle(324);
        Wrist.turnToAngle(165);
    }

    public void grab() { Gripper.turnToAngle(closedPosition); }
    public void release() { Gripper.turnToAngle(openPosition); }

    public boolean withinUncertainty(double currentPos, double wantedPos, double range) {
        if((currentPos < wantedPos + range) && currentPos > wantedPos - range) {
            return true;
        } else {
            return false;
        }
    }
}
