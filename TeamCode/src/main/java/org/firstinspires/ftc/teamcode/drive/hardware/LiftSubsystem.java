package org.firstinspires.ftc.teamcode.drive.hardware;

import static org.firstinspires.ftc.teamcode.drive.Robotv9.RobotInfo.RobotConstants.*;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class LiftSubsystem extends SubsystemBase {
    protected DepositSubsystem deposit;
    protected LiftSubsystem lift;
    public ElapsedTime armRuntime = new ElapsedTime();
    public MotorEx liftLM;
    public MotorEx liftRM;
    protected SlideModel slideModel;
    Telemetry telemetry;
    Gamepad gamepad2;

    public static int top;
    public int liftOffset = 0;
    public int targetLiftPosition;

    public LiftSubsystem(HardwareMap hMap, Telemetry telemetry, Gamepad gamepad2, DepositSubsystem deposit, LiftSubsystem lift){
        this.telemetry = telemetry;
        this.gamepad2 = gamepad2;
        this.deposit = deposit;
        this.lift = lift;

        targetLiftPosition = 0;
        top = SlideModel.MAXENCODER;
        slideModel = new SlideModel();

        liftLM = new MotorEx(hMap,LIFT_L, Motor.GoBILDA.RPM_1150);
        liftRM = new MotorEx(hMap, LIFT_R, Motor.GoBILDA.RPM_1150);
        liftLM.setInverted(true);
        liftLM.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        liftRM.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        liftLM.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRM.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // note: this is only run IF deposit.outtakeBusy is false.
    public void run(double joystickY) {
        PassiveResetCheck(); // note: passive check should come first
        ManualLift(joystickY);
    }

    public void ManualLift(double joystickY) {
        int target = CalculateTargetFromJoystick(joystickY);
        double motorPowers = Range.clip(Math.abs(joystickY), 0.2, 1);

        targetLiftPosition = target;
        //UpdateLift(true, motorPowers); // TODO: re-enable manual joystick controls when i figure out what this thing does

        telemetry.addData("Target", joystickY);
        telemetry.addData("Slide Power", motorPowers);
    }

    public void UpdateLift(boolean joystick, double power) {
        /*
        note: set targetLiftPosition then call UpdateLift()
         */
        liftRM.setTargetPosition(targetLiftPosition);
        liftLM.setTargetPosition(targetLiftPosition);
        liftLM.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRM.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRuntime.reset();
        liftRM.motor.setPower(joystick ? power : MAX_LIFT_SPEED);
        liftLM.motor.setPower(joystick ? power : MAX_LIFT_SPEED);
        telemetry.update();
    }

    public void PassiveResetCheck() {
        /*
        note: automatically cuts power if two conditions are met:
        note: 1) both motors are below a threshold, 2) the target position is below a threshold, or 3) timeout has expired
         */
        int thresh = 20;
        if (targetLiftPosition <= thresh + 10) {
            if ((liftLM.getCurrentPosition() <= thresh && liftRM.getCurrentPosition() <= thresh) || armRuntime.seconds() >= LIFT_RESET_TIMEOUT) {
                liftRM.setVelocity(0);
                liftLM.setVelocity(0);
            }
        }
    }

    public int CalculateTargetFromJoystick(double joystick) {
        return (joystick > 0) ? (top + liftOffset) : ((joystick == 0) ? liftLM.motor.getCurrentPosition() : liftOffset);
    }

    // note: for lift operation during autonomous
    public void AutoRun() {
        int target = 250;
        while (getPosition() < target - 10) {
            liftLM.motor.setTargetPosition(target);
            liftRM.motor.setTargetPosition(target);

            liftLM.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftRM.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            liftLM.motor.setPower(1);
            liftRM.motor.setPower(1);
        }
    }

    public void AutoHome() {
        int target = 0;
        deposit.elbow.turnToAngle(260);
        deposit.wrist.turnToAngle(170);
        deposit.spin.turnToAngle(SPIN_HOME);
        deposit.clawGrab();
        deposit.outtakeBusy = false;

        while(getPosition() > target + 10) {
            liftLM.motor.setTargetPosition(target);
            liftRM.motor.setTargetPosition(target);

            liftLM.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftRM.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            liftLM.motor.setPower(1);
            liftRM.motor.setPower(1);
        }
    }

    @Override
    public void periodic() { }

    public double getPosition() { return liftLM.getCurrentPosition(); }
}