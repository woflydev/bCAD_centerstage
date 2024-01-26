package org.firstinspires.ftc.teamcode.drive.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class LiftSubsystem extends SubsystemBase {
    protected DepositSubsystem deposit;
    protected LiftSubsystem lift;
    public MotorEx leftMotor;
    public MotorEx rightMotor;
    protected SlideModel slideModel;
    Telemetry telemetry;
    Gamepad gamepad2;

    public static int top;
    public int liftOffset = 0;


    public LiftSubsystem(HardwareMap hMap, Telemetry telemetry, Gamepad gamepad2, DepositSubsystem deposit, LiftSubsystem lift){
        this.telemetry = telemetry;
        this.gamepad2 = gamepad2;
        this.deposit = deposit;
        this.lift = lift;

        slideModel = new SlideModel();

        leftMotor = new MotorEx(hMap,"DS1", Motor.GoBILDA.RPM_1150);
        rightMotor = new MotorEx(hMap, "DS2", Motor.GoBILDA.RPM_1150);
        leftMotor.setInverted(true);
        leftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        leftMotor.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        top = slideModel.MAXENCODER;

    }


    @Override
    public void periodic() {

    }

    public void autoStabilize() {

    }

    public void run(double joystick) {
        leftMotor.motor.setTargetPosition(target(joystick));
        rightMotor.motor.setTargetPosition(target(joystick));

        telemetry.addData("Target: ", target(joystick));
        telemetry.addData("Power: ", Math.abs(joystick));

        leftMotor.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double motorPowers = Range.clip(Math.abs(joystick), 0.2, 1);

        telemetry.addData("Slide Power", motorPowers);

        leftMotor.motor.setPower(motorPowers);
        rightMotor.motor.setPower(motorPowers);
    }

    public int target(double joystick) {
        if(joystick > 0) return top+liftOffset;
        else if(joystick == 0) return leftMotor.motor.getCurrentPosition();
        else return 0+liftOffset; };

    public double getPosition(){
        return leftMotor.getCurrentPosition();
    }

    public boolean autoRun() {
        int target = 250;
        while(getPosition() < target-10) {
            leftMotor.motor.setTargetPosition(target);
            rightMotor.motor.setTargetPosition(target);

            leftMotor.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftMotor.motor.setPower(1);
            rightMotor.motor.setPower(1);
        }
        return true;
    }

    public boolean autoHome() {
        int target = 0;
        deposit.V4B.turnToAngle(260);
        deposit.Wrist.turnToAngle(170);
        deposit.Spin.turnToAngle(deposit.transferSpin);
        deposit.grab();
        deposit.outtaking = false;

        while(getPosition() > target+10) {
            leftMotor.motor.setTargetPosition(target);
            rightMotor.motor.setTargetPosition(target);

            leftMotor.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftMotor.motor.setPower(1);
            rightMotor.motor.setPower(1);
        }

        return(true);

    }
}