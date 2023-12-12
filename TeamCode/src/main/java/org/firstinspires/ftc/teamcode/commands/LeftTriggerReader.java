package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class LeftTriggerReader extends Trigger {
    GamepadEx gamepad1;
    GamepadEx gamepad2;
    boolean doubleGamepad;
    public LeftTriggerReader(GamepadEx gamepad1) {
        this.gamepad1 = gamepad1;
        doubleGamepad = false;
    }

    public LeftTriggerReader(GamepadEx gamepad1, GamepadEx gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        doubleGamepad = true;

    }
    public boolean get() {
        if(!doubleGamepad) {
            if(gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2) return true;
            else return false;
        } else {
            if(gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2 || gamepad2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2) return true;
            else return false;
        }
    }
}
