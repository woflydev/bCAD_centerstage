package org.firstinspires.ftc.teamcode.drive.commands.teleopCommands;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class LeftTriggerReader extends Trigger {
    GamepadEx gamepad1;
    GamepadEx gamepad2;
    boolean doubleGamepad;

    public LeftTriggerReader(GamepadEx gamepad1, GamepadEx gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        doubleGamepad = true;
    }

    // note: instead of returning a float from 0 to 1, it returns a bool
    @Override
    public boolean get() {
        if(!doubleGamepad) {
            return gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2;
        } else {
            return gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2 || gamepad2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2;
        }
    }
}
