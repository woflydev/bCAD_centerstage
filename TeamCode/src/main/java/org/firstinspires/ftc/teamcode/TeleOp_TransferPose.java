package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotAutoConstants.BLUE_STARTING_POSES;
import static org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.RobotAutoConstants.RED_STARTING_POSES;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.AAutoState.RobotAlliance;
import org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.AAutoState.RobotStartingPosition;
import org.firstinspires.ftc.teamcode.Robotv9.RobotInfo.GlobalStorage;

@TeleOp(name="TeleOp ", group="!RC")
public class TeleOp_TransferPose extends LinearOpMode {
    public RobotAlliance alliance;
    public RobotStartingPosition startingPosition;

    public static Pose2d START_POSE = new Pose2d();

    public TeleOp_TransferPose(RobotAlliance alliance, RobotStartingPosition startPos) {
        this.alliance = alliance;
        this.startingPosition = startPos;
    }

    public void runOpMode() {
        DetermineStartFinishPoses();
        GlobalStorage.switchoverPose = START_POSE;
    }

    private void DetermineStartFinishPoses() {
        START_POSE =
                this.startingPosition == RobotStartingPosition.BACKDROP
                    ? (this.alliance == RobotAlliance.RED
                        ? RED_STARTING_POSES[0]
                        : BLUE_STARTING_POSES[0])
                    : (this.alliance == RobotAlliance.RED
                        ? RED_STARTING_POSES[1]
                        : BLUE_STARTING_POSES[1]);
    }
}