package team492;

import frclib.FrcMotionProfile;
import frclib.FrcMotionProfileFollower;
import trclib.TrcPidController;
import trclib.TrcRobot;


public class MotionProfileTest
{
    private static final double kP = 0.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kF = 0.0;

    private String instanceName;
    private FrcMotionProfile profile;
    private FrcMotionProfileFollower follower;
    public MotionProfileTest(String instanceName, Robot robot)
    {
        this.instanceName = instanceName;
        profile = FrcMotionProfile.loadProfileFromCsv("test_left.csv",
                "test_right.csv",
                true);
        TrcPidController.PidCoefficients pidCoefficients = new TrcPidController.PidCoefficients(kP,kI,kD,kF);
        follower = new FrcMotionProfileFollower(instanceName + ".profileFollower",
                pidCoefficients, RobotInfo.ENCODER_Y_INCHES_PER_COUNT);
        follower.setLeftMotors(robot.leftFrontWheel,robot.leftRearWheel);
        follower.setRightMotors(robot.rightFrontWheel,robot.rightRearWheel);
    }

    public void start()
    {
        follower.start(profile);
    }
}
