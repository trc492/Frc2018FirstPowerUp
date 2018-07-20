package team492;

import common.CmdTimedDrive;
import hallib.HalDashboard;
import trclib.TrcRobot;

public class DriveBaseWidth implements TrcRobot.RobotCommand
{
    private Robot robot;
    private double cumulativeAngle;
    private CmdTimedDrive timedDrive;

    public DriveBaseWidth(Robot robot)
    {
        this.robot = robot;
        this.cumulativeAngle = 0.0;
    }

    public void reset()
    {
        robot.driveBase.resetPosition(true);
        this.cumulativeAngle = 0.0;
    }
    
    public void start()
    {
        timedDrive = new CmdTimedDrive(robot, 0.0, 8.0, 0.0, 0.0, 0.7);
        reset();
    }

    public double getCumulativeAngle()
    {
        return cumulativeAngle;
    }

    public double getEffectiveDriveBaseWidth()
    {
        // C = pi*d
        // R*C = R*pi*d
        // (R*C)/(R*pi) = d where R = numRotations and R*C = encoder distance
        double numRotations = Math.abs(cumulativeAngle) / 360.0;
        if(numRotations == 0.0)
            return 0.0;
        double leftDistance =
            RobotInfo.ENCODER_Y_INCHES_PER_COUNT * (robot.leftFrontWheel.getPosition() + robot.leftRearWheel
                .getPosition()) / 2.0;
        double rightDistance =
            RobotInfo.ENCODER_Y_INCHES_PER_COUNT * (robot.rightFrontWheel.getPosition() + robot.rightRearWheel
                .getPosition()) / 2.0;
        double avgDistance = (Math.abs(leftDistance) + Math.abs(rightDistance)) / 2.0;
        return avgDistance / (numRotations * Math.PI);
    }

    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        cumulativeAngle = robot.driveBase.getHeading();
        timedDrive.cmdPeriodic(elapsedTime);
        
        HalDashboard.putNumber("Test/EffectiveDriveBaseWidth", getEffectiveDriveBaseWidth());
        return false;
    }
}
