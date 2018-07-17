package team492;

import trclib.TrcTankMotionProfile;
import frclib.FrcTankMotionProfileFollower;
import trclib.TrcPidController;
import trclib.TrcRobot;
import trclib.TrcUtil;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.text.SimpleDateFormat;
import java.util.Date;

public class MotionProfileTest implements TrcRobot.RobotCommand
{
    private static final double kP = 1.275;
    private static final double kI = 0.0;
    private static final double kD = 0.0956;
    private static final double kF = 0.8525; // TODO: Calculate this according to Phoenix docs

    private static final boolean WRITE_CSV = false;

    private String instanceName;
    private TrcTankMotionProfile profile;
    private FrcTankMotionProfileFollower follower;
    private Robot robot;
    private PrintStream fileOut;
    private double startTime;

    public MotionProfileTest(String instanceName, Robot robot)
    {
        this.instanceName = instanceName;
        this.robot = robot;
        profile = TrcTankMotionProfile
            .loadProfileFromCsv("/home/lvuser/test_left_Jaci.csv", "/home/lvuser/test_right_Jaci.csv");
        TrcPidController.PidCoefficients pidCoefficients = new TrcPidController.PidCoefficients(kP, kI, kD, kF);
        follower = new FrcTankMotionProfileFollower(instanceName + ".profileFollower", pidCoefficients,
            RobotInfo.ENCODER_Y_INCHES_PER_COUNT);
        follower.setLeftMotors(robot.leftFrontWheel, robot.leftRearWheel);
        follower.setRightMotors(robot.rightFrontWheel, robot.rightRearWheel);
    }

    public void start()
    {
        follower.start(profile);
        robot.globalTracer.traceInfo(instanceName + ".start", "Started following path!");

        if (WRITE_CSV)
        {
            try
            {
                startTime = TrcUtil.getCurrentTime();
                String timeStamp = new SimpleDateFormat("dd-MM-yy_HHmm").format(new Date());
                File dir = new File("/home/lvuser/MP_logs");
                if (dir.isDirectory() || dir.mkdir())
                {
                    fileOut = new PrintStream(new FileOutputStream(new File(dir, timeStamp + "_profilelog.csv")));
                    fileOut.println("Time," + "TargetPosLeft,ActualPosLeft,TargetVelLeft,ActualVelLeft,"
                        + "TargetPosRight,ActualPosRight,TargetVelRight,ActualVelRight");
                }
            }
            catch (IOException e)
            {
                robot.globalTracer.traceErr(instanceName + ".start", e.toString());
            }
        }
    }

    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        boolean isActive = follower.isActive();

        double targetPosLeft = follower.leftTargetPosition();
        double actualPosLeft = follower.getLeftMaster().getPosition() * RobotInfo.ENCODER_Y_INCHES_PER_COUNT;
        double targetVelLeft = follower.leftTargetVelocity();
        double actualVelLeft = follower.getLeftMaster().getSpeed() * 10 * RobotInfo.ENCODER_Y_INCHES_PER_COUNT;

        double targetPosRight = follower.rightTargetPosition();
        double actualPosRight = follower.getRightMaster().getPosition() * RobotInfo.ENCODER_Y_INCHES_PER_COUNT;
        double targetVelRight = follower.rightTargetVelocity();
        double actualVelRight = follower.getRightMaster().getSpeed() * 10 * RobotInfo.ENCODER_Y_INCHES_PER_COUNT;

        String message = String.format(
            "MotionProfile: %s - Running: %b, Bottom Buffer: [%d,%d], Top Buffer: [%d,%d], Target Positions: [%.2f,%.2f], Target Velocities: [%.2f,%.2f]",
            follower.getInstanceName(), isActive, follower.leftBottomBufferCount(), follower.rightBottomBufferCount(),
            follower.leftTopBufferCount(), follower.rightTopBufferCount(), targetPosLeft, targetPosRight, targetVelLeft,
            targetVelRight);

        robot.dashboard.displayPrintf(1, message);
        robot.globalTracer.traceInfo(instanceName + ".cmdPeriodic", message);

        if (fileOut != null && isActive)
        {
            String line = String
                .format("%.2f," + "%.2f,%.2f,%.2f,%.2f," + "%.2f,%.2f,%.2f,%.2f", TrcUtil.getCurrentTime() - startTime,
                    targetPosLeft, actualPosLeft, targetVelLeft, actualVelLeft, targetPosRight, actualPosRight,
                    targetVelLeft, actualVelRight);
            fileOut.println(line);
        }

        return !isActive;
    }
}