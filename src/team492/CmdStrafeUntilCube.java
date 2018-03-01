package team492;

import trclib.TrcAnalogSensor;
import trclib.TrcAnalogTrigger;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;

public class CmdStrafeUntilCube implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdStrafeUntilCube";

    public enum State
    {
        START_STRAFE, DONE
    }

    private static final double[] visionThresholds =
        {-RobotInfo.FIND_CUBE_X_TOLERANCE*2, 0.0, RobotInfo.FIND_CUBE_X_TOLERANCE*2};

    private Robot robot;
    private TrcAnalogSensor visionSensor;
    private TrcAnalogTrigger<TrcAnalogSensor.DataType> visionTrigger;
    private TrcStateMachine<State> sm;
    private TrcEvent event, visionEvent;
    private double xDistance;
    private TrcEvent visionTriggerEvent, onFinishedEvent;
//    private boolean strafeRight;
//    private double startTime;
//    private double startX, startY;

    public CmdStrafeUntilCube(Robot robot)
    {
        this.robot = robot;
        visionSensor = new TrcAnalogSensor("pixyVision", robot::getPixyTargetX);
        visionTrigger = new TrcAnalogTrigger<>(
            "VisionTrigger", visionSensor, 0, TrcAnalogSensor.DataType.RAW_DATA, visionThresholds,
            this::visionTriggerHandler);
        sm = new TrcStateMachine<>(moduleName);
        event = new TrcEvent(moduleName);
        visionEvent = new TrcEvent(moduleName + ".VisionEvent");
    }

    void setVisionTriggerEnabled(boolean enabled, TrcEvent event)
    {
        visionTriggerEvent = event;
        visionTrigger.setTaskEnabled(enabled);
    }

    /**
     * Is the state machine enabled?
     * 
     * @return Is the state machine enabled?
     */
    public boolean isEnabled()
    {
        return sm.isEnabled();
    }

    public void start(double xDistance, TrcEvent event)
    {
        stop();
        this.xDistance = xDistance;
        onFinishedEvent = event;
        sm.start(State.START_STRAFE);
    }

    public void start(double xDistance)
    {
        start(xDistance, null);
    }

    public void stop()
    {
        if (sm.isEnabled())
        {
            robot.pidDrive.cancel();
            setVisionTriggerEnabled(false, null);
            sm.stop();
        }
    }

//    /**
//     * @param strafeRight
//     * @param stopTrigger Should return true when this should stop
//     */
//    public void start(boolean strafeRight, StopTrigger stopTrigger)
//    {
//        this.strafeRight = strafeRight;
//        sm.start(State.START_STRAFE);
//
//        startTime = Robot.getModeElapsedTime();
//        startX = robot.driveBase.getXPosition();
//        startY = robot.driveBase.getYPosition();
//        
//        this.stopTrigger = stopTrigger;
//    }
//
//    /**
//     * Default stopTrigger. Continue indefinitely.
//     */
//    public boolean shouldStop(double elapsedTime, double changeX, double changeY, boolean xStop)
//    {
//        return false;
//    }
//
//    /**
//     * Is this task current active?
//     * 
//     * @return True if active, false if not.
//     */
//    public boolean isRunning()
//    {
//        return sm.isEnabled();
//    }
//
//    /**
//     * How much time has passed since this task was started?
//     * 
//     * @return Time in seconds
//     */
//    public double elapsedTime()
//    {
//        return Robot.getModeElapsedTime() - startTime;
//    }
//
//    /**
//     * Return distance moved since this task was started
//     * 
//     * @return scaled distance
//     */
//    public double[] distanceMoved()
//    {
//        return new double[] { changeX(), changeY() };
//    }
//
//    public double changeX()
//    {
//        robot.tracer.traceInfo("ChangeX", "changeX");
//        return robot.driveBase.getXPosition() - startX;
//    }
//
//    private double changeY()
//    {
//        robot.tracer.traceInfo("ChangeY", "changeY");
//        return robot.driveBase.getYPosition() - startY;
//    }
//
    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        boolean done = !sm.isEnabled();

        if (!done)
        {
            State state = sm.checkReadyAndGetState();
            //
            // Print debug info.
            //
            robot.dashboard.displayPrintf(1, "State: %s", state == null? "NotReady": state);

            if (state != null)
            {
                switch (state)
                {
                    case START_STRAFE:
                        setVisionTriggerEnabled(true, visionEvent);
                        sm.addEvent(visionEvent);
                        robot.pidDrive.setTarget(xDistance, 0.0, robot.targetHeading, false, event);
                        sm.addEvent(event);
                        sm.waitForEvents(State.DONE);
//                        double xPower = RobotInfo.FIND_CUBE_STRAFE_POWER * (strafeRight ? 1 : -1);
//                        robot.pidDrive.driveMaintainHeading(xPower, 0.0, robot.targetHeading);
//                        sm.setState(State.CHECK_FOR_CUBE);
                        break;

//                    case CHECK_FOR_CUBE:
//                        TargetInfo target = robot.pixy.getTargetInfo();
//                        if (target != null && Math.abs(target.angle) <= RobotInfo.FIND_CUBE_ANGLE_TOLERANCE)
//                        {
//                            sm.setState(State.DONE);
//                        }
//                        break;
                    case DONE:
                        stop();
                        if (onFinishedEvent != null)
                        {
                            onFinishedEvent.set(true);
                        }
                        done = true;
                        break;
                }
                robot.traceStateInfo(Robot.getModeElapsedTime(), state.toString());
            }
        }

        return done;
    }

    public void visionTriggerHandler(int zone, double value)
    {
        if (zone == 1 || zone == 2)
        {
            if (visionTriggerEvent != null)
            {
                visionTriggerEvent.set(true);
            }
        }
    }
//    public interface StopTrigger
//    {
//        public boolean shouldStop(double elapsedTime, double changeX, double changeY, boolean xStop);
//    }

}
