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
        START_VISION_STRAFE, VISION_DONE
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
        sm.start(State.START_VISION_STRAFE);
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

    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        final String funcName = "VisionStrafePeriodic";
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
                    case START_VISION_STRAFE:
                        setVisionTriggerEnabled(true, visionEvent);
                        sm.addEvent(visionEvent);
                        robot.pidDrive.setTarget(xDistance, 0.0, robot.targetHeading, false, event);
                        sm.addEvent(event);
                        sm.waitForEvents(State.VISION_DONE);
                        break;

                    case VISION_DONE:
                        robot.globalTracer.traceInfo(funcName, "visionEvent=%b, driveEvent=%b",
                            visionEvent.isSignaled(), event.isSignaled());
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

    public void visionTriggerHandler(int currZone, int prevZone, double value)
    {
        robot.globalTracer.traceInfo("VisionTrigger", "prevZone=%d, currZone=%d, value=%.2f", prevZone, currZone, value);
        if (currZone == 1)
        {
            if (visionTriggerEvent != null)
            {
                visionTriggerEvent.set(true);
            }
        }
    }

}
