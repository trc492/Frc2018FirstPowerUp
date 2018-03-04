package team492;

import frclib.FrcPneumatic;
import trclib.TrcEvent;
import trclib.TrcRobot.RunMode;
import trclib.TrcStateMachine;
import trclib.TrcTaskMgr;
import trclib.TrcTaskMgr.TaskType;
import trclib.TrcTimer;

public class Flipper extends FrcPneumatic
{
    private static final String moduleName = "Flipper";
    
    private enum State
    {
        EXTEND, DELAY, RETRACT
    }
    
    private TrcStateMachine<State> sm;
    private TrcEvent event;
    private TrcTimer timer;
    private TrcTaskMgr.TaskObject taskObj;
    private double delay;
    private boolean taskEnabled;
    
    public Flipper(String name, int canId, int channel1, int channel2)
    {
        super(name, canId, channel1, channel2);
        
        sm = new TrcStateMachine<State>(moduleName);
        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        taskObj = TrcTaskMgr.getInstance().createTask("flipperTask", this::flipTask);
    }
    
    private void setTaskEnabled(boolean enabled)
    {
        taskEnabled = enabled;
        if(enabled)
        {
            taskObj.registerTask(TaskType.POSTPERIODIC_TASK);
            sm.start(State.EXTEND);
        } else
        {
            taskObj.unregisterTask(TaskType.POSTPERIODIC_TASK);
            sm.stop();
        }
    }
    
    public boolean isTaskEnabled()
    {
        return taskEnabled;
    }
    
    public void extendAndRetract()
    {
        extendAndRetract(RobotInfo.FLIPPER_DELAY);
    }
    
    public void extendAndRetract(double delay)
    {
        if(!isTaskEnabled())
        {
            this.delay = delay;
            setTaskEnabled(true);            
        }
    }
    
    public void flipTask(TaskType taskType, RunMode runMode)
    {
        State state = sm.checkReadyAndGetState();
        
        if(state != null)
        {
            switch(state)
            {
                case EXTEND:
                    this.extend(0.0, event);
                    sm.waitForSingleEvent(event, State.DELAY);
                    break;
                    
                case DELAY:
                    timer.set(delay, event);
                    sm.waitForSingleEvent(event, State.RETRACT);
                    break;
                    
                case RETRACT:
                    this.retract();
                    setTaskEnabled(false);
                    break;
            }
        }     
    }
}
