package team492;

import trclib.TrcRobot.RunMode;
import frclib.FrcEmic2TextToSpeech;
import trclib.TrcTaskMgr;
import trclib.TrcTaskMgr.TaskType;
import trclib.TrcUtil;

public class SpeakStandClearWhenEnabledTask
{

    private static final double SPEAK_PERIOD_SECONDS = 20.0; // Speaks once every this # of second.
    private static final String TASK_NAME = "SpeakStandClearWhenEnabledTask";

    private final FrcEmic2TextToSpeech tts;
    private double nextTimeToSpeakSeconds = 0; // 0 means disabled, no need to speak

    public SpeakStandClearWhenEnabledTask(final FrcEmic2TextToSpeech tts)
    {
        this.tts = tts;

        TrcTaskMgr taskMgr = TrcTaskMgr.getInstance();
        TrcTaskMgr.TaskObject startTaskObj = taskMgr.createTask(TASK_NAME + ".start", this::startTask);
        TrcTaskMgr.TaskObject postPeriodicTaskObj = taskMgr.createTask(
            TASK_NAME + ".postPeriodic", this::postPeriodicTask);
        startTaskObj.registerTask(TaskType.START_TASK);
        postPeriodicTaskObj.registerTask(TaskType.POSTPERIODIC_TASK);
    }

    public void startTask(TrcTaskMgr.TaskType taskType, RunMode runMode)
    {
        if (runMode != RunMode.DISABLED_MODE)
        {
            // Robot is unsafe
            tts.speak("Robot enabled, stand clear");
            nextTimeToSpeakSeconds = TrcUtil.getCurrentTime() + SPEAK_PERIOD_SECONDS;
        }
        else
        {
            // Robot is safe. Note: "disaibled" is not a typo. It forces the speech board to pronounce it correctly.
            tts.speak("Robot disaibled");
            nextTimeToSpeakSeconds = 0;
        }
    }

    public void postPeriodicTask(TrcTaskMgr.TaskType taskType, RunMode runMode)
    {
        double currentTime = TrcUtil.getCurrentTime();
        if (nextTimeToSpeakSeconds > 0 && currentTime >= nextTimeToSpeakSeconds)
        {
            tts.speak("Stand clear");
            nextTimeToSpeakSeconds = currentTime + SPEAK_PERIOD_SECONDS;
        }
    }

}
