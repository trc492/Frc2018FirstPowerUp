package team492;

import trclib.TrcRobot.RunMode;
import frclib.FrcEmic2TextToSpeech;
import trclib.TrcTaskMgr;
import trclib.TrcTaskMgr.TaskType;
import trclib.TrcUtil;

public class SpeakStandClearWhenEnabledTask implements TrcTaskMgr.Task {
	
	private static final double SPEAK_PERIOD_SECONDS = 10.0; // Speaks once every this # of seconds.
	private static final String TASK_NAME = "SpeakStandClearWhenEnabledTask";
	
	private final FrcEmic2TextToSpeech tts;
	private double nextTimeToSpeakSeconds = 0; // 0 means disabled, no need to speak
	
	public SpeakStandClearWhenEnabledTask(FrcEmic2TextToSpeech tts) {
		this.tts = tts;
		
		TrcTaskMgr taskMgr = TrcTaskMgr.getInstance();
		taskMgr.registerTask(TASK_NAME, this, TaskType.START_TASK);
		taskMgr.registerTask(TASK_NAME, this, TaskType.POSTPERIODIC_TASK);
	}
		
	@Override
	public void startTask(RunMode runMode) {
		if (runMode != RunMode.DISABLED_MODE && runMode != RunMode.INVALID_MODE) {
			// Robot is unsafe
			nextTimeToSpeakSeconds = TrcUtil.getCurrentTime();
		} else {
			// Robot is safe
			nextTimeToSpeakSeconds = 0;
		}
	}

	@Override
	public void postPeriodicTask(RunMode runMode) {
		double currentTime = TrcUtil.getCurrentTime();
		if (nextTimeToSpeakSeconds > 0 && currentTime >= nextTimeToSpeakSeconds) {
			tts.speak("stand clear");
			nextTimeToSpeakSeconds = currentTime + SPEAK_PERIOD_SECONDS; 
		}
	}

	@Override
	public void stopTask(RunMode runMode) {
		// Not used.
	}

	@Override
	public void prePeriodicTask(RunMode runMode) {
		// Not used.
	}

	
	@Override
	public void preContinuousTask(RunMode runMode) {
		// Not used.
	}

	@Override
	public void postContinuousTask(RunMode runMode) {
		// Not used.
	}
}
