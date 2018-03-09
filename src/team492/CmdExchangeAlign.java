/*
 * Copyright (c) 2018 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package team492;

import frclib.FrcDigitalInput;
import trclib.TrcDigitalTrigger;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;

public class CmdExchangeAlign implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdExchangeAlign";

    private enum State
    {
    	START, START_STRAFE, ADJUST_POSITION, DONE
    }

    private Robot robot;
    private FrcDigitalInput proximitySensor;
    private TrcDigitalTrigger proximityTrigger;
    private TrcEvent proximityEvent, pidEvent;
    private TrcEvent onFinishedEvent = null;
    private TrcStateMachine<State> sm;
    private double strafeDistance;
    private boolean failed;

    public CmdExchangeAlign(Robot robot)
    {
        this.robot = robot;
        proximitySensor = new FrcDigitalInput("LeftProximitySensor", RobotInfo.DIO_LEFT_PROXIMITY_SENSOR);
        proximitySensor.setInverted(true);
        proximityTrigger = new TrcDigitalTrigger("ExchangeTrigger", proximitySensor, this::proximityTriggerEvent);
        proximityEvent = new TrcEvent(moduleName + ".proximityEvent");
        pidEvent = new TrcEvent(moduleName + ".pidEvent");
        sm = new TrcStateMachine<State>(moduleName);
    }

    public boolean isOpenSpace()
    {
        return proximitySensor.isActive();
    }

    public void start(boolean goRight)
    {
        start(goRight ? RobotInfo.EXCHANGE_ALIGN_STRAFE_DIST : -RobotInfo.EXCHANGE_ALIGN_STRAFE_DIST, null);
    }

    public void start(double strafeDistance, TrcEvent onFinishedEvent)
    {
        if(onFinishedEvent != null)
        {
            onFinishedEvent.clear();
        }

        this.strafeDistance = strafeDistance;
        this.onFinishedEvent = onFinishedEvent;

        stop();
        proximityTrigger.setTaskEnabled(true);
        sm.start(State.START);
        failed = false;
    }

    public void stop()
    {
        if(sm.isEnabled())
        {
            robot.pidDrive.cancel();
            proximityTrigger.setTaskEnabled(false);
            sm.stop();
        }
    }

    public boolean isEnabled()
    {
        return sm.isEnabled();
    }

    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        boolean done = !sm.isEnabled();

        if(done) return true;

        State state = sm.checkReadyAndGetState();
        //
        // Print debug info.
        //
        robot.dashboard.displayPrintf(1, "State: %s", state == null? "NotReady": state);
        if(state != null)
        {
            switch(state)
            {
                case START:
                    proximitySensor.setInverted(false); //CodeReview: Please explain!
                    if(!proximitySensor.isActive()) // If the proximity sensor can't detect the wall, attempt to find the wall
                    {
                        robot.pidDrive.setTarget(0.0, RobotInfo.EXCHANGE_ALIGN_WALL_DIST, robot.targetHeading, false, pidEvent);
                        sm.addEvent(pidEvent);
                        sm.addEvent(proximityEvent);
                        sm.waitForEvents(State.START_STRAFE);
                    }
                    else
                    {
                        sm.setState(State.START_STRAFE);
                    }
                    break;

                case START_STRAFE:
                    if(pidEvent.isSignaled() && !proximityEvent.isSignaled()) // Can't detect the wall for some reason
                    {
                        failed = true;
                        sm.setState(State.DONE);
                    } else
                    {
                        robot.pidDrive.cancel();
                        proximitySensor.setInverted(true);  //CodeReview: Please explain!
                        proximityEvent.clear();
                        
                        robot.pidDrive.setTarget(strafeDistance, 0.0, robot.targetHeading, false, pidEvent);
                        sm.addEvent(pidEvent);
                        sm.addEvent(proximityEvent);
                        sm.waitForEvents(State.ADJUST_POSITION);                        
                    }

                    break;

                case ADJUST_POSITION:
                    robot.pidDrive.cancel();
                    proximityTrigger.setTaskEnabled(false);
                    double xDistance = RobotInfo.EXCHANGE_WIDTH/2.0 + RobotInfo.EXCHANGE_ALIGN_SENSOR_OFFSET;
                    xDistance *= (strafeDistance < 0) ? 1 : -1;
                    robot.pidDrive.setTarget(xDistance, 0.0, robot.targetHeading, false, pidEvent, RobotInfo.EXCHANGE_ALIGN_TIMEOUT);
                    sm.waitForSingleEvent(pidEvent, State.DONE);
                    break;

                case DONE:
                    stop();
                    done = true;
                    if(onFinishedEvent != null)
                    {
                        if(failed)
                        {
                            onFinishedEvent.cancel();
                        } else
                        {
                            onFinishedEvent.set(true);
                        }
                        onFinishedEvent.set(true);
                    }
                    break;
            }
            robot.traceStateInfo(elapsedTime, state.toString());
        }

        return done;
    }

    private void proximityTriggerEvent(boolean active)
    {
        proximityEvent.set(active);
    }
}
