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
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
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

import com.ctre.phoenix.motorcontrol.ControlMode;

import frclib.FrcPneumatic;
import frclib.FrcCANTalon;
import frclib.FrcDigitalInput;
import trclib.TrcAnalogSensor;
import trclib.TrcAnalogTrigger;
import trclib.TrcDigitalTrigger;
import trclib.TrcEvent;
import trclib.TrcRobot.RunMode;
import trclib.TrcStateMachine;
import trclib.TrcTaskMgr;
import trclib.TrcTaskMgr.TaskType;
import trclib.TrcTimer;
import trclib.TrcUtil;

public class CubePickup
{
    private enum State
    {
        START, DETECT_CUBE, CLOSE_CLAW, PULLIN_CUBE, DONE
    }

    private static final double[] currentThresholds =
        {RobotInfo.GRABBER_FREE_SPIN_CURRENT, RobotInfo.GRABBER_STALL_CURRENT};
    private Robot robot;
    private FrcCANTalon controlMotor, slaveMotor;
    private FrcPneumatic claw, deployer;
    private FrcDigitalInput cubeProximitySensor;
    private TrcDigitalTrigger cubeProximityTrigger;
    private TrcAnalogSensor currentSensor;
    private TrcAnalogTrigger<TrcAnalogSensor.DataType> currentTrigger;
    private TrcTaskMgr.TaskObject grabberTaskObj;
    private TrcStateMachine<State> sm;
    private TrcTimer timer;
    private TrcEvent timerEvent;
    private TrcEvent proximityEvent;
    private TrcEvent possessionEvent;
    private TrcEvent cubeEvent;
    public double startTime;

    /**
     * Initialize the CubePickup class.
     */
    public CubePickup(Robot robot)
    {
        this.robot = robot;

        controlMotor = new FrcCANTalon("LeftPickupMotor", RobotInfo.CANID_LEFT_PICKUP);
        controlMotor.setInverted(false);

        slaveMotor = new FrcCANTalon("RightPickupMotor", RobotInfo.CANID_RIGHT_PICKUP);
        slaveMotor.setInverted(true);
        slaveMotor.motor.set(ControlMode.Follower, RobotInfo.CANID_LEFT_PICKUP);

        claw = new FrcPneumatic("CubePickupClaw", RobotInfo.CANID_PCM1, RobotInfo.SOL_CUBEPICKUP_CLAW_EXTEND,
            RobotInfo.SOL_CUBEPICKUP_CLAW_RETRACT);
        deployer = new FrcPneumatic("CubePickupDeploy", RobotInfo.CANID_PCM1, RobotInfo.SOL_CUBEPICKUP_ARM_EXTEND,
            RobotInfo.SOL_CUBEPICKUP_ARM_RETRACT);

        cubeProximitySensor = new FrcDigitalInput("CubeProximitySensor", RobotInfo.DIO_CUBE_PROXIMITY_SENSOR);
        cubeProximityTrigger = new TrcDigitalTrigger(
            "CubeProximityTrigger", cubeProximitySensor, this::cubeProximityEvent);

        currentSensor = new TrcAnalogSensor("grabberCurrent", this::getGrabberCurrent);
        currentTrigger = new TrcAnalogTrigger<TrcAnalogSensor.DataType>(
            "PickupCurrentTrigger", currentSensor, 0, TrcAnalogSensor.DataType.RAW_DATA, currentThresholds,
            this::currentTriggerEvent);

        grabberTaskObj = TrcTaskMgr.getInstance().createTask("grabberTask", this::grabberTask);
        sm = new TrcStateMachine<>("grabberStateMachine");
        timer = new TrcTimer("grabberTimer");
        timerEvent = new TrcEvent("grabberTimerEvent");
        proximityEvent = new TrcEvent("cubeProximityEvent");
        possessionEvent = new TrcEvent("cubePossessionEvent");
    }

    /**
     * Open the claw.
     */
    public void openClaw()
    {
        claw.extend();
    }

    /**
     * Close the claw
     */
    public void closeClaw()
    {
        claw.retract();
    }

    /**
     * Get whether the claw is opened or not.
     *
     * @return true if opened, false if not.
     */
    public boolean isClawOpen()
    {
        return claw.isExtended();
    }

    /**
     * Set the state of the claw.
     *
     * @param open If true, open the claw. If false, close it.
     */
    public void setClawOpen(boolean open)
    {
        if (open)
            openClaw();
        else
            closeClaw();
    }

    /**
     * Lift the pickup so the claw is off the ground.
     */
    public void raisePickup()
    {
        deployer.retract();
    }

    /**
     * Lower the pickup so the claw is on the ground.
     */
    public void deployPickup()
    {
        deployer.extend();
    }

    /**
     * Set the state of the pickup.
     *
     * @param down If true, lower the pickup. Otherwise, lift.
     */
    public void setPickupDeployed(boolean down)
    {
        if (down)
            deployPickup();
        else
            raisePickup();
    }

    /**
     * Is the pickup up or down?
     *
     * @return Returns true if the pickup is up, (claw is off ground) and
     *         returns false if the pickup is down. (claw is on ground)
     */
    public boolean isPickupDeployed()
    {
        return deployer.isExtended();
    }

    /**
     * @return Returns true of there is a cube in the pickup
     */
     public boolean cubeDetected()
     {
         return cubeProximitySensor.isActive();
     }

    public double getPower()
    {
        return controlMotor.getPower();
    }

    private void setGrabberTaskEnabled(boolean enabled)
    {
        if (enabled)
        {
            grabberTaskObj.registerTask(TaskType.POSTCONTINUOUS_TASK);
            startTime = TrcUtil.getCurrentTime();
            sm.start(State.START);
        } else
        {
            grabberTaskObj.unregisterTask(TaskType.POSTCONTINUOUS_TASK);
            sm.stop();
        }
    }

    /**
     * spins the motors to pickup a cube and signals an event when done
     */
    public void grabCube(double power, TrcEvent event)
    {
        if (!sm.isEnabled())
        {
            controlMotor.setPower(power);
            cubeEvent = event;
            setGrabberTaskEnabled(true);
        }
        else
        {
            robot.tracer.traceInfo("grabCube", "***** Caught double trigger");
        }
    }

    /**
     * spin the pickup motors to pull in a cube
     */
    public void grabCube(double power)
    {
        controlMotor.setPower(power);
    }

    /**
     * spin the motors to push out a cube
     */
    public void dropCube(double power)
    {
        controlMotor.setPower(-power);
        robot.cubeIndicator.showNoCube();
    }

    public double getGrabberCurrent()
    {
        return controlMotor.motor.getOutputCurrent() + slaveMotor.motor.getOutputCurrent();
    }

    /**
     * stops the pickup motors, use after cube has been picked up or dropped.
     * caller can call this method to abort a cube pickup sequence in progress.
     */
    public void stopPickup()
    {
        controlMotor.setPower(0.0);
        if (sm.isEnabled())
        {
            setGrabberTaskEnabled(false);
        }
    }

    public void stopGrabberTask()
    {
        setGrabberTaskEnabled(false);
        stopPickup();
    }

    // Step 1:
    // - set timer for 0.3 second, when done goto step 2.
    // - this allows us to ignore the current spike when the motors start up.
    // Step 2:
    // - enable both cube proximity and current spike triggers, wait either events then goto step 3.
    // - the proximity trigger will monitor if a cube is close enough to be grabbed.
    // - the current spike trigger will monitor if we have the cube in possession (i.e. sucked in).
    // Step 3:
    // - a cube is detected close by or in possession, disable the proximity trigger, close the claw to secure it.
    // - if only proximity event was triggered, wait again for the current spike trigger, otherwise go to the
    //   next state directly.
    // Step 4:
    // - the cube is in possession, disable current spike trigger.
    // - set timer for 0.3 second so that we will pull in the cube firmly before stopping the motor.
    // Step 5:
    // - stop motor.
    // - set the given event to true if any.
    // - stop the state machine.

    public void grabberTask(TaskType taskType, RunMode runMode)
    {
        State state = sm.checkReadyAndGetState();

        if (state != null)
        {
            switch (state)
            {
                case START:
                    // wait a bit to let the start up current spike past.
                    timer.set(0.3, timerEvent);
                    sm.waitForSingleEvent(timerEvent, State.DETECT_CUBE);
                    robot.cubeIndicator.showNoCube();
                    break;

                case DETECT_CUBE:
                    // enable both proximity and current triggers to detect the cube close by or in possession.
                    // either events will move us to the next state.
                    robot.tracer.traceInfo("GrabberCurrent", "grabberCurrent=%.2f", getGrabberCurrent());
                    cubeProximityTrigger.setTaskEnabled(true);
                    currentTrigger.setTaskEnabled(true);
                    sm.addEvent(proximityEvent);
                    sm.addEvent(possessionEvent);
                    sm.waitForEvents(State.CLOSE_CLAW);
                    break;

                case CLOSE_CLAW:
                    // disable proximity trigger.
                    // close the claw to grab the cube.
                    // enable current trigger to detect cube in possession.
                    closeClaw();
                    cubeProximityTrigger.setTaskEnabled(false);
                    if (possessionEvent.isSignaled())
                    {
                        // current trigger has also happened, move onto the next state.
                        sm.setState(State.PULLIN_CUBE);
                    }
                    else
                    {
                        // still don't have cube in possession, wait for the current spike.
                        sm.waitForSingleEvent(possessionEvent, State.PULLIN_CUBE);
                    }
                    break;

                case PULLIN_CUBE:
                    // disable current trigger.
                    // wait a bit to make sure we pull in the cube firmly.
                    currentTrigger.setTaskEnabled(false);
                    timer.set(0.3, timerEvent);
                    sm.waitForSingleEvent(timerEvent, State.DONE);
                    break;

                case DONE:
                default:
                    // we have the cube, stop the motor and tell somebody if necessary.
                    controlMotor.setPower(0.0);
                    robot.cubeIndicator.showCubeFullyGrabbed();
                    if (cubeEvent != null)
                    {
                        cubeEvent.set(true);
                    }
                    setGrabberTaskEnabled(false);
                    break;
            }

            robot.traceStateInfo(TrcUtil.getCurrentTime() - startTime, state.toString());
        }
    }

    public void cubeProximityEvent(boolean active)
    {
        robot.tracer.traceInfo("ProximityTrigger", "active=%b", active);
        if (active)
        {
            proximityEvent.set(true);
        }
    }

    public void currentTriggerEvent(int zoneIndex, double zoneValue)
    {
        robot.tracer.traceInfo("CurrentTrigger", "zone=%d, grabberCurrent=%.2f", zoneIndex, zoneValue);
        if (zoneIndex == 1)
        {
            // Detected current spike beyond current threshold, let's tell
            // somebody.
            possessionEvent.set(true);
        }
    }

}
