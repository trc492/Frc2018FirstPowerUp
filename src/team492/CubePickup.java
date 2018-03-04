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
        START, DETECT_CUBE, PULLIN_CUBE, DONE
    }

    private static final double[] currentThresholds =
        {RobotInfo.PICKUP_FREE_SPIN_CURRENT, RobotInfo.PICKUP_STALL_CURRENT};
    private Robot robot;
    private FrcCANTalon controlMotor, slaveMotor;
    private FrcPneumatic claw, deployer;
    private FrcDigitalInput cubeProximitySensor;
    private TrcDigitalTrigger cubeProximityTrigger;
    private TrcAnalogSensor currentSensor;
    private TrcAnalogTrigger<TrcAnalogSensor.DataType> currentTrigger;
    private TrcTaskMgr.TaskObject pickupTaskObj;
    private TrcStateMachine<State> sm;
    private TrcTimer timer;
    private TrcEvent timerEvent, currentUpEvent, currentDownEvent;
    private TrcEvent cubeInPossessionEvent, cubeInProximityEvent;
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

        currentSensor = new TrcAnalogSensor("pickupCurrent", this::getPickupCurrent);
        currentTrigger = new TrcAnalogTrigger<TrcAnalogSensor.DataType>(
            "PickupCurrentTrigger", currentSensor, 0, TrcAnalogSensor.DataType.RAW_DATA, currentThresholds,
            this::currentTriggerEvent);

        pickupTaskObj = TrcTaskMgr.getInstance().createTask("pickupTask", this::pickupTask);
        sm = new TrcStateMachine<>("pickupStateMachine");
        timer = new TrcTimer("pickupTimer");
        timerEvent = new TrcEvent("TimerEvent");
        currentUpEvent = new TrcEvent("CurrentUpEvent");
        currentDownEvent = new TrcEvent("CurrentDownEvent");
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
     * Is the pickup up or down?
     *
     * @return Returns true if the pickup is up, (claw is off ground) and
     *         returns false if the pickup is down. (claw is on ground)
     */
    public boolean isPickupDeployed()
    {
        return deployer.isExtended();
    }

    //
    // Pickup motor methods.
    //

    public double getPickupPower()
    {
        return controlMotor.getPower();
    }

    public double getPickupCurrent()
    {
        return controlMotor.motor.getOutputCurrent() + slaveMotor.motor.getOutputCurrent();
    }

    /**
     * @return Returns true of there is a cube in the pickup
     */
    public boolean cubeInProximity()
    {
        return cubeProximitySensor.isActive();
    }

    public boolean cubeInPossession()
    {
        return getPickupCurrent() > RobotInfo.PICKUP_CURRENT_THRESHOLD;
    }

    private void setPickupTaskEnabled(boolean enabled)
    {
        if (enabled)
        {
            pickupTaskObj.registerTask(TaskType.POSTCONTINUOUS_TASK);
            startTime = TrcUtil.getCurrentTime();
            sm.start(State.START);
        }
        else
        {
            pickupTaskObj.unregisterTask(TaskType.POSTCONTINUOUS_TASK);
            sm.stop();
        }
    }

    /**
     * spin the pickup motors to the given power.
     */
    public void setPickupPower(double power)
    {
        if (sm.isEnabled())
        {
            setPickupTaskEnabled(false);
        }
        controlMotor.setPower(power);
    }

    /**
     * spin the motors to push out a cube
     */
    public void dropCube(double power)
    {
        setPickupPower(-power);
        robot.ledStrip.setPattern(RobotInfo.LED_CUBE_NONE);
    }

    /**
     * stops the pickup motors, use after cube has been picked up or dropped.
     * caller can call this method to abort a cube pickup sequence in progress.
     */
    public void stopPickup()
    {
        setPickupPower(0.0);
    }

    public void setProximityTriggerEnabled(boolean enabled, TrcEvent event)
    {
        cubeInProximityEvent = event;
        cubeProximityTrigger.setTaskEnabled(enabled);
    }

    public void setCurrentTriggerEnabled(boolean enabled, TrcEvent event)
    {
        cubeInPossessionEvent = event;
        currentTrigger.setTaskEnabled(enabled);
    }

    /**
     * spins the motors to pickup a cube and signals an event when done
     */
    public void grabCube(double power, TrcEvent event)
    {
        if (!sm.isEnabled())
        {
            controlMotor.setPower(power);
            cubeInPossessionEvent = event;
            setPickupTaskEnabled(true);
        }
        else
        {
            robot.tracer.traceWarn("grabCube", "***** Caught double trigger *****");
        }
    }

    public void pickupTask(TaskType taskType, RunMode runMode)
    {
        State state = sm.checkReadyAndGetState();

        if (state != null)
        {
            switch (state)
            {
                case START:
                    // Enable both proximity and current triggers.
                    // Proximity trigger will automatically close the claws.
                    // Current trigger will detect the startup current spike subsiding so we can start monitoring
                    //      the real current spike for cube possession.
                    // The timer is for catching the case where we already have the cube in possession so the
                    //      current is already spiking and will not come down. The timer timeout will move us to the
                    //      next state so we don't wait forever for the startup current spike to subside.
                    cubeProximityTrigger.setTaskEnabled(true);
                    currentTrigger.setTaskEnabled(true);
                    timer.set(1.0, timerEvent);
                    robot.ledStrip.setPattern(RobotInfo.LED_CUBE_NONE);
                    sm.addEvent(timerEvent);
                    sm.addEvent(currentDownEvent);
                    sm.waitForEvents(State.DETECT_CUBE);
                    break;

                case DETECT_CUBE:
                    // We are now waiting for the current spike ramping up indicating cube in possession.
                    robot.tracer.traceInfo("PickupCurrent", "pickupCurrent=%.2f", getPickupCurrent());
                    sm.waitForSingleEvent(currentUpEvent, State.PULLIN_CUBE);
                    break;

                case PULLIN_CUBE:
                    // Disable both triggers.
                    // Wait a bit to make sure we pull in the cube firmly before stopping.
                    cubeProximityTrigger.setTaskEnabled(false);
                    currentTrigger.setTaskEnabled(false);
                    timer.set(0.3, timerEvent);
                    sm.waitForSingleEvent(timerEvent, State.DONE);
                    break;

                case DONE:
                default:
                    // We have the cube, we can stop now.
                    controlMotor.setPower(0.0);
                    robot.ledStrip.setPattern(RobotInfo.LED_CUBE_IN_POSSESSION);
                    setPickupTaskEnabled(false);
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
            // Detected cube close by, grab it.
            closeClaw();
            if (cubeInProximityEvent != null)
            {
                cubeInProximityEvent.set(true);
            }
        }
    }

    public void currentTriggerEvent(int currZone, int prevZone, double zoneValue)
    {
        robot.tracer.traceInfo("CurrentTrigger", "curZone=%d, prevZone=%d, pickupCurrent=%.2f",
            currZone, prevZone, zoneValue);
        //TODO: debug this. The condition may be too restrictive. It may be okay now that the trigger state
        // is reset before enabling. So the last state does not persist across enable/disable.
        if (prevZone == 0 && currZone == 1)
        {
            // Current is ramping up.
            currentUpEvent.set(true);
            if (cubeInPossessionEvent != null)
            {
                cubeInPossessionEvent.set(true);
            }
        }
        else if (prevZone == 1 && currZone == 0)
        {
            // Current is ramping down.
            currentDownEvent.set(true);
        }
    }

}
