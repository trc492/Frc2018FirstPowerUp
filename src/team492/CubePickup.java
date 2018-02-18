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
import frclib.FrcCANTalonLimitSwitch;
import trclib.TrcAnalogSensor;
import trclib.TrcAnalogTrigger;
import trclib.TrcEvent;
import trclib.TrcRobot.RunMode;
import trclib.TrcStateMachine;
import trclib.TrcTaskMgr;
import trclib.TrcTaskMgr.TaskType;
import trclib.TrcTimer;

public class CubePickup
{
    private enum State
    {
        START, ENABLE_TRIGGER, DISABLE_TRIGGER, DONE
    }

    private FrcCANTalon controlMotor, slaveMotor;
    private FrcPneumatic claw, deployer;
    private FrcCANTalonLimitSwitch cubeSensor;
    private TrcEvent cubeEvent;
    private TrcAnalogSensor currentSensor;
    private TrcAnalogTrigger<TrcAnalogSensor.DataType> currentTrigger;
    private TrcTaskMgr.TaskObject grabberTaskObj;
    private TrcStateMachine<State> sm;
    private TrcTimer timer;
    private TrcEvent event;
    private double[] currentThreshold = {RobotInfo.GRABBER_CURRENT_THRESHOLD};

    /**
     * Initialize the CubePickup class.
     */
    public CubePickup()
    {
        controlMotor = new FrcCANTalon("LeftPickupMotor", RobotInfo.CANID_LEFT_PICKUP);
        controlMotor.setInverted(true);

        slaveMotor = new FrcCANTalon("RightPickupMotor", RobotInfo.CANID_RIGHT_PICKUP);
        slaveMotor.setInverted(true);
        slaveMotor.motor.set(ControlMode.Follower, RobotInfo.CANID_LEFT_PICKUP);

        claw = new FrcPneumatic("CubePickupClaw", RobotInfo.CANID_PCM1, RobotInfo.SOL_CUBEPICKUP_CLAW_EXTEND,
            RobotInfo.SOL_CUBEPICKUP_CLAW_RETRACT);
        deployer = new FrcPneumatic("CubePickupDeploy", RobotInfo.CANID_PCM1, RobotInfo.SOL_CUBEPICKUP_ARM_EXTEND,
            RobotInfo.SOL_CUBEPICKUP_ARM_RETRACT);

        cubeSensor = new FrcCANTalonLimitSwitch("CubeSensor", controlMotor, true);

        currentSensor = new TrcAnalogSensor("grabberCurrent", this::getGrabberCurrent);
        currentTrigger = new TrcAnalogTrigger<TrcAnalogSensor.DataType>(
            "PickupCurrentTrigger", currentSensor, 0, TrcAnalogSensor.DataType.RAW_DATA,
            currentThreshold, this::triggerEvent);
        grabberTaskObj = TrcTaskMgr.getInstance().createTask("grabberTask", this::grabberTask);
        sm = new TrcStateMachine<>("grabberStateMachine");
        timer = new TrcTimer("grabberTimer");
        event = new TrcEvent("grabberEvent");
    }

    private void setGrabberTaskEnabled(boolean enabled)
    {
        if (enabled)
        {
            grabberTaskObj.registerTask(TaskType.POSTCONTINUOUS_TASK);
            sm.start(State.START);
        }
        else
        {
            grabberTaskObj.unregisterTask(TaskType.POSTCONTINUOUS_TASK);
            sm.stop();
        }
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
        return cubeSensor.isActive();
    }

    public double getPower()
    {
        return controlMotor.getPower();
    }

    /**
     * spins the motors to pickup a cube and signals an event when done
     */
    public void grabCube(double power, TrcEvent event)
    {
        controlMotor.setPower(power);
        cubeEvent = event;
        setGrabberTaskEnabled(true);
    }

    // CodeReview: this method should call the grabCube with the event parameter
    // but set the event parameter to null.

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
    }

    public double getGrabberCurrent()
    {
        return controlMotor.motor.getOutputCurrent() + slaveMotor.motor.getOutputCurrent();
    }

    /**
     * stops the pickup motors, use after cube has been picked up or dropped
     */
    public void stopPickup()
    {
        controlMotor.setPower(0.0);
    }

    // Step 1:
    // - set timer for 0.5 second, when done goto step 2.
    // - this allows us to ignore the current spike when the motor starts up.
    // Step 2:
    // - enable analog trigger, wait for the event then goto step 3.
    // - the analog trigger will monitor for current spike which will happen when the cube is in possession.
    // Step 3:
    // - disable analog trigger.
    // - we are done with cube detection, set timer for 0.5 second, when done goto step 4.
    // - the delay allows the motor to firmly pull the cube in before we stop the motor.
    // Step 4:
    // - stop motor.
    // - set the given event to true.
    // - stop the state machine.

    public void grabberTask(TaskType taskType, RunMode runMode)
    {
        State state = sm.getState();

        switch (state)
        {
            case START:
                timer.set(0.5, event);
                sm.waitForSingleEvent(event, State.ENABLE_TRIGGER);
                break;

            case ENABLE_TRIGGER:
                currentThreshold[0] = getGrabberCurrent() + RobotInfo.GRABBER_CURRENT_OFFSET;
                currentTrigger.setTaskEnabled(true);
                sm.waitForSingleEvent(event, State.DISABLE_TRIGGER);
                break;

            case DISABLE_TRIGGER:
                currentTrigger.setTaskEnabled(false);
                timer.set(0.5, event);
                sm.waitForSingleEvent(event, State.DONE);
                break;

            case DONE:
            default:
                controlMotor.setPower(0.0);
                if (cubeEvent != null)
                {
                    cubeEvent.set(true);
                }
                setGrabberTaskEnabled(false);
                break;
        }
    }

    public void triggerEvent(int zoneIndex, double zoneValue)
    {
        if (zoneIndex == 1)
        {
            // Detected current spike beyond current threshold, let's tell somebody.
            event.set(true);
        }
    }

}
