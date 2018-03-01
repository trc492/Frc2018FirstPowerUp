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
    private TrcEvent timerEvent, proximityEvent, currentEvent;
    private TrcEvent cubePossessionEvent, proximityTriggerEvent, currentTriggerEvent;
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
        proximityEvent = new TrcEvent("ProximityEvent");
        currentEvent = new TrcEvent("CurrentEvent");
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

    /**
     * @return Returns true of there is a cube in the pickup
     */
     public boolean cubeDetected()
     {
         return cubeProximitySensor.isActive();
     }

    public double getPickupPower()
    {
        return controlMotor.getPower();
    }

    public double getPickupCurrent()
    {
        return controlMotor.motor.getOutputCurrent() + slaveMotor.motor.getOutputCurrent();
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
        robot.cubeIndicator.showNoCube();
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
        proximityTriggerEvent = event;
        cubeProximityTrigger.setTaskEnabled(enabled);
    }

    public void setCurrentTriggerEnabled(boolean enabled, TrcEvent event)
    {
        currentTriggerEvent = event;
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
            cubePossessionEvent = event;
            setPickupTaskEnabled(true);
        }
        else
        {
            robot.tracer.traceWarn("grabCube", "***** Caught double trigger *****");
        }
    }

    // Step 1:
    // - set timer for 0.3 second, when done goto step 2.
    // - this allows us to ignore the current spike when the motors start up.
    // Step 2:
    // - enable both cube proximity and current spike triggers.
    // - the proximity trigger will monitor if a cube is close enough to be grabbed and will automatically close
    //   the claws on it.
    // - the current spike trigger will monitor if we have the cube in possession (i.e. sucked in) and move on to
    //   step 3.
    // Step 3:
    // - the cube is in possession, disable both triggers.
    // - set timer for 0.3 second so that we will pull in the cube firmly before stopping the motor.
    // Step 4:
    // - stop motor.
    // - set the given event to true if any.
    // - stop the state machine.

    public void pickupTask(TaskType taskType, RunMode runMode)
    {
        State state = sm.checkReadyAndGetState();

        if (state != null)
        {
            switch (state)
            {
                case START:
                    // wait a bit to let the start up current spike past.
                    timer.set(0.5, timerEvent);
                    sm.waitForSingleEvent(timerEvent, State.DETECT_CUBE);
                    robot.cubeIndicator.showNoCube();
                    break;

                case DETECT_CUBE:
                    // enable both proximity and current triggers to detect the cube close by or in possession.
                    // proximity trigger will automatically close the claws.
                    // current trigger will move us to the next state.
                    robot.tracer.traceInfo("PickupCurrent", "pickupCurrent=%.2f", getPickupCurrent());
                    setProximityTriggerEnabled(true, proximityEvent);
                    setCurrentTriggerEnabled(true, currentEvent);
                    sm.waitForSingleEvent(currentEvent, State.PULLIN_CUBE);
                    break;

                case PULLIN_CUBE:
                    // disable both trigger.
                    // wait a bit to make sure we pull in the cube firmly.
                    setCurrentTriggerEnabled(false, null);
                    setProximityTriggerEnabled(false, null);
                    timer.set(0.3, timerEvent);
                    sm.waitForSingleEvent(timerEvent, State.DONE);
                    break;

                case DONE:
                default:
                    // we have the cube, stop the motor and tell somebody if necessary.
                    controlMotor.setPower(0.0);
                    robot.cubeIndicator.showCubeFullyGrabbed();
                    if (cubePossessionEvent != null)
                    {
                        cubePossessionEvent.set(true);
                    }
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
            if (proximityTriggerEvent != null)
            {
                proximityTriggerEvent.set(true);
            }
        }
    }

    public void currentTriggerEvent(int zoneIndex, double zoneValue)
    {
        robot.tracer.traceInfo("CurrentTrigger", "zone=%d, pickupCurrent=%.2f", zoneIndex, zoneValue);
        if (zoneIndex == 1 && currentTriggerEvent != null)
        {
            // Detected current spike beyond current threshold, let's tell
            // somebody.
            currentTriggerEvent.set(true);
        }
    }

}
