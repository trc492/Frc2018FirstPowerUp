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

import com.ctre.phoenix.motorcontrol.ControlMode;

import frclib.FrcPneumatic;
import frclib.FrcCANTalon;
import frclib.FrcCANTalonLimitSwitch;
import trclib.TrcDigitalTrigger;
import trclib.TrcEvent;

public class CubePickup
{
    private FrcCANTalon controlMotor, slaveMotor;
    private FrcPneumatic claw, deployer;
    private FrcCANTalonLimitSwitch cubeSensor;
    private TrcDigitalTrigger cubeTrigger;
    private TrcEvent cubeEvent;

    /**
     * Initialize the CubePickup class.
     */
    public CubePickup()
    {
        controlMotor = new FrcCANTalon("LeftPickupMotor", RobotInfo.CANID_LEFT_PICKUP);
        controlMotor.setInverted(true);
        controlMotor.configFwdLimitSwitchNormallyOpen(true);
        controlMotor.motor.overrideLimitSwitchesEnable(true);

        slaveMotor = new FrcCANTalon("RightPickupMotor", RobotInfo.CANID_RIGHT_PICKUP);
        slaveMotor.setInverted(true);
        slaveMotor.motor.set(ControlMode.Follower, RobotInfo.CANID_LEFT_PICKUP);

        claw = new FrcPneumatic(
            "CubePickupClaw", RobotInfo.CANID_PCM1,
            RobotInfo.SOL_CUBEPICKUP_CLAW_EXTEND, RobotInfo.SOL_CUBEPICKUP_CLAW_RETRACT);
        deployer = new FrcPneumatic(
            "CubePickupDeploy", RobotInfo.CANID_PCM1,
            RobotInfo.SOL_CUBEPICKUP_ARM_EXTEND, RobotInfo.SOL_CUBEPICKUP_ARM_RETRACT);
        cubeSensor = new FrcCANTalonLimitSwitch("CubeSensor", controlMotor, true);
        cubeTrigger = new TrcDigitalTrigger("CubeTrigger", cubeSensor, this::triggerEvent);
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
     * 
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

    // CodeReview: grabCube should always enable the digital trigger so it will turn off the motor when cube is
    // detected. So it should first turn on the pickup motor, then enable the digital trigger. When trigger event
    // is called, it will turn off the motor and disable the digital trigger.

    /**
     * spins the motors to pickup a cube and signals an event when done
     */
    public void grabCube(double power, TrcEvent event)
    {
        controlMotor.setPower(power);
        cubeTrigger.setEnabled(true);
        cubeEvent = event;
    }

    // CodeReview: this method should call the grabCube with the event parameter but set the event parameter to null.

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

    /**
     * stops the pickup motors, use after cube has been picked up or dropped
     */
    public void stopPickup()
    {
        controlMotor.setPower(0.0);
    }

    public void triggerEvent(boolean active)
    {
        stopPickup();
        if (cubeEvent != null)
        {
            cubeEvent.set(true);
        }
        cubeTrigger.setEnabled(false);
    } // DigitalTriggerEvent

}
