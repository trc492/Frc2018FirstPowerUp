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

package team492.diagnostics;

import java.util.function.Supplier;

import trclib.TrcDiagnostics;
import trclib.TrcPidActuator;

public class TestActuatorLimitSwitchStuck<T> extends TrcDiagnostics.Test<T>
{
    private final TrcPidActuator actuator;
    private final Supplier<Boolean> limitSwitch;
    private final double posChangeThreshold;

    private Double actuatorPosWhenLimitSwitchActivatedInches = null;
    private boolean limitSwitchEverStuck = false;

    public TestActuatorLimitSwitchStuck(
        T group, TrcPidActuator actuator, Supplier<Boolean> limitSwitch, double posChangeThreshold)
    {
        super(limitSwitch.toString() + ".StuckTest", group);
        this.actuator = actuator;
        this.limitSwitch = limitSwitch;
        this.posChangeThreshold = posChangeThreshold;
    }

    @Override
    public String runTest()
    {
        boolean limitSwitchActive = limitSwitch.get();

        // Save current position when limit switch is first activated
        if (limitSwitchActive && actuatorPosWhenLimitSwitchActivatedInches == null)
        {
            actuatorPosWhenLimitSwitchActivatedInches = actuator.getPosition();
        }

        if (actuatorPosWhenLimitSwitchActivatedInches != null)
        {
            final double changeInActuatorPositionInches =
                Math.abs(actuator.getPosition() - actuatorPosWhenLimitSwitchActivatedInches);

            // If limit switch is still pressed and we moved more than we should have, consider limit switch stuck
            if (limitSwitchActive && changeInActuatorPositionInches > posChangeThreshold)
            {
                limitSwitchEverStuck = true;
            }

            // Once the limit switch resets, clear the elevator position for next time
            if (!limitSwitchActive)
            {
                actuatorPosWhenLimitSwitchActivatedInches = null;
            }
        }

        return limitSwitchEverStuck? limitSwitch.toString() + " got stuck at least once": null;
    }

}
