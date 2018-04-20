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

import frclib.FrcCANTalon;
import trclib.TrcDiagnostics;

public class TestTalonErrorRate<T> extends TrcDiagnostics.Test<T>
{
    private static final int ERROR_COUNT_THRESHOLD = 100;

    private final FrcCANTalon motor;

    public TestTalonErrorRate(String name, T group, FrcCANTalon motor)
    {
        super(name, group);
        this.motor = motor;
    }

    @Override
    public String runTest()
    {
        int errorCount = motor.getErrorCount();
        return errorCount > ERROR_COUNT_THRESHOLD?
            String.format("%s reported %d errors (> %d) - " +
                "there might be a problem with the CAN wiring.",
                this.getTestName(), errorCount, ERROR_COUNT_THRESHOLD): null;
    }

}
