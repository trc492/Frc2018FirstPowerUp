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

package trclib;

/**
 * This class implements a platform independent AnalogInput. Typically, this class is extended by a platform dependent
 * sensor class that produces value data. The sensor doesn't have to be connected to the AnalogInput port. It could be
 * connected to an I2C port as long as it produces a value data. The platform dependent sensor class must implement
 * the abstract methods required by this class. The abstract methods allow this class to get raw data from the sensor.
 * Depending on the options specified in the constructor, this class may create an integrator. If it needs data
 * integration, it can set the INTEGRATE or the DOUBLE_INTEGRATE options.
 */
public class TrcAnalogSensor extends TrcAnalogInput
{
    public interface AnalogDataSource
    {
        double getData();
    }

    private AnalogDataSource dataSource;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param numAxes specifies the number of axes.
     * @param options specifies the AnalogInput options. Multiple options can be OR'd together.
     *                ANALOGINPUT_INTEGRATE - do integration on sensor data.
     *                ANALOGINPUT_DOUBLE_INTEGRATE - do double integration on sensor data.
     * @param filters specifies an array of filter objects, one for each axis, to filter sensor data. If no filter
     *                is used, this can be set to null.
     */
    public TrcAnalogSensor(final String instanceName, int numAxes, int options, TrcFilter[] filters, AnalogDataSource dataSource)
    {
        super(instanceName, numAxes, options, filters);
        this.dataSource = dataSource;
    }   //TrcAnalogSensor

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param numAxes specifies the number of axes.
     * @param options specifies the AnalogInput options. Multiple options can be OR'd together.
     *                ANALOGINPUT_INTEGRATE - do integration on sensor data.
     *                ANALOGINPUT_DOUBLE_INTEGRATE - do double integration on sensor data.
     */
    public TrcAnalogSensor(final String instanceName, int numAxes, int options, AnalogDataSource dataSource)
    {
        this(instanceName, numAxes, options, null, dataSource);
    }   //TrcAnalogSensor

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public TrcAnalogSensor(final String instanceName, int numAxes, AnalogDataSource dataSource)
    {
        this(instanceName, numAxes, 0, null, dataSource);
    }   //TrcAnalogSensor

    /**
     * This abstract method returns the raw data with the specified index and type.
     *
     * @param index specifies the data index.
     * @param dataType specifies the data type.
     * @return raw data with the specified type.
     */
    public SensorData<Double> getRawData(int index, DataType dataType)
    {
        return new SensorData<>(TrcUtil.getCurrentTime(), dataSource.getData());
    }

}   //class TrcAnalogSensor
