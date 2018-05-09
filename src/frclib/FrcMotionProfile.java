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

package frclib;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class FrcMotionProfile
{
    public static FrcMotionProfile loadFromCsv(String path)
    {
        if(!path.endsWith(".csv")) throw new IllegalArgumentException("path must be a csv file!");

        try(BufferedReader in = new BufferedReader(new FileReader(path)))
        {
            List<FrcMotionProfilePoint> points = new ArrayList<>();
            String line;
            in.readLine(); // Get rid of the first line
            while((line = in.readLine()) != null)
            {
                Double[] parts = Arrays.stream(line.split(",")).map(Double::parseDouble).toArray(Double[]::new);
                if(parts.length != 8) throw new IllegalArgumentException("There must be 8 columns in the csv file!");
                FrcMotionProfilePoint point = new FrcMotionProfilePoint(parts[0], parts[1], parts[2], parts[3], parts[4],
                        parts[5], parts[6], parts[7]);
                points.add(point);
            }
            return new FrcMotionProfile(points.toArray(new FrcMotionProfilePoint[0]));
        }
        catch(IOException e)
        {
            throw new RuntimeException(e);
        }
    }

    private FrcMotionProfilePoint[] points;
    public FrcMotionProfile(FrcMotionProfilePoint[] points)
    {
        this.points = points;
    }

    public FrcMotionProfilePoint[] getPoints()
    {
        return points;
    }

    public void scale(double worldUnitsPerEncoderTick) {
        for(FrcMotionProfilePoint point:points)
        {
            point.encoderPosition /= worldUnitsPerEncoderTick;
        }
    }

    public Double[] getTimeSteps()
    {
        return Arrays.stream(points).map(e -> e.timeStep).toArray(Double[]::new);
    }

    public Double[] getXPositions()
    {
        return Arrays.stream(points).map(e -> e.x).toArray(Double[]::new);
    }

    public Double[] getYPositions()
    {
        return Arrays.stream(points).map(e -> e.y).toArray(Double[]::new);
    }

    public Double[] getEncoderPositions()
    {
        return Arrays.stream(points).map(e -> e.encoderPosition).toArray(Double[]::new);
    }

    public Double[] getVelocities()
    {
        return Arrays.stream(points).map(e -> e.velocity).toArray(Double[]::new);
    }

    public Double[] getAccelerations()
    {
        return Arrays.stream(points).map(e -> e.acceleration).toArray(Double[]::new);
    }

    public Double[] getJerks()
    {
        return Arrays.stream(points).map(e -> e.jerk).toArray(Double[]::new);
    }

    public Double[] getHeadings()
    {
        return Arrays.stream(points).map(e -> e.heading).toArray(Double[]::new);
    }

    public static class FrcMotionProfilePoint
    {
        public double timeStep, x, y, encoderPosition, velocity, acceleration, jerk, heading;
        public FrcMotionProfilePoint(double timeStep, double x, double y, double position, double velocity,
                                     double acceleration, double jerk, double heading)
        {
            this.timeStep = timeStep;
            this.x = x;
            this.y = y;
            this.encoderPosition = position;
            this.velocity = velocity;
            this.acceleration = acceleration;
            this.jerk = jerk;
            this.heading = heading;
        }
    }
}
