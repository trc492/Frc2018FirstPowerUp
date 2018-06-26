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
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.OptionalDouble;
import java.util.stream.Stream;

public class FrcMotionProfile
{

    public static FrcMotionProfile loadProfileFromCsv(String leftPath, String rightPath)
    {
        return loadProfileFromCsv(leftPath, rightPath, false);
    }

    public static FrcMotionProfile loadProfileFromCsv(String leftPath, String rightPath, boolean loadFromResources)
    {
        return new FrcMotionProfile(loadPointsFromCsv(leftPath, loadFromResources), loadPointsFromCsv(rightPath, loadFromResources));
    }

    public static FrcMotionProfilePoint[] loadPointsFromCsv(String path, boolean loadFromResources)
    {
        if(!path.endsWith(".csv")) throw new IllegalArgumentException(String.format("%s is not a csv file!", path));
        File file = loadFromResources ?
                new File(FrcMotionProfile.class.getClassLoader().getResource(path).getFile()) :
                new File(path);
        try(BufferedReader in = new BufferedReader(new FileReader(file)))
        {
            List<FrcMotionProfilePoint> points = new ArrayList<>();
            String line;
            in.readLine(); // Get rid of the first line
            while((line = in.readLine()) != null)
            {
                double[] parts = Arrays.stream(line.split(",")).mapToDouble(Double::parseDouble).toArray();
                if(parts.length != 8) throw new IllegalArgumentException("There must be 8 columns in the csv file!");
                FrcMotionProfilePoint point = new FrcMotionProfilePoint(parts[0], parts[1], parts[2], parts[3], parts[4],
                        parts[5], parts[6], parts[7]);
                points.add(point);
            }
            return points.toArray(new FrcMotionProfilePoint[0]);
        }
        catch(IOException e)
        {
            throw new RuntimeException(e);
        }
    }

    private FrcMotionProfilePoint[] leftPoints, rightPoints;
    public FrcMotionProfile(FrcMotionProfilePoint[] leftPoints, FrcMotionProfilePoint[] rightPoints)
    {
        if(leftPoints.length != rightPoints.length)
        {
            throw new IllegalArgumentException("leftPoints and rightPoints must have the same length!");
        }
        this.leftPoints = leftPoints;
        this.rightPoints = rightPoints;
    }

    public FrcMotionProfilePoint[] getLeftPoints()
    {
        return leftPoints;
    }

    public FrcMotionProfilePoint[] getRightPoints()
    {
        return rightPoints;
    }

    public void scale(double worldUnitsPerEncoderTick)
    {
        for(FrcMotionProfilePoint point:leftPoints)
        {
            point.encoderPosition /= worldUnitsPerEncoderTick;
        }

        for(FrcMotionProfilePoint point:rightPoints)
        {
            point.encoderPosition /= worldUnitsPerEncoderTick;
        }
    }

    public int getNumPoints()
    {
        return leftPoints.length;
    }

    public double getMinTimeStep()
    {
        OptionalDouble minTimeStep = Stream.concat(Arrays.stream(leftPoints), Arrays.stream(rightPoints))
                .mapToDouble(p -> p.timeStep).min();
        if(minTimeStep.isPresent())
        {
            return minTimeStep.getAsDouble();
        }
        throw new IllegalStateException("For some reason the streaming returned a null! I don't know why!");
    }

    public FrcMotionProfile copy()
    {
        FrcMotionProfilePoint[] left = Arrays.stream(leftPoints)
                .map(FrcMotionProfilePoint::new)
                .toArray(FrcMotionProfilePoint[]::new);
        FrcMotionProfilePoint[] right = Arrays.stream(rightPoints)
                .map(FrcMotionProfilePoint::new)
                .toArray(FrcMotionProfilePoint[]::new);
        return new FrcMotionProfile(left, right);
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

        public FrcMotionProfilePoint(FrcMotionProfilePoint other)
        {
            this.timeStep = other.timeStep;
            this.x = other.x;
            this.y = other.y;
            this.encoderPosition = other.encoderPosition;
            this.velocity = other.velocity;
            this.acceleration = other.acceleration;
            this.jerk = other.jerk;
            this.heading = other.heading;
        }
    }
}
